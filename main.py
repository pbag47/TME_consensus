import asyncio
import cf_info
import cflib.crtp
import csv
import logging
import pynput.keyboard
import qasync
import qtm_tools
import sys

from joystick_class import Joystick
from main_ui import Window
from PyQt5.QtWidgets import QApplication
from qtm import QRTConnection
from qtm.packet import QRTPacket
from swarm_object_class import SwarmObject


logger = logging.getLogger(__name__)


async def start_qtm_streaming(connection: QRTConnection, callback):
    """ Starts a QTM stream, and assigns a callback method to run each time a QRTPacket is received from QTM
     This method is made to run forever in an asyncio event loop """
    logger.info('QTM streaming started')
    await connection.stream_frames(components=['3dnolabels'], on_packet=callback)


async def stop_qtm_streaming(connection: QRTConnection):
    await connection.stream_frames_stop()
    logger.info('QTM streaming stopped')


async def keyboard_handler():
    global SWARM_MANAGER
    key_queue = detect_keyboard_input()
    while True:
        key = await key_queue.get()
        if key == pynput.keyboard.Key.esc:
            logger.info('Esc key pressed, disconnecting')
            for agt in SWARM_MANAGER.swarm_agent_list:
                agt.cf.commander.send_stop_setpoint()
                agt.stop()
            asyncio.get_event_loop().stop()


def detect_keyboard_input():
    queue = asyncio.Queue()
    loop = asyncio.get_event_loop()

    def on_press_callback(key):
        try:
            loop.call_soon_threadsafe(queue.put_nowait, key.char)
        except AttributeError:
            loop.call_soon_threadsafe(queue.put_nowait, key)

    pynput.keyboard.Listener(on_press=on_press_callback).start()
    return queue


def packet_reception_callback(packet: QRTPacket):
    global SWARM_MANAGER
    global RUN_TRACKER

    if not RUN_TRACKER:
        logger.error('QTM packet callback interrupted by the reception of a new packet')
        for agents_to_stop in SWARM_MANAGER.swarm_agent_list:
            agents_to_stop.stop()
        raise BlockingIOError('High delay detected, unable to keep up with real-time processing rate')

    RUN_TRACKER = False
    timestamp = packet.timestamp * 10 ** -6
    _, markers = packet.get_3d_markers_no_label()
    lost_uav = qtm_tools.uav_tracking(SWARM_MANAGER.swarm_agent_list, markers, timestamp)
    if lost_uav:
        [SWARM_MANAGER.remove_agent(agt) for agt in lost_uav]
    SWARM_MANAGER.flight_sequence()
    RUN_TRACKER = True


def main():
    global SWARM_MANAGER
    global RUN_TRACKER

    # -- Flight parameters ------------------------------------------------------- #
    qtm_ip_address: str = '192.168.0.1'
    agents_list = cf_info.init_agents()

    # -- Logging configuration --------------------------------------------------- #
    log_level = logging.WARNING
    all_loggers = [logging.getLogger(name) for name in logging.root.manager.loggerDict]
    for log in all_loggers:
        log.setLevel(log_level)
    logging.getLogger('joystick_class').setLevel(logging.INFO)
    logger.setLevel(logging.INFO)

    # -- User interface setup ---------------------------------------------------- #
    app_test = QApplication(sys.argv)
    user_window = Window(uavs=agents_list, parameters_filename='flight_parameters.txt')

    # -- Asyncio loop setup ------------------------------------------------------ #
    q_loop = qasync.QEventLoop(app_test)
    asyncio.set_event_loop(q_loop)

    # -- QTM connection ---------------------------------------------------------- #
    qtm_connection: QRTConnection = asyncio.get_event_loop().run_until_complete(
        qtm_tools.connect_to_qtm(qtm_ip_address))

    # -- Asyncio loop run (user interface + QTM streaming) ----------------------- #
    if not qtm_connection:
        logger.warning('QTM not connected, displaying UI in settings-only mode')
        q_loop.run_forever()
        return

    asyncio.ensure_future(start_qtm_streaming(qtm_connection, user_window.update_graph))
    q_loop.run_forever()
    asyncio.get_event_loop().run_until_complete(stop_qtm_streaming(qtm_connection))

    # -- UAV objects retrieval --------------------------------------------------- #
    agents = user_window.agents_list
    [agent.set_agents_to_avoid([agt.name for agt in agents if agt.name != agent.name]) for agent in agents]

    # -- Logs initialization ----------------------------------------------------- #
    file = open('logs.csv', 'w')
    writer = csv.writer(file)
    writer.writerow(['Crazyflie name', 'QTM packet timestamp (s)',
                     'x (m)', 'y (m)', 'z (m)', 'yaw (°)',
                     'vx (m/s)', 'vy (m/s)', 'vz (m/s)',
                     'vx_c (m/s)', 'vy_c (m/s)', 'vz_c (m/s)',
                     'roll_c (°)', 'pitch_c (°)', 'yaw_rate_c (°/s)', 'thrust_c (PWM)'])

    # -- Initial frame acquisition ----------------------------------------------- #
    header, markers, timestamp = qtm_tools.frame_acquisition(qtm_connection)
    logger.info(str(header.marker_count) + ' markers found by QTM during initialization')
    qtm_tools.initial_uav_detection(agents, markers, timestamp)

    # -- Crazyflie connection and swarm initialization procedure ----------------- #
    cflib.crtp.init_drivers()
    SWARM_MANAGER = SwarmObject()
    _ = Joystick(SWARM_MANAGER)
    for agent in agents:
        agent.connect_cf()
        agent.csv_logger = writer
        SWARM_MANAGER.add_agent(agent)

    SWARM_MANAGER.manual_flight_agents_list = user_window.manual_agents_list

    # -- Real-time stream start -------------------------------------------------- #
    RUN_TRACKER = True
    asyncio.ensure_future(start_qtm_streaming(qtm_connection, packet_reception_callback))
    asyncio.ensure_future(keyboard_handler())
    asyncio.get_event_loop().run_forever()

    # -- Disconnect procedure ---------------------------------------------------- #
    for agent in agents:
        agent.stop()
        agent.enabled = False
        agent.cf.close_link()

    asyncio.get_event_loop().run_until_complete(qtm_tools.disconnect_qtm(qtm_connection))
    file.close()

    for agent in agents:
        if agent.error:
            raise agent.error


if __name__ == '__main__':
    global SWARM_MANAGER
    global RUN_TRACKER
    main()
