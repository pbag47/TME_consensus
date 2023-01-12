import asyncio
import qtm
import numpy as np

from agent_class import Agent
from qtm import QRTConnection
from qtm.packet import RT3DMarkerPositionNoLabel
from typing import List


async def connect_to_qtm(ip: str):
    connection: QRTConnection = await qtm.connect(ip)
    if connection:
        print('QTM connected @', ip)
    else:
        print(' ---- Warning ---- Error during QTM connection @', ip, '***')
    return connection


def frame_acquisition(connection: QRTConnection):
    frame: qtm.QRTPacket = asyncio.get_event_loop().run_until_complete(
        connection.get_current_frame(components=['3dnolabels']))
    timestamp = frame.timestamp * 10 ** -6
    headers, markers = frame.get_3d_markers_no_label()
    return headers, markers, timestamp


def initial_uav_detection(agents: List[Agent], markers: List[RT3DMarkerPositionNoLabel], timestamp: float):
    if len(markers) < len(agents):
        print(' -- Warning -- : Too few detected makers than expected, flight disabled')
        for agt in agents:
            agt.enabled = False
    elif len(markers) > len(agents):
        print(' -- Warning -- : Too many detected markers than expected')

    for mk in markers:
        d = [distance_init_pos_to_marker(agt.initial_position,
                                         [mk.x * 10 ** -3, mk.y * 10 ** -3, mk.z * 10 ** -3]) for agt in agents]
        try:
            min_d_index = d.index(min(d))
            agt = agents[min_d_index]
            agt.update_extpos(mk, timestamp)
            print(agt.name, 'found @', agt.extpos)
            if d[min_d_index] > 0.5:
                print(' ---- Warning ---- UAV <', agt.name, '> marker found too far from expected initial position')
                print('                   High risk of marker mismatch')
        except ValueError:
            break


def uav_tracking(agents: List[Agent], markers: List[RT3DMarkerPositionNoLabel], timestamp: float) -> List[Agent]:
    markers_ids = [mk.id for mk in markers]
    lost_uav = []
    for agt in agents:
        try:
            index = markers_ids.index(agt.extpos.id)
            agt.update_extpos(markers[index], timestamp)
            agt.send_external_position()
        except ValueError:
            print(' ---- Warning ----', agt.name, 'tracking lost, switching the engines off')
            agt.stop()
            lost_uav.append(agt)
    return lost_uav


def distance_init_pos_to_marker(position_1: [float] * 3, position_2: [float] * 3):
    d = np.sqrt((position_1[0] - position_2[0]) ** 2
                + (position_1[1] - position_2[1]) ** 2
                + (position_1[2] - position_2[2]) ** 2)
    return d


def distance_between_markers(marker_1: RT3DMarkerPositionNoLabel, marker_2: RT3DMarkerPositionNoLabel):
    """
    Method that returns the distance (m) between marker_1 position (m) and
    marker_2 position (mm)
    """
    d = np.sqrt((marker_1.x - marker_2.x * 10 ** -3) ** 2
                + (marker_1.y - marker_2.y * 10 ** -3) ** 2
                + (marker_1.z - marker_2.z * 10 ** -3) ** 2)
    return d


async def disconnect_qtm(connection: QRTConnection):
    if connection is not None:
        await connection.stream_frames_stop()
        connection.disconnect()
        print('QTM disconnected')
    else:
        print('QTM connection already closed')


if __name__ == '__main__':
    qtm_ip_address = '192.168.0.1'
    qtm_connection = asyncio.get_event_loop().run_until_complete(connect_to_qtm(qtm_ip_address))
    h, m, t = frame_acquisition(qtm_connection)
    print('QTM packet received:')
    print('    Headers:', h)
    print('    Markers:', m)
    print('    Timestamp:', t)
    disconnect_qtm(qtm_connection)