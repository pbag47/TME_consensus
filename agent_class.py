import _csv
import numpy
# noinspection PyProtectedMember
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from qtm.packet import RT3DMarkerPositionNoLabel
from typing import List, Union


class Agent:
    def __init__(self, name, uri):
        self.name: str = name
        self.uri: str = uri

        # ---- Parameters ---- #
        self.max_roll: float = 45  # Maximum |roll| tolerance before emergency stop (°)
        self.max_pitch: float = 45  # Maximum |pitch| tolerance before emergency stop (°)

        self.x_boundaries: List[float] = [-1.10, 1.10]  # [Minimum x coordinate (m), Maximum x coordinate (m)]
        self.y_boundaries: List[float] = [-1.10, 1.1]  # [Minimum y coordinate (m), Maximum y coordinate (m)]
        self.z_boundaries: List[float] = [-0.1, 0.8]  # [Minimum z coordinate (m), Maximum z coordinate (m)]

        # Crazyflie parameters:
        # -- Warning -- May cause chaotic flight when badly configured, do not attempt to change these settings without
        # a consistent knowledge of the Crazyflie parameters documentation
        self.stabilizer_estimator = 2  # 2: stabilizer based on an Extended Kalman Filter
        self.flightmode_pos_set = 0

        # Crazyflie variables log:
        # -- Warning -- The program must at least retrieve the attitude information (roll, pitch and yaw) and the
        # battery mode for each Crazyflie to perform attitude and battery security checks and to control their yaw.
        # The user may add other variables to log (e.g. battery level, radio link quality, etc...) with respects to
        # the radio communication limits.
        # Please check the validity of a custom log configuration with the cfclient before implementing it on this
        # program.
        self.log_config = LogConfig(name=self.name + ' attitude', period_in_ms=50)
        self.log_config.add_variable('stabilizer.roll', 'float')  # Retrieves the roll from Crazyflie stabilizer (EKF)
        self.log_config.add_variable('stabilizer.pitch', 'float')  # Retrieves the pitch from Crazyflie stabilizer (EKF)
        self.log_config.add_variable('stabilizer.yaw', 'float')  # Retrieves the yaw from Crazyflie stabilizer (EKF)
        # Retrieves the battery mode (0 = nominal mode, 1 = loading, 2 = loading completed, 3 = Low energy mode)
        self.log_config.add_variable('pm.state', 'float')

        # ---- Attributes initialization ---- #
        self.cf: Crazyflie = Crazyflie()
        self.state: str = 'Not flying'
        self.is_flying: bool = False
        self.enabled: bool = False
        self.battery_test_passed = False
        self.extpos_test_passed = False

        self.setup_finished: bool = False
        self.initial_battery_voltage: float = 0.0
        self.initial_battery_level: int = 0

        self.initial_position: [float] * 3 = [0, 0, 0]

        self.takeoff_position: List[float] = []
        self.takeoff_yaw: float = 0.0
        self.takeoff_height: float = 0.5

        self.land_position: List[float] = []
        self.land_yaw: float = 0.0

        self.standby_position: List[float] = []
        self.standby_yaw: float = 0.0

        self.consensus_connectivity: List[str] = []
        self.z_consensus_xy_position: List[float] = []
        self.xy_consensus_z: float = 0.0
        self.xy_consensus_offset: [float, float] = [0, 0]

        self.xy_auto_avoid_obstacles_list: List[[float] * 3] = []
        self.xy_auto_avoid_agents_list: List[str] = []

        self.back_to_init_yaw: float = 0

        self.timestamp: float = 0.0
        self.extpos: RT3DMarkerPositionNoLabel = RT3DMarkerPositionNoLabel(0, 0, 0, None)
        self.yaw: float = 0.0
        self.velocity: [float] * 3 = [0.0, 0.0, 0.0]
        self.delta_t: float = 1.0
        self.invalid_6dof_count: int = 0
        self.previous_iz: int = 0

        self.csv_logger: Union[None, _csv.writer] = None

    def connect_cf(self):
        self.cf.open_link(self.uri)
        self.cf.connected.add_callback(self.cf_connected_callback)
        self.cf.connection_failed.add_callback(self.cf_connection_failed_callback)
        self.cf.disconnected.add_callback(self.cf_disconnected_callback)

    def cf_connected_callback(self, _):
        self.enabled = True
        self.cf.commander.send_setpoint(0, 0, 0, 0)

    def cf_connection_failed_callback(self, _, __):
        self.battery_test_passed = True
        self.extpos_test_passed = True
        self.enabled = False
        print(self.name, 'connection attempt failed')

    def setup_parameters(self):
        log_config = LogConfig(name=self.name + ' battery voltage', period_in_ms=50)
        log_config.add_variable('pm.vbat', 'float')  # Retrieves the battery level
        self.cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(self.check_battery_level)
        log_config.start()
        self.cf.param.set_value('stabilizer.estimator', str(self.stabilizer_estimator))
        self.cf.param.set_value('flightmode.posSet', str(self.flightmode_pos_set))
        self.check_initial_extpos()
        if not self.enabled:
            self.stop()

    def check_battery_level(self, _, data, logconf):
        self.initial_battery_voltage = data['pm.vbat']
        logconf.stop()
        voltage_points = [3.0, 3.6, 3.65, 3.75, 3.85, 4.0, 4.2]
        battery_level_points = [0, 5, 10, 25, 50, 75, 100]
        self.initial_battery_level = numpy.interp(self.initial_battery_voltage, voltage_points, battery_level_points)
        if self.initial_battery_level <= 20:
            print('---- Warning ----', self.name, ': Low battery level (', round(self.initial_battery_level),
                  '% ), flight disabled')
            self.enabled = False
        else:
            self.battery_test_passed = True

    def check_initial_extpos(self):
        if self.extpos.id is None:
            print(self.name, '---- Warning ---- Crazyflie not tracked, flight disabled')
            self.enabled = False
        else:
            self.extpos_test_passed = True

    def start_attitude_logs(self):
        self.cf.log.add_config(self.log_config)
        self.log_config.data_received_cb.add_callback(self.check_attitude)
        self.log_config.start()

    def check_attitude(self, _, data, __):
        if self.enabled and (abs(data['stabilizer.roll']) > self.max_roll
                             or abs(data['stabilizer.pitch']) > self.max_pitch
                             or self.extpos.x < self.x_boundaries[0] or self.extpos.x > self.x_boundaries[1]
                             or self.extpos.y < self.y_boundaries[0] or self.extpos.y > self.y_boundaries[1]
                             or self.extpos.z < self.z_boundaries[0] or self.extpos.z > self.z_boundaries[1]):
            print(self.name, '---- Warning ---- Abnormal attitude detected')
            print(self.name, 'attitude : {Roll =', round(data['stabilizer.roll']),
                  '° ; Pitch =', round(data['stabilizer.pitch']),
                  '° ; x =', round(self.extpos.x, 2), 'm ; y =', round(self.extpos.y, 2), 'm ; z =',
                  round(self.extpos.z, 2), 'm}')
            print(self.name, 'nominal attitude boundaries : {|Roll| <', self.max_roll, '° ; |Pitch| <',
                  self.max_pitch,
                  '°, x in', self.x_boundaries, 'm, y in', self.y_boundaries, 'm, z in', self.z_boundaries, 'm}')
            self.stop()
        else:
            self.yaw = data['stabilizer.yaw']

        if self.enabled and round(data['pm.state']) == 3 and self.state != 'Land':
            print('---- Warning ----', self.name, ': Low battery level, automatic landing triggered')
            self.land()

    def cf_disconnected_callback(self, _):
        self.battery_test_passed = True
        self.extpos_test_passed = True
        self.enabled = False
        print(self.name, 'disconnected')

    def stop(self):
        if self.setup_finished:
            try:
                self.log_config.stop()
            except AttributeError:
                print(self.name, 'Warning : Unable to stop attitude logging')
        self.state = 'Not flying'
        self.is_flying = False
        self.enabled = False
        self.cf.commander.send_stop_setpoint()

    def send_external_position(self):
        self.cf.extpos.send_extpos(self.extpos.x, self.extpos.y, self.extpos.z)

    def update_extpos(self, marker: RT3DMarkerPositionNoLabel, timestamp: float):
        if timestamp > self.timestamp:
            self.delta_t = timestamp - self.timestamp
            self.update_velocity(marker, timestamp)
            self.timestamp = timestamp
            self.extpos = RT3DMarkerPositionNoLabel(marker.x * 10 ** -3, marker.y * 10 ** -3, marker.z * 10 ** -3,
                                                    marker.id)
        else:
            print('Warning :', self.name, ': Several extpos packets received for the same timestamp')

    def update_velocity(self, marker: RT3DMarkerPositionNoLabel, timestamp: float):
        self.velocity[0] = (marker.x * 10 ** -3 - self.extpos.x) / (timestamp - self.timestamp)
        self.velocity[1] = (marker.y * 10 ** -3 - self.extpos.y) / (timestamp - self.timestamp)
        self.velocity[2] = (marker.z * 10 ** -3 - self.extpos.z) / (timestamp - self.timestamp)

    def set_initial_position(self, position: [float] * 3):
        self.initial_position = position

    def set_takeoff_height(self, height: float):
        self.takeoff_height = height

    def set_consensus_connectivity(self, connections: List[str]):
        self.consensus_connectivity = connections

    def set_xy_consensus_offset(self, offset: [float, float]):
        self.xy_consensus_offset = offset

    def set_agents_to_avoid(self, agents_names: List[str]):
        self.xy_auto_avoid_agents_list = agents_names

    def set_xy_auto_avoid_obstacles(self, obstacles: List[List[float]]):
        self.xy_auto_avoid_obstacles_list = obstacles

    def standby(self):
        print(self.name, ': Standby')
        self.state = 'Standby'
        self.standby_position = [self.extpos.x, self.extpos.y, self.extpos.z]
        self.standby_yaw = self.yaw

    def takeoff(self):
        print(self.name, ': Takeoff')
        self.state = 'Takeoff'
        self.is_flying = True
        self.takeoff_position = [self.extpos.x, self.extpos.y, self.takeoff_height]
        self.takeoff_yaw = self.yaw

    def land(self):
        print(self.name, ': Landing')
        self.state = 'Land'
        self.land_position = [self.extpos.x, self.extpos.y, 0.0]
        self.land_yaw = self.yaw

    def manual_flight(self):
        print(self.name, ': Manual flight')
        self.state = 'Manual'
        self.standby_position = [self.extpos.x, self.extpos.y, self.extpos.z]
        self.standby_yaw = self.yaw

    def xy_consensus(self):
        print(self.name, ': x and y consensus')
        self.state = 'xy_consensus'
        self.xy_consensus_z = self.extpos.z

    def z_consensus(self):
        print(self.name, ': z consensus')
        self.state = 'z_consensus'
        self.z_consensus_xy_position = [self.extpos.x, self.extpos.y]

    def back_to_initial_position(self):
        print(self.name, ': Back to initial position')
        self.state = 'Back_to_init'
        self.back_to_init_yaw = self.yaw

    def log_flight_data(self):
        self.csv_logger.writerow([self.name, self.timestamp,
                                  self.extpos.x, self.extpos.y, self.extpos.z, self.yaw,
                                  self.velocity[0], self.velocity[1], self.velocity[2]])
