import traceback

from Niveau3 import *
from time import perf_counter, sleep
from Niveau1 import *
import Outils as outils
from typing import Tuple


# CT = Conditional Transition
# SVC = State Value Condition
def add_CT_with_SVC(state_from: 'RobotState', state_to: MonitoredState, expected_value):
    stateValueCondition = StateValueCondition(expected_value=expected_value, monitored_state=state_from)
    condition_transition = ConditionalTransition(
        condition=stateValueCondition, next_state=state_to)
    state_from.add_transition(condition_transition)


# CT = Conditional Transition
# ATC = Always True Condition
def add_CT_with_ATC(state_from: 'RobotState', state_to: MonitoredState):
    condition_transition = ConditionalTransition(condition=AlwaysTrueCondition(), next_state=state_to)
    state_from.add_transition(condition_transition)


# CT = Conditional Transition
# SEDC = State Entry Duration Condition
def add_CT_with_SEDC(state_from: 'RobotState', state_to: MonitoredState, duration: float):
    stateEntryDurationCondition = StateEntryDurationCondition(duration=duration, monitored_state=state_from)
    condition_transition = ConditionalTransition(condition=stateEntryDurationCondition, next_state=state_to)
    state_from.add_transition(condition_transition)


# RCT = RemoteControlCondition
def add_RCT(state_from: 'RobotState', state_to, expected_input: str, remote_control: 'RemoteControl'):
    remoteControlCondition = RemoteControlCondition(remote_control=remote_control,
                                                    expected_input=expected_input)
    conditional_transition = ConditionalTransition(condition=remoteControlCondition, next_state=state_to)
    state_from.add_transition(conditional_transition)


# RFT = RemoteFinderTransition
def add_RFT(state_from: 'RobotState', state_to, robot, wanted_distance):
    rangeFinderCondition = RangeFinderCondition(robot=robot, wanted_distance=wanted_distance)
    conditional_transition = ConditionalTransition(condition=rangeFinderCondition, next_state=state_to)
    state_from.add_transition(conditional_transition)


class LedBlinkers(SideBlinker):
    class LedBlinkerState(MonitoredState):
        def __init__(self):
            self._led = None
            self._robot: Robot
            super().__init__()

        @property
        def led(self):
            return self._led

        @property
        def robot(self):
            return self._robot

        @led.setter
        def led(self, port: str):
            verify_instance(port, str)
            if 'right' == port or port == 'left':
                self._led = port
            else:
                raise ValueError(
                    "The entered string value needs to be right or left.")

        @robot.setter
        def robot(self, value: 'Robot'):
            if value is not None:
                verify_instance(value, Robot)
            self._robot = value

    class OffLedBlinkersState(LedBlinkerState):
        def __int__(self):
            super().__init__()

        def _do_entering_action(self):
            try:
                self._robot.action_led(led=self.led)
            except:
                print("off")

    class OnLedBlinkersState(LedBlinkerState):
        def __init__(self):
            super().__init__()

        def _do_entering_action(self):
            try:
                self._robot.action_led(close=False, led=self.led)
            except:
                print("on")

    def __init__(self, robot):
        self.__robot: Robot = robot

        super().__init__(left_off_state_generator=self.initialised_led_off_left,
                         left_on_state_generator=self.initialised_led_on_left,
                         right_off_state_generator=self.initialised_led_off_right,
                         right_on_state_generator=self.initialised_led_on_right)

    def initialised_led_off_left(self):
        state = self.OffLedBlinkersState()
        state.led = "left"
        state.robot = self.__robot
        return state

    def initialised_led_on_left(self):
        state = self.OnLedBlinkersState()
        state.led = "left"
        state.robot = self.__robot
        return state

    def initialised_led_off_right(self):
        state = self.OffLedBlinkersState()
        state.led = "right"
        state.robot = self.__robot
        return state

    def initialised_led_on_right(self):
        state = self.OnLedBlinkersState()
        state.led = "right"
        state.robot = self.__robot
        return state


class EyeBlinkers(SideBlinker):
    class EyeBlinkerState(MonitoredState):
        def __init__(self):
            self.__eye: [int, str] = None
            self.__robot: Robot
            super().__init__()

        @property
        def eye(self):
            return self.__eye

        @property
        def robot(self):
            return self.__robot

        @eye.setter
        def eye(self, value: str):
            verify_instance(value, str)
            if 'right' == value or value == 'left':
                self.__eye = value
            else:
                raise ValueError(
                    "The entered value is wrong. It need to be a str of value right or left.")

        @robot.setter
        def robot(self, value: 'Robot'):
            if value is not None:
                verify_instance(value, Robot)
            self.__robot = value

    class OffEyeBlinkersState(EyeBlinkerState):
        def __int__(self):
            super().__init__()

        def _do_entering_action(self):
            try:
                if self.eye == 'left':
                    self.robot.action_eyes(eye=self.eye)
                elif self.eye == 'right':
                    self.robot.action_eyes(eye=self.eye)
            except:
                print("ferme blinker")

    class OnEyeBlinkersState(EyeBlinkerState):
        def __init__(self):
            super().__init__()

        def _do_entering_action(self):
            try:
                if self.eye == 'left':
                    self.robot.action_eyes(close=False, eye=self.eye)
                elif self.eye == 'right':
                    self.robot.action_eyes(close=False, eye=self.eye)
            except:
                print("ouvre blinker")

    def __init__(self, robot: 'Robot'):
        self.__robot: Robot = robot
        super().__init__(left_off_state_generator=self.initialised_eyes_off_left,
                         left_on_state_generator=self.initialised_eyes_on_left,
                         right_off_state_generator=self.initialised_eyes_off_right,
                         right_on_state_generator=self.initialised_eyes_on_right)

    def initialised_eyes_off_left(self):
        state = self.OffEyeBlinkersState()
        state.eye = "left"
        state.robot = self.__robot
        return state

    def initialised_eyes_on_left(self):
        state = self.OnEyeBlinkersState()
        state.eye = "left"
        state.robot = self.__robot
        return state

    def initialised_eyes_off_right(self):
        state = self.OffEyeBlinkersState()
        state.eye = "right"
        state.robot = self.__robot
        return state

    def initialised_eyes_on_right(self):
        state = self.OnEyeBlinkersState()
        state.eye = "right"
        state.robot = self.__robot
        return state

    def change_eyes_color(self, color: Tuple[int, int, int], **kwargs):
        '''
        This function change the color of the specified eye(s).
        :param color: tuple[int, int, int] need to be > 0
        :param kwargs :
        :param right -> bool
        :param left -> bool
        :return: None
        '''

        def clamp(tup: Tuple[int, int, int]):
            new_tuple = []
            for value in tup:
                verify_instance(value, int)
                value = 0 if value < 0 else 255 if value > 255 else value
                new_tuple.append(value)
            tup = tuple(new_tuple)
            return tup

        if verify_instance(color, tuple):
            color = clamp(color)

        key = set(kwargs.keys())
        if key == set():
            raise ValueError("Need to have an eye and a color to work with.")
        elif key == {'left', 'right'}:
            verify_instance(kwargs["left"], bool)
            verify_instance(kwargs["right"], bool)

            if kwargs['left'] and kwargs['right']:
                self.__robot.set_eyes_color(color)
            elif kwargs['left'] and not kwargs['right']:
                self.__robot.set_eyes_color(color, "left")
            elif not kwargs['left'] and kwargs['right']:
                self.__robot.set_eyes_color(color, "right")

            else:
                print("aucun des deux yeux")
        else:
            raise ValueError("Les éléments ne sont pas pris en charge")


class RemoteControl:
    def __init__(self, robot, initialized: bool = False):  # gpg.GoPiGo3
        # verify_instance(robot, gpg.GoPiGo3)
        verify_instance(initialized, bool)
        self.__robot: Robot = robot
        self.__port = 'AD1'
        self.__current_key = ""
        self.__remote_control = None
        self.__key_codes = ['', 'up', 'left', 'ok', 'right', 'down', '1', '2', '3', '4', '5', '6', '7', '8', '9', '*',
                            '0', '#']
        self.__duration = 2.0
        if initialized:
            self.init_remote()

    @property
    def port(self):
        return self.__port

    @property
    def current_key(self):
        return self.__current_key

    @current_key.setter
    def current_key(self, value: str):
        verify_instance(value, str)
        self.__current_key = value

    @port.setter
    def port(self, value: str):
        verify_instance(value, str)
        self.__port = value

    def init_remote(self):
        self.__remote_control = self.__robot.gpg.init_remote(port=self.port)
        if self.__remote_control is not None:
            print("Remote Control instantiate")
            return True
        else:
            print("Remote Control coudn't be instantiate")
            return False

    @property
    def return_key(self):
        return self.__key_codes[self.__remote_control.read()]
    

class Robot:
    def __init__(self):
        self.__instantiate = "succeeded"
        self.__integrity = "succeeded"
        try:
            import easygopigo3 as gpg
            self.__gpg = gpg.EasyGoPiGo3()
        except:
            self.__instantiate = "failed"
            self.__integrity = "failed"
        self.__led_blinkers = LedBlinkers(self)
        self.__eyes_blinkers = EyeBlinkers(self)
        self._side = self.__eyes_blinkers.Side
        self.remote_control = RemoteControl(self)
        self.__speed = 10.0
        self.__distance_sensor = None
        self.__servo_camera = None
        self.__servo_distance_sensor = None
        self.__set_blinker = {"right", "left", "both"}

    @property
    def is_instanciate(self):
        return self.__instantiate

    @property
    def check_integrity(self):
        return self.__integrity

    @property
    def gpg(self):
        return self.__gpg

    @property
    def led_blinkers(self):
        return self.__led_blinkers

    @property
    def eyes_blinkers(self):
        return self.__eyes_blinkers

    @property
    def speed(self):
        return self.__speed

    @property
    def side(self):
        return self._side

    @property
    def distance_sensor(self):
        return self.__distance_sensor

    @gpg.setter
    def gpg(self, value):
        self.__gpg = value

    @led_blinkers.setter
    def led_blinkers(self, value):
        self.__led_blinkers = value

    @eyes_blinkers.setter
    def eyes_blinkers(self, value):
        self.__eyes_blinkers = value

    @speed.setter
    def speed(self, value: float):
        self.__speed = value
        self.gpg.set_speed(self.__speed)

    def eyes_on(self, side: SideBlinker.Side, **kwargs):
        if side > self.eyes_blinkers.Side.RIGHT_RECIPROCAL:
            raise ValueError("The specified side is not an option.")
        self.eyes_blinkers.turn_on(side, **kwargs)

    def eyes_off(self, side: SideBlinker.Side, **kwargs):
        if side > self.eyes_blinkers.Side.RIGHT_RECIPROCAL:
            raise ValueError("The specified side is not an option.")
        self.eyes_blinkers.turn_off(side, **kwargs)

    def eye_blink(self, side: SideBlinker.Side, percent_on: float = 0.5, begin_on: bool = True, **kwargs):
        if side > self.eyes_blinkers.Side.RIGHT_RECIPROCAL:
            raise ValueError("The specified side is not an option.")
        self.eyes_blinkers.blink(side=side, percent_on=percent_on, begin_on=begin_on, **kwargs)

    def change_eyes_color(self, color: Tuple[int, int, int], right_eye: bool, left_eye: bool):
        self.eyes_blinkers.change_eyes_color(color, right=right_eye, left=left_eye)

    def wink(self, side, wink_duration, n_winks, wait_duration, percent_on, wait_off, begin_on):
        self.eyes_blinkers.wink(side=side, wink_duration=wink_duration, n_winks=n_winks, wait_duration=wait_duration,
                                percent_on=percent_on, wait_off=wait_off, begin_on=begin_on)

    def action_eyes(self, close: bool = True, eye: str = "both"):
        """
        This function close or open the eyes of the robot.
        :param close: bool, default True
        :param eye: right, left, both
        :return:
        """
        verify_instance(close, bool)
        if eye not in self.__set_blinker:
            raise ValueError("The eye value '{eye}' is not valid")
        if eye == "right":
            if close:
                self.__gpg.close_right_eye()
            else:
                self.__gpg.open_right_eye()
        elif eye == "left":
            if close:
                self.__gpg.close_left_eye()
            else:
                self.__gpg.open_left_eye()
        elif eye == "both":
            if close:
                self.__gpg.close_eyes()
            else:
                self.__gpg.open_eyes()

    def set_eyes_color(self, color: Tuple[int, int, int], eye: str = "both"):
        """
        This function change the color on the eyes of the GoPiGo3
        :param color:
        :param eye: a str that take 3 value right, left and both
        :return:
        """
        if eye not in self.__set_blinker:
            raise ValueError("The eye value '{eye}' is not valid")
        if eye == 'right':
            self.__gpg.set_right_eye_color(color)
        elif eye == 'left':
            self.__gpg.set_left_eye_color(color)
        elif eye == 'both':
            self.__gpg.set_eye_color(color)

    def led_on(self, side: SideBlinker.Side, **kwargs):
        if side > self.led_blinkers.Side.RIGHT_RECIPROCAL:
            raise ValueError("The specified side is not an option.")
        self.led_blinkers.turn_on(side, **kwargs)

    def led_off(self, side: SideBlinker.Side, **kwargs):
        self.led_blinkers.turn_on(side, **kwargs)

    def led_blink(self, side: SideBlinker.Side, percent_on: float = 0.5, begin_on: bool = True, **kwargs):
        self.led_blinkers.blink(side=side, percent_on=percent_on, begin_on=begin_on, **kwargs)

    # todo ajouter les kwargs
    def led_wink(self, side, wink_duration, wait_duration, n_winks, percent_on, wait_off, begin_on):
        self.led_blinkers.wink(side=side, wink_duration=wink_duration, n_winks=n_winks, wait_duration=wait_duration,
                               percent_on=percent_on, wait_off=wait_off, begin_on=begin_on)

    def action_led(self, close: bool = True, led: str = "both"):
        """
        This function turn on or close the specified led.
        :param close:
        :param kwargs:
        :return:
        """
        verify_instance(close, bool)
        if led not in self.__set_blinker:
            raise ValueError("The eye value '{eye}' is not valid")
        if led == 'right':
            if close:
                self.__gpg.led_off(led)
            else:
                self.__gpg.led_on(led)
        elif led == 'left':
            if close:
                self.__gpg.led_off(led)
            else:
                self.__gpg.led_on(led)
        elif led == 'both':
            if close:
                self.__gpg.led_off(led)
            else:
                self.__gpg.led_on(led)

    def move_forward(self):
        self.gpg.forward()

    def turn_right(self):
        self.gpg.right()

    def turn_left(self):
        self.gpg.left()

    def stop_moving(self):
        self.gpg.stop()

    def rotate(self, degrees: int):
        self.gpg.turn_degrees(degrees)

    def move_backward(self):
        self.gpg.backward()

    def track(self):
        self.__eyes_blinkers.track()
        self.__led_blinkers.track()

    def instantiate_rangefinder(self, port: str = "I2C") -> bool:
        success = False
        self.__distance_sensor = self.__gpg.init_distance_sensor(port=port)
        if self.__distance_sensor is not None:
            print("RangeFinder instantiate")
            success = True
        else:
            print("RangeFinder coudn't be instantiate")
        return success

    def read_distance(self, value: str = "mm"):
        """
        This fonction use the RangeFinder to detected the distance between the robot and
        an obstacle in front of him.
        :param value:
        :return:
        """
        valeur = 0
        if value == "mm":
            valeur = self.__distance_sensor.read_mm()
        elif value == "cm":
            valeur = self.__distance_sensor.read()
        elif value == "inches":
            valeur = self.__distance_sensor.read_inches()
        return valeur

    def instantiate_servo_camera(self, port: str = "SERVO1"):
        self.__servo_camera = self.__gpg.init_servo(port=port)
        if self.__servo_camera is not None:
            print("Camera Servo instantiate")
            return True
        else:
            print("Camera Servo cound't be instantiate")
            return False

    def instanciate_servo_range_finder(self, port: str = "SERVO2"):
        self.__servo_distance_sensor = self.__gpg.init_servo(port=port)
        if self.__servo_distance_sensor is not None:
            print("RangeFinder instantiate")
            return True
        else:
            print("RangeFinder Servo cound't be instantiate")
            return False

    def center_servo(self, servo: str = "distance_finder"):
        if servo == "distance_finder":
            self.__servo_distance_sensor.reset_servo()
        elif servo == "camera":
            self.__servo_camera.reset_servo()
        else:
            raise ValueError(f"The selected brain {servo} is not valid")

    # ===========================================================================================
    # Vérification d'intégrité
    # ===========================================================================================

    def validate_integrity(self):
        """
        Cette fonction valide que les éléments externe du robot sont bien fonctionnels.
        Soit le RangeFinder, la manette et les cerveaux moteurs
        :return:
        """
        validation = [False] * 4
        validation[0] = self.instantiate_rangefinder()
        validation[1] = self.remote_control.init_remote()
        validation[2] = self.instantiate_servo_camera()
        validation[3] = self.instanciate_servo_range_finder()
        self.__integrity = "failed" if False in validation else "succeeded"

    def shut_down_actions(self):
        self.center_servo()
        self.center_servo(servo="camera")
        del self.__distance_sensor
        del self.__servo_camera
        del self.__servo_distance_sensor
        del self.remote_control
        self.led_off(self._side.BOTH)
        self.eyes_off(self._side.BOTH)
        del self.__gpg


class RemoteControlCondition(Condition):

    def __init__(self, remote_control: RemoteControl, expected_input: str):
        super().__init__()
        self.remote_control = remote_control
        self.expected_input = expected_input


    def _compare(self) -> bool:
        words_array = self.expected_input.split()
        if len(words_array) >= 1 and words_array[0] == 'not':
            return self.remote_control.return_key != words_array[1]
        return self.remote_control.return_key == self.expected_input


class RangeFinderCondition(Condition):

    def __init__(self, robot: Robot, wanted_distance: float, metric: str = "mm"):
        # todo SAMMMMMMMM
        super().__init__()
        self.robot = robot
        self.wanted_distance = wanted_distance
        self.metric = metric

    def _compare(self) -> bool:
        return self.robot.read_distance(self.metric) <= self.wanted_distance


class RobotState(MonitoredState):
    def __init__(self, robot, parameters: State.Parameters = State.Parameters()):
        super().__init__(parameters)
        self._robot = robot
        self.custom_value = None

    def _exec_in_state_action(self):
        super()._exec_in_state_action()
        self._robot.track()


class ManualControlFSM(FiniteStateMachine):
    def __init__(self, robot):
        self.__robot = robot
        self.__remote_control = robot.remote_control
        self.__MC_layout = FiniteStateMachine.Layout()
        self.__MC_states = []

        ########STATES##########
        self.__MC_RS_stop = RobotState(self.__robot)
        self.__MC_RS_foward = RobotState(self.__robot)
        self.__MC_RS_rotate_right = RobotState(self.__robot)
        self.__MC_RS_rotate_left = RobotState(self.__robot)
        self.__MC_RS_backward = RobotState(self.__robot)
        ########ACTIONS##########
        self.__MC_RS_stop.add_in_state_action(self.MC_stop_action)
        self.__MC_RS_foward.add_in_state_action(self.MC_forward_action)
        self.__MC_RS_backward.add_in_state_action(self.MC_backward_action)
        self.__MC_RS_rotate_left.add_in_state_action(self.MC_left_action)
        self.__MC_RS_rotate_right.add_in_state_action(self.MC_right_action)

        add_RCT(state_from=self.__MC_RS_stop, state_to=self.__MC_RS_foward, expected_input='up',
                remote_control=self.__remote_control)
        add_RCT(state_from=self.__MC_RS_foward, state_to=self.__MC_RS_stop, expected_input='not up',
                remote_control=self.__remote_control)

        add_RCT(state_from=self.__MC_RS_stop, state_to=self.__MC_RS_rotate_left, expected_input='left',
                remote_control=self.__remote_control)
        add_RCT(state_from=self.__MC_RS_rotate_left, state_to=self.__MC_RS_stop, expected_input='not left',
                remote_control=self.__remote_control)

        add_RCT(state_from=self.__MC_RS_stop, state_to=self.__MC_RS_backward, expected_input='down',
                remote_control=self.__remote_control)
        add_RCT(state_from=self.__MC_RS_backward, state_to=self.__MC_RS_stop, expected_input='not down',
                remote_control=self.__remote_control)

        add_RCT(state_from=self.__MC_RS_stop, state_to=self.__MC_RS_rotate_right, expected_input='right',
                remote_control=self.__remote_control)
        add_RCT(state_from=self.__MC_RS_rotate_right, state_to=self.__MC_RS_stop, expected_input='not right',
                remote_control=self.__remote_control)

        self.__MC_states.append(self.__MC_RS_stop)
        self.__MC_states.append(self.__MC_RS_foward)
        self.__MC_states.append(self.__MC_RS_rotate_right)
        self.__MC_states.append(self.__MC_RS_rotate_left)
        self.__MC_states.append(self.__MC_RS_backward)

        self.__MC_layout.add_states(self.__MC_states)
        self.__MC_layout.initial_state = self.__MC_RS_stop

        super().__init__(self.__MC_layout, initialized=False)

    def MC_stop_action(self):
        self.__robot.stop_moving()
        self.__robot.led_off(side=self.__robot.side.BOTH)

    def MC_forward_action(self):
        self.__robot.move_forward()
        self.__robot.led_blink(side=self.__robot.side.BOTH, cycle_duration=1.0, percent_on=.25)

    def MC_backward_action(self):
        self.__robot.move_backward()
        self.__robot.led_blink(side=self.__robot.side.BOTH, cycle_duration=1.0, percent_on=.75)

    def MC_left_action(self):
        self.__robot.turn_left()
        self.__robot.led_blink(side=self.__robot.side.LEFT_RECIPROCAL, cycle_duration=1.0, percent_on=.50)

    def MC_right_action(self):
        self.__robot.turn_right()
        self.__robot.led_blink(side=self.__robot.side.RIGHT_RECIPROCAL, cycle_duration=1.0, percent_on=.50)


class WinkTaskFSM(FiniteStateMachine):
    def __init__(self, robot_instance):
        self.__robot = robot_instance
        self.__remote_control = self.__robot.remote_control
        self._WT_layout = FiniteStateMachine.Layout()
        self.__WT_states = []

        ########STATES##########
        self.__WT_RS_start = RobotState(self.__robot)
        self.__WT_RS_foward_left = RobotState(self.__robot)
        self.__WT_RS_foward_right = RobotState(self.__robot)
        self.__WT_RS_turn_left = RobotState(self.__robot)
        self.__WT_RS_turn_right = RobotState(self.__robot)

        ########ACTIONS##########
        self.__WT_RS_start.add_in_state_action(self.wt_rs_start_action)
        self.__WT_RS_foward_left.add_in_state_action(self.WT_RS_foward_action)
        self.__WT_RS_foward_right.add_in_state_action(self.WT_RS_foward_action)
        self.__WT_RS_turn_left.add_in_state_action(self.WT_RS_rotate_left_action)
        self.__WT_RS_turn_right.add_in_state_action(self.WT_RS_rotate_right_action)

        #############TRANSITIONS###########################################
        add_RCT(state_from=self.__WT_RS_start, state_to=self.__WT_RS_foward_right, expected_input='right',
                remote_control=self.__remote_control)
        add_RCT(state_from=self.__WT_RS_start, state_to=self.__WT_RS_foward_left, expected_input='left',
                remote_control=self.__remote_control)
        add_RFT(state_from=self.__WT_RS_foward_right, state_to=self.__WT_RS_turn_left, wanted_distance=100,
                robot=self.__robot)
        add_RFT(state_from=self.__WT_RS_foward_left, state_to=self.__WT_RS_turn_left, wanted_distance=100,
                robot=self.__robot)
        add_CT_with_SEDC(state_from=self.__WT_RS_turn_left, state_to=self.__WT_RS_foward_left, duration=1.2)
        add_CT_with_SEDC(state_from=self.__WT_RS_turn_right, state_to=self.__WT_RS_foward_right, duration=1.2)

        self.__WT_states.append(self.__WT_RS_start)
        self.__WT_states.append(self.__WT_RS_foward_left)
        self.__WT_states.append(self.__WT_RS_foward_right)
        self.__WT_states.append(self.__WT_RS_turn_left)
        self.__WT_states.append(self.__WT_RS_turn_right)
        self._WT_layout.add_states(self.__WT_states)
        self._WT_layout.initial_state = self.__WT_RS_start
        super().__init__(self._WT_layout, initialized=False)

    def wt_rs_start_action(self):
        self.__robot.set_eyes_color((199, 0, 255), 'right')
        self.__robot.set_eyes_color((199, 0, 255), 'left')

    def WT_RS_foward_action(self):
        self.__robot.move_forward()

    def WT_RS_rotate_left_action(self):
        self.__robot.turn_left()
        self.__robot.set_eyes_color((120, 120, 120), 'left')
        self.__robot.wink(side=self.__robot.side.LEFT_RECIPROCAL, wink_duration=10.0, n_winks=4, wait_duration=5.0,
                          percent_on=0.5, wait_off=True,
                          begin_on=True)

    def WT_RS_rotate_right_action(self):
        self.__robot.turn_right()
        self.__robot.set_eyes_color((120, 120, 120), 'right')
        self.__robot.wink(side=self.__robot.side.RIGHT_RECIPROCAL, wink_duration=10.0, n_winks=4, wait_duration=5.0,
                          percent_on=0.5, wait_off=True,
                          begin_on=True)


class ManualControlState(RobotState):
    def __init__(self, robot):
        super().__init__(robot)
        self.__stateMachine = ManualControlFSM(robot)

    def _exec_entering_action(self):
        print("Entrez dans le task 1")
        self._robot.change_eyes_color((255, 0, 0), right_eye=True, left_eye=False)
        self._robot.change_eyes_color((0, 0, 255), right_eye=False, left_eye=True)
        self._robot.eye_blink(self._robot.side.RIGHT_RECIPROCAL, cycle_duration=1.)
        self.__stateMachine.reset()

    def _exec_in_state_action(self):
        super()._exec_in_state_action()
        self.__stateMachine.track()

    def _exec_exiting_action(self):
        self.__stateMachine.stop()


class WinkTaskState(RobotState):

    def __init__(self, robot):
        super().__init__(robot)
        self.__stateMachine = WinkTaskFSM(robot)

    def _exec_entering_action(self):
        print("Entrez dans le task 2")
        self.__stateMachine.reset()

    def _exec_in_state_action(self):
        self.__stateMachine.track()

    def _exec_exiting_action(self):
        self._robot.stop()
        self.__stateMachine.stop()


class C64(FiniteStateMachine):
    """
    La class C64 est la sous-classe du FSM qui utilise ce dernier pour créer des tâches applicables au
Robot GoPiGo3.

La classe possède un Layout contenant plusieurs States, notamment:
    -__RS_robot_instantiation: effectue l'initialisation du Robot;
    -__RS_robot_integrity: effectue l'initialisation du rangeFinder, de la télécommande et du servo
        de la caméra et du rangeFinder;
    -__RS_instantiation_failed: imprime dans la console un message d'erreur indiquant que l'initialisation du
        Robot a échoué;
    -__RS_integrity_failed: imprime dans la console un message d'erreur indiquant que l'initialisation du
        RangeFinder et/ou d'autres composantes du Robots a/ont échoué, puis clignote les lumières
        du Robot GoPiGo en rouge;
    -__RS_integrity_succeeded: imprime dans la console un message indiquant que l'initialisation du
        RangeFinder et tous les autres composantes du Robots ont réussi, puis clignote les lumières
        du Robot GoPiGo en vert;
    -__RS_shut_down_robot: imprime dans la console un message indiquant que le robot est en train de s'éteindre,
        puis clignote les lumières du Robot GoPiGo en jaune;
    -__RS_end: imprime "Chow!". C'est le State terminal.
    -__RS_home: les yeux du robots clignote l'un après l'autre en jaune, et est en attente d'une commande
        de la télécommande;
    -__MCS_task1: permet de déplacer le robot selon les touches up down left ou right. S'il tourne à gauche, le
        blinker gauche est le seul à clignoter (pour signaler son virage), et vice-versa pour la droite;
    -__MCS_task2: Le robot avance tout seul jusqu'à ce que le télémètre détecte un obstacele, puis va éviter
        l'objet en tournant à gauche ou à droite, selon les paramètres fournis par l'utilisateur. lorsqu'il tourne
        à gauche, le Robot va clignoter sa lumière en jaune gauche seulement, et vice-versa pour la droite. Le
        robot tournera pendant 1.2 secondes et recommencera à avancer s'il n'y a pas d'obstacles en avant;
    -__MCS_task3: La tâche 2 n'a pas encore été programmée, elle ne fait rien.

__RS_robot_instantiation transitera vers __RS_robot_integrity si l'initialisation du Robot est couronnée de succès,
sinon elle transitera au State __RS_instantiation_failed, puis __RS_end.


__RS_robot_integrity transitera vers __RS_integrity_succeeded si l'initialisation du Robot est couronnée de succès
puis vers __RS_Home après 3 secondes, sinon elle transitera au State __RS_integrity_failed, puis
__RS_shut_down_robot après 5 secondes, et finalement __RS_end après 3 secondes.

Lorsqu'on est dans __RS_home, il faut appuyer sur la touche 1 pour transiter vers la tâche __MCS_task1, sur 2
pour transiter vers la tâche __MCS_task2 et 3 pour la tâche 3.

Pour transiter de l'une des 3 tâches mentionnés à __RS_home, il suffit d'appuyer sur 'OK'.

Pour passer de __RS_home à __RS_shut_down_robot, il faut appuyer sur '*' sur la télécommande.

C64 contient aussi:
    -_states: contient tous les states mentionnés ci-dessus avec leurs transitions respectifs;
    -layout: le layout composé des states mentionnés ci-dessus avec leurs transitions respectifs.

et les fonctions appelés par les execute_actions des States:
    -robot_instanciation: appelée par __RS_robot_instantiation;
    -check_integrity: appelée par __RS_robot_integrity en tant qu'entering action (donc appelée seulement 1 fois);
    -instantiation_failed_action: appelée par __RS_instantiation_failed en tant qu'entering action;
    -integrity_failed_action: appelée par __RS_integrity_failed en tant qu'entering action;
    -end_action: appelée par __RS_end en tant qu'entering action;
    -integrity_succeeded_action: appelée par __RS_integrity_succeeded en tant qu'entering action;
    -shut_down_robot_action: appelée par __RS_shut_down_robot en tant qu'entering action;
    -home_idle: appelée par __RS_home en tant qu'entering action, et finalement
    -in_home: appelée par __RS_home en tant qu'action principale, qui tourne en boucle.

Voici ce qui arrive si l'on ne possède pas de Robot GoPiGo3:
    >>> c64 = C64()
    >>> c64.start()
    Robot instantiation has failed
    Chow!
    """

    def __init__(self):
        self._robot: Robot = Robot()
        self._remote_control = self._robot.remote_control
        self._states = []
        self.layout = FiniteStateMachine.Layout()

        #######################STATES#########################
        self.__RS_robot_instantiation = RobotState(self._robot)
        self.__RS_instantiation_failed = RobotState(self._robot)
        self.__RS_robot_integrity = RobotState(self._robot)
        self.__RS_integrity_failed = RobotState(self._robot)
        self.__RS_integrity_succeeded = RobotState(self._robot)
        self.__RS_shut_down_robot = RobotState(self._robot)
        end_params = RobotState.Parameters()
        end_params.terminal = True
        self.__RS_end = RobotState(self._robot, end_params)
        self.__RS_end.terminal = end_params
        self.__RS_home = RobotState(self._robot)
        self.__MCS_task1 = ManualControlState(self._robot)
        self.__RS_task2 = WinkTaskState(self._robot)

        #######################ACTIONS#########################
        self.__RS_robot_instantiation.add_in_state_action(self.robot_instanciation)
        self.__RS_robot_integrity.add_entering_action(self.check_integrity)
        self.__RS_instantiation_failed.add_entering_action(self.instantiation_failed_action)
        self.__RS_integrity_failed.add_entering_action(self.integrity_failed_action)
        self.__RS_integrity_succeeded.add_entering_action(self.integrity_succeeded_action)
        self.__RS_shut_down_robot.add_entering_action(self.shut_down_robot_action)
        self.__RS_end.add_entering_action(self.end_action)
        self.__RS_home.add_entering_action(self.home_idle)
        self.__RS_home.add_in_state_action(self.in_home)

        #######################TRANSITIONS#########################
        add_CT_with_SVC(state_from=self.__RS_robot_instantiation,
                        state_to=self.__RS_instantiation_failed,
                        expected_value="failed")
        add_CT_with_ATC(state_from=self.__RS_instantiation_failed, state_to=self.__RS_end)
        add_CT_with_SVC(state_from=self.__RS_robot_instantiation,
                        state_to=self.__RS_robot_integrity,
                        expected_value="succeeded")
        add_CT_with_SVC(state_from=self.__RS_robot_integrity,
                        state_to=self.__RS_integrity_failed,
                        expected_value="failed")
        add_CT_with_SVC(state_from=self.__RS_robot_integrity,
                        state_to=self.__RS_integrity_succeeded,
                        expected_value="succeeded")
        add_CT_with_SEDC(state_from=self.__RS_integrity_failed,
                         state_to=self.__RS_shut_down_robot,
                         duration=5.0)
        add_CT_with_SEDC(state_from=self.__RS_shut_down_robot,
                         state_to=self.__RS_end,
                         duration=3.0)
        add_CT_with_SEDC(state_from=self.__RS_integrity_succeeded,
                         state_to=self.__RS_home,
                         duration=3.0)

        add_RCT(state_from=self.__RS_home, state_to=self.__MCS_task1, expected_input='1',
                remote_control=self._remote_control)
        add_RCT(state_from=self.__RS_home, state_to=self.__RS_task2, expected_input='2',
                remote_control=self._remote_control)

        add_RCT(state_from=self.__MCS_task1, state_to=self.__RS_home, expected_input='ok',
                remote_control=self._remote_control)
        add_RCT(state_from=self.__RS_task2, state_to=self.__RS_home, expected_input='ok',
                remote_control=self._remote_control)

        add_RCT(state_from=self.__RS_home, state_to=self.__RS_shut_down_robot, expected_input='*',
                remote_control=self._remote_control)

        self._states.append(self.__RS_robot_instantiation)
        self._states.append(self.__RS_instantiation_failed)
        self._states.append(self.__RS_robot_integrity)
        self._states.append(self.__RS_integrity_failed)
        self._states.append(self.__RS_integrity_succeeded)
        self._states.append(self.__RS_shut_down_robot)
        self._states.append(self.__RS_home)
        self._states.append(self.__MCS_task1)
        self._states.append(self.__RS_task2)

        self.layout.add_states(self._states)
        self.layout.initial_state = self.__RS_robot_instantiation
        super().__init__(self.layout)

    def robot_instanciation(self):
        self.__RS_robot_instantiation.custom_value = self._robot.is_instanciate

    def check_integrity(self):
        self._robot.validate_integrity()
        self.__RS_robot_integrity.custom_value = self._robot.check_integrity

    def instantiation_failed_action(self):
        print("Robot instantiation has failed")

    def integrity_failed_action(self):
        print("Robot integrity test has failed")
        self._robot.change_eyes_color((255, 0, 0), right_eye=True, left_eye=True)
        self._robot.eye_blink(self._robot.side.BOTH, cycle_duration=0.5)

    def end_action(self):
        print("Chow!")

    def integrity_succeeded_action(self):
        print("Robot integrity test has succeeded")
        self._robot.change_eyes_color((0, 255, 0), right_eye=True, left_eye=True)
        self._robot.eye_blink(self._robot.side.BOTH, cycle_duration=1.0)

    def shut_down_robot_action(self):
        print("Robot is shuting down")
        self._robot.change_eyes_color((255, 255, 0), right_eye=True, left_eye=True)
        self._robot.eye_blink(self._robot.side.BOTH, end_off=True, cycle_duration=0.75, total_duration=3.)
        self._robot.shut_down_actions()

    def home_idle(self):
        print("Home, prêt à écouter votre commande...")
        self._robot.change_eyes_color((255, 255, 0), right_eye=True, left_eye=True)
        self._robot.eye_blink(self._robot.side.RIGHT_RECIPROCAL, cycle_duration=1.5)

    def in_home(self):
        self.__RS_home.custom_value = self._robot.remote_control.return_key

def __main_doctest():
    import doctest
    doctest.testmod()


if __name__ == "__main__":
    __main_doctest()

