from Niveau1 import *
from time import perf_counter
from Outils import verify_instance
from abc import ABC
from typing import Union, Callable, List, Tuple


class ActionState(State):
    Action = Callable[[], None]  # alias de type

    def __init__(self, parameters: State.Parameters = State.Parameters()):
        super().__init__(parameters)
        self.__entering_actions: List[ActionState.Action] = []
        self.__in_state_actions: List[ActionState.Action] = []
        self.__exiting_actions: List[ActionState.Action] = []

    def _do_entering_action(self):
        for action in self.__entering_actions:
            action()

    def _do_in_state_action(self):
        for action in self.__in_state_actions:
            action()

    def _do_exiting_action(self):
        for action in self.__exiting_actions:
            action()

    def add_entering_action(self, action: Action) -> None:
        self.__entering_actions.append(action)

    def add_in_state_action(self, action: Action) -> None:
        self.__in_state_actions.append(action)

    def add_exiting_action(self, action: Action) -> None:
        self.__exiting_actions.append(action)


class MonitoredState(ActionState):

    def __init__(self, parameters: State.Parameters = State.Parameters()):
        super().__init__(parameters)
        self.__counter_last_entry: float = 0
        self.__counter_last_exit: float = 0
        self.__entry_count: int = 0
        self.custom_value: any

    @property
    def entry_count(self) -> int:
        return self.__entry_count

    @property
    def last_entry_time(self) -> float:
        return self.__counter_last_entry

    @property
    def last_exit_time(self) -> float:
        return self.__counter_last_exit

    def reset_entry_count(self) -> None:
        self.__entry_count = 0

    def reset_last_times(self) -> None:
        self.__counter_last_entry = perf_counter()
        self.__counter_last_exit = perf_counter()

    def _exec_entering_action(self):
        self.__counter_last_entry = perf_counter()
        self.__entry_count += 1
        super()._exec_entering_action()

    def _exec_exiting_action(self):
        super()._exec_exiting_action()
        self.__counter_last_exit = perf_counter()



class ConditionalTransition(Transition):

    def __init__(self, next_state: State = None, condition: "Condition" = None):
        verify_instance(condition, Condition)
        self.__condition = condition
        super().__init__(next_state)

    @property
    def condition(self):
        return self.__condition

    @condition.setter
    def condition(self, value):
        self.__condition = value

    @property
    def is_valid(self) -> bool:
        return self.__condition is not None

    @property
    def is_transiting(self) -> bool:
        return bool(self.__condition)


class ActionTransition(ConditionalTransition):
    # Ajouter le datatype action type Alias (prends une fonction vide f() -> None)

    Action = Callable[[], None]

    def __init__(self, next_state: State = None, condition: 'Condition' = None):
        verify_instance(next_state, State)
        super().__init__(next_state, condition)
        self.__transiting_actions: list[ActionTransition.Action] = []

    def _do_transiting_action(self):
        for action in self.__transiting_actions:
            action()

    def add_transiting_action(self, action: Action):
        self.__transiting_actions.append(action)


class MonitoredTransition(ActionTransition):

    def __init__(self, next_state: State = None, condition: 'Condition' = None):
        verify_instance(next_state, State)
        self.__transit_count: int = 0
        self.__last_transit_time: float = 0
        self.custom_value: any
        super().__init__(next_state, condition)

    @property
    def transit_count(self) -> int:
        return self.__transit_count

    @property
    def last_transit_time(self) -> float:
        return self.__last_transit_time

    def reset_transit_count(self):
        self.__transit_count = 0

    def reset_last_transit_time(self):
        self.__last_transit_time = perf_counter()

    def _exec_transiting_action(self):
        self.__counter_last_entry = perf_counter()
        self.__transit_count += 1
        super()._exec_transiting_action()


class Condition(ABC):

    def __init__(self, inverse: bool = False):
        verify_instance(inverse, bool)
        self.__inverse = inverse

    @abstractmethod
    def _compare(self) -> bool:
        pass

    def __bool__(self) -> bool:
        return self._compare() ^ self.__inverse


ConditionList = List[Condition]


class AlwaysTrueCondition(Condition):

    def __init__(self, inverse: bool = False):
        super().__init__(inverse)

    def _compare(self) -> bool:
        return True


class ValueCondition(Condition):

    def __init__(self, initial_value: any, expected_value: any, inverse: bool = False):
        self.expected_value = expected_value
        self.value = initial_value
        super().__init__(inverse)

    def _compare(self) -> bool:
        return self.value == self.expected_value  # Question


class TimedCondition(Condition):

    def __init__(self, duration: float = 1., time_reference: float = None, inverse: bool = False):
        verify_instance(duration, float)
        if time_reference is not None:
            verify_instance(time_reference, float)
        self.__counter_duration: float = duration
        self.__counter_reference: float = time_reference
        super().__init__(inverse)

    def _compare(self) -> bool:
        try:
            return self.__counter_duration >= self.__counter_reference
        except:
            return False

    @property
    def duration(self) -> float:
        return self.__counter_duration

    @duration.setter
    def duration(self, value: float) -> None:
        verify_instance(value, float)
        self.__counter_duration = value


class MonitoredStateCondition(Condition, ABC):
    def __init__(self, monitored_state: MonitoredState, inverse: bool = False):
        verify_instance(monitored_state,MonitoredState)
        verify_instance(inverse,bool)
        self._monitored_state = monitored_state
        super().__init__(inverse)

    @property
    def monitored_state(self):
        return self._monitored_state

    @monitored_state.setter
    def monitored_state(self, value: MonitoredState):
        verify_instance(value,MonitoredState)
        self._monitored_state = value


class StateEntryDurationCondition(MonitoredStateCondition):
    def __init__(self, duration: float, monitored_state: MonitoredState, inverse: bool = False):
        verify_instance(duration,float)
        verify_instance(monitored_state, MonitoredState)
        verify_instance(inverse,bool)
        self.__duration = duration
        super().__init__(monitored_state, inverse)

    @property
    def duration(self):
        return self.__duration

    @duration.setter
    def duration(self, value: float):
        verify_instance(value, float)
        self.__duration = value

    def _compare(self) -> bool:
        return perf_counter() - self.monitored_state.last_entry_time >= self.__duration


class StateEntryCountCondition(MonitoredStateCondition):
    def __init__(self, expected_count: int, monitored_state: MonitoredState, auto_reset: bool = True,
                 inverse: bool = False):
        super().__init__(monitored_state, inverse)
        verify_instance(expected_count,int)
        verify_instance(monitored_state,MonitoredState)
        verify_instance(auto_reset,bool)
        verify_instance(inverse,bool)
        self.__auto_reset = auto_reset
        self.__ref_count = self._monitored_state.entry_count
        self.__expected_count = expected_count

    @property
    def expected_count(self):
        return self.__expected_count

    @expected_count.setter
    def expected_count(self, value: int):
        verify_instance(value, int)
        self.__expected_count = value

    def _compare(self) -> bool:
        if self._monitored_state.entry_count - self.__ref_count >= self.__expected_count:
            if self.__auto_reset:
                self.reset_count()
            return True
        return False

    def reset_count(self):
        self.__ref_count = self._monitored_state.entry_count


class StateValueCondition(MonitoredStateCondition):
    def __init__(self, expected_value, monitored_state: MonitoredState, inverse: bool = False):
        super().__init__(monitored_state, inverse)
        self.__expected_value = expected_value

    @property
    def expected_value(self):
        return self.__expected_value

    @expected_value.setter
    def expected_value(self, value):
        self.__expected_value = value

    def _compare(self) -> bool:
        try:
            return self._monitored_state.custom_value == self.__expected_value
        except:
            return False


class ManyConditions(Condition):
    def __init__(self, inverse: bool = False):
        super().__init__(inverse)
        self._conditions: ConditionList = []

    def add_condition(self, condition: Condition):
        verify_instance(condition,Condition)
        self._conditions.append(condition)

    def add_conditions(self, conditions: ConditionList):
        verify_instance(conditions,ConditionList)
        for condition in conditions:
            self.add_condition(condition)


class NoneConditions(ManyConditions):
    def __init__(self, inverse: bool = False):
        super().__init__(inverse)

    def _compare(self) -> bool:
        return not self._conditions


class AllConditions(ManyConditions):
    def __init__(self, inverse: bool = False):
        super().__init__(inverse)

    def _compare(self) -> bool:
        compt = 0
        if self._conditions:

            for condition in self._conditions:
                compt += bool(condition)
            return compt == len(self._conditions)
        return False



class AnyConditions(ManyConditions):
    def __init__(self, inverse: bool = False):
        super().__init__(inverse)

    def _compare(self) -> bool:
        if self._conditions:
            for condition in self._conditions:
                if bool(condition):
                    return True
        return False
