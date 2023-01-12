from Niveau2 import *
from Outils import verify_instance, verify_blink_number_parameter, verify_instances
from time import perf_counter


class Blinker(FiniteStateMachine):
    StateGenerator = Callable[[], MonitoredState]

    def __init__(self, off_state_generator: StateGenerator, on_state_generator: StateGenerator):
        self.__off_state_generator = off_state_generator()
        self.__on_state_generator = on_state_generator()
        self.layout = FiniteStateMachine.Layout()

        states = []
        # block rouge et blanc
        self.__on = on_state_generator()
        self.__on.__BLINKER_EST_ALLUMEUR = True

        self.__off = off_state_generator()
        self.__off.__BLINKER_EST_ALLUMEUR = False

        self.__off_duration = off_state_generator()
        self.__off_duration.__BLINKER_EST_ALLUMEUR = False

        self.__on_duration = on_state_generator()
        self.__off_duration.__BLINKER_EST_ALLUMEUR = True

        self.__blink_off = off_state_generator()
        self.__off_duration.__BLINKER_EST_ALLUMEUR = False

        self.__blink_on = on_state_generator()
        self.__off_duration.__BLINKER_EST_ALLUMEUR = True

        self.__blink_stop_off = off_state_generator()
        self.__off_duration.__BLINKER_EST_ALLUMEUR = False

        self.__blink_stop_on = on_state_generator()
        self.__off_duration.__BLINKER_EST_ALLUMEUR = True

        self.__wink_off = off_state_generator()
        self.__wink_off.__BLINKER_EST_ALLUMEUR = False
        self.__wink_off.add_entering_action(lambda: print("entrering wink off"))

        self.__wink_on = on_state_generator()
        self.__wink_on.__BLINKER_EST_ALLUMEUR = True
        self.__wink_on.add_entering_action(lambda: print("entrering wink on"))

        self.__wink_wait_off = off_state_generator()
        self.__wink_wait_off.__BLINKER_EST_ALLUMEUR = False
        self.__wink_wait_off.add_entering_action(lambda: print("entrering wink off"))

        self.__wink_wait_on = on_state_generator()
        self.__wink_wait_on.__BLINKER_EST_ALLUMEUR = True
        self.__wink_wait_on.add_entering_action(lambda: print("entrering __wink_wait_on"))

        # block bleu et blanc
        self.__blink_begin = MonitoredState()
        self.__blink_stop_begin = MonitoredState()
        self.__blink_stop_end = MonitoredState()

        self.__wink_begin = MonitoredState()
        self.__wink_wait_transit = MonitoredState()
        self.__wink_wait = MonitoredState()
        self.__wink_end = MonitoredState()

        # Arrows


        self.__SECC_wink_wait_to_wink_end = self.__yellow_arrow(self.__wink_wait, self.__wink_end, expected_count=0)

        self.__SEDC_on_duration_to_off = self.__greenArrow(self.__on_duration, self.__off)
        self.__SEDC_off_duration_to_on = self.__greenArrow(self.__off_duration, self.__on)
        self.__SEDC_blink_off_to_blink_on = self.__greenArrow(self.__blink_off, self.__blink_on)
        self.__SEDC_blink_on_to_blink_off = self.__greenArrow(self.__blink_on, self.__blink_off)
        self.__SEDC_blink_stop_off_to_blink_stop_on = self.__greenArrow(self.__blink_stop_off, self.__blink_stop_on)
        self.__SEDC_blink_stop_on_to_blink_stop_off = self.__greenArrow(self.__blink_stop_on, self.__blink_stop_off)

        self.__SEDC_wink_wait_to_wink_begin = self.__greenArrow(self.__wink_wait, self.__wink_begin, duration=1.0)
        self.__SEDC_wink_off_to_wink_on = self.__greenArrow(self.__wink_off, self.__wink_on, duration=0.1)
        self.__SEDC_wink_on_to_wink_off = self.__greenArrow(self.__wink_on, self.__wink_off, duration=0.1)

        self.__SVC_blink_begin_to_blink_off = self.__orange_arrow(self.__blink_begin, self.__blink_off, False)
        self.__SVC_blink_begin_to_blink_on = self.__orange_arrow(self.__blink_begin, self.__blink_on, True)

        self.__SVC_blink_stop_begin_to_blink_stop_off = self.__orange_arrow(self.__blink_stop_begin,
                                                                            self.__blink_stop_off, False)
        self.__SVC_blink_stop_begin_to_blink_stop_on = self.__orange_arrow(self.__blink_stop_begin,
                                                                           self.__blink_stop_on, True)

        self.__SVC_blink_stop_end_to_on = self.__orange_arrow(self.__blink_stop_end, self.__on, True)
        self.__SVC_blink_stop_end_to_off = self.__orange_arrow(self.__blink_stop_end, self.__off, False)

        self.__SVC_wink_begin_to_wink_off = self.__orange_arrow(self.__wink_begin, self.__wink_off, False)
        self.__SVC_wink_begin_to_wink_on = self.__orange_arrow(self.__wink_begin, self.__wink_on, True)

        self.__SVC_wink_wait_transit_to_wink_wait_off = self.__orange_arrow(self.__wink_wait_transit,
                                                                            self.__wink_wait_off, False)
        self.__SVC_wink_wait_transit_to_wink_wait_on = self.__orange_arrow(self.__wink_wait_transit,
                                                                           self.__wink_wait_on, True)

        self.__SVC_wink_end_to_on = self.__orange_arrow(self.__wink_end, self.__on, False)
        self.__SVC_wink_end_to_off = self.__orange_arrow(self.__wink_end, self.__off, True)

        self.__SEDC_blink_stop_off_to_blink_stop_end_MS_blink_stop_begin = self.__dotted_green_arrow(
            self.__blink_stop_off, self.__blink_stop_end, self.__blink_stop_begin)
        self.__SEDC_blink_stop_on_to_blink_stop_end_MS_blink_stop_begin = self.__dotted_green_arrow(
            self.__blink_stop_on, self.__blink_stop_end, self.__blink_stop_begin)

        self.__ATC_wink_wait_off_to_wink_wait = self.__blue_arrow(self.__wink_wait_off, self.__wink_wait)
        self.__ATC_wink_wait_on_to_wink_wait = self.__blue_arrow(self.__wink_wait_on, self.__wink_wait)


        self.__SECC_wink_off_to_wink_wait_transit = self.__yellow_arrow(self.__wink_off, self.__wink_wait_transit,
                                                                        expected_count=1)
        self.__SECC_wink_on_to_wink_wait_transit = self.__yellow_arrow(self.__wink_on, self.__wink_wait_transit,
                                                                       expected_count=1)

        states.append(self.__on)
        states.append(self.__off)
        states.append(self.__off_duration)
        states.append(self.__on_duration)
        states.append(self.__blink_on)
        states.append(self.__blink_off)
        states.append(self.__blink_begin)
        states.append(self.__blink_stop_begin)
        states.append(self.__blink_stop_off)
        states.append(self.__blink_stop_end)
        states.append(self.__blink_stop_on)
        states.append(self.__wink_begin)
        states.append(self.__wink_on)
        states.append(self.__wink_off)
        states.append(self.__wink_wait_transit)
        states.append(self.__wink_wait)
        states.append(self.__wink_end)
        states.append(self.__wink_wait_on)
        states.append(self.__wink_wait_off)
        self.layout.initial_state = self.__off
        self.layout.add_states(states)
        super().__init__(self.layout, initialized=False)

    @property
    def is_on(self) -> bool:
        return self.current_applicative_state.__BLINKER_EST_ALLUMEUR

    @property
    def is_off(self) -> bool:
        return self.current_applicative_state.__BLINKER_EST_ALLUMEUR

    def __greenArrow(self, state_from: MonitoredState, state_to: MonitoredState,
                     duration: float = 1.0) -> StateEntryDurationCondition:
        stateEntryDurationCondition = StateEntryDurationCondition(duration=duration, monitored_state=state_from)
        condition_transition = ConditionalTransition(condition=stateEntryDurationCondition, next_state=state_to)
        state_from.add_transition(condition_transition)
        return condition_transition.condition

    def __orange_arrow(self, state_from: MonitoredState, state_to: MonitoredState,
                       expected_value) -> StateValueCondition:
        stateValueCondition = StateValueCondition(expected_value=expected_value, monitored_state=state_from)
        condition_transition = ConditionalTransition(
            condition=stateValueCondition, next_state=state_to)
        state_from.add_transition(condition_transition)
        return condition_transition.condition

    def __dotted_green_arrow(self, state_from: MonitoredState, state_to: MonitoredState,
                             monitored_state: MonitoredState):
        stateEntryDurationCondition = StateEntryDurationCondition(duration=1.0, monitored_state=monitored_state)
        condition_transition = ConditionalTransition(condition=stateEntryDurationCondition, next_state=state_to)
        state_from.add_transition(condition_transition)
        return condition_transition.condition

    def __blue_arrow(self, state_from: MonitoredState, state_to: MonitoredState) -> AlwaysTrueCondition:
        condition_transition = ConditionalTransition(condition=AlwaysTrueCondition(), next_state=state_to)
        state_from.add_transition(condition_transition)
        return condition_transition.condition

    def __yellow_arrow(self, state_from: MonitoredState, state_to: MonitoredState,
                       expected_count: int) -> StateEntryCountCondition:
        stateEntryCountCondition = StateEntryCountCondition(expected_count=expected_count, monitored_state=state_from)
        condition_transition = ConditionalTransition(
            condition=stateEntryCountCondition, next_state=state_to)
        state_from.add_transition(condition_transition)
        return condition_transition.condition

    def turn_off(self, **kwargs):
        key = set(kwargs.keys())
        if key == set():
            self._transit_to(self.__off)
        elif key == key == {'duration'}:
            duration = kwargs['duration']
            verify_instance(duration, float)
            verify_blink_number_parameter(duration=duration)
            if duration > 0:
                self.__SEDC_on_duration_to_off.duration = duration
                self._transit_to(self.__off_duration)
            else:
                ValueError("La duration doit être > 0")
        else:
            raise ValueError("Les paramètres fournis ne sont pas pris en charge")

    def turn_on(self, **kwargs):
        key = set(kwargs.keys())
        if key == set():
            self._transit_to(self.__on)
        elif key == {'duration'}:
            duration = kwargs['duration']
            verify_instance(duration, float)
            verify_blink_number_parameter(duration=duration)
            if duration > 0:
                self.__SEDC_off_duration_to_on.duration = duration
                self._transit_to(self.__on_duration)
            else:
                ValueError("La duration doit être > 0")
        else:
            raise ValueError("Les paramètres fournis ne sont pas pris en charge")

    def blink(self, percent_on: float = 0.5, begin_on: bool = True, **kwargs):
        key = set(kwargs.keys())
        if key == {'cycle_duration'}:
            cycle_duration = kwargs['cycle_duration']
            verify_instances([cycle_duration, float], [percent_on, float], [begin_on, bool])
            verify_blink_number_parameter(cycle_duration=cycle_duration, percent_on=percent_on)
            self.__blink_begin.custom_value = begin_on
            self.__SEDC_blink_off_to_blink_on.duration = cycle_duration * (1 - percent_on)
            self.__SEDC_blink_on_to_blink_off.duration = cycle_duration * percent_on
            self._transit_to(self.__blink_begin)
        elif key == {'cycle_duration', 'total_duration', 'end_off'}:
            cycle_duration = kwargs['cycle_duration']
            total_duration = kwargs['total_duration']
            end_off = kwargs['end_off']
            verify_instances([total_duration, float], [cycle_duration, float], [percent_on, float],
                             [begin_on, bool], [end_off, bool])
            verify_blink_number_parameter(total_duration=total_duration, cycle_duration=cycle_duration,
                                          percent_on=percent_on)
            n_cycles = total_duration / cycle_duration
            self.__blink_stop_begin.custom_value = begin_on
            self.__SEDC_blink_stop_off_to_blink_stop_on.duration = cycle_duration * (1 - percent_on)
            self.__SEDC_blink_stop_off_to_blink_stop_end_MS_blink_stop_begin.duration = total_duration
            self.__SEDC_blink_stop_on_to_blink_stop_off.duration = cycle_duration * percent_on
            self.__SEDC_blink_stop_on_to_blink_stop_end_MS_blink_stop_begin.duration = total_duration
            self.__blink_stop_end.custom_value = end_off
            self._transit_to(self.__blink_stop_begin)

        elif key == {'total_duration', 'n_cycles', 'end_off'}:
            total_duration = kwargs['total_duration']
            n_cycles = kwargs['n_cycles']
            end_off = kwargs['end_off']
            verify_instances([total_duration, float], [n_cycles, int], [percent_on, float],
                             [begin_on, bool], [end_off, bool])
            verify_blink_number_parameter(total_duration=total_duration, n_cycles=n_cycles,
                                          percent_on=percent_on)
            cycle_duration = total_duration / n_cycles
            self.__blink_stop_begin.custom_value = begin_on
            self.__SEDC_blink_stop_off_to_blink_stop_on.duration = cycle_duration * (1 - percent_on)

            self.__SEDC_blink_stop_off_to_blink_stop_end_MS_blink_stop_begin.duration = total_duration
            self.__SEDC_blink_stop_on_to_blink_stop_end_MS_blink_stop_begin.duration = total_duration

            self.__SEDC_blink_stop_on_to_blink_stop_off.duration = cycle_duration * percent_on

            self.__blink_stop_end.custom_value = end_off
            self._transit_to(self.__blink_stop_begin)

        elif key == {'n_cycles', 'cycle_duration', 'end_off'}:
            cycle_duration = kwargs['cycle_duration']
            n_cycles = kwargs['n_cycles']
            end_off = kwargs['end_off']
            verify_instances([cycle_duration, float], [n_cycles, int], [percent_on, float],
                             [begin_on, bool], [end_off, bool])
            verify_blink_number_parameter(n_cycles=n_cycles, cycle_duration=cycle_duration,
                                          percent_on=percent_on)
            total_duration = n_cycles * cycle_duration
            self.__blink_stop_begin.custom_value = begin_on
            self.__SEDC_blink_stop_off_to_blink_stop_on.duration = cycle_duration * (1 - percent_on)
            self.__SEDC_blink_stop_off_to_blink_stop_end_MS_blink_stop_begin.duration = total_duration
            self.__SEDC_blink_stop_on_to_blink_stop_end_MS_blink_stop_begin.duration = total_duration
            self.__SEDC_blink_stop_on_to_blink_stop_off.duration = cycle_duration * percent_on
            self.__blink_stop_end.custom_value = end_off
            self._transit_to(self.__blink_stop_begin)
        else:
            raise ValueError("Les paramètres fournis ne sont pas pris en charge")

    def wink(self, wink_duration: float, n_winks: int, percent_on: float = 0.5, begin_on: bool = True,
             wait_off: bool = True,
             **kwargs):

        end_off = True
        n_cycle = 99999999999999999999999999999999

        key = set(kwargs.keys())
        ################WAIT DURATION#####################
        if key == {'wait_duration'}: #infinie
            wait_duration = kwargs['wait_duration']
            cycle_duration = (wink_duration - wait_duration)/n_winks
        elif key == {'wait_duration', 'end_off', 'n_cycles'}:
            wait_duration = kwargs['wait_duration']
            end_off = kwargs['end_off']
            n_cycle = kwargs['n_cycles']

            #cycle_duration = (wink_duration/n_cycle) - wait_duration
            cycle_duration = ((wink_duration - wait_duration)/n_winks)/n_cycle
            wait_total_duration = wait_duration*n_cycle
            #cycle_duration = (wink_duration - wait_total_duration)/n_cycle

        ################CYCLE DURATION#####################
        elif key == {'cycle_duration'}:
            cycle_duration = kwargs['cycle_duration']
            wait_duration = wink_duration - (cycle_duration*n_winks)
        elif key == {'cycle_duration', 'end_off', 'n_cycles'}:
            cycle_duration = kwargs['cycle_duration']
            n_cycle = kwargs['n_cycles']
            end_off = kwargs['end_off']
            wait_duration = (wink_duration - (cycle_duration*n_winks))/n_cycle
        else:
            raise ValueError("Les paramètres fournis ne sont pas pris en charge")


        verify_instance(wink_duration, float)
        verify_instance(n_winks, int)
        verify_instance(percent_on, float)
        if wink_duration <= 0.0:
            raise ValueError("wink_duration doit être > 0.0")
        if n_winks <= 1:
            raise ValueError("n_winks doit être > 1")
        if 0.0 > percent_on or percent_on > 1.0:
            raise ValueError("On doit avoir 0.0 <= percent_on <= 1.0")

        #######BEGIN ON
        verify_instance(begin_on, bool)
        self.__wink_begin.custom_value = begin_on

        #######WAIT OFF
        verify_instance(wait_off, bool)
        self.__wink_wait_transit.custom_value = not wait_off

        ########END 0FF
        verify_instance(end_off, bool)
        self.__wink_end.custom_value = end_off

        #####WAIT DURATION
        verify_instance(wait_duration, float)
        if wait_duration <= 0.0:
            raise ValueError("wait_duration doit être > 0.0")

        self.__SEDC_wink_wait_to_wink_begin.duration = wait_duration

        #######N_WINKS
        if begin_on:
            self.__SECC_wink_off_to_wink_wait_transit.expected_count = n_winks
            self.__SECC_wink_on_to_wink_wait_transit.expected_count = n_winks + 1
        else:
            self.__SECC_wink_off_to_wink_wait_transit.expected_count = n_winks + 1
            self.__SECC_wink_on_to_wink_wait_transit.expected_count = n_winks

        ######N_CYCLE
        self.__SECC_wink_wait_to_wink_end.expected_count = n_cycle

        #####CYCLE DURATION
        verify_instance(percent_on, float)
        verify_instance(cycle_duration, float)
        if cycle_duration <= 0.0:
            raise ValueError("cycle_duration doit être > 0.0")
        self.__SEDC_wink_off_to_wink_on.duration = cycle_duration * percent_on
        self.__SEDC_wink_on_to_wink_off.duration = cycle_duration * (1 - percent_on)

        self._transit_to(self.__wink_begin)


class SideBlinker:
    class Side(Enum):
        LEFT = 1
        RIGHT = 2
        BOTH = 3
        LEFT_RECIPROCAL = 4
        RIGHT_RECIPROCAL = 6
        
        def __gt__(self, other):
            if isinstance(other, SideBlinker.Side):
                return self.value > other.value
            return False
                
    def __init__(self, left_off_state_generator: Blinker.StateGenerator,
                 left_on_state_generator: Blinker.StateGenerator, right_off_state_generator: Blinker.StateGenerator,
                 right_on_state_generator: Blinker.StateGenerator):
        self._left_blinker = Blinker(left_off_state_generator, left_on_state_generator)
        self._right_blinker = Blinker(right_off_state_generator, right_on_state_generator)

    def is_on(self, side: Side) -> bool:
        if side == self.Side.BOTH:
            if self._right_blinker.is_on and self._left_blinker.is_on:
                return True
        elif side == self.Side.LEFT:
            if self._left_blinker.is_on:
                return True
        elif side == self.Side.RIGHT:
            if self._right_blinker.is_on:
                return True
        elif side == self.Side.LEFT_RECIPROCAL:
            if self._right_blinker.is_off and self._left_blinker.is_on:
                return True
        elif side == self.Side.RIGHT_RECIPROCAL:
            if self._right_blinker.is_on and self._left_blinker.is_off:
                return True
        return False

    def is_off(self, side: Side) -> bool:
        return not self.is_on(side)

    def turn_on(self, side: Side, **kwargs):
        if side == self.Side.BOTH:
            self._left_blinker.turn_on(**kwargs)
            self._right_blinker.turn_on(**kwargs)
        elif side == self.Side.LEFT:
            self._left_blinker.turn_on(**kwargs)
        elif side == self.Side.RIGHT:
            self._right_blinker.turn_on(**kwargs)
        elif side == self.Side.LEFT_RECIPROCAL:
            self._right_blinker.turn_off(**kwargs)
            self._left_blinker.turn_on(**kwargs)
        elif side == self.Side.RIGHT_RECIPROCAL:
            self._right_blinker.turn_on(**kwargs)
            self._left_blinker.turn_off(**kwargs)

    def turn_off(self, side: Side, **kwargs):
        if side == self.Side.BOTH:
            self._left_blinker.turn_off(**kwargs)
            self._right_blinker.turn_off(**kwargs)
        elif side == self.Side.LEFT:
            self._left_blinker.turn_off(**kwargs)
        elif side == self.Side.RIGHT:
            self._right_blinker.turn_off(**kwargs)
        elif side == self.Side.LEFT_RECIPROCAL:
            self._right_blinker.turn_on(**kwargs)
            self._left_blinker.turn_off(**kwargs)
        elif side == self.Side.RIGHT_RECIPROCAL:
            self._right_blinker.turn_off(**kwargs)
            self._left_blinker.turn_on(**kwargs)

    def blink(self, side: Side, percent_on: float = 0.5, begin_on: bool = True, **kwargs):
        if side == self.Side.BOTH:
            self._left_blinker.blink(percent_on=percent_on, begin_on=begin_on, **kwargs)
            self._right_blinker.blink(percent_on=percent_on, begin_on=begin_on, **kwargs)
        elif side == self.Side.LEFT:
            self._left_blinker.blink(percent_on=percent_on, begin_on=begin_on, **kwargs)
        elif side == self.Side.RIGHT:
            self._right_blinker.blink(percent_on=percent_on, begin_on=begin_on, **kwargs)
        elif side == self.Side.LEFT_RECIPROCAL:
            self._left_blinker.blink(percent_on=percent_on, begin_on=begin_on, **kwargs)
            self._right_blinker.blink(percent_on=percent_on, begin_on=False, **kwargs)
        elif side == self.Side.RIGHT_RECIPROCAL:
            self._right_blinker.blink(percent_on=percent_on, begin_on=begin_on, **kwargs)
            self._left_blinker.blink(percent_on=percent_on, begin_on=False, **kwargs)

    def wink(self, side: Side, wink_duration: float, n_winks: int, percent_on: float = 0.5, begin_on: bool = True, wait_off: bool = True,
             **kwargs):
        if side == self.Side.BOTH:
            self._left_blinker.wink(wink_duration=wink_duration, n_winks=n_winks, percent_on=percent_on,
                                    begin_on=begin_on, wait_off=wait_off, **kwargs)
            self._right_blinker.wink(wink_duration=wink_duration, n_winks=n_winks, percent_on=percent_on,
                                     begin_on=begin_on, wait_off=wait_off, **kwargs)
        elif side == self.Side.LEFT:
            self._left_blinker.wink(wink_duration=wink_duration, n_winks=n_winks, percent_on=percent_on,
                                    begin_on=begin_on, wait_off=wait_off, **kwargs)
        elif side == self.Side.RIGHT:
            self._right_blinker.wink(wink_duration=wink_duration, n_winks=n_winks, percent_on=percent_on,
                                     begin_on=begin_on, wait_off=wait_off, **kwargs)
        elif side == self.Side.LEFT_RECIPROCAL:
            self._left_blinker.wink(wink_duration=wink_duration, n_winks=n_winks, percent_on=percent_on,
                                    begin_on=begin_on, wait_off=wait_off, **kwargs)
            self._right_blinker.turn_off()
        elif side == self.Side.RIGHT_RECIPROCAL:
            self._right_blinker.wink(wink_duration=wink_duration, n_winks=n_winks, percent_on=percent_on,
                                     begin_on=begin_on, wait_off=wait_off, **kwargs)
            self._left_blinker.turn_off()

    def track(self):
        self._right_blinker.track()
        self._left_blinker.track()
