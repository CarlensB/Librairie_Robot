from enum import Enum

def verify_instance(var: any, instance: any):
    if not isinstance(var, instance):
        raise ValueError(f'La variable passée en paramètre n\'est pas une instance de {instance}')

def verify_instances(*params):
    for var, instance in params:
        verify_instance(var, instance)


def verify_blink_number_parameter(duration:float= None, total_duration:float = None, cycle_duration:float = None,
                                  percent_on:float = None, wink_duration:float = None, wait_duration:float = None,
                                  n_cycles:int = None, n_winks:int = None):
    if duration is not None:
        if duration <= 0.0:
            raise ValueError("La variable duration n'est pas au dessus de 0.")


    if total_duration is not None:
        if total_duration <= 0.0:
            raise ValueError("La variable total_duration n'est pas au dessus de 0.")

    if cycle_duration is not None:
        if cycle_duration <= 0.0:
            raise ValueError("La variable cycle_duration n'est pas au dessus de 0.")

    if wink_duration is not None:
        if wink_duration <= 0.0:
            raise ValueError("La variable wink_duration n'est pas au dessus de 0.")

    if wait_duration is not None:
        if wait_duration <= 0.0:
            raise ValueError("La variable wait_duration n'est pas au dessus de 0.")

    if percent_on is not None:
        if percent_on < 0.0 or percent_on > 1.0:
            raise ValueError("La variable percent_on n'est pas entre 0 et 1 inclusivement.")

    if n_cycles is not None:
        if n_cycles < 1:
            raise ValueError("La variable n_cycles n'est pas au dessus de 1.")

    if n_winks is not None:
        if n_winks < 1:
            raise ValueError("La variable n_winks n'est pas au dessus de 1.")

