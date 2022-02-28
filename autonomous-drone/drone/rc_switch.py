import warnings
import time


class Switch(object):
    """
    A SwitchState class to handle switch detection on RC Transmitter.
    Does not rely on transmitter mapping.
    """

    def __init__(self, number_of_states=None, initial_state=None):
        self._time_last_updated = time.time()
        if number_of_states is None:
            self.number_of_states = 3
        else:
            self.number_of_states = number_of_states
        if initial_state is None:
            self.state = "down"
        else:
            self.set_state(initial_state)

    def __str__(self):
        return self.state

    def set_state(self, state):
        if not (state in ["up", "middle", "down"]):
            raise ValueError("set_state string argument must be either up, middle or down")

        if self.number_of_states == 3:
            if self.state != state:
                self._time_last_updated = time.time()
            self.state = state

        elif self.number_of_states == 2 and state != "middle":
            if self.state != state:
                self._time_last_updated = time.time()
            self.state = state

        else:
            raise ValueError("middle is not an acceptable value for a 2-way switch")

    def is_up(self):
        return self.state == "up"

    def is_down(self):
        return self.state == "down"

    def is_middle(self):
        if self.number_of_states == 2:
            warnings.warn("Warning : you tried to check whether a 2 state switch could be in the middle state")
        return self.state == "middle"

    def was_updated_since(self, amount_of_time):
        """
        Function that checks if a switch was updated since amount_of_time time.
        Used mainly to avoid making multiple unwanted inputs.
        :param: amount_of_time: an amount of time in seconds. Use decimal for time under a second
        :return: True: was updated since amount_of time, False, was not updated since amount_of_time
        """
        return time.time() - self._time_last_updated > amount_of_time
