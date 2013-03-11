import threading
import time
import numpy

# start at zero
start_point = 0.0
speed = 10 # mm/s

class multiaxis_linear_slides(object):

    def __init__(self, connection, devices, *args, **kwargs):
        self._devices = devices
        self._current_position = dict.fromkeys(devices, start_point)
        self._position_lock = threading.Lock()
        self.command_queue = []

    def set_position(self, position):

        self._current_position = position

    def get_current_position(self):

        with self._position_lock:
            position = self._current_position.copy()

        return position

    def move_absolute(self, positions):
        self.command_queue.append(('move_absolute', positions))

    def _simulate_move_absolute(self, position):
        '''Move in 1ms increments
        '''
        t_step = 1e-3
        start_position = self.get_current_position()
        step_delta = speed*t_step
        
        signs = {}
        for each_key in position:
            signs[each_key] = numpy.sign(float(position[each_key]) - 
                    start_position[each_key])

        n_completed = 0

        n_positions = len(position)
        while True:
            time.sleep(t_step)
            with self._position_lock:
                completed_keys = []
                for each_key in position:
                    if abs(float(position[each_key]) - 
                            self._current_position[each_key]) > step_delta:
                        self._current_position[each_key] += (signs[each_key] *
                                step_delta)

                    else:
                        self._current_position[each_key] = float(
                                position[each_key])
                        n_completed += 1
                        completed_keys.append(each_key)

                        if n_completed >= n_positions:
                            return

                for each_key in completed_keys:
                    del position[each_key]


    def _simulate_command(self, command):
        if command[0] == 'move_absolute':
            positions = command[1]
            thread = threading.Thread(target=self._simulate_move_absolute,
                    args=(positions,))
            thread.start()


    def step(self):
        self._simulate_command(self.command_queue.pop())
