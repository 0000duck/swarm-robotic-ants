from pyrep import PyRep
import numpy as np

class Unit():
    def __init__(self, pyrep: PyRep):
        self._pyrep = pyrep

        # unit traits
        self.max_speed = 1.5
        self.max_force = 2.0

    def getLocation(self):
        ints, floats, strings, byte = self._pyrep.script_call(
            function_name_at_script_name='getLocation@unitScript',
            script_handle_or_type=1,
            ints=(),
            floats=(),
            strings=(),
            bytes=''
        )

        return np.array([floats[0], floats[1]])

    def getVelocity(self):
        ints, floats, strings, byte = self._pyrep.script_call(
            function_name_at_script_name='getVelocity@unitScript',
            script_handle_or_type=1,
            ints=(),
            floats=(),
            strings=(),
            bytes=''
        )

        return np.array([floats[0], floats[1]])

    def applyForce(self, force):
        self._pyrep.script_call(
            function_name_at_script_name='applyForce@unitScript',
            script_handle_or_type=1,
            ints=(),
            floats=([force[0], force[1]]),
            strings=(),
            bytes=''
        )

        self._pyrep.step()
