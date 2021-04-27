from pyrep import PyRep
import numpy as np
from numpy import linalg as la

class Unit():
    def __init__(self, pyrep: PyRep, index: int):
        self._pyrep = pyrep
        self._index = index

        self._targets = []

        # unit traits
        self.max_speed = 1.0
        self.max_force = 5.0

        self.min_sep_dist = 2.0
        self.max_sep_speed = 10.0
        self.max_sep_force = 5.0

        self.min_follow_dist = 2.0

        self.arrival_radius = 1.5

    def getLocation(self):
        ints, floats, strings, byte = self._pyrep.script_call(
            function_name_at_script_name='getLocation@unitScript',
            script_handle_or_type=1,
            ints=([self._index]),
            floats=(),
            strings=(),
            bytes=''
        )

        return np.array([floats[0], floats[1]])

    def getVelocity(self):
        ints, floats, strings, byte = self._pyrep.script_call(
            function_name_at_script_name='getVelocity@unitScript',
            script_handle_or_type=1,
            ints=([self._index]),
            floats=(),
            strings=(),
            bytes=''
        )

        return np.array([floats[0], floats[1]])

    def applyForce(self, force):
        self._pyrep.script_call(
            function_name_at_script_name='applyForce@unitScript',
            script_handle_or_type=1,
            ints=([self._index]),
            floats=([force[0], force[1]]),
            strings=(),
            bytes=''
        )

    def seek(self, behavior=None):
        if behavior == 'arrival':
            radius = self.arrival_radius
            dist = self.distTo(self.getCurrTarget())

            rated_speed = self.max_speed
            
            if dist < radius:
                rated_speed = self.max_speed * (dist / radius)
                rated_speed = min(rated_speed, self.max_speed)

            position = self.getLocation()
            desired  = np.subtract(self._targets[0], position)
            desired = (desired / la.norm(desired)) * rated_speed

            steer = np.subtract(desired, self.getVelocity())
            steer = np.clip(steer, None, self.max_force)

            self.applyForce(steer)
        else:
            position = self.getLocation()
            desired  = np.subtract(self._targets[0], position)
            desired = (desired / la.norm(desired)) * self.max_speed

            steer = np.subtract(desired, self.getVelocity())
            steer = np.clip(steer, None, self.max_force)

            self.applyForce(steer)

    def separate(self, units):
        steer = np.array([np.nan, np.nan])
        
        for unit in units:
            curr_pos = self.getLocation()
            unit_pos = unit.getLocation()

            dist = la.norm(curr_pos - unit_pos)
            if dist < self.min_sep_dist:
                rep = np.subtract(curr_pos, unit_pos)
                rep = (rep / la.norm(rep)) * (1 / self.max_sep_speed)

                if np.isnan(steer).any():
                    steer = rep
                else:
                    steer = steer + rep

        if not np.isnan(steer).any():
            steer = np.clip(steer, None, self.max_sep_force)
            self.applyForce(steer)

    def addTarget(self, target) -> None:
        print('#{}: ADDED TARGET: {}'.format(self._index, target))
        self._targets.append(target)
            
    def nextTarget(self) -> None:
        self._targets.pop(0)
        print('#{}: NEXT TARGET: {}'.format(self._index, self._targets[0]))

    def getCurrTarget(self):
        return self._targets[0]
    
    # helper function(s)
    def distTo(self, target) -> float:
        return la.norm(target - self.getLocation())

    
