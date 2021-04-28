from pyrep import PyRep
import numpy as np
from numpy import linalg as la

class Unit():
    def __init__(self, pyrep: PyRep, queue, index: int):
        self._pyrep = pyrep
        self._queue = queue
        self._index = index

        # unit properties
        self._mode = 'idle'
        self._submode = 'gather'
        self._targets = []
        self._holding_item = False
        self._item = np.array([])

        self._home_base = np.array([0, 0])

        # unit movement properties
        self._max_speed = 1.0
        self._max_force = 5.0

        self._find_item_max_speed = 1.0
        self._find_item_max_force = 5.0

        # unit separation properties
        self._min_sep_dist = 2.0
        self._max_sep_speed = 5.0
        self._max_sep_force = 5.0

        # unit arrival properties
        self._arrival_rad = 1.5

    # functions associated with PyRep
    def getPosition(self):
        ints, floats, strings, byte = self._pyrep.script_call(
            function_name_at_script_name='getPosition@unitScript',
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

    def actuateGripper(self, pose: str):
        val = 0

        if pose == 'open':
            val = 0
        elif pose == 'close':
            val = 1

        self._pyrep.script_call(
            function_name_at_script_name='actuateGripper@unitScript',
            script_handle_or_type=1,
            ints=([self._index, val]),
            floats=(),
            strings=(),
            bytes=''
        )

    def getNearestItem(self):
        ints, floats, strings, byte = self._pyrep.script_call(
            function_name_at_script_name='getNearestItem@unitScript',
            script_handle_or_type=1,
            ints=([self._index]),
            floats=(),
            strings=(),
            bytes=''
        )

        if floats:
            return np.array([floats[0], floats[1]])
        else:
            return np.array([])

    def isHoldingItem(self):
        ints, floats, strings, byte = self._pyrep.script_call(
            function_name_at_script_name='isHoldingItem@unitScript',
            script_handle_or_type=1,
            ints=([self._index]),
            floats=(),
            strings=(),
            bytes=''
        )

        if ints[0] == 1:
            return True
        else:
            return False

    def setReverse(self, isReverse=0):
        ints, floats, strings, byte = self._pyrep.script_call(
            function_name_at_script_name='setUnitReverse@unitScript',
            script_handle_or_type=1,
            ints=([self._index, isReverse]),
            floats=(),
            strings=(),
            bytes=''
        )

    # unit functions
    def idle(self):
        steer = self.getVelocity()
        steer = steer * -1

        if la.norm(steer) > 0:
            steer = np.clip(steer, None, self._max_force)
            self.applyForce(steer)

    def seek(self, behavior=None):
        if behavior == 'arrival':
            radius = self._arrival_rad
            dist = self.distTo(self.getCurrTarget())

            rated_speed = self._max_speed
            
            if dist < radius:
                rated_speed = self._max_speed * (dist / radius)
                rated_speed = min(rated_speed, self._max_speed)

            position = self.getPosition()
            desired  = np.subtract((self._targets[0])[1], position)
            desired = (desired / la.norm(desired)) * rated_speed

            steer = np.subtract(desired, self.getVelocity())
            steer = np.clip(steer, None, self._max_force)

            self.applyForce(steer)
        else:
            position = self.getPosition()
            desired  = np.subtract((self._targets[0])[1], position)
            desired = (desired / la.norm(desired)) * self._max_speed

            steer = np.subtract(desired, self.getVelocity())
            steer = np.clip(steer, None, self._max_force)

            self.applyForce(steer)

    def separate(self, units):
        steer = np.array([np.nan, np.nan])
        
        for unit in units:
            curr_pos = self.getPosition()
            unit_pos = unit.getPosition()

            dist = la.norm(curr_pos - unit_pos)
            if dist < self._min_sep_dist:
                if unit.getSubMode() == 'wait':
                    if self.getSubMode() == 'gather':
                        self.setSubMode('wait')
                        self._queue.append(self._index)
                        print('(inside) added {} to queue...'.format(self._index))
                        print(self._queue)

                rep = np.subtract(curr_pos, unit_pos)
                rep = (rep / la.norm(rep)) * (1 / self._max_sep_speed)

                if np.isnan(steer).any():
                    steer = rep
                else:
                    steer = steer + rep

        if not np.isnan(steer).any():
            steer = np.clip(steer, None, self._max_sep_force)
            self.applyForce(steer)

    def findItem(self):
        target = self.getNearestItem()

        if target.size != 0:
            self._item = target
            radius = self._arrival_rad
            dist = self.distTo(target)

            rated_speed = self._find_item_max_speed

            if dist < radius:
                rated_speed = self._find_item_max_speed * (dist / radius)
                rated_speed = min(rated_speed, self._find_item_max_speed)

            position = self.getPosition()            
            desired = np.subtract(target, position)
            desired = (desired / la.norm(desired)) * rated_speed

            steer = np.subtract(desired, self.getVelocity())
            steer = np.clip(steer, None, self._find_item_max_force)

            self.applyForce(steer)
            return dist
        else:
            return None

    def goHome(self):
        position = self.getPosition()
        target = self._home_base

        desired = np.subtract(target, position)
        desired = (desired / la.norm(desired)) * self._max_speed

        steer = np.subtract(desired, self.getVelocity())
        steer = np.clip(steer, None, self._find_item_max_force)

        self.applyForce(steer)
        return self.distTo(target)

    def addTarget(self, target) -> None:
        self._targets.append(target)
            
    def nextTarget(self) -> None:
        return self._targets.pop(0)

    def getCurrTarget(self):
        return (self._targets[0])[1]

    # mode controller
    def setMode(self, mode: str) -> None:
        self._mode = mode

    def getMode(self) -> str:
        return self._mode

    def setSubMode(self, submode: str) -> None:
        self._submode = submode
    
    def getSubMode(self) -> str:
        return self._submode

    def nextMode(self) -> None:
        self._instructions.pop(0)
        self._mode = self._instructions[0]

    # helper function(s)
    def distTo(self, target) -> float:
        return la.norm(target - self.getPosition())

    def holdingItem(self) -> bool:
        self._holding_item = self.isHoldingItem()
        return self._holding_item
