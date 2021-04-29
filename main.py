from pyrep import PyRep
import time
import os

import numpy as np
from unit import Unit

class CoppeliaSimulator():
    def __init__(self, scene: str) -> None:
        self._pyrep = PyRep()
        self._pyrep.launch(os.getcwd() + "/" + scene, headless=False)
        time.sleep(2.0)

    def start(self) -> None:
        self._pyrep.start()

    def shutdown(self) -> None:
        self._pyrep.shutdown()

    def getPyRep(self) -> PyRep:
        return self._pyrep

    def step(self) -> None:
        self._pyrep.step()

if __name__ == '__main__':
    cpsim = CoppeliaSimulator('main-scene.ttt')
    cpsim.start()

    queue = []
    units = []
    for i in range(1, 8):
        units.append(Unit(cpsim.getPyRep(), queue, i))

    for unit in units:
        unit.setMode('work')
        unit.setSubMode('gather')

    targets = [
        [-1, [-5, 5]],
        [1, [5, 5]],
        [-2, [5, -5]],
        [1, [-5, -5]]
    ]

    for unit in units:
        for target in targets:
            unit.addTarget(target)

    active = True

    while active:
        active = False
        for unit in units:
            if unit._start_pos[0] == 0 and unit._start_pos[1] == 0:
                unit._start_pos = unit.getPosition()
            
            if unit.getMode() != 'idle':
                active = True

            mode = unit.getMode()

            if mode == 'base':
                dist = unit.goTo(unit._start_pos)

                if dist < 0.5:
                    unit.setMode('idle')
                continue
            if mode == 'idle':
                unit.separate(units)
                unit.idle()
            if mode == 'goHome':
                dist = unit.goHome()

                if dist < 2.0:
                    unit._targets.clear()
            elif mode == 'seek':
                unit.seek()
                unit.separate(units)

                dist = unit.distTo(unit.getCurrTarget())
                if dist < 1.0:
                    unit.nextTarget()
            elif mode == 'work':
                submode = unit.getSubMode()

                if submode == 'wait':
                    if (unit.getNearestItem()).size == 0:
                        # no more items
                        unit.setSubMode('return')
                    
                    q_index = queue.index(unit._index)

                    if q_index > 0:
                        in_front = queue[q_index - 1]
                        dist = unit.distTo(units[in_front - 1].getPosition())

                        if dist > unit._min_sep_dist:
                            unit.goTo(units[in_front - 1].getPosition())
                        else:
                            unit.idle()
                    elif q_index == 0:
                        dist = unit.distTo(targets[2][1])

                        if dist > 1.0:
                            unit.goTo(targets[2][1])
                        else:
                            unit.idle()
                        
                    count = 0
                    for u in units:
                        su = u.getSubMode()
                        if su == 'pickupItem' or su == 'reverse':
                            count = count + 1

                    if count == 0:
                        nextUnit = queue.pop(0)
                        print('removed {} from queue...'.format(nextUnit))
                        print(queue)
                        units[nextUnit - 1].setSubMode('pickupItem')
                        continue
                elif submode == 'gather':
                    unit.seek()
                    unit.separate(units)
                elif submode == 'return':
                    unit.seek()
                    unit.separate(units)
                elif submode == 'pickupItem':
                    if unit.holdingItem():
                        unit.actuateGripper('close')
                        unit.setSubMode('reverse')

                    dist = unit.findItem()
                    if dist == None:
                        if not unit.holdingItem():
                            unit.setSubMode('return')

                            continue
                elif submode == 'reverse':
                    if unit.distTo(unit._item) < 2.0:
                        unit.setReverse(1)
                    else:
                        unit._item = np.array([])
                        unit.setReverse(0)
                        unit.setSubMode('return')
                        print('[#{}]: (reverse) setting submode to `return`...'.format(unit._index))
                elif submode == 'dropOff':
                    dist = unit.goHome()

                    if dist < 1.0:
                        unit.actuateGripper('open')
                        unit.setSubMode('dropItem')
                        print('[#{}]: setting sub-mode to `dropItem`'.format(unit._index))
                elif submode == 'dropItem':
                    if unit.distTo(unit._home_base) < 2.0:
                        unit.setReverse(1)
                    else:
                        unit.setReverse(0)
                        unit.setSubMode('gather')
                        print('[#{}]: setting sub-mode to `gather`'.format(unit._index))

                dist = unit.distTo(unit.getCurrTarget())
                if dist < 2.0:
                    waypoint = unit.nextTarget()
                    unit.addTarget(waypoint) # move waypoint to back

                    if waypoint[0] == -1:
                        # start target
                        if unit.holdingItem():
                            unit.setSubMode('dropOff')
                            print('[#{}]: setting sub-mode to `goHome`'.format(unit._index))
                        elif unit.getNearestItem().size == 0:
                            # no more items, return back to base
                            unit.setMode('base')
                    elif waypoint[0] == -2:
                        # end target
                        if unit.getSubMode() != 'pickupItem':
                            count = 0
                            for u in units:
                                if u.getSubMode() == 'pickupItem' or u.getSubMode() == 'reverse':
                                    count = count + 1

                            if count == 0:
                                unit.setSubMode('pickupItem')
                                print('[#{}]: setting sub-mode to `pickupItem`'.format(unit._index))
                            else:
                                if unit.getSubMode() != 'wait':
                                    queue.append(unit._index)
                                    print('added {} to queue...'.format(unit._index))
                                    print(queue)
                                    unit.setSubMode('wait')
                                    print('[#{}]: setting sub-mode to `wait`'.format(unit._index))

        # increment simulator
        cpsim.step()

    # end of simulation
    cpsim.shutdown()

'''
UNIT MODES:
-----
1. Seek (seek): When a unit is in `seek` mode, the unit will continuously seek
out the set of targets in its unit._targets list. When the last target is reached,
its mode is changed to `idle`
---
2. Idle (idle): When a unit is in `idle` mode, the unit will continuously to remain
unmoved from where it is.
---
3. Work (work): When a unit is in `work` mode, the unit will have a path
(set of waypoints) to follow from the home base to the final destination that it
will continually seek. When the unit is at the final destination target, it will
search for the closest supply, pick it up, then return home, drop it off,
and return back to its path.
'''
