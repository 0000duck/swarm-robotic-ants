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

    units = []
    for i in range(1, 4):
        units.append(Unit(cpsim.getPyRep(), i))

    units[0].setMode('work')
    units[0].setSubMode('gather')

    units[1].setMode('idle')
    units[1].setSubMode('gather')

    units[2].setMode('idle')

    targets0 = [
        [-1, [-5, 5]],
        [1, [5, 5]],
        [-2, [5, -5]],
        [1, [5, 5]]
    ]
    
    targets1 = [
        [-2, [5, -5]],
        [-1, [-5, 5]],
        [-2, [5, -5]],
        [-1, [-5, 5]],
        [-2, [5, -5]],
        [-1, [-5, 5]]
    ]

    for t in targets0:
        units[0].addTarget(t)
        units[1].addTarget(t)

    for t in targets1:
        units[2].addTarget(t)

    while units[0]._targets or units[1]._targets or units[2]._targets:
        for unit in units:
            mode = unit.getMode()

            if mode == 'idle':
                unit.separate(units)
                unit.idle()
            elif mode == 'seek':
                unit.seek()
                unit.separate(units)

                dist = unit.distTo(unit.getCurrTarget())
                if dist < 0.5:
                    unit.nextTarget()
            elif mode == 'work':
                submode = unit.getSubMode()

                if submode == 'gather':
                    unit.seek()
                    unit.separate(units)
                elif submode == 'return':
                    unit.seek()
                    unit.separate(units)
                elif submode == 'pickupItem':
                    if unit.holdingItem():
                        unit.actuateGripper('close')
                        unit.setSubMode('return')

                    dist = unit.findItem()
                    if dist == None:
                        if not unit.holdingItem():
                            unit.setMode('idle')

                dist = unit.distTo(unit.getCurrTarget())
                if dist < 0.5:
                    waypoint = unit.nextTarget()
                    unit.addTarget(waypoint) # move waypoint to back

                    if waypoint[0] == -1:
                        # start target
                        unit.setSubMode('gather')
                        unit.actuateGripper('open')
                        print('#{}: setting mode to GATHER'.format(unit._index))
                    elif waypoint[0] == -2:
                        # end target
                        unit.setSubMode('pickupItem')
                        print('#{}: setting mode to PICKUPITEM'.format(unit._index))
            elif mode == 'scout':
                # TODO: implement wander/scouting
                continue

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
