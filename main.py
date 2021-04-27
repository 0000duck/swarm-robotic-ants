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
    for i in range(1, 6):
        units.append(Unit(cpsim.getPyRep(), i))

    targets0 = [
        np.array([-10, 10]),
        np.array([10, -10]),
        np.array([-10, 10]),
        np.array([10, -10]),
        np.array([-10, 10]),
        np.array([10, -10])
    ]
    
    targets1 = [
        np.array([10, -10]),
        np.array([-10, 10]),
        np.array([10, -10]),
        np.array([-10, 10]),
        np.array([10, -10]),
        np.array([-10, 10])
    ]

    for t in targets0:
        units[0].addTarget(t)
        units[1].addTarget(t)

    for t in targets1:
        units[2].addTarget(t)
        units[3].addTarget(t)
        units[4].addTarget(t)

    while units[0]._targets or units[1]._targets or units[2]._targets or units[3]._targets or units[4]._targets:
        for unit in units:
            unit.seek('arrival')
            unit.separate(units)
            
            dist = unit.distTo(unit.getCurrTarget())
            if dist < 0.5:
                unit.nextTarget()
            
        cpsim.step()

    cpsim.shutdown()
