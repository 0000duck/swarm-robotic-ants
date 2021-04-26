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
    
    targets = [
        np.array([10.0, 5.0]),
        np.array([10.0, -5.0]),
        np.array([-10.0, 10.0]),
        np.array([-10.0, -10.0]),
        np.array([0, 0])
    ]

    for target in targets:
        dist = 1000
        for unit in units:
            dist = min(dist, unit.seek(target))

        while dist > 0.5:
            cpsim.step()
            for unit in units:
                dist = min(dist, unit.seek(target))
                unit.separate(units)

    cpSim.shutdown()
