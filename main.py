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
    cpSim = CoppeliaSimulator('main-scene.ttt')
    cpSim.start()

    unit = Unit(cpSim.getPyRep(), 1)

    target = np.array([10.0, 5.0])

    targets = [
        np.array([10.0, 5.0]),
        np.array([10.0, -5.0]),
        np.array([-10.0, 10.0]),
        np.array([-10.0, -10.0]),
        np.array([0, 0])
    ]

    for target in targets:
        dist = unit.seek(target)

        while dist > 0.5:
            cpSim.step()
            dist = unit.seek(target)

    cpSim.shutdown()
