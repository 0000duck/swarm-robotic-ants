from pyrep import PyRep
import time
import os

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
        
if __name__ == '__main__':
    cpSim = CoppeliaSimulator('main-scene.ttt')
    cpSim.start()

    cpSim.shutdown()
