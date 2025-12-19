from abc import ABC, abstractmethod

class AbstractObject(ABC):
    def __init__(self, mass, center, static=False):
        self.center = center
        self.mass = mass
        self.static = static

    
    @abstractmethod
    def updateObject(self):
        """Initiates the objects"""
        pass

