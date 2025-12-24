from abc import ABC, abstractmethod

class AbstractObject(ABC):
    def __init__(self, mass, center, calc, static=False):
        self.center = center
        self.mass = mass
        self.static = static
        self.calculator = calc
        self.calcIdx = self.calculator.addObject(self)
    
    @abstractmethod
    def updateObject(self):
        """Initiates the objects"""
        pass

