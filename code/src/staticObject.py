class StaticObject(AbstractObject):

    def __init__(self, hitbox):
        super().__init__(3, np.array([0,0,0]), True) # random non zero value for mass, doesn't matter since the object will never move
        self.hitbox = hitbox