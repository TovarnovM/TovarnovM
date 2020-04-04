from easyvec import Vec2, Mat2
from easyvec.geometry import _convert

class SK2:
    def __init__(self, pos=None, angle=0):
        self.pos = _convert(pos if pos is not None else (0,0))
        self.angle = angle

    def to_world(self, vec: Vec2, is_dir_vector=False) -> Vec2:
        if is_dir_vector:
            return self._mat_1 * vec
        return self._mat_1 * vec + self.pos

    def to_local(self, world: Vec2, is_dir_vector=False) -> Vec2:
        if is_dir_vector:
            return self._mat * world
        return self._mat * (world - self.pos)

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, value):
        self._angle = value
        self._mat = Mat2.from_angle(value)
        self._mat_1 = self.mat._1

    def move(self, anch1_world, to_anch1_world, anch2_world, to_anch2_world):
        dangle = (to_anch2_world - to_anch1_world).angle_to(anch2_world - anch1_world)
        self.angle += dangle
        self.pos += anch1_world - to_anch1_world
