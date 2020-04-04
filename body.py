from core import DObject, DParam, doafter, doafter
from easyvec import Vec2, Mat2
from easyvec.geometry import _convert

class DValue(DObject):
    value_p = DParam()

    def __init__(self, value, name=None, parent=None):
        super().__init__(name=name, parent=parent)
        self.value_p.set_value(value)

    @property
    def value(self):
        return self.value_p.value

    @value.setter
    def value(self, value):
        self.value_p.set_value(value)


class Position2d(DObject):
    x_p = DParam()
    y_p = DParam()
    def __init__(self, x, y, name=None, parent=None):
        super().__init__(name=name, parent=parent)
        self.x = x
        self.y = y

    @property
    def vec(self):
        return Vec2(self.x_p.value, self.y_p.value)

    @vec.setter
    def vec(self, value):
        self.x_p.set_value(value[0])
        self.y_p.set_value(value[1])

    def set_diff(self, dpos: Position2d):
        self.px.diff = dpos.x_p
        self.py.diff = dpos.y_p

    @property
    def x(self):
        return self.x_p.value
    
    @x.setter
    def x(self, value):
        self.x_p.set_value(value)

    @property
    def y(self):
        return self.y_p.value
    
    @y.setter
    def y(self, value):
        self.y_p.set_value(value) 

class Orient2d(Position2d):
    alpha_p = DParam()
    def __init__(self, x, y, alpha, name=None, parent=None):
        super().__init__(x, y, name=name, parent=parent)
        self.alpha = alpha
        
    @property
    def alpha(self):
        return self.alpha_p.value

    @alpha.setter
    def alpha(self, value):
        self.alpha.set_value(value)
        self._mat = Mat2.from_angle(value)
        self._mat_1 = self._mat._1

    def to_world(self, vec: Vec2, is_dir_vector=False) -> Vec2:
        if is_dir_vector:
            return self._mat_1 * vec
        return self._mat_1 * vec + self.vec

    def to_local(self, world: Vec2, is_dir_vector=False) -> Vec2:
        if is_dir_vector:
            return self._mat * world
        return self._mat * (world - self.vec)

    def set_orient(self, curr_world_p: Vec2, move_to_world_p: Vec2, fixed_p_world=None):
        if fixed_p_world is None:
            self.pos += move_to_world_p - curr_world_p
        else:
            fixed_p_world = _convert(fixed_p_world)
            angle = (curr_world_p - fixed_p_world).angle_to(move_to_world_p - fixed_p_world)
            sk0 = self.vec
            if sk0 != fixed_p_world:
                r0 = sk0 - fixed_p_world
                r1 = r0.rotate(angle)
                self.pos = fixed_p_world + r1
            self.alpha += angle


class RelPoint2d(Position2d):
    def __init__(self, x, y, sk=None, name=None, parent=None):
        super().__init__(x, y, name=name, parent=parent)
        self.sk = sk
    
    @property
    def vec_world(self):
        if self.sk is None:
            return self.vec
        return self.sk.to_world(self.vec)

    @property
    def dir_world(self):
        if self.sk is None:
            return self.vec
        return self.sk.to_world(self.vec, is_dir_vector=True)


class Force(DObject):
    value_p = DParam()
    
    def __init__(self, value: float, direction:RelPoint2d, apply_point:RelPoint2d, 
                name=None, parent=None):
        super().__init__(name=name, parent=parent)
        self.value_p.set_value(value)
        self.direction = direction
        self.apply_point = apply_point
        self.add_child(direction)
        self.add_child(apply_point)

    @property
    def vec(self):
        return self.value_p.value * self.direction.vec.norm()

    @property
    def vec_world(self):
        return self.value_p.value * self.direction.vec_world.norm()

    def get_momentum(self, point_world: Vec2):
        r = self.apply_point.pos_world - point_world
        return r & self.vec_world

    
class MatPoint2d(Orient2d):
    mass_p = DParam()

    def __init__(self, x, y, alpha, vx, vy, mass, name=None, parent=None):
        super().__init__(x, y, alpha, name=name, parent=parent)
        self.vel = Position2d(vx, vy, "vel")
        self.add_child(self.vel)
        self.set_diff(self.vel)

        self.acc = Position2d(0,0, "acc")
        self.vel.add_child(self.acc)
        self.vel.set_diff(self.acc)

        self.mass_p.set_value(mass)

        self.forces = []
        self.forced_neg = []

    @doafter
    def calc_acc(self, t, y):
        acc = Vec2(0, 0)
        for f in self.forces:
            if f.enabled:
                acc += f.vec_world
        for f in self.forced_neg:
            if f.enabled:
                acc -= f.vec_world
        acc /= self.mass_p.get_value(t)
        self.acc.vec = acc
        return acc

    def add_force(self, force: Force):
        self.forces.append(force)
        if force.parent is None:
            self.add_child(force)

    def add_force_neg(self, force: Force):
        self.forces_neg.append(force)
        if force.parent is None:
            self.add_child(force)

class Body2d(MatPoint2d):
    iz_p = DParam()
    omega_p = DParam()
    epsilon_p = DParam()

    def __init__(self, x, y, alpha, vx, vy, mass, iz, omega, name=None, parent=None):
        super().__init__(x, y, alpha, vx, vy, mass, name=name, parent=parent)
        self.iz_p.set_value(iz)
        self.omega_p.set_value(omega)

        self.alpha_p.diff = self.omega_p
        self.omega_p.diff = self.epsilon_p

        self.moments = []
        self.moments_neg = []

    def add_moment(self, moment: DValue):
        self.moments.append(moment)
        if moment.parent is None:
            self.add_child(moment)

    def add_moment_neg(self, moment: DValue):
        self.moments_neg.append(moment)
        if moment.parent is None:
            self.add_child(moment)

    @doafter
    def calc_epsilon(self, t, y):
        moment = 0.0
        for m in self.moments:
            if m.enabled:
                moment += m.value
        for m in self.moments_neg:
            if m.enabled:
                moment -= m.value
        pos = self.vec
        for f in self.forces:
            if f.enabled:
                moment += f.get_momentum(pos)
        for f in self.forces_neg:
            if f.enabled:
                moment -= f.get_momentum(pos)
        epsilon = moment / self.iz_p.get_value(t)
        self.epsilon_p.set_value(epsilon)
        return epsilon
    

