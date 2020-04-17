from core import DObject, DParam, doafter, dobefore
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

    def set_diff(self, dpos):
        self.x_p.diff = dpos.x_p
        self.y_p.diff = dpos.y_p

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
        self.alpha_p.set_value = self.__alpha_set_value
        
    def __alpha_set_value(self, value):
        self.alpha = value

    @property
    def alpha(self):
        return self.alpha_p.value

    @alpha.setter
    def alpha(self, value):
        self.alpha_p.value = value
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

    def set_orient(self, curr_world_p, move_to_world_p, fixed_p_world=None):
        curr_world_p = _convert(curr_world_p)
        move_to_world_p = _convert(move_to_world_p)
        if fixed_p_world is None:
            self.vec += move_to_world_p - curr_world_p
        else:
            fixed_p_world = _convert(fixed_p_world)
            angle = (curr_world_p - fixed_p_world).angle_to(move_to_world_p - fixed_p_world)
            sk0 = self.vec
            if sk0 != fixed_p_world:
                r0 = sk0 - fixed_p_world
                r1 = r0.rotate(angle)
                self.vec = fixed_p_world + r1
            self.alpha += angle


class RelPoint2d(Position2d):
    def __init__(self, x, y, sk=None, name=None, parent=None):
        self.sk = sk
        super().__init__(x, y, name=name, parent=parent)
    
    @property
    def vec_world(self):
        if self.sk is None:
            return self.vec
        return self.sk.to_world(self.vec)

    @vec_world.setter
    def vec_world(self, value):
        value = _convert(value)
        if self.sk is None:
            self.vec = value
        else:
            self.vec = self.sk.to_local(value)

    @property
    def dir_world(self):
        if self.sk is None:
            return self.vec
        return self.sk.to_world(self.vec, is_dir_vector=True)

    @dir_world.setter
    def dir_world(self, value):
        value = _convert(value)
        if self.sk is None:
            self.vec = value
        else:
            self.vec = self.sk.to_local(value, is_dir_vector=True)

    @property
    def vel_world(self):
        if self.sk is not None:
            return self.sk.vel_world(self.vec, is_local_point=True)
        else:
            return Vec2(0,0)


class Force(DObject):
    value_p = DParam()
    
    def __init__(self, value: float, direction:RelPoint2d, apply_point:RelPoint2d, 
                name=None, parent=None):
        self.direction = direction
        self.apply_point = apply_point
        super().__init__(name=name, parent=parent)
        self.value_p.set_value(value)
        self.add_child(direction)
        self.add_child(apply_point)

    @property
    def vec(self):
        return self.value_p.value * self.direction.vec.norm()

    @property
    def dir_world(self):
        return self.value_p.value * self.direction.dir_world.norm()

    def get_momentum(self, point_world: Vec2):
        r = self.apply_point.vec_world - point_world
        return r & self.dir_world

    
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
        self.forces_neg = []

    @doafter
    def calc_acc(self, t, y):
        acc = Vec2(0, 0)
        for f in self.forces:
            if f.enabled:
                acc += f.dir_world
        for f in self.forces_neg:
            if f.enabled:
                acc -= f.dir_world
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

    def add_GForce(self, direction=None, value=9.81):
        if direction is None:
            direction = Vec2(0, -1)
        else:
            direction = _convert(direction)
        dir_point = RelPoint2d(direction.x, direction.y, name='GForce_direction')
        apply_point = RelPoint2d(0,0, sk=self, name='GForce_apply_p')
        gforce = Force(self.mass_p.value, dir_point, apply_point, name='GForce')
        def calc_g_force(t, y):
            nonlocal self
            gforce.value_p.set_value(self.mass_p.get_value(t) * value)
        gforce._add_dobefore_foo(calc_g_force)
        for f in self.forces:
            if f.name == 'GForce':
                return
        self.add_force(gforce)
        

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

    def vel_local(self, point: Vec2, is_local_point=False) -> Vec2:
        if not is_local_point:
            point = self.to_local(point)
        return point.rotate90().norm() * self.omega_p.value         

    def vel_world(self, point: Vec2, is_local_point=False) -> Vec2:
        return self.to_world(self.vel_local(point, is_local_point), is_dir_vector=True) + self.vel.vec
    
class ForceKMu(Force):
    k_p = DParam()
    mu_p = DParam()
    len0_p = DParam()

    def __init__(self, p1: RelPoint2d, p2: RelPoint2d, len0: float, k: float, mu: float, 
                dxlimits=None, vlimits=None,
                name=None, parent=None):
        direction = RelPoint2d(0,0,name='direction')
        self.dxlimits = dxlimits
        self.vlimits = vlimits
        super().__init__(0, direction, apply_point=p1, name=name, parent=parent)
        self.p2 = p2
        self.add_child(p2)
        self.k_p.set_value(k)
        self.mu_p.set_value(mu)
        self.len0_p.set_value(len0)

    def is_dx_in_range(self, dx: float) -> bool:
        if self.dxlimits is None:
            return True
        return self.dxlimits[0] <= dx <= self.dxlimits[1]
    
    def is_v_in_range(self, v: float) -> bool:
        if self.vlimits is None:
            return True
        return self.vlimits[0] <= v <= self.vlimits[1]

    @dobefore
    def calc_force(self, t, y):
        p1_world = self.apply_point.vec_world
        p2_world = self.p2.vec_world
        
        r = p2_world - p1_world
        len1 = r.len()
        dx = len1 - self.len0_p.get_value(t)
        kforce = 0
        if self.is_dx_in_range(dx):
            kforce = dx * self.k_p.get_value(t)
        
        rn = r.norm() if len1 > 1e-9 else Vec2(1,0)
        self.direction.vec_world = rn
        v1 = self.apply_point.vel_world
        v2 = self.p2.vel_world
        v_otn = v2 * rn - v1 * rn
        muforce = 0
        if self.is_v_in_range(v_otn):
            muforce = self.mu_p.get_value(t) * v_otn
        self.value_p.set_value(kforce+muforce)

def connect_two_bodies2d(body1: Body2d, body2: Body2d, p1_loc, p2_loc, k, mu, len0=0):
    p1_loc = _convert(p1_loc)
    p2_loc = _convert(p2_loc)
    p1 = RelPoint2d(p1_loc.x, p1_loc.y, body1)
    p2 = RelPoint2d(p2_loc.x, p2_loc.y, body2)
    force1 = ForceKMu(p1, p2, len0, k, mu)
    body1.add_force(force1)

    p1 = RelPoint2d(p1_loc.x, p1_loc.y, body1)
    p2 = RelPoint2d(p2_loc.x, p2_loc.y, body2)
    force2 = ForceKMu(p2, p1, len0, k, mu)
    body2.add_force(force2)
    return force1, force2

def connect_body_to_world(body: Body2d, p_body, p_world, k, mu, len0=0):
    p_body = _convert(p_body)
    p_world = _convert(p_world)
    p1 = RelPoint2d(p_body.x, p_body.y, body)
    p2 = RelPoint2d(p_world.x, p_world.y)
    force = ForceKMu(p1, p2, len0, k, mu)
    body.add_force(force)
    return force

    
    

