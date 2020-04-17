from easyvec import PolyLine, Vec2
from body import Body2d

class PolygonBody(Body2d):
    def __init__(self, polyline, mass, name=None, parent=None):
        center_mass = polyline.get_center_mass()
        iz = polyline.get_Iz(center_mass) * mass
        self.polyline = polyline.add_vec(-center_mass)
        super().__init__(x=0, y=0, alpha=0, vx=0, vy=0, mass=mass, iz=iz, omega=0, name=name, parent=parent)

    def plot(self, ax, **kwargs):
        transormed_polyline = self.polyline.transform(self._mat_1).add_vec(self.vec)
        xs, ys = [], []
        for v in transormed_polyline.vecs:
            xs.append(v.x)
            ys.append(v.y)
        xs.append(xs[0])
        ys.append(ys[0])
        ax.plot(xs, ys, **kwargs)
        cm = self.to_world(Vec2(0,0))
        ax.plot([cm.x], [cm.y], 'o', color='red')



def main():
    import matplotlib.pyplot as plt
    pb = PolygonBody(PolyLine([(0,0), (3,0), (4,1), (2,2)], copy_data=True), 2)
    pb2 = PolygonBody(PolyLine([(0,0), (3,0), (4,1), (2,2)], copy_data=True), 2, name='polygon')
    fig, ax = plt.subplots()
    pb.plot(ax)
    pb2.set_orient((1,0), (1,3),(-2,0))
    pb2.plot(ax)
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')
    plt.show()
    print(pb2.get_y0())
    print(pb2.get_full_names())

if __name__ == "__main__":
    main()