import numpy as np

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

from bezier import bezier

class Plane(g.Geometry):

    def __init__(self, width=1, height=1, widthSegments=1, heightSegments=1):
        super().__init__()
        self.width = width
        self.height = height
        self.widthSegments = widthSegments
        self.heightSegments = heightSegments

    def lower(self, object_data):
        return {
            u"uuid": self.uuid,
            u"type": u"PlaneGeometry",
            u"width": self.width,
            u"height": self.height,
            u"widthSegments": self.widthSegments,
            u"heightSegments": self.heightSegments,
        }

class Visualizer:
    def __init__(self):
        self.vis = meshcat.Visualizer()
        self.robots = dict()
        # self.vis.open()

    def jupyter(self):
        return self.vis.jupyter_cell()

    def add_robot(self, name, r):
        self.vis[name].set_object(g.Box([0.14, 0.13, 0.1]))
        self.robots[name] = r

    def update_robots(self):
        for name, r in self.robots.items():
            x, y, theta = r.state
            self.vis[name].set_transform(
                    tf.translation_matrix([x, y, 0.1]).dot(
                        tf.euler_matrix(0, 0, theta)))

    # where vertices are [[x1,y1],[x2,y2],...]
    def add_line2d(self, name, line, color=0xff0000):
        # convert to 3xN of float32
        vertices = np.zeros((3, line.shape[1]), dtype=np.float32)
        print(line.shape)
        vertices[0:2,:] = line
        self.vis[name].set_object(g.Line(g.PointsGeometry(vertices), g.MeshBasicMaterial(color=color)))

    # where line_segments are [[[xs, ys], [xg,yg]], ...]
    def add_line_segments2d(self, name, line_segments, color=0xff0000):
        # convert to 3xN of float32
        vertices = np.zeros((3, len(line_segments)*2), dtype=np.float32)
        for k, ls in enumerate(line_segments):
            vertices[0:2, k*2] = ls[0]
            vertices[0:2, k*2+1] = ls[1]
        self.vis[name].set_object(g.LineSegments(g.PointsGeometry(vertices), g.MeshBasicMaterial(color=color)))

    def add_bezier(self, name, p, color=0xff0000):
        ts = np.linspace(0, 1, 100)
        ps = np.empty((len(ts), 2))
        for k, t in enumerate(ts):
            ps[k] = bezier(p, t)
        self.add_line2d(name, ps.T, color)

    def add_image(self, name, file):
        self.vis[name].set_object(g.Mesh(Plane(width=6.4, height=4.8), g.MeshPhongMaterial(map=g.ImageTexture(
                    image=g.PngImage.from_file(file)))))
        self.vis[name].set_transform(
                    tf.translation_matrix([-3, 0, 2.4]).dot(
                        tf.euler_matrix(np.pi/2, 0, np.pi/2)))
