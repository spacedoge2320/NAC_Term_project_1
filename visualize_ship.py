from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np


class visualization:

    def main(self, results):

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # visualize ship
        self.visualize_cube(ax, results['polygons'][0])

        # visualize submerged portion
        self.visualize_convex_hull_3d(ax, results['polygons'][1], color='blue', alpha=0.5)
        self.visualize_water_plane(ax, results['polygons'][2], color='blue', alpha=0.5)

        # visualize buoyancy
        self.visualize_arrow(ax, results['forces'][0][1], (0, 0, 1), length=results['forces'][0][0][2] / 2000,
                             color='black')
        # visualize gravity
        self.visualize_arrow(ax, results['forces'][1][1], (0, 0, 1), length=results['forces'][1][0][2] / 2000,
                             color='red')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_xlim([-30, 30])
        ax.set_ylim([-30, 30])
        ax.set_zlim([-30, 30])

        ax.set_xscale('linear')
        ax.set_yscale('linear')
        ax.set_zscale('linear')

        ax.set_axis_off()

        ax.set_box_aspect([1, 1, 1])
        ax.view_init(elev=20, azim=20)

        # 그래프 출력
        plt.show()

    def gz_curve(self, angles, gz_arm):
        plt.plot(angles, gz_arm)
        plt.xlabel('Angle (degrees)')
        plt.ylabel('gz_arm')
        plt.title('Angle and gz_arm')
        plt.grid(True)

        # left_area에 해당하는 영역을 파란색으로 채우기

        plt.show()

    def visualize_cube(self, ax, vertices, line_color='-k', face_color='grey', alpha=0.1):
        # Extract x, y, and z coordinates from vertices
        x = [v[0] for v in vertices]
        y = [v[1] for v in vertices]
        z = [v[2] for v in vertices]

        # Define the edges of the cube
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom face
            [4, 5], [5, 6], [6, 7], [7, 4],  # Top face
            [0, 4], [1, 5], [2, 6], [3, 7]  # Vertical edges
        ]

        # Plot the edges of the cube
        for edge in edges:
            ax.plot([x[edge[0]], x[edge[1]]],
                    [y[edge[0]], y[edge[1]]],
                    [z[edge[0]], z[edge[1]]],
                    line_color)

        # Define the surfaces of the cube
        surfaces = [
            [0, 1, 2, 3],  # Bottom face
            [4, 5, 6, 7],  # Top face
            [0, 1, 5, 4],  # Side face
            [1, 2, 6, 5],  # Side face
            [2, 3, 7, 6],  # Side face
            [3, 0, 4, 7]  # Side face
        ]

        # Plot the surfaces of the cube
        cube = Poly3DCollection([[(x[idx], y[idx], z[idx]) for idx in surface] for surface in surfaces], alpha=alpha)
        cube.set_facecolor(face_color)
        ax.add_collection3d(cube)

    def visualize_convex_hull_3d(self, ax, points, color='blue', alpha=0.01):
        hull = ConvexHull(points)

        hull_faces = hull.simplices

        # 도형의 원본 점을 그림
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='g', marker='o')

        ax.plot_trisurf(points[:, 0], points[:, 1], points[:, 2], triangles=hull_faces, color=color, alpha=alpha,
                        shade=False)

    def visualize_water_plane(self, ax, points, color='blue', alpha=0.1):
        points_2d = points[:, :2]
        hull = ConvexHull(points_2d)
        boundary_points = points_2d[hull.vertices]
        points_3d = np.column_stack((points_2d, np.zeros(points_2d.shape[0])))
        center_point = np.mean(points_3d, axis=0)
        angles = np.arctan2(points[:, 1] - center_point[1], points[:, 0] - center_point[0])
        sorted_indices = np.argsort(angles)
        sorted_points = points[sorted_indices]

        lines = []
        for i in range(len(sorted_points)):
            lines.append([sorted_points[i], sorted_points[(i + 1) % len(sorted_points)]])

        line_collection = Line3DCollection(lines, colors='black')
        ax.add_collection(line_collection)

    def visualize_arrow(self, ax, point, angle, length, color):
        ax.quiver(point[0], point[1], point[2], angle[0], angle[1], angle[2], length=length, color=color)
