import numpy as np
from scipy.spatial import Delaunay


class calculate_ship:
    def validation(self, bargeship_obj):
        # Validate inputs
        if (not (
                self.validate_com_pos(bargeship_obj.ship_com,
                                      bargeship_obj.ship_size))):  # The ship has invalid COM position
            raise ValueError("Center of mass should be inside ship boundaries")

        return "COM pos valid"

    def main_calc(self, bargeship_obj):
        # Transform COM
        # print(bargeship_obj.ship_com)

        rotated_com = self.rotate_polygon(bargeship_obj.angle, np.array([bargeship_obj.ship_com]))[0]
        transformed_com = self.transform_polygon(bargeship_obj.z_pos, np.array([rotated_com]))[0]

        # Transform Vertices
        rotated_polygon = self.rotate_polygon(bargeship_obj.angle, bargeship_obj.vertices)
        transformed_polygon = self.transform_polygon(bargeship_obj.z_pos, rotated_polygon)
        # print(rotated_polygon)

        # Generate new polygon from volume under water plane
        points_underwater = self.return_points_underwater(transformed_polygon)

        intersections = self.find_intersection_points(transformed_polygon)

        if len(intersections) == 0: raise ValueError(
            "Ship is sunken or levitating! (no intersection between waterplane and ship)")

        # print("points_underwater",points_underwater,"intersections",intersections)

        submerged_polygon = np.concatenate((intersections, points_underwater), axis=0)

        # print("submerged_polygon is",subemrged_polygon)

        # Find the centroid of the shape
        total_volume, centroid_of_submerged_volume = self.hexahedron_volume_and_centroid(submerged_polygon)

        # print("centroid is",centroid_of_submerged_volume)

        # Find buoyancy and gravity
        buoyancy = np.array([0, 0, total_volume * bargeship_obj.water_density * 9.81])
        gravity = np.array([0, 0, -bargeship_obj.mass * 9.81])
        # print("volume", total_volume,"buoyancy",buoyancy,"gravity: ",gravity)

        # Find the moment from the buoyant force & gravity
        moment_buoyancy = self.calculate_moment(buoyancy, centroid_of_submerged_volume)
        moment_gravity = self.calculate_moment(gravity, transformed_com)
        net_moment = moment_buoyancy + moment_gravity
        net_force = buoyancy + gravity
        # print("net_force ",net_force)
        # print("moment is", moment)

        rev_angle = []

        for i in range(0, 3):
            rev_angle.append(-bargeship_obj.angle[i])

        transformed_cob = self.transform_polygon(-bargeship_obj.z_pos, [centroid_of_submerged_volume])[0]
        rotated_cob = self.rotate_polygon(rev_angle, transformed_cob)

        draft = {"mean": (transformed_polygon[0][2] + transformed_polygon[1][2] + transformed_polygon[2][2] +
                          transformed_polygon[3][2]) / 4, "aft_port": transformed_polygon[3][2],
                 "aft_star": transformed_polygon[0][2], "fore_port": transformed_polygon[1][2],
                 "fore_star": transformed_polygon[2][2]}
        trim = (((transformed_polygon[1][2] + transformed_polygon[2][2]) / 2) - (
                    (transformed_polygon[3][2] + transformed_polygon[0][2]) / 2))
        local_cob = rotated_cob + np.array([bargeship_obj.ship_size[0] / 2, 0, bargeship_obj.ship_size[2] / 2])
        # print(local_cob)

        return {"moment_and_force": [net_moment, net_force],
                "polygons": [transformed_polygon, submerged_polygon, intersections],
                "forces": [[buoyancy, centroid_of_submerged_volume], [gravity, transformed_com]],
                "Output_values": {'draft': draft, 'trim': trim, 'local_cob': local_cob, 'z_pos': bargeship_obj.z_pos},
                "Ship_size": bargeship_obj.ship_size, "Ship_angle": bargeship_obj.angle}

    def validate_com_pos(self, ship_com, ship_size):
        # this function validates the center of mass position by comparing it to the ship dimensions
        inside_bounds = (
                abs(ship_com[0]) <= ship_size[0] / 2 and
                abs(ship_com[1]) <= ship_size[1] / 2 and
                abs(ship_com[2]) <= ship_size[2] / 2
        )
        return inside_bounds

    def transform_polygon(self, z_height, vertices):
        transformed_vertex = []
        for vertex in vertices:
            transformed_vertex.append(np.array([vertex[0], vertex[1], vertex[2] + z_height]))
        return transformed_vertex

    def rotate_polygon(self, angle, vertices):
        # Rotation matrices
        rx = np.array([
            [1, 0, 0],
            [0, np.cos(angle[0]), -np.sin(angle[0])],
            [0, np.sin(angle[0]), np.cos(angle[0])]
        ])

        ry = np.array([
            [np.cos(angle[1]), 0, np.sin(angle[1])],
            [0, 1, 0],
            [-np.sin(angle[1]), 0, np.cos(angle[1])]
        ])

        rz = np.array([
            [np.cos(angle[2]), -np.sin(angle[2]), 0],
            [np.sin(angle[2]), np.cos(angle[2]), 0],
            [0, 0, 1]
        ])

        # Combine rotations
        rotation_matrix = np.dot(np.dot(rx, ry), rz)

        # Apply rotation to vertices
        transformed_vertices = np.dot(vertices, rotation_matrix.T)

        return transformed_vertices

    def return_points_underwater(self, vertices):
        # This function returns all vertices that has <0 height
        below_z = []
        for i in range(0, len(vertices)):
            if vertices[i][2] < 0:
                below_z.append(vertices[i])

        below_z = np.array(below_z)
        # print("below_z is ",below_z)
        return below_z

    def find_intersection_points(self, vertices, Z=0):
        # This function first finds vertices intersect with plane of height Z. Then, it goes through every edge of the bargeship and finds edges that intersect with plane of height Z.
        edges = [
            (vertices[0], vertices[1]),
            (vertices[1], vertices[2]),
            (vertices[2], vertices[3]),
            (vertices[3], vertices[0]),
            (vertices[4], vertices[5]),
            (vertices[5], vertices[6]),
            (vertices[6], vertices[7]),
            (vertices[7], vertices[4]),
            (vertices[0], vertices[4]),
            (vertices[1], vertices[5]),
            (vertices[2], vertices[6]),
            (vertices[3], vertices[7]),
        ]
        intersection_points = []
        for v in vertices:
            if v[2] == Z:
                intersection_points.append(v)
        for edge in edges:
            v1, v2 = edge
            if (v1[2] > Z and v2[2] < Z) or (v1[2] < Z and v2[2] > Z):
                t = (Z - v1[2]) / (v2[2] - v1[2])
                intersection_point = v1 + t * (v2 - v1)
                intersection_points.append(intersection_point)

        # print(np.array(intersection_points))
        return np.array(intersection_points)

    def calculate_moment(self, force, point):
        Moment_i = point[1] * force[2] - point[2] * force[1]
        Moment_j = -(point[0] * force[2] - point[2] * force[0])
        Moment_k = point[0] * force[1] - point[1] * force[0]

        Moment = np.array([Moment_i, Moment_j, Moment_k])

        # print(Moment)

        return Moment

    def tetrahedron_volume(self, A, B, C, D):
        V = np.abs(np.dot(A - D, np.cross(B - D, C - D))) / 6
        centroid = (A + B + C + D) / 4
        return V, centroid

    def hexahedron_volume_and_centroid(self, vertices):
        hexahedron = np.array(vertices)

        # Delaunay triangulation
        tri = Delaunay(hexahedron)
        tetrahedrons = hexahedron[tri.simplices]

        total_volume = 0
        centroid_times_volume = np.zeros(3)
        for tetrahedron in tetrahedrons:
            volume, centroid = self.tetrahedron_volume(*tetrahedron)
            total_volume += volume
            centroid_times_volume += centroid * volume

        total_centroid = centroid_times_volume / total_volume
        return total_volume, total_centroid
