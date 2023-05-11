import numpy as np

import bargeship_obj
import calculate_ship as cs
import visualize_ship as vis_ship


class calc_loops:
    def stability_calculation_loop(self, ship_size, ship_com, mass, water_density):

        bargeship1 = bargeship_obj.bargeship_obj(ship_size, ship_com, mass, water_density)

        # print("starting ship stability calculations")

        calc = cs.calculate_ship()
        vis = vis_ship.visualization()
        # Validate COM position
        validity = calc.validation(bargeship_obj=bargeship1)
        # print(validity)
        # console_output.setPlainText("Starting Calculations")
        # console_output.append(format("{}\n".format(validity)))

        # Gradient Descent Loop
        # moment, force = calc.main_calc(bargeship_obj=bargeship1)

        gd_params = {'max_iteration': 5000,
                     'angle_tolerance': 0.0001,
                     'force_tolerance': 0.0001,
                     'angle_scale': 0.01,
                     'force_scale': 0.02,
                     'current_point': [0, 0, 0, 0]
                     }

        stability_result = self.find_stable_point(bargeship1=bargeship1, gd_params=gd_params)

        return stability_result

    def gz_arm_calculation_loop(self, ship_size, ship_com, mass, water_density):

        bargeship1 = bargeship_obj.bargeship_obj(ship_size, ship_com, mass, water_density)
        # print("starting gz_arm calculations")

        # print("start",ship_size)

        calc = cs.calculate_ship()
        vis = vis_ship.visualization()
        # Validate COM position
        validity = calc.validation(bargeship_obj=bargeship1)
        # print(validity)
        # console_output.setPlainText("Starting Calculations")
        # console_output.append(format("{}\n".format(validity)))

        # Gradient Descent Loop
        # moment, force = calc.main_calc(bargeship_obj=bargeship1)
        gd_params2 = {'max_iteration': 5000,
                      'angle_tolerance': 0.0001,
                      'force_tolerance': 0.0001,
                      'angle_scale': 0.01,
                      'force_scale': 0.02,
                      'current_point': [0, 0, 0, 0]
                      }

        gz_arm_result = self.gz_curve(bargeship1=bargeship1, gd_params=gd_params2)

        return gz_arm_result

    def find_stable_point(self, bargeship1, gd_params):

        max_iteration = gd_params['max_iteration']
        angle_tolerance = gd_params['angle_tolerance']
        force_tolerance = gd_params['force_tolerance']
        angle_scale = gd_params['angle_scale']
        force_scale = gd_params['force_scale']
        current_point = gd_params['current_point']
        iterations = 1

        calc = cs.calculate_ship()
        vis = vis_ship.visualization()
        prev_grad = np.empty(4)

        for i in range(0, 2):
            Ix = bargeship1.mass * (bargeship1.ship_size[1] ** 2 + bargeship1.ship_size[2] ** 2) / 12
            Iy = bargeship1.mass * (bargeship1.ship_size[0] ** 2 + bargeship1.ship_size[2] ** 2) / 12
            Iz = bargeship1.mass * (bargeship1.ship_size[0] ** 2 + bargeship1.ship_size[1] ** 2) / 12

        while True:
            # print("iterations: ", iterations)
            bargeship1.angle, bargeship1.z_pos = current_point[0:3], current_point[3]
            results = calc.main_calc(bargeship_obj=bargeship1)
            moment, net_force = results['moment_and_force']

            grad = np.array(
                [angle_scale * moment[0] / Ix, angle_scale * moment[1] / Iy, angle_scale * moment[2] / Iz,
                 force_scale * net_force[2] / bargeship1.mass])

            # print("grad",grad)
            if iterations >= max_iteration:
                print("iterations reached max value")
                break

            if all(abs(m / Ix) <= angle_tolerance for m in moment) and net_force[
                2] / bargeship1.mass <= force_tolerance:
                print("grad within tolerance")
                break
            current_point = current_point + grad
            # print(current_point)
            iterations += 1

        optimal_point = current_point

        print("Final_result", iterations)

        # vis.main(results=results)
        return results, iterations

    def gz_curve(self, bargeship1, gd_params):
        max_iteration = gd_params['max_iteration']
        force_tolerance = gd_params['force_tolerance']
        force_scale = gd_params['force_scale']
        gz_arm = []

        calc = cs.calculate_ship()
        vis = vis_ship.visualization()

        range2 = range(-30, 30, 2)
        range3 = range(-100, 110, 10)
        # Convert ranges to lists and concatenate them
        combined_range = list(range2) + list(range3)

        # Sort the combined range in ascending order
        sorted_range = sorted(combined_range)

        new_range = range2
        # Find GZ curve
        for angle in sorted_range:
            iterations = 1
            current_point = [np.radians(angle), 0, 0, 0]
            print(angle)

            while True:
                # print("iterations: ", iterations)
                bargeship1.angle, bargeship1.z_pos = current_point[0:3], current_point[3]
                results = calc.main_calc(bargeship_obj=bargeship1)
                moment, net_force = results['moment_and_force']
                # print("!!!:", moment, net_force)
                # print(bargeship1.angle, bargeship1.z_pos)
                grad = np.array([0, 0, 0, force_scale * net_force[2] / bargeship1.mass])

                if iterations >= max_iteration:
                    print("iterations reached max value")
                    break

                if net_force[2] / bargeship1.mass <= force_tolerance:
                    # print("grad", grad)
                    print("grad within tolerance", iterations)
                    break

                current_point = current_point + grad * 2
                # print(current_point)
                iterations += 1

            x_moment = results['moment_and_force'][0][0]
            gz_arm.append(-x_moment / (9.81 * bargeship1.mass))
            # print()

        # vis.gz_curve(angles=range2, gz_arm=gz_arm)
        return {'angles': sorted_range, "gz_arm": gz_arm}
