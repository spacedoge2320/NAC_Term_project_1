import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from ship_calc_ui import Ui_MainWindow
import calc_loops
from PyQt5.QtCore import QObject, QThread, pyqtSignal, qDebug
from PyQt5.QtWidgets import *
import traceback
import visualize_ship
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D


class CalculationResult(QObject):
    result = pyqtSignal(dict)

    def __init__(self, result2):
        super().__init__()
        self.result = result2

    def get_result(self):
        return self.result


class CalculationThread(QThread):
    result = pyqtSignal(dict)

    def __init__(self, ship_size, cog, mass, water_density, mode):
        super().__init__()
        self.ship_size = ship_size
        self.com = cog
        print(self.com)
        self.mass = mass
        self.water_density = water_density
        self.calc_loop1 = calc_loops.calc_loops()
        self.mode = mode

    def run(self):
        print("1")
        try:
            if self.mode == 0:
                print("mode = 0")
                result = \
                    self.calc_loop1.stability_calculation_loop(ship_size=self.ship_size, ship_com=self.com,
                                                               mass=self.mass,
                                                               water_density=self.water_density)[0]
            if self.mode == 1:
                print("mode = 1")
                result = self.calc_loop1.gz_arm_calculation_loop(ship_size=self.ship_size, ship_com=self.com,
                                                                 mass=self.mass,
                                                                 water_density=self.water_density)

            print("run")
            result_obj = CalculationResult(result)
            # self.wait()
            self.result.emit(result)
            time.sleep(0.5)

        except Exception as e:
            result = {"1": "Calculation Unsuccessful"}
            print("ERROR")
            self.result.emit(result)
            time.sleep(0.5)
            traceback.print_exc()


class MainUI():
    def main(self):
        self.app = QApplication([])
        self.app.setQuitOnLastWindowClosed(False)
        self.ui = Ui_MainWindow()
        self.window = QMainWindow()
        self.ui.setupUi(self.window)

        self.figure_3d = Figure(figsize=(10, 10))
        self.canvas_3d = FigureCanvas(self.figure_3d)
        self.ui.vis_3dplot.addWidget(self.canvas_3d)
        self.canvas_3d.setFixedSize(1200, 1200)

        self.figure_2d = Figure()
        self.canvas_2d = FigureCanvas(self.figure_2d)
        self.ui.vis_gzcurve.addWidget(self.canvas_2d)

        self.canvas_2d.setFixedSize(800, 250)

        self.reset_console(self.ui.Console_output_2)
        self.write_to_console(self.ui.Console_output_2,
                              "Welcome! \nNaval Architeture Calculation Term Project by 2022-16903 이준승, 2023-05-10" + "\n")
        self.write_to_console(self.ui.Console_output_2,
                              "Enter ship properties and press 'Analyze Equilibrium' or 'generate GZ curve'" + "\n")

        self.ui.reset_button.clicked.connect(self.reset)
        self.ui.calculate_button.clicked.connect(self.calculate_stable_pos)
        self.ui.calculate_button_2.clicked.connect(self.calculate_gz_arm)

        self.window.show()
        sys.exit(self.app.exec())

    def reset(self):
        print("Resetting Variables")
        self.ui.size1.setValue(10)
        self.ui.size2.setValue(5)
        self.ui.size3.setValue(2)
        self.ui.cog1.setValue(5)
        self.ui.cog2.setValue(0)
        self.ui.cog3.setValue(1)
        self.ui.mass.setValue(40)
        self.ui.water_density.setValue(1)
        self.reset_console(self.ui.Console_output_2)
        self.write_to_console(self.ui.Console_output_2, "Reset Complete !" + "\n")
        self.write_to_console(self.ui.Console_output_2,
                              "Enter ship properties and press 'Analyze Equilibrium' or 'generate GZ curve'" + "\n")

    def calculate_stable_pos(self):
        print("Receiving Variables")
        qDebug("Starting Calculations")
        self.reset_console(self.ui.Console_output_2)
        self.write_to_console(self.ui.Console_output_2, "Equilibrium Analysis Start" + "\n")
        self.write_to_console(self.ui.Console_output_2, "This may take 10s ~" + "\n")
        self.write_to_console(self.ui.Console_output_2, "Please Wait...." + "\n")

        try:
            ship_size = [self.ui.size1.value(), self.ui.size2.value(), self.ui.size3.value()]
            cog = [-ship_size[0] / 2 + self.ui.cog1.value(), self.ui.cog2.value(),
                   -ship_size[2] / 2 + self.ui.cog3.value()]
            # print(cog)
            mass = self.ui.mass.value()
            water_density = self.ui.water_density.value()

            # Opening Thread for Calculation
            self.thread1 = CalculationThread(ship_size=ship_size, cog=np.array(cog), mass=mass,
                                             water_density=water_density, mode=0)
            self.thread1.setStackSize(8 * 1024 * 1024)  # 8MB 스택 크기 설정
            self.thread1.result.connect(self.on_stability_calculation_complete)
            time.sleep(1)
            self.thread1.start()
            time.sleep(1)

        except Exception as e:
            traceback.print_exc()

    def calculate_gz_arm(self):
        print("Receiving Variables")
        qDebug("Starting Calculations")
        self.reset_console(self.ui.Console_output_2)
        self.write_to_console(self.ui.Console_output_2, "GZ_arm Calculation Start" + "\n")
        self.write_to_console(self.ui.Console_output_2, "This may take 20s ~" + "\n")
        self.write_to_console(self.ui.Console_output_2, "Please Wait...." + "\n")

        try:
            ship_size = [self.ui.size1.value(), self.ui.size2.value(), self.ui.size3.value()]
            cog = [-ship_size[0] / 2 + self.ui.cog1.value(), self.ui.cog2.value(),
                   -ship_size[2] / 2 + self.ui.cog3.value()]
            mass = self.ui.mass.value()
            water_density = self.ui.water_density.value()
            self.thread2 = CalculationThread(ship_size=ship_size, cog=np.array(cog), mass=mass,
                                             water_density=water_density, mode=1)
            self.thread2.setStackSize(8 * 1024 * 1024)  # 8MB 스택 크기 설정
            self.thread2.result.connect(self.on_gz_calculation_complete)
            time.sleep(1)
            self.thread2.start()
            time.sleep(1)
        except Exception as e:
            traceback.print_exc()

    def visualize_3d(self, results):
        print("visualize_3d")
        vis_ship = visualize_ship.visualization()
        self.figure_3d.clear()
        ax = self.figure_3d.add_subplot(111, projection='3d')

        max_size = max(results['Ship_size'])
        try:
            fig = plt.figure()
            # visualize ship
            vis_ship.visualize_cube(ax, results['polygons'][0])

            # visualize submerged portion
            vis_ship.visualize_convex_hull_3d(ax, results['polygons'][1], color='blue', alpha=0.5)
            vis_ship.visualize_water_plane(ax, results['polygons'][2], color='blue', alpha=0.5)

            # visualize buoyancy
            vis_ship.visualize_arrow(ax, results['forces'][0][1], (0, 0, 1), length=results['forces'][0][0][2] / 2000,
                                     color='blue')
            # visualize ship forward
            vector = (results['polygons'][0][0] - results['polygons'][0][1])

            z_pos = results['Output_values']['z_pos']
            centerpoint = [0, 0, z_pos]
            # rotate (1,0,0) by angle 
            vis_ship.visualize_arrow(ax, centerpoint, -vector / np.linalg.norm(vector), length=np.linalg.norm(vector),
                                     color='black')

            # visualize gravity
            vis_ship.visualize_arrow(ax, results['forces'][1][1], (0, 0, 1), length=results['forces'][1][0][2] / 2000,
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
            self.canvas_3d.draw()
            print("YAY!")

        except Exception as e:
            traceback.print_exc()

    def visualize_2d(self, results):
        vis_ship = visualize_ship.visualization()
        self.figure_2d.clear()
        ax = self.figure_2d.add_subplot(111)

        angles, gz_arm = results['angles'], results['gz_arm']

        print(results)

        try:
            ax.plot(angles, gz_arm)
            ax.set_xlabel('Angle (degrees)')
            ax.set_ylabel('gz_arm')
            ax.set_title('Angle and gz_arm')
            ax.grid(True)

            # left_area에 해당하는 영역을 파란색으로 채우기

            self.canvas_2d.draw()
            print("YAY!")

        except Exception as e:
            traceback.print_exc()

    def write_to_console(self, ui_console, text):
        ui_console.append(text)
        QApplication.processEvents()

    def reset_console(self, ui_console):
        ui_console.setText("")
        QApplication.processEvents()

    def on_stability_calculation_complete(self, result):
        # 계산이 완료된 후 호출되는 콜백 함수
        print("on_stability_calculation_complete")
        try:
            print("final!!!!", result['Output_values'])
            time.sleep(0.5)
            print("3")
            d = result['Output_values']['draft']
            print(d)
            t = result['Output_values']['trim']
            cob = result['Output_values']['local_cob']
            draft = "draft (in meters)" + "\nmean: " + str(round(-d['mean'], 4)) + "\naft-starboard: " + str(
                round(-d['aft_star'], 4)) + "\naft-port: " + str(round(-d['aft_port'], 4)) + "\nfore-starboard: " + str(
                round(-d['fore_star'], 4)) + "\nfore-port: " + str(round(-d['fore_port'], 4))
            heel = "heel (in degrees) " + str(round(-np.degrees(result['Ship_angle'][0]),4))
            trim = "\ntrim (in meters) : " + str(round(t, 4))
            local_cob = "\nlocal_cob (in meters)" + "\nLCB: " + str(round(cob[0], 4)) + "\nTCB : " + str(
                round(cob[1], 4)) + "\nVCB : " + str(round(cob[2], 4))

            self.reset_console(self.ui.Console_output_2)
            self.write_to_console(self.ui.Console_output_2, "Calculation Complete" + "\n")
            self.write_to_console(self.ui.Console_output_2, "Output values" + "\n")
            self.write_to_console(self.ui.Console_output_2, draft)
            self.write_to_console(self.ui.Console_output_2, heel)
            self.write_to_console(self.ui.Console_output_2, trim)
            self.write_to_console(self.ui.Console_output_2, local_cob)
            time.sleep(0.5)
            self.visualize_3d(result)
            time.sleep(1)
            self.thread1.terminate()
        except Exception as e:
            self.write_to_console(self.ui.Console_output_2, "Calculation Error!")
            traceback.print_exc()

    def on_gz_calculation_complete(self, result):
        # 계산이 완료된 후 호출되는 콜백 함수
        time.sleep(0.5)
        print("on_gz_calculation_complete")
        try:
            print("final!!!!", result)
            self.reset_console(self.ui.Console_output_2)
            self.write_to_console(self.ui.Console_output_2, "Calculation Complete" + "\n")
            self.write_to_console(self.ui.Console_output_2, "Outputting values" + "\n")
            time.sleep(0.5)
            self.visualize_2d(result)
            time.sleep(0.5)
            self.thread2.terminate()


        except Exception as e:
            self.write_to_console(self.ui.Console_output_2, "Calculation Error!")
            traceback.print_exc()


if __name__ == "__main__":
    MainUI1 = MainUI()
    MainUI1.main()
