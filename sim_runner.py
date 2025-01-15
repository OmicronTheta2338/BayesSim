import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.animation import FuncAnimation
from itertools import count

from sim_v2 import RocketSimulation

sim = RocketSimulation()

# sim.set_fin_displacements([0.3, 0.0, 0.3, 0.0])

time_vals   = []
x_positions = []
y_positions = []
tilt_vals   = []

index = count()

fig, (ax1, ax2) = plt.subplots(2, 1, gridspec_kw={'height_ratios': [2, 1]})
plt.subplots_adjust(bottom=0.3)
def lateral_position():
    return sim.rocket.lateral_position

def update(i):
    time_vals.append(next(index))
    x_positions.append(lateral_position()[0])
    y_positions.append(lateral_position()[1])
    tilt_vals.append(sim.rocket.vector @ np.array([0, 0, 1]))


    ax1.cla()
    ax1.plot(x_positions, y_positions, label ='Trajectory')
    ax1.set_title("Rocket Trajectory")
    ax1.set_xlabel("X Position")
    ax1.set_ylabel("Y Position")
    ax1.legend()

    ax2.cla()
    ax2.plot(time_vals, tilt_vals, label ='Tilt Vals')

    current_fins = []
    for fin in sim.fins:
        current_fins.append(fin.displacement)

    slider1.set_val(current_fins[0])
    slider2.set_val(current_fins[1])
    slider3.set_val(current_fins[2])
    slider4.set_val(current_fins[3])


    sim.update()

slider_ax1 = plt.axes([0.1, 0.2, 0.8, 0.03])
slider_ax2 = plt.axes([0.1, 0.15, 0.8, 0.03])
slider_ax3 = plt.axes([0.1, 0.1, 0.8, 0.03])
slider_ax4 = plt.axes([0.1, 0.05, 0.8, 0.03])

slider1 = Slider(slider_ax1, 'Fin 1', -np.pi / 4, np.pi / 4, valinit=0.0)
slider2 = Slider(slider_ax2, 'Fin 2', -np.pi / 4, np.pi / 4, valinit=0.0)
slider3 = Slider(slider_ax3, 'Fin 3', -np.pi / 4, np.pi / 4, valinit=0.0)
slider4 = Slider(slider_ax4, 'Fin 4', -np.pi / 4, np.pi / 4, valinit=0.0)


def update_fin_displacements(val):
    fin_displacements = [slider1.val, slider2.val, slider3.val, slider4.val]
    sim.set_fin_displacements(fin_displacements)
    # print("Fin displacements set to: ", fin_displacements)

slider1.on_changed(update_fin_displacements)
slider2.on_changed(update_fin_displacements)
slider3.on_changed(update_fin_displacements)
slider4.on_changed(update_fin_displacements)


ani = FuncAnimation(plt.gcf(), update, interval=1000, cache_frame_data=False)
plt.show()
