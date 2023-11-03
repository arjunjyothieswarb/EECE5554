import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))

def get_arrow(theta):
    # x = np.cos(theta)
    # y = np.sin(theta)
    x = 0
    y = 0
    z = 0
    u = np.sin(2*theta)
    v = np.sin(3*theta)
    w = np.cos(3*theta)
    return x,y,z,u,v,w

quiver1 = ax.quiver(*get_arrow(0))
quiver2 = ax.quiver(*get_arrow(0))

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-2, 2)

def update(theta):
    global quiver1, quiver2
    quiver1.remove()
    quiver2.remove()
    quiver1 = ax.quiver(*get_arrow(theta))
    quiver2 = ax.quiver(*get_arrow(theta + (np.pi/2)))

ani = FuncAnimation(fig, update, frames=np.linspace(0,2*np.pi,200), interval=50)
plt.show()