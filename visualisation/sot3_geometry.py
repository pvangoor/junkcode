from matplotlib import pyplot as plt
import numpy as np
from pylie import SOT3

from matplotlib.animation import FuncAnimation

eta0 = np.reshape((0,0,1), (3,1))

n_angles = 20
n_scales = 8
max_angle = np.pi / 4.0
max_scale = 1.0

omega1, omega2, s = np.meshgrid(np.linspace(-max_angle,max_angle,n_angles), np.linspace(-max_angle,max_angle,n_angles), np.linspace(-max_scale,max_scale,n_scales))

def sot3normal(omega1, omega2, s):
    x = SOT3.exp(np.reshape([omega1,omega2,0,s], (4,1))) * eta0
    return x.ravel()

sot3normal_vec = np.vectorize(sot3normal, signature="(),(),()->(3)")

result = sot3normal_vec(omega1, omega2, s)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
for i in range(result.shape[0]):
    for j in range(result.shape[1]):
        ax.plot(result[i,j,:,0], result[i,j,:,1], result[i,j,:,2], 'r-')
for i in range(result.shape[1]):
    for j in range(result.shape[2]):
        ax.plot(result[:,i,j,0], result[:,i,j,1], result[:,i,j,2], 'b-')
for i in range(result.shape[0]):
    for j in range(result.shape[2]):
        ax.plot(result[i,:,j,0], result[i,:,j,1], result[i,:,j,2], 'b-')
# for i in range(result.shape[1]):
#     ax.plot(result[:,i,0], result[:,i,1], result[:,i,2], 'b-')

# ax.plot(result[:,:,0], result[:,:,1], result[:,:,2], 'r-')
# ax.plot(result[:,:,0].T, result[:,:,1].T, result[:,:,2].T, 'r-')
# plt.show()

def update(angle):
    ax.view_init(30, angle)
    return []

hz = 50
rev_time = 15
ani = FuncAnimation(fig, update, frames=np.linspace(0, 360, int(rev_time*hz)), blit=True, interval=int(1000/hz))
ani.save("sot3_geometry.mp4")
# ani.save("sot3_geometry.gif")

# plt.show()