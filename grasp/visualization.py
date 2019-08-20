import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

file_path = "/home/sai2/sai2/apps/allergo_grasping/build/grasp/output.txt"
with open(file_path) as f:
	content = f.read()
	# print(content)
content = content.rstrip("\n")
content = content.split("\n")
coordinates = np.zeros((3,5,3))
finger_index = -1
point_index = -1
coordinate_index = -1
for i,line in enumerate(content):
	coordinate_index += 1
	if i % 3 == 0:
		coordinate_index = 0
		point_index += 1
	if i % 15 == 0:
		point_index = 0
		finger_index += 1
	coordinates[finger_index, point_index, coordinate_index] = float(content[i])
fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')
color = ['r','g','b','y','k']
for finger_index in range(3):
	for point_index in range(5):
		ax.scatter(coordinates[finger_index,point_index,0], coordinates[finger_index,point_index,1],coordinates[finger_index,point_index,2], c = color[point_index])

# picture = plt.subplot(111, projection = '3d')
ax.set_zlabel("Z")
ax.set_ylabel("Y")
ax.set_xlabel("X")
plt.show()
	