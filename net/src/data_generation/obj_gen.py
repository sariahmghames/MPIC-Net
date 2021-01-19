import os
import numpy as np
from PIL import Image
import os.path
import random
import math
#import additions.rigid_body
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from visual import *
 
import logging
import time

# 3D Cluster Generation
# N is the nb of fruits in the cluster (in one view)
# R is the fruit diameter
# S is the cluster size/diameter

class fruit:
    fruit_ID = None
    fruit_primitive = None
    object_type = None

class Clusters_Gen(fruit):

    def __init__(self,):
        self.tabletop_in_world = np.array([10, 2.5, 1])
        self.N = [10]
        self.R = 0.03
        self.S = 0.15
        self.cluster_count = 0
        self.scene = []
        self.Pi = np.pi
        self.color = [color.red, color.green]

    def position(self, center):
        u = np.random.random_sample()
        v = np.random.random_sample()
        w = np.random.random_sample()
        ## 2D
        r = (self.S)/2 * sqrt(w)
        #theta = 2 * self.Pi * v
        #x = r * cos(theta)
        #y = r * sin(theta)
        ## 3D
        lamda = np.arcos(2*u-1)-(Pi/2)
        phi = 2*Pi*v
        x =  center[0] + r * np.cos(lamda)*np.cos(phi)
        y =  center[1] + r * np.cos(lamda)*np.sin(phi)
        z =  center[2] + r * np.sin(lamda)
        location = vector(x, y, z)

        return location

    def orientation(self, center):
        u = np.random.random_sample()
        v = np.random.random_sample()
        w = np.random.random_sample()

        lamda = np.arcos(2*u-1)-(Pi/2)
        phi = 2*Pi*v
        x_unit =  center[0] + 1 * np.cos(lamda)*np.cos(phi)
        y_unit =  center[1] + 1 * np.cos(lamda)*np.sin(phi)
        z_unit =  center[2] + 1 * np.sin(lamda)
        rotation = vector(x_unit, y_unit, z_unit)

        return rotation


    def color(self,):

        return random.choice(self.color)


	def gen_cluster(self, kwargs, nb):
		shape_type = kwargs['shape_type']
        center_location = kwargs['center_location']
        #rotation = kwargs['rotation']
        size = (self.R)/2
        group_name = kwargs.get('name')
        group_ID = kwargs.get('ID')
        elements = fruit()
        for fruit_ID in range(nb):
            location = self.position(center_location)
            rotation = self.orientation(center_location)
            fID = '{}-{}%s'.format(group_ID, fruit_ID)
            fcol = self.color()

            #if shape_type == rigid_body.SHAPE_SPHERE:
            if shape_type == 0:
            	fruit_primitve = sphere(pos=location, radius=size, color=fcol)
                object_type = 'SPHERE'
            elif shape_type == 1:
            	fruit_primitve = box(pos=location, size = vector(size,size,size), color=fcol)
            	object_type = 'BOX'
            else:
            	raise ValueError('Unknown shape type: %s'%(str(shape_type)))
            
            elements.fruit_primitve.append(fruit_primitve)
            elements.fruit_ID.append(fID)
            elements.object_type.append(object_type)
        self.scene.append(elements)
        cluster_orig = elements
        return cluster_orig


    # The following function should be replaced by a permute_color function
	# def permute_dataset(self, cluster_orig, NP=4000):  
 #        for pm in range(NP):
 #            Pfruit_primitve = random.shuffle(cluster_orig.fruit_primitve)
 #            PfID = random.shuffle(cluster_orig.fruit_ID)
 #            Pelements = fruit()
 #            Pelements.fruit_primitve.append(Pfruit_primitve)
 #            Pelements.fruit_ID.append(PfID)
 #            Pelements.object_type.append(cluster_orig.object_type)
 #            self.scene.append(Pelements)

    def regenerate_dataset(self, kwargs, NP=4000, N= 10):  
        for pm in range(NP):
            new_cluster = self.gen_cluster(kwargs, N)


    def print_cluster(self, cluster):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for sphere in cluster:
            ax.scatter(sphere[0],sphere[1],sphere[2],s = 100, c='r', marker='o')
        ax.set_xlabel('Xc')
        ax.set_ylabel('Yc')
        ax.set_zlabel('Zc')
        ax.set_title('Random Cluster Generation')
        plt.show()



				
if __name__ == '__main__':
    # here we should call data_gen for each obj_gen element
    object_init = Clusters_Gen()
    centerX = object_init.tabletop_in_world[0]
    centerY = object_init.tabletop_in_world[1] - 0.05
    centerZ = object_init.tabletop_in_world[2] - 0.1
    center = np.array([centerX, centerY, centerZ])

    for nb in object_init.N:
        kwargs = {'shape_type': 0, 'center_location': center, 'name':'straw', 'ID':'{}%s'.format(nb)}
        cluster_orig = object_init.gen_cluster(kwargs, nb)

        for perm in range(0, object_init.NP):
            permutations = object_init.regenerate_dataset(kwargs, NP=100, N= nb)









#### visual library needs python3 , so instead of running from a virtual environment, we can use the following to generate a sphere:

    # def sphere(self, ax, radius, centre):
    #     u = np.linspace(0, 2 * np.pi, 13)  # start , goal , num of pts
    #     v = np.linspace(0, np.pi, 7)
    #     x = centre[0] + radius * np.outer(np.cos(u), np.sin(v))
    #     y = centre[1] + radius * np.outer(np.sin(u), np.sin(v))
    #     z = centre[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    #     xdata = scipy.ndimage.zoom(x, 3) # 3 dim image
    #     ydata = scipy.ndimage.zoom(y, 3)
    #     zdata = scipy.ndimage.zoom(z, 3)
    #     ax.plot_surface(xdata, ydata, zdata, rstride=3, cstride=3, color='w', shade=0) # rstride: array row stride (step size), default = 1







