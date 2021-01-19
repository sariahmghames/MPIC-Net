import numpy as np
import os
import numpy as np
from PIL import Image
import os.path
import random
import math
#import additions.rigid_body
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
 
import logging
import time

# 3D Cluster Generation
# N is the nb of fruits in the cluster (in one view)
# R is the fruit diameter
# S is the cluster size/diameter

class fruit:
    fruit_ID = 0
    fruit_primitive = []
    object_type = 'sphere'

class Clusters_Gen(fruit):

    def __init__(self,):
        self.tabletop_in_world = np.array([9.15, -2.5, 1.19])
        self.N = np.array([5, 10, 15, 20])
        self.R = 0.03
        self.S = 0.1
        self.cluster_count = 0
        self.scene = []
        self.Pi = np.pi
        self.color = ['red', 'green']
        self.NP = 10
	self.loop = 0

    def position(self, center):
        u = np.random.random_sample()
        v = np.random.random_sample()
        w = np.random.random_sample()
        ## 2D
        r = (self.S)/2 * np.sqrt(w)
        #theta = 2 * self.Pi * v
        #x = r * cos(theta)
        #y = r * sin(theta)
        ## 3D
        lamda = np.arccos(2*u-1)-(self.Pi/2)
        phi = 2*self.Pi*v
        x =  center[0] + r * np.cos(lamda)*np.cos(phi)
        y =  center[1] + r * np.cos(lamda)*np.sin(phi)
        z =  center[2] + r * np.sin(lamda)
        location = [x, y, z]

        return x, y, z

    def orientation(self, center):
        u = np.random.random_sample()
        v = np.random.random_sample()
        w = np.random.random_sample()

        lamda = np.arccos(2*u-1)-(self.Pi/2)
        phi = 2*(self.Pi/2)*v
        x_unit =  center[0] + 1 * np.cos(lamda)*np.cos(phi)
        y_unit =  center[1] + 1 * np.cos(lamda)*np.sin(phi)
        z_unit =  center[2] + 1 * np.sin(lamda)
        rotation = [x_unit, y_unit, z_unit]

        return rotation


    def orient_sim(self,): # limited to 1D orientation
	theta = np.linspace(-self.Pi/4, self.Pi/4, num=10, endpoint = True)

        return random.choice(theta)


    def ripening(self,):

        return random.choice(self.color)

    def gen_cluster(self, center, nb):
        center_location = center
        size = (self.R)/2
        elements = fruit()
        for fruit_ID in range(nb):
            locX, locY, locZ = self.position(center_location)
            rotation = self.orient_sim()
            fcol = self.ripening()
            
            elements.fruit_primitive.append([locX, locY, locZ, rotation, fcol])
        self.scene.append(elements)
        cluster_orig = elements
        return cluster_orig


    def regenerate_dataset(self, center, NP=4000, N= 10):  
        for pm in range(NP):
            new_cluster = self.gen_cluster(center, N)

				
if __name__ == '__main__':
    # here we should call data_gen for each obj_gen element
    object_init = Clusters_Gen()

    for nb in object_init.N:

   	centerX = object_init.tabletop_in_world[0] + object_init.loop
    	centerY = object_init.tabletop_in_world[1] 
    	centerZ = object_init.tabletop_in_world[2] 
    	center = np.array([centerX, centerY, centerZ])
        cluster_orig = object_init.gen_cluster(center, nb)
        print('cluster_orig=', cluster_orig.fruit_primitive)
        permutations = object_init.regenerate_dataset(center, NP=object_init.NP, N= nb)

        #print('scene size=', len(object_init.scene))
        #print('scene 1=', object_init.scene[1].fruit_primitive)

	with open('cluster{}file.txt'.format(nb), 'w') as filehandle:
		filehandle.write('This is a permutation of a cluster of 5 elements')
    		for listitem in object_init.scene:
        		filehandle.write('%s\n' % listitem.fruit_primitive)

	object_init.loop += 1








