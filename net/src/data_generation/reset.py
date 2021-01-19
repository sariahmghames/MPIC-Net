import numpy as np
import os
from PIL import Image
import os.path
import random
import math
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetLinkState, GetLinkState, SetLinkProperties
from gazebo_msgs.srv import SetVisualProperties
from gazebo_msgs.srv import GetModelProperties
from gazebo_msgs.srv import DeleteModel
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import smach_ros
import smach
#from gazebo_msgs.srv import SetVisualProperties
#from gazebo_msgs.srv import SetVisualPropertiesRequest
 
import logging
import time



class Clusters_Gen():

    def __init__(self,):
    	rospy.init_node('reset_pose')
        self.tabletop_in_world = np.array([10, 2.5, 1])
        self.N = [10]
        self.R = 0.03
        self.S = 0.15
        self.cluster_count = 0
        self.scene = []
        self.Pi = np.pi
        self.StemOrient = np.linspace(-self.Pi/4, self.Pi/4, 10)
        self.color = ['Gazebo/Red', 'Gazebo/green']
        self.episodes = []
        self.object_to_remove = ['panda_arm', 'ground_plane', 'strut', 'pole_21']
        self.robot_name = 'panda_arm'
        self.set_property = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_link_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.del_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        self.repetitions = 10
        self.demos = 0



    def getRandomModelPosition(self, center):
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

    def getRandomModelOrientation(self, center):
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


    def getRandomModelColor(self,link_name):
    	update_material = SetLinkProperties()
    	update_material.link_name = link_name
    	update_material.material = random.choice(self.color)

        return update_material


	def set_color_proxy(self,):

        return random.choice(self.color)

    def set_stem_orient(self,):

        return random.choice(self.StemOrient)


    def Reset_Robot(self,robot_name):
    	init_pose = SetModelState()
		init_pose.name = robot_name
        init_pose.pose = [0, 0, 0, 0, 0, 0, 0] # to modify

        return init_pose

    def check_pose(self, updated_objects_list, tmp_pose):
        if tmp_pose != objects_list.any()
            return True
        else:
            return False


	def gen_cluster(self, kwargs, nb):
		shape_type = kwargs['shape_type']
        center_location = kwargs['center_location']
        #rotation = kwargs['rotation']
        size = (self.R)/2
        group_name = kwargs.get('name')
        group_ID = kwargs.get('ID')
        elements = fruit()
        for fruit_ID in range(nb):
            location = self.getRandomModelPosition(center_location)
            rotation = self.getRandomModelOrientation(center_location)
            fID = '{}-{}%s'.format(group_ID, fruit_ID)
            fcol = self.getRandomModelColor()

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



	def reset(self, world_xyz, object_to_remove, episode=1, set = 1):
	        """
	        Clean the environment and reset the position of all the models
	        :param target_world: the area where the model cannot be generated --> not into account
	        :param world_xy: the area where the model can be generated
	        :param robot_xy: the area where the robot can ge generated --> assume manipulator fix 
	        :param object_to_remove: the object which do not need to be regenerated
	        """
	        # print("Waiting for /gazebo/set_model_state service")
	        if set == 1:
                for rep in range(0, self.repetitions):
                    self.reset_color(self, world_xyz, object_to_remove, episode)
                    rospy.loginfo('you can initialte the script of teleoperation and data logging')
                    rospy.sleep(120.0) # check with other methods
                    self.demos +=1
                    ## Reset robot pose at every demo/episode
                    print("Resetting robot position")

                    pose_acceptable = False
                    while pose_acceptable is False:
                        objects_list = None
                        while objects_list is None:
                            #  Wait the callback updates the object list
                            objects_list = rospy.wait_for_message("/gazebo/model_states", ModelStates)
                        initial_RobotPose = self.ResetRobot(model_name=self.robot_name)
                    try:
                        print("Assign same init pose to robot!")
                        resp = self.set_state(initial_RobotPose)
                    except (rospy.ServiceException) as e:
                        rospy.logerr("/gazebo/set_model_state service call failed")
                self.episodes.append(episode)

	        elif set ==2:
                for rep in range(0, self.repetitions):
                    self.reset_fruit_pose(self, world_xyz, object_to_remove, episode)
                    rospy.loginfo('you can initialte the script of teleoperation and data logging')
                    rospy.sleep(120.0) # check with other methods
                    self.demos +=1
                    ## Reset robot pose at every demo/episode
                    print("Resetting robot position")

                    pose_acceptable = False
                    while pose_acceptable is False:
                        objects_list = None
                        while objects_list is None:
                            #  Wait the callback updates the object list
                            objects_list = rospy.wait_for_message("/gazebo/model_states", ModelStates)
                        initial_RobotPose = self.ResetRobot(model_name=self.robot_name)
                    try:
                        print("Assign same init pose to robot!")
                        resp = self.set_state(initial_RobotPose)
                    except (rospy.ServiceException) as e:
                        rospy.logerr("/gazebo/set_model_state service call failed")
                self.episodes.append(episode)
	        elif set ==3:
                for rep in range(0, self.repetitions):
                    self.reset_stem_inclin(self, world_xyz, object_to_remove, episode)
                    rospy.loginfo('you can initialte the script of teleoperation and data logging')
                    rospy.sleep(120.0) # check with other methods
                    self.demos +=1
                    ## Reset robot pose at every demo/episode
                    print("Resetting robot position")

                    pose_acceptable = False
                    while pose_acceptable is False:
                        objects_list = None
                        while objects_list is None:
                            #  Wait the callback updates the object list
                            objects_list = rospy.wait_for_message("/gazebo/model_states", ModelStates)
                        initial_RobotPose = self.ResetRobot(model_name=self.robot_name)
                    try:
                        print("Assign same init pose to robot!")
                        resp = self.set_state(initial_RobotPose)
                    except (rospy.ServiceException) as e:
                        rospy.logerr("/gazebo/set_model_state service call failed")
                self.episodes.append(episode)
	        else: 
	        	rospy.loginfo('no valid cluster regeneration scenario requested')


    def reset_color(self, world_xyz, object_to_remove, episode=1):

        resp = rospy.wait_for_service('/gazebo/delete_model')
        objects_list = None
        while objects_list is None:
            #  Wait the callback updates the object list
            # print("Waiting for object list")
            objects_list = rospy.wait_for_message(
                "/gazebo/model_states", ModelStates) # Publish complete model states
        clean_object_list = np.setdiff1d(np.array(objects_list.name), object_to_remove)
        print("First, clean the environment removing the objects")
        for i in range(0, len(clean_objects_list.name)):
            #updated_objects_list = rospy.wait_for_message("/gazebo/model_states", ModelStates) # --> ?? why 

            try:
                resp = self.del_model(clean_object_list.name[i])
            except (rospy.ServiceException) as e:
                rospy.logerr("/gazebo/delete_model service call failed")
        # Remove the ground_plane and the robot from the list of objects
        #clean_object_list = np.setdiff1d(np.array(objects_list.name), object_to_remove)
        # print("Cleaned object list: ", clean_object_list)
        print("Second, re-generate models at same initial pose...")
        for i in range(0, len(clean_object_list)):
            try:
                resp = self.set_state(clean_object_list[i])
            except (rospy.ServiceException) as e:
                rospy.logerr("/gazebo/set_model_state service call failed")

        rospy.sleep(1.0)
        print("Third, generate random colors...")
        mask = np.ones(shape=np.shape(clean_object_list))
        for i in range(0, len(clean_object_list)):
            if mask[i] == 1:

	            update_material = self.getRandomModelColor(link_name=objects_list.name[i])
	            try:
	                resp = self.set_property(update_material)
	            except (rospy.ServiceException) as e:
	                rospy.logerr("/gazebo/set_link_properties service call failed")
            else:
                pass
        rospy.sleep(2.0)



	    

	def reset_fruit_pose(self, world_xyz, object_to_remove, episode=1):

        resp = rospy.wait_for_service('/gazebo/delete_model')
        objects_list = None
        while objects_list is None:
            #  Wait the callback updates the object list
            # print("Waiting for object list")
            objects_list = rospy.wait_for_message(
                "/gazebo/model_states", ModelStates) # Publish complete model states
        clean_object_list = np.setdiff1d(np.array(objects_list.name), object_to_remove)
        print("First, clean the environment removing the objects")
        for i in range(0, len(clean_objects_list.name)):
            #updated_objects_list = rospy.wait_for_message("/gazebo/model_states", ModelStates) # --> ?? why 

            try:
                resp = self.del_model(clean_object_list.name[i])
            except (rospy.ServiceException) as e:
                rospy.logerr("/gazebo/delete_model service call failed")

        print("Second, generate random position...")
        updated_objects_list = []
        mask = np.ones(shape=np.shape(clean_object_list))
        for i in range(0, len(clean_object_list)):
            if mask[i] == 1:
                pose_acceptable = False
                if clean_object_list[i] != self.robot_name:
                    # print("[2]Object selected is: ", clean_object_list[i])
                    while pose_acceptable is False:
                        # print("[{}] generating pose".format(objects_list.name[i]))
                        # objects_list.name[i] = 'obstacle'
                        tmp_pose.pose = self.getRandomModelPosition(center=world_xyz)
                        tmp_pose.model_name = clean_object_list.name[i]

                        pose_acceptable = self.checkPose(updated_objects_list, tmp_pose.pose)
                        updated_objects_list.append(tmp_pose.pose)
                        else:
                            break
                    try:
                        resp = self.set_link_state(tmp_pose) # make model composed of straw + stem or update stems pose (TO DO)
                    except (rospy.ServiceException) as e:
                        rospy.logerr(
                            "/gazebo/set_link_state service call failed")
            else:
                pass
        rospy.sleep(2.0)



    def reset_stem_inclin(self, world_xyz, object_to_remove, episode=1):

        resp = rospy.wait_for_service('/gazebo/delete_model')
        objects_list = None
        while objects_list is None:
            #  Wait the callback updates the object list
            # print("Waiting for object list")
            objects_list = rospy.wait_for_message(
                "/gazebo/model_states", ModelStates) # Publish complete model states
        clean_object_list = np.setdiff1d(np.array(objects_list.name), object_to_remove)
        print("First, clean the environment removing the objects")
        for i in range(0, len(clean_objects_list.name)):
            #updated_objects_list = rospy.wait_for_message("/gazebo/model_states", ModelStates) # --> ?? why 

            try:
                resp = self.del_model(clean_object_list.name[i])
            except (rospy.ServiceException) as e:
                rospy.logerr("/gazebo/delete_model service call failed")

        print("Second, re-generate models at same initial pose...")
        for i in range(0, len(clean_object_list)):
            if straw in clean_object_list[i].name:
                try:
                    resp = self.set_link_state(clean_object_list[i])
                except (rospy.ServiceException) as e:
                    rospy.logerr("/gazebo/set_link_state service call failed")

            rospy.sleep(1.0)

        print("Third, generate random stem inclination..")
        updated_objects_list = []
        mask = np.ones(shape=np.shape(clean_object_list))
        for i in range(0, len(clean_object_list)):
            if mask[i] == 1:
                pose_acceptable = False
                if stem in clean_object_list[i]:
                    # print("[2]Object selected is: ", clean_object_list[i])
                    while pose_acceptable is False:
                        # print("[{}] generating pose".format(objects_list.name[i]))
                        # objects_list.name[i] = 'obstacle'
                        tmp_pose.pose = self.set_stem_orient()
                        tmp_pose.model_name = clean_object_list.name[i]

                        pose_acceptable = self.checkPose(updated_objects_list, tmp_pose.pose)
                        updated_objects_list.append(tmp_pose.pose)
                        else:
                            break
                    try:
                        resp = self.set_link_state(tmp_pose) # make model composed of straw + stem or update stems pose (TO DO)
                    except (rospy.ServiceException) as e:
                        rospy.logerr(
                            "/gazebo/set_link_state service call failed")
            else:
                pass
        rospy.sleep(2.0)
    

	    # (112 lines)
	    #Collapse
    

if __name__ == '__main__':
	## Set I: regenerate only straw color --> couldn't find the service!!!!!!!!!!!!!!!!!!!!!!!!!!! try setLinkProperties and check GetLinkProperties when run gazebo with world file .material or create custom service from those -  try permute same elements as a way to change 1 color ##
	# Episode1: regenerate 1st cluster type (elements nb = 5) 10 times 
	# Episode2: regenerate 2nd cluster type (elements nb = 10) 10 times
	# Episode3: regenerate 3rd cluster type (elements nb = 15) 10 times 
	# Episode4: regenerate 4th cluster type (elements nb = 20) 10 times 

	## Set II: regenerate only straw pose and update stems based on new straw pose  ##
	# Episode1: regenerate 1st cluster type (elements nb = 5) 10 times 
	# Episode2: regenerate 2nd cluster type (elements nb = 10) 10 times
	# Episode3: regenerate 3rd cluster type (elements nb = 15) 10 times 
	# Episode4: regenerate 4th cluster type (elements nb = 20) 10 times 

	## Set III: regenerate only stems position/inclination and update straw pose based on new stem position/inclination ##
	# Episode1: regenerate 1st cluster type (elements nb = 5) 10 times 
	# Episode2: regenerate 2nd cluster type (elements nb = 10) 10 times
	# Episode3: regenerate 3rd cluster type (elements nb = 15) 10 times 
	# Episode4: regenerate 4th cluster type (elements nb = 20) 10 times 

	## Set IV: regenerate the center of cluster (TO ADD @ later stage)
    try:
        cluster_init = Clusters_Gen()
        # each urdf corresponds to a different episode
        centerX = cluster_init.tabletop_in_world[0]
    	centerY = cluster_init.tabletop_in_world[1] - 0.05
    	centerZ = cluster_init.tabletop_in_world[2] - 0.1
    	center_xyz = np.array([centerX, centerY, centerZ])

        cluster_init.reset(center_xyz, self.object_to_remove, episode=1, set=1)
    except rospy.ROSInterruptException:
        pass