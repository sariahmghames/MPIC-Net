import torch
import torch.utils.data as data
import os
import pickle
import numpy as np
import nltk
from PIL import Image
import os.path
import random
from torch.autograd import Variable
import torch.nn as nn
import math

#N=number of environments; NP=Number of Paths
def load_dataset(N=7,NP=70):

	############ Continue here #######
				
	dataset=[]
	targets=[]
	for i in range(0,N):
		for j in range(0,NP):
			
						
					targets.append()
					dataset.append()
			
	data=zip(dataset,targets)
	random.shuffle(data)	
	dataset,targets=zip(*data)
	return 	np.asarray(dataset),np.asarray(targets) 

	


