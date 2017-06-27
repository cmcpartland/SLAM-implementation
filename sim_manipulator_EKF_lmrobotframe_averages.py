# Connor McPartland
# SLAM Implementation for a P-P Manipulator with Laser Scanner
# 5/16/2017


import numpy as np
import time
import matplotlib.pyplot as plt

try:
	import vrep
except:
	print '--------------------------------------------------------------'
	print '"vrep.py" could not be imported. This means very probably that'
	print 'either "vrep.py" or the remoteApi library could not be found.'
	print 'Make sure both are in the same folder as this file,'
	print 'or appropriately adjust the file "vrep.py"'
	print '--------------------------------------------------------------'
	print ''



print 'Program started'

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP

r = .35

plt.ion()
plt.xticks(np.arange(-.5,3,.5))
plt.yticks(np.arange(-3,3,.5))

class landmark:
	def __init__(self,x_init,y_init,landmark_id):
		# x_init and y_init are initial pts of new landmark
		self.x_range = np.array([x_init-r,x_init+r])
		self.y_range = np.array([y_init-r,y_init+r])
		self.center = np.array([x_init,y_init])
		self.landmark_id = landmark_id
		# if landmark is not seen in 140 scanned pts, kill it off
		self.landmark_lifetime = 2
		self.observed = False
	
	# Determine if given point is within the x,y range (a square) of this landmark
	def within_range(self,x_i,y_i):
		if x_i > self.x_range[0] and x_i < self.x_range[1]:
			#within x_range
			if y_i > self.y_range[0] and y_i < self.y_range[1]:
				return True
		else:
			return False
			
	
	def update_bounds(self, x_i, y_i):
		# pseudo: if x_i is close to lower range, update the lower range
		#		  else, if x_i is close to upper range, update the upper range
		range_updated = False
		if abs(x_i - self.x_range[0]) < r:
			self.x_range[0] = x_i - r
			range_updated = True
		elif abs(x_i - self.x_range[1]) < r:
			self.x_range[1] = x_i + r
			range_updated = True
			
		
		if abs(y_i - self.y_range[0]) < r:
			self.y_range[0] = y_i - r
			range_updated = True
		elif abs(y_i - self.y_range[1]) < r:
			self.y_range[1] = y_i + r
			range_updated = True
		
		if range_updated:
			# Update center pt to be midpoint of x and y ranges
			self.center = np.array([(self.x_range[1]-self.x_range[0])/2. + self.x_range[0], (self.y_range[1]-self.y_range[0])/2. + self.y_range[0]])
		
	def landmark_observed(self):
		# max out landmark_lifetime to be 100
		if self.landmark_lifetime <= 98:
			self.landmark_lifetime += 2 # add 2 because each lifetime is subtracted by 1 at first to get rid of outliers
		self.landmark = True
						

# The known landmark database in world frame			
lm_db = []

x_robots = []
y_robots = []
known_x = []
known_y = []

if clientID!=-1:
	print 'Connected to remote API server'

	# Now try to retrieve data in a blocking fashion (i.e. a service call):
	res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
	if res==vrep.simx_return_ok:
		print 'Number of objects in the scene: ',len(objs)
	else:
		print 'Remote API function call returned with error code: ',res
	
	pos = []
	
	[res,handle_cuboid] = vrep.simxGetObjectHandle(clientID,'Cuboid',vrep.simx_opmode_oneshot_wait)	 # get handle for cuboid
	[res,handle_revolute_joint] = vrep.simxGetObjectHandle(clientID,'Revolute_joint',vrep.simx_opmode_oneshot_wait)	 # get handle for revolute joint
	[res,handle_prismatic_joint] = vrep.simxGetObjectHandle(clientID,'Prismatic_joint',vrep.simx_opmode_oneshot_wait) # get handle for prismatic joint
	# vrep.simxSetJointTargetVelocity(clientID,handle_prismatic_joint,0.2,vrep.simx_opmode_oneshot_wait) # set desired velocity for prismatic joint
	[res,handle_prismatic_joint1] = vrep.simxGetObjectHandle(clientID,'Prismatic_joint1',vrep.simx_opmode_oneshot_wait) # get handle for second prismatic joint
	[res,laser_signal] = vrep.simxGetStringSignal(clientID,'laser_data',vrep.simx_opmode_streaming) # initialize getting of laser data in streaming mode
	
	initializing_scan = True
	
	q1_prev = 0
	t_prev = 0
	
	x_robot = 0.0
	y_robot = 0.0
	while(vrep.simxGetConnectionId(clientID) != -1):  # while v-rep connection is still active
		t = vrep.simxGetLastCmdTime(clientID) / 1000.0	# get current simulation time
		if (t > 20):
			break  # stop after t = 20 seconds
		[res,q1] = vrep.simxGetJointPosition(clientID,handle_prismatic_joint,vrep.simx_opmode_oneshot_wait) # get prismatic joint position (q1)
		q1_noisy = q1 + np.random.normal(0,.01) # add noise to q1 value to simulate odometry
		
		[res,q2] = vrep.simxGetJointPosition(clientID,handle_revolute_joint,vrep.simx_opmode_oneshot_wait) # get revolute joint position (q2)
		[res,q3] = vrep.simxGetJointPosition(clientID,handle_prismatic_joint1,vrep.simx_opmode_oneshot_wait)
		[res,laser_signal] = vrep.simxGetStringSignal(clientID,'laser_data',vrep.simx_opmode_buffer); # get laser data
		
		# Define the homoegenous transformation from frame 3 to frame 0 using the noisy state variables
		H = np.array([[np.cos(q2), -np.sin(q2), 0, x_robot],[np.sin(q2), np.cos(q2), y_robot, 0],[0, 0, 1, 0.69],[0, 0, 0, 1]])
		
		
		# delta values are for the control inputs! they are what the robot 'should' have moved
		delta_x = q1-q1_prev
		delta_y = 0
		delta_t = t-t_prev
		
		
		if (res == vrep.simx_return_ok):
			laser_data = np.array(vrep.simxUnpackFloats(laser_signal)) # unpack into floating-point numbers
			n_laser_points = len(laser_data)/3	# each laser data point is an (x,y,z) -- hence, number of points is length(laser_data)/3
			laser_points = np.reshape(laser_data,(n_laser_points,3)) # get points as n_laser_points x 3 array, so each row is a point like [x y z]
			xy_laserpoints = np.delete(laser_points,2,1)

			# Calculate P_0 for each laser point using P_3 and homogenous
			# transformation H. Add this to an array of points, pos
			xy_conv = []
			xy_robotframe = []
			for i in range(0,n_laser_points):
				p_3 = laser_points[i]
				P_3 = np.append(p_3,1)
				P_0 = np.dot(H,P_3)
				
				# extract only the x,y coordinates from each P_0
				
				pos.append(P_0[0:2].tolist())
				xy_robotframe.append(P_3[0:2].tolist())
				
			xy_robotframelp = np.array(xy_robotframe).reshape((n_laser_points,2))
			
			x_sorted_rf_indices = np.argsort(xy_robotframelp[:,0])
			x_sorted_rf = np.array([xy_robotframelp[:,0][x_sorted_rf_indices]]).reshape(n_laser_points,1)
			y_sorted_rf = np.array([xy_robotframelp[:,1][x_sorted_rf_indices]]).reshape(n_laser_points,1)
			

			x_0_rf = x_sorted_rf[0]
			y_0_rf = y_sorted_rf[0]
			x_sorted_rf_indices = np.delete(x_sorted_rf_indices,0)

			lm_observed_rf = []

			# CREATE THE OBSERVED LANDMARK SET
			# Create the first landmark using the first observed point
			lm_observed_rf.append(landmark(x_0_rf,y_0_rf,0))

			# Build observed landmarks in reference frame
			for indx in x_sorted_rf_indices:
				x_i = x_sorted_rf[indx]
				y_i = y_sorted_rf[indx]
				
				part_of_existing_landmark = False
				
				for lm_indx in range(len(lm_observed_rf)):
					if lm_observed_rf[lm_indx].within_range(x_i,y_i):
						lm_observed_rf[lm_indx].update_bounds(x_i,y_i)
						# print 'bounds updated: ', lm_observed[lm_indx].center
						part_of_existing_landmark = True
				
				if not part_of_existing_landmark:
					lm_observed_rf.append(landmark(x_i,y_i,len(lm_observed_rf)+1))
			

			
			
			
			landmarks_associated = [] # the set of landmarks that have been associated to newly observed ones. We only want to use the associated (re-observed) landmarks to improve robot pos estimate
			if not initializing_scan:
				# At this point, the observed landmark set is created. Now, pass this throught the validation gate to update the known database
				# LANDMARK VALIDATION GATE
				# The center of each lm observed in rf will be transformed to world frame for validation
				threshold = .35 # slightly larger than average landmark size
				for lm_o in lm_observed_rf:
					lm_associated = False
					lm_o_center_wf = H.dot([[lm_o.center[0]],[lm_o.center[1]],[0.69],[1]])[0:2]
					# print "observed center, wf: ",lm_o_center_wf
					
					for lm in lm_db:
						# print 'lm.center: ',lm.center
						
						distance = np.linalg.norm(lm_o_center_wf - lm.center)
						# print 'distance: ',distance
						if distance < threshold: # then the two landmarks are associated
							lm_db[lm_db.index(lm)].landmark_observed() # mark this landmark as observed
							lm_associated = True
							lm_observed_rf[lm_observed_rf.index(lm_o)].landmark_id = lm.landmark_id # associate these two landmarks by giving them the same id
							# only associate with landmarks observed more than 10 times
							if lm.landmark_lifetime > 10:
								landmarks_associated.append([lm,lm_o])
							break # break because the landmark was observed, move to the next one
							
					if not lm_associated: #if the landmark was not associated to any of those in the known database, add it to the database
						print 'ADDING NEW LANDMARK'
						# Make sure the center of the newly observed object is in the world frame (also adjust boundaries), since lm_db is all written in world frame
						lm_center_offset = lm_o_center_wf - lm_o.center
						# print lm_o_center_wf
						lm_temp = lm_o
						lm_temp.center = np.array(lm_o_center_wf)
						lm_temp.x_range = np.array([lm_temp.x_range[0] + lm_center_offset[0],lm_temp.x_range[1] + lm_center_offset[0]])
						lm_temp.y_range = np.array([lm_temp.y_range[0] + lm_center_offset[1],lm_temp.y_range[1] + lm_center_offset[1]])
						lm_temp.landmark_id = lm_db[-1].landmark_id+1
						lm_db.append(lm_temp)			
				
			
				# Step 2: Now that landmarks are observed and associated, update the state based on the associated landmark positions
				# Use the observed lm centers in robot frame and compare them to known lm centers in world frame. The displacement will be equal to the robot position
				robot_displacements = []
				weight_sum = 0
				for lm_pair in landmarks_associated:
					lm_fromdb = lm_pair[0]
					lm_inrf = lm_pair[1]
					
					weight = lm_fromdb.landmark_lifetime

					robot_displacements.append((lm_fromdb.center - lm_inrf.center)*weight)
					# robot_displacements.append((lm_fromdb.center - lm_inrf.center)) # UNCOMMENT FOR POOR TUNING EXAMPLE
					weight_sum += weight
					# print lm_fromdb.center - lm_inrf.center
					
					
				avg_displacement = np.sum(np.array(robot_displacements),axis=0)/(weight_sum)
				# avg_displacement = np.sum(np.array(robot_displacements),axis=0)/len(robot_displacements) # UNCOMMENT FOR POOR TUNING EXAMPLE
				
				# print 'Average displacement: ',avg_displacement
				x_robot = avg_displacement[0]
				y_robot = avg_displacement[1]
				
			# Because the initial landmark db is empty, first set the landmark db equal to the first observed landmark set
			if initializing_scan:
				lm_db = lm_observed_rf
				for lm in lm_db:
					lm.landmark_lifetime = 12 # Give initial landmarks high confidence since robot's starting position is well known - it is OK to associate with these in the next scan
				initializing_scan = False
				print 'INITIALIZING SCAN, ADDING %d LANDMARKS'%(len(lm_db))
				
			lifetimes = []
			# After landmarks are updated, decrease timer by 1. If landmark_timer for lm was already at 0, remove it.
			for lm in lm_db:
				lm.landmark_lifetime -= 1
				if lm.landmark_lifetime <= 0:
					lm_db.pop(lm_db.index(lm))
					print 'LANDMARK REMOVED'
				lifetimes.append(lm.landmark_lifetime)
			
			# print 'lifetimes: ', lifetimes
			
				

		x_db = []
		y_db = []
		known_x.append(q1)
		known_y.append(q3-0.29)
		for lm in lm_db:
			x_db.append(lm.center[0])
			y_db.append(lm.center[1])
		x_robots.append(x_robot)
		y_robots.append(y_robot)
		plt.grid()
		plt.scatter(x_db,y_db, color='red', label='Landmarks')
		plt.scatter(x_robots,y_robots, color='blue', s=10, label='Estimated robot position')
		plt.scatter(known_x,known_y,color='black',marker='x', label='Known robot position')
		plt.xticks(np.arange(-.5,5,.5))
		plt.yticks(np.arange(-3,5,.5))
		plt.legend(loc='upper left')
		plt.pause(0.001)
		plt.cla()

		q2_des = 0.1*t # desired rotation angle of revolute joint
		v = -2*(q2 - q2_des) # desired velocity of revolute joint (simple proportional controller)
		# vrep.simxSetJointTargetVelocity(clientID,handle_revolute_joint,v,vrep.simx_opmode_oneshot_wait)	 # set desired velocity for revolute joint
		
		q1_prev = q1
		t_prev = t
		# print 'x_robot: ',x_robot,' q1: ',q1
	vrep.simxFinish(clientID)
	
	
	pos_array = np.array(pos)
	# 2-d plot of environment
	# plt.scatter(pos_array[:,0],pos_array[:,1])
	# plt.show()
else:
	print 'Failed connecting to remote API server'

print 'Program ended'
