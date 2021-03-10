import pybullet as p
import time
import pybullet_data
import numpy as np
import math


## setup
useMaximalCoordinates = False
p.connect(p.GUI)
p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

pole = p.loadURDF("onelink_robot/onelink.urdf", [0, 0, 0], useMaximalCoordinates=useMaximalCoordinates)
p.resetBasePositionAndOrientation(pole, [0, 0, 0], [0, 0, 0, 1])

p.setJointMotorControl2(pole, 0, p.POSITION_CONTROL, targetPosition=0, force=0)

timeStepId = p.addUserDebugParameter("timeStep", 0.001, 0.1, 0.01)
useRealTimeSim = False
p.setRealTimeSimulation(useRealTimeSim)


numJoints = p.getNumJoints(pole)

desired_q = np.array([0]);
desired_qdot = np.array([0]);

prev_q = 0

kps = [10]
kds = [10]
maxF = np.array([10]);


while p.isConnected():
	timeStep = p.readUserDebugParameter(timeStepId)
	p.setTimeStep(timeStep)
	jointStates = p.getJointStates(pole,[0])
	q = np.array([jointStates[0][0]])
	qdot = np.array((q-prev_q )/timeStep)


###       CONTROL LOOP

	qError = desired_q - q;
	qdotError = desired_qdot - qdot;
	Kp = np.diagflat(kps)
	Kd = np.diagflat(kds)
	torques = Kp.dot(qError)+Kd.dot(qdotError)
	
	torques = np.clip(torques,-maxF,maxF)
	prev_q = q

###	


	p.setJointMotorControlArray(pole, [0], controlMode=p.TORQUE_CONTROL, forces=torques)
	
	if (not useRealTimeSim):
		p.stepSimulation()
		time.sleep(timeStep)
