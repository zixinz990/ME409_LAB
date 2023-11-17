import struct
import math

import numpy as np

def usr(robot):
    robot.delay(10)
    
	# set color and virtual radius
    R = 0
    if (robot.assigned_id == 0):
        robot.set_led(255, 0, 0)
        R = 0.1
    elif (robot.assigned_id == 1):
        robot.set_led(0, 255, 0)
        R = 0.25
    elif (robot.assigned_id == 2):
        robot.set_led(0, 0, 255)
        R = 0.4
    
    state = 0
    v_gravity = np.array([0.0, 0.0])
    v_repulsion = np.array([0.0, 0.0])
    v_random = np.array([0.0, 0.0])
    v_target = np.array([0.0, 0.0])
    
    while True:
        pose = robot.get_pose()
        pos = np.array([pose[0], pose[1]])
        robot.send_msg(struct.pack("ffi", pose[0], pose[1], robot.id))

        if (state == 0):        
			# gravity
            v_gravity = -pos / math.sqrt(pos[0]*pos[0] + pos[1]*pos[1])
			
			# repulsion

			# random motion
            direction_random = np.random.uniform(-3.14, 3.14, 1)
            v_random = np.array([math.cos(direction_random), math.sin(direction_random)])

            # sum
            v_target = v_gravity + 0.6 * v_random
            state = 1
        elif (state == 1):
		    # turn to the target direction
            angle_target = math.atan2(v_target[1], v_target[0])
            angle_err = angle_target - pose[2]
            if abs(angle_err) > 0.2:
                robot.set_vel(1, -1)
                state = 1
            else:
                robot.set_vel(0, 0)
                state = 2
        elif (state == 2):
            # move towards
            robot.set_vel(50, 50)
            state = 0
