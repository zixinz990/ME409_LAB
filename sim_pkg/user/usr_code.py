import struct
import math

import numpy as np

def to_unit_vec(vec):
    magnitude = np.linalg.norm(vec)
    if magnitude == 0: return vec
    return vec / magnitude

def usr(robot):
    # initialize target velocities
    v_gravity = np.zeros(2)
    v_repulsion = np.zeros(2)
    v_random = np.zeros(2)
    v_target = np.zeros(2)

    # initialize parameters
    max_repulsion = 15.0 # maximum repulsion velocity
    k_repulsion = 10.0 # magnitude of repulsion velocity
    k_random = 0.9 # magnitude of random velocity
    k_turning = 1.0 # for turning control
    
    # state 0: calculate target velocity, only run 1 time in each control cycle
    # state 1: turn to the target direction, run multi times, must reach t1 = 15 s
    # state 2: move, run until the end of control cycle, must reach t2 = 17 s
    state = 0

    while True:
        # set color and virtual radius
        R = 0
        if (robot.assigned_id == 0):
            robot.set_led(150, 0, 0)
            R = 0.1
        elif (robot.assigned_id == 1):
            robot.set_led(0, 150, 0)
            R = 0.6
        elif (robot.assigned_id == 2):
            robot.set_led(0, 0, 150)
            R = 1.2

        pose = robot.get_pose()
        position = np.array([pose[0], pose[1]])
        robot.send_msg(struct.pack("ffi", position[0], position[1], robot.id))

        if (state == 0):
            # gravity
            v_gravity = -to_unit_vec(position)

            # repulsion
            msgs = robot.recv_msg()
            id_count = np.zeros(150)
            if len(msgs) > 0:
                for i in range(len(msgs)):
                    recv_msg = struct.unpack("ffi", msgs[i])
                    if id_count[recv_msg[2]] == 0: # if haven't use the msg from this robot
                        id_count[recv_msg[2]] = 1
                        pos_i = np.array([recv_msg[0], recv_msg[1]])
                        vec_i = position - pos_i
                        d_i = np.linalg.norm(vec_i)
                        if (d_i < 2 * R):
                            repulsion_mag_i = k_repulsion * (2 * R - d_i)
                            v_repulsion = v_repulsion + repulsion_mag_i * to_unit_vec(vec_i)
            if np.linalg.norm(v_repulsion) > max_repulsion:
                v_repulsion = max_repulsion * to_unit_vec(v_repulsion)

            # random motion
            direction_random = np.random.uniform(-math.pi, math.pi, 1)
            v_random = np.array([math.cos(direction_random[0]), math.sin(direction_random[0])])

            # sum
            v_target = v_gravity + v_repulsion + k_random * v_random
            # v_target = v_gravity
            state = 1
        elif (state == 1):
            # turn to the target direction
            angle_target = math.atan2(v_target[1], v_target[0])
            angle_err = angle_target - pose[2]
            if abs(angle_err) > 0.22:
                control_mag = 0
                if abs(angle_err) > math.pi:
                    control_mag = k_turning * (2 * math.pi - abs(angle_err))
                else:
                    control_mag = k_turning * abs(angle_err)

                if angle_err > 0: # current angle < target angle
                    if abs(angle_err) < math.pi:
                        robot.set_vel(-control_mag, control_mag)
                    else:
                        robot.set_vel(control_mag, -control_mag)
                else: # current angle > target angle
                    if abs(angle_err) < math.pi:
                        robot.set_vel(control_mag, -control_mag)
                    else:
                        robot.set_vel(-control_mag, control_mag)
                state = 1
            else:
                state = 2
        elif (state == 2):
            # move towards
            robot.set_vel(5, 5)
            v_gravity = np.zeros(2)
            v_repulsion = np.zeros(2)
            v_random = np.zeros(2)
            v_target = np.zeros(2)
            state = 0
        
        robot.delay(5)