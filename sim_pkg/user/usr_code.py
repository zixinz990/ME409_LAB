import struct


def usr(robot):
    desired_distance = .5  # will vary from 0.3-0.5
    p = (desired_distance + 0.04) / (desired_distance - 0.04) # v1/v2 based on desired distance and robot geometry
    v = 10  # (v1+v2)/2
    curr_r = 0.4 # this is the control input
    distance = 0.0  # distance between robots
    distance_last = 0.0  # distance between robots last iteration
    distance_last_inited = False  # whether distance_last has been initialized
    robot_0_pose = [0.0, 0.0]  # robot 0 pose

    Kp = 0.5  # proportional gain
    Kd = 10.0  # derivative gain

    while True:
        if (robot.id == 0):
            msgs = robot.recv_msg()

            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12])
                # blink led if message is received
                robot.set_led(100, 0, 0)
                robot.delay(10)
                robot.set_led(0, 0, 0)

            pose_t = robot.get_pose()

            if pose_t:  # check pose is valid before using
                # send pose x,y in message
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.id))
            
            # robot.set_vel(1, 1)

        if (robot.id == 1):
            # if we received a message, print out info in message
            recv_msg_from_0 = False
            msgs = robot.recv_msg()

            if len(msgs) > 0:
                pose_rxed = struct.unpack('ffi', msgs[0][:12])
                robot_0_pose = [pose_rxed[0], pose_rxed[1]]
                # blink led if message is received
                robot.set_led(0, 100, 0)
                robot.delay(10)
                robot.set_led(0, 0, 0)
                recv_msg_from_0 = True
            else:
                recv_msg_from_0 = False
            
            pose_t = robot.get_pose()

            if pose_t:  # check pose is valid before using
                # send pose x,y in message
                robot.send_msg(struct.pack('ffi', pose_t[0], pose_t[1], robot.id))

                if recv_msg_from_0:
                    distance = ((pose_t[0] - robot_0_pose[0]) ** 2 + (pose_t[1] - robot_0_pose[1]) ** 2) ** 0.5
                    if not distance_last_inited:  # if distance_last has not been initialized
                        distance_last = distance  # initialize distance_last
                        distance_last_inited = True
                        robot.set_vel(0, 0)
                    else:
                        # PD control
                        error = desired_distance - distance
                        change_in_distance = distance_last - distance
                        curr_r = desired_distance + Kp * error + Kd * change_in_distance
                        distance_last = distance
                        print('Distance between robots is ', distance)

                        p = (curr_r + 0.04) / (curr_r - 0.04) # update v1/v2
                        v2 = 2 * v / (p + 1)
                        v1 = p * v2
                        robot.set_vel(v1, v2)                        
                else:
                    robot.set_vel(0, 0)
