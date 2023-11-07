import struct
import math
import numpy as np


def usr(robot):
    smooth = True
        
    while True:
        sum_hop_1 = 0
        sum_hop_2 = 0
        num_nbrs = 0

        robot.delay(10)

        # update hop count
        hop_1 = 100
        hop_2 = 100

        msgs = robot.recv_msg()
        if len(msgs) > 0:
            for i in range(len(msgs)):
                recv_hop_msg = struct.unpack("ii", msgs[i])
                recv_hop_1 = recv_hop_msg[0]
                recv_hop_2 = recv_hop_msg[1]
                if recv_hop_1 + 1 < hop_1:
                    hop_1 = recv_hop_1 + 1
                if recv_hop_2 + 1 < hop_2:
                    hop_2 = recv_hop_2 + 1
                sum_hop_1 += recv_hop_1
                sum_hop_2 += recv_hop_2
                num_nbrs += 1

        # reset hop count for seeds
        if robot.assigned_id == 1:
            hop_1 = 0
        elif robot.assigned_id == 2:
            hop_2 = 0
        
        if hop_1 < 24 and hop_2 < 24:
            # estimate distance to seed 1 and seed 2
            h1 = hop_1 * 1 # distance to seed 1
            h2 = hop_2 * 1 # distance to seed 2

            if smooth == True:
                h1 = (sum_hop_1 + hop_1) / (num_nbrs + 1) - 0.5 # distance to seed 1
                h2 = (sum_hop_2 + hop_2) / (num_nbrs + 1) - 0.5 # distance to seed 2   
            
            # estimate position
            x, y = solve_lse(h1, h2)

            # paint N
            if x < 4 or x > 12 or (y + 1.9 * x < 27 and y + 1.9 * x > 20):
                robot.set_led(78, 42, 132)
            else:
                robot.set_led(255, 255, 255)

        # send hop count
        msg = struct.pack("ii", hop_1, hop_2)
        robot.send_msg(msg)

def solve_lse(hop_1, hop_2):
    x = 0.0
    y = 0.0
    x_best = 0.0
    y_best = 0.0
    d1_est = hop_1 * 1.0
    d2_est = hop_2 * 1.0
    E = 1000

    for i in range(0, 34):
        for j in range(0, 27):
            x = i / 2.0
            y = j / 2.0
            d1_cal = math.sqrt(x**2 + y**2)
            d2_cal = math.sqrt(x**2 + (y - 15)**2)

            E1 = (d1_cal - d1_est)**2
            E2 = (d2_cal - d2_est)**2
            E_new = E1 + E2

            if E_new < E:
                E = E_new
                x_best = x
                y_best = y

    return x_best, y_best
