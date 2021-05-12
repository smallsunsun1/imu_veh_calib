import os
import math

filename = "odo.bag"
out_filename = "formatted_odometry_v2.txt"
out_f = open(out_filename, "w")

time_interval = 0.01
axle = 0.57
v_left = 2
v_right = 2
radius = 0.1

with open(filename, "r") as f:
    out_f.write("time,v_left,v_right\n")
    for idx, line in enumerate(f):
        cur_t = idx * time_interval
        if idx == 0:
            continue
        line = line.strip()
        data = line.split(",")
        time = data[0]

        line_vec = float(data[1])
        angle_vec = float(data[2])
        speed_left = (2 * line_vec - axle * angle_vec) / 2 / radius
        speed_right = (2 * line_vec + axle * angle_vec) / 2 / radius

        # speed_left = v_left * math.cos(cur_t) * radius
        # speed_right = v_right * math.sin(cur_t) * radius
        out_str = ",".join([time, str(speed_left), str(speed_right)]) + '\n'
        out_f.write(out_str)
    out_f.close()