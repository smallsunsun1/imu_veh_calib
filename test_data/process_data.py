import os

filename = "camera.bag"
out_filename = "formatted_camera.txt"
out_f = open(out_filename, "w")

with open(filename, "r") as f:
    out_f.write("start_t,end_t, delta_theta,angle_axis[w,x,y,z],rot_quaterian[w,x,y,z],trans_vec[x,y,z]\n")
    for idx, line in enumerate(f):
        if idx == 0:
            continue
        line = line.strip()
        data = line.split(",")
        start_t = data[0]
        end_t = data[1]
        delta_theta = data[4]
        angle_axis = ",".join(data[-4:])
        rot_quaterian = ",".join(data[-11:-7])
        trans_vec = ",".join(data[-7:-4])
        out_f.write(",".join([start_t, end_t, delta_theta, angle_axis, rot_quaterian, trans_vec]) + "\n")
    out_f.close()