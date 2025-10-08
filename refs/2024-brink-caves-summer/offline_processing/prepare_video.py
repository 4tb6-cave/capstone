import os
import yaml
import subprocess
import csv
import numpy as np
from tabulate import tabulate
import argparse

def main():
    parser = argparse.ArgumentParser("prepare_video.py")
    parser.add_argument("--source", nargs='?', help="source directory, with recorded data (if blank, cwd)")
    parser.add_argument("--output", nargs='?', help="destination directory (if blank, cwd)")
    parser.add_argument("--old", default=False, action=argparse.BooleanOptionalAction, help="set if data was collected before aug 8 2024 (time.txt files were messed up)")
    args = parser.parse_args()

    if args.source:
        data_input = args.source
    else:
        data_input = os.getcwd()
    if args.output:
        data_output = args.output
    else:
        data_output = os.getcwd()

    recorded_before_aug_8 = args.old

    # find format of video files
    with open(os.path.join(data_input, "config_save.yaml")) as file:
        config = yaml.safe_load(file)

    times = np.loadtxt(os.path.join(data_input, "time.txt"), skiprows=2)
    print(times)

    encoding = {"rgb": config["rgb"]["encoding"], "left": config["mono"]["encoding"], "right": config["mono"]["encoding"]}

    for stream in ["rgb", "left", "right"]:
        dest_dir = os.path.join(data_output, stream)

        try:   
            os.makedirs(dest_dir, exist_ok=False) # raise exception if it already exists
        except OSError:
            print(stream, "output folder already exists, not ffmpeging again")
            continue

        subprocess.check_call(
            [
                "ffmpeg",
                "-i",
                os.path.join(data_input, f'{stream}.{encoding[stream]}'),
                os.path.join(dest_dir, "%06d.png") # width of 6 means 10 hours (which we never want to reach) before it overflows
            ]
        )

    # copy depth
    try:
        os.makedirs(os.path.join(data_output, "depth"), exist_ok=False)
        image_list = os.listdir(os.path.join(data_input, "depth"))
        for f in image_list:
            if not os.path.isfile(os.path.join(data_input, "depth", f)):
                print("skipping", f, "because not a file")
            else:
            # make a hard link (file name points to same content on disk)
                os.link(os.path.join(data_input, "depth", f), os.path.join(data_output, "depth", f))

    except OSError:
        print("depth output folder already exists, not copying")

    num = [0,0,0,0]
    header = ["rgb", "left", "right", "depth"] # titles for each column
    deleted = [0,0,0,0]
    skipped = [0,0,0,0]

    # make list of timestamps on depth files (different from table if table was recorded with 4 decimal places)
    depth_files = os.listdir(os.path.join(data_output, "depth"))
    depth_timestamps = []
    for df in depth_files:
        sec, subsec, ext = df.split('.')
        t = float(f'{sec}.{subsec}')
        depth_timestamps.append(t)

    new_times = []

    for t in times:
        no_good = not t[0] or not t[1] or not t[2] or not t[3] # if any element in the row is missing a timestamp the whole thing is no good
        
        if not no_good:
            new_times.append(t)

        for i in range(3): # rgb left and right
            if t[i] != 0: # if timestamp was saved
                num[i] += 1

                # rgb left and right are initially named by number
                full_path = os.path.join(data_output, header[i], f'{num[i]:06d}.png')
                
                try:
                    if no_good: # delete
                        os.remove(full_path)
                        deleted[i] += 1
                    else:
                        # rename to timestamp
                        os.rename(full_path, os.path.join(data_output, header[i], f'{t[i]:.6f}.png'))
                except FileNotFoundError:
                    skipped[i] += 1
                    #print(full_path, "already deleted or renamed")
    
        # depth
        if t[3] != 0: # if timestamp was saved
            num[3] += 1

            # find closest timestamp to the one that has been rounded (t[3])
            guess = 0
            for dt in depth_timestamps:
                if abs(t[3] - dt) < abs(t[3] - guess):
                    guess = dt

            if abs(guess - t[3]) > 0.00005: # max rounding error when rounding to x.xxxx
                #print("no good association in depth timestamp", t[3], "-- maybe already deleted?")
                skipped[3] += 1
                continue

            full_path = os.path.join(data_output, header[3], f'{guess:.6f}.png')
            
            if no_good: # delete
                os.remove(full_path)
                deleted[3] += 1
            elif recorded_before_aug_8: # rename it, but not necessary if table was made properly
                os.rename(full_path, os.path.join(data_output, header[3], f'{t[3]:.6f}.png'))

    # could also use numpy.savetxt() but for consistency I'll do it the same as before
    table_string = tabulate(new_times, headers=["Color", "Left", "Right", "Depth"], floatfmt=".6f")
    
    # save timestamp table
    with open(os.path.join(data_output, "time.txt"), 'w') as file:
        file.write(table_string)

    for i in range(4):
        n = len(os.listdir(os.path.join(data_output, header[i])))
        print("number of files remaining in", header[i], ":", n)
        print("deleted from", header[i], ":", deleted[i])
        print("skpped files in", header[i], ":", skipped[i])

main()