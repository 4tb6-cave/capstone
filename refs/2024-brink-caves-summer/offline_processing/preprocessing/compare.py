# Script for comparing timestamps. Intended for checking alignment of rgb and depth images.
#
# Filenames are expected to be of the form topic_name_seconds.nanoseconds.extension
# for example oak_rgb_image_raw_compressed_1720630267.729555944.png

# to do:
# extend to more than two folders if required (eg right and left stereo + rgbd)
#   although you could also just run it on each combination of two folders.


import os
import sys
import matplotlib.pyplot as plt
import numpy as np

class timestampedFile:
    '''
    This class stores a file's name and location
    and extracts a timestamp from its filename
    '''
    def __init__(self, directory, filename):
        self.directory = directory
        self.filename = filename
        self.seconds = self.__extract_time()

    def __extract_time(self):
        try:
            seconds, nanos, ext = (self.filename.split('_')[-1]).split('.')
            while (len(nanos) < 9):
                nanos = nanos + '0'
            time = float(seconds) + float(nanos)*1e-9
        except:
            time = 0
            print("error extracting timestamp from ", self.filename)
        return time
    
    def is_valid(self):
        if (self.seconds != 0):
            return True
        else:
            return False
        
    def __eq__(self, other):
        return (self.directory == other.directory) and (self.filename == other.filename) and (self.seconds == other.seconds)

class association:
    '''
    The association class stores two instances of the timestampedFile class
    and provides a method to find the difference in their times
    '''
    def __init__(self, timestamp_a, timestamp_b):
        self.a = timestamp_a
        self.b = timestamp_b
    
    def get_error(self):
        return self.b.seconds - self.a.seconds
    
    def __eq__(self, other):
        return (self.a == other.a and self.b == other.b) or (self.a == other.b and self.b == other.a)

def get_timestamps(directory):
    files = os.listdir(directory) #[f for f in os.listdir(directory) if os.path.isfile(f)]
    timestamps = []
    for filename in files:
        t = timestampedFile(directory, filename)
        if (t.is_valid()):
            timestamps.append(t)
    return timestamps

def compare_timestamps(files_a, files_b, threshold):
    good_assoc = []
    bad_assoc = []
    for fa in files_a:
        closestFile = files_b[0]
        error = abs(closestFile.seconds - fa.seconds)
        for fb in files_b:
            if abs(fb.seconds - fa.seconds) < error:
                closestFile = fb
                error = abs(closestFile.seconds - fa.seconds)

        if error > threshold:
            bad_assoc.append(association(fa, closestFile))
        else:
            good_assoc.append(association(fa, closestFile))
    return good_assoc, bad_assoc

def show_stats(values, title):
    print("\n", title)
    if len(values) == 0:
        print("No elements")
        return
    
    print("Number of elements: ", len(values))
    print("mean: ", np.mean(values))
    print("median: ", np.median(values))
    print("std dev: ", np.std(values))
    print("min: ", min(values))    
    print("max: ", max(values))

    plt.hist(values)
    plt.title(title)
    plt.show()

def main():

    # check user input
    if len(sys.argv) != 3 and len(sys.argv) != 4:
        print("Usage: compare.py [directory A] [directory B] [threshold (optional)]")
        return
    dirA = sys.argv[1]
    dirB = sys.argv[2]
    if (len(sys.argv)== 4):
        threshold = float(sys.argv[3])
    else:
        threshold = 0.01

    # information
    print("A: ", dirA, "\tB: ", dirB)
    print("max difference in timestamps allowed: ", threshold)
    filesA = get_timestamps(dirA)
    filesB = get_timestamps(dirB)
    print("number in A: ", len(filesA), "\tnumber in B: ", len(filesB))
    
    # compare files
    goodA, badA = compare_timestamps(filesA, filesB, threshold)
    goodB, badB = compare_timestamps(filesB, filesA, threshold)

    # show statistics
    error_good = []
    for f in goodA:
        error_good.append(f.get_error())
    show_stats(error_good, "Error in good associations")

    error_bad_a = []
    for f in badA:
        error_bad_a.append(f.get_error())
    show_stats(error_bad_a, "Error in failed associations, directory A")

    error_bad_b = []
    for f in badB:
        error_bad_b.append(f.get_error())
    show_stats(error_bad_b, "Error in failed associations, directory B")
    
    # show bad files
    print("\nEnter y to list files with no associations?")
    if (input() != 'y'):
        return
    for f in badA + badB:
        print(os.path.join(f.a.directory, f.a.filename), "\tError: ", f.get_error())
    
    # delete bad files
    print("\nEnter y to permanently delete all files listed above")
    if (input() != 'y'):
        return
    for f in badA + badB:
        print("Deleting ", os.path.join(f.a.directory, f.a.filename))
        os.remove(os.path.join(f.a.directory, f.a.filename))
    print("Deletion succesful")
    
if __name__ == "__main__":
    main()