import cv2
from os import path, listdir

data_dir = "/home/arco3/Desktop/aug_12_data/data_009"
right_file = "right.mjpeg"
left_file = "left.mjpeg"
right_path = path.join(data_dir, right_file)
left_path = path.join(data_dir, left_file)

def file_key(filename: str):
    i, j, ext = filename.split('.')
    return float(f'{i}.{j}')

def main():

    depth_list = listdir(path.join(data_dir, "depth"))
    depth_list.sort(key=file_key)

    vr = cv2.VideoCapture(right_path)
    vl = cv2.VideoCapture(left_path)

    if (vl.isOpened() == False):
        print("error opening", left_path)
        return
    
    if (vr.isOpened() == False):
        print("error opening", right_path)
        return

    print("right file opened:", right_path)
    print("left file opened:", left_path)

    print("using backend", vr.getBackendName())
    fps1 = vr.get(cv2.CAP_PROP_FPS)
    fps2 = vl.get(cv2.CAP_PROP_FPS)

    if (abs(fps1 - fps2) > 1e-5):
        print("fps right:", fps1)
        print("fps left :", fps2)
        print("not close enough")
        return
    
    print('Frames per second: ', fps1)

    countl = vl.get(cv2.CAP_PROP_FRAME_COUNT)
    countr = vr.get(cv2.CAP_PROP_FRAME_COUNT)
    print('Frame count (R):', countr)
    print('Frame count (L):', countl)

    stereo = cv2.StereoBM_create()

    while(vr.isOpened() and vl.isOpened()):
        retr, framer = vr.read()
        retl, framel = vl.read()

        if retr and retl:
            cv2.imshow('Right', framer)
            cv2.imshow('Left', framel)
            

            framel = cv2.cvtColor(framel, cv2.COLOR_BGR2GRAY)
            framer = cv2.cvtColor(framer, cv2.COLOR_BGR2GRAY)
            
            DEPTH_VISUALIZATION_SCALE = 2048
            depth = stereo.compute(framel, framer)
            cv2.imshow('Depth', depth/DEPTH_VISUALIZATION_SCALE)

            depth_path = path.join(data_dir, "depth", depth_list.pop(0))
            depth2 = cv2.imread(depth_path)
            disp2 = 75*2.35/depth2
            cv2.imshow('Depth2', disp2/DEPTH_VISUALIZATION_SCALE)

            key = cv2.waitKey(50)
            
            if key == ord('q'):
                print("q pressed")
                break
        else:
            print("end of one or both files")
            break

    # will not be executed if program quits early...
    vr.release()
    vl.release()
    cv2.destroyAllWindows

if __name__=="__main__":
    main()