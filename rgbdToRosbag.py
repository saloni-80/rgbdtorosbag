import time, sys, os
from ros import rosbag
import roslib
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image

from PIL import ImageFile
import rospy

def GetFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    print( "Searching directory %s" % dir )
    all = []
    depth = []
    left_files = []
    right_files = []
    if os.path.exists(dir+'/stereo'):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    if 'left' in f or 'left' in path:
                        left_files.append( os.path.join( path, f ) )
                    elif 'right' in f or 'right' in path:
                        right_files.append( os.path.join( path, f ) )
                    #all.append( os.path.join( path, f ) )
    if os.path.exists(dir+'/rgb'):
        for path, names, files in os.walk(dir+'/rgb'):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    all.append( os.path.join( path, f ) )
    if os.path.exists(dir+'/depth'):
        for path, names, files in os.walk(dir+'/depth'):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    depth.append( os.path.join( path, f ) )
    return all, depth, left_files, right_files

def CreateStereoBag(left_imgs, right_imgs, bagname):
    '''Creates a bag file containing stereo image pairs'''
    bag =rosbag.Bag(bagname, 'w')

    try:
        for i in range(len(left_imgs)):
            print("Adding %s" % left_imgs[i])
            fp_left = open( left_imgs[i], "r" )
            p_left = ImageFile.Parser()

            while 1:
                s = fp_left.read(1024)
                if not s:
                    break
                p_left.feed(s)

            im_left = p_left.close()

            fp_right = open( right_imgs[i], "r" )
            print("Adding %s" % right_imgs[i])
            p_right = ImageFile.Parser()

            while 1:
                s = fp_right.read(1024)
                if not s:
                    break
                p_right.feed(s)

            im_right = p_right.close()

            Stamp = roslib.rostime.Time.from_sec(time.time())

            Img_left = Image()
            Img_left.header.stamp = Stamp
            Img_left.width = im_left.size[0]
            Img_left.height = im_left.size[1]
            Img_left.encoding = "rgb8"
            Img_left.header.frame_id = "camera/left"
            Img_left_data = [pix for pixdata in im_left.getdata() for pix in pixdata]
            Img_left.data = Img_left_data
            Img_right = Image()
            Img_right.header.stamp = Stamp
            Img_right.width = im_right.size[0]
            Img_right.height = im_right.size[1]
            Img_right.encoding = "rgb8"
            Img_right.header.frame_id = "camera/right"
            Img_right_data = [pix for pixdata in im_right.getdata() for pix in pixdata]
            Img_right.data = Img_right_data

            bag.write('camera/left/image_raw', Img_left, Stamp)
            bag.write('camera/right/image_raw', Img_right, Stamp)
    finally:
        bag.close()

def CreateMonoBag(imgs, depth_imgs, bagname):
    '''Creates a bag file with camera images'''
    bag =rosbag.Bag(bagname, 'w')

    try:
        for i in range(len(imgs)):
            print("Addingg %s" % imgs[i])
            fp_rgb = open( imgs[i], "rb" )
            p_rgb = ImageFile.Parser()

            while 1:
                s = fp_rgb.read(1024)
                if not s:
                    break
                p_rgb.feed(s)

            im_rgb = p_rgb.close()

            print("Adding %s" % depth_imgs[i])
            fp_depth = open( depth_imgs[i], "rb" )
            p_depth = ImageFile.Parser()

            while 1:
                s = fp_depth.read(1024)
                if not s:
                    break
                p_depth.feed(s)

            im_depth = p_depth.close()


            Stamp = rospy.rostime.Time.from_sec(time.time())
            Img_rgb = Image()
            Img_rgb.header.stamp = Stamp
            Img_rgb.width = im_rgb.size[0]
            Img_rgb.height = im_rgb.size[1]
            Img_rgb.encoding = "rgb8"
            Img_rgb.header.frame_id = "camera/rgb"
            Img_rgb_data = [pix for pixdata in im_rgb.getdata() for pix in pixdata]
            Img_rgb.data = Img_rgb_data
            
            Img_depth = Image()
            Img_depth.header.stamp = Stamp
            Img_depth.width = im_depth.size[0]
            Img_depth.height = im_depth.size[1]
            Img_depth.encoding = "mono8"
            Img_depth.header.frame_id = "camera/depth_registered/sw_registered"
            Img_depth_data = [int(pix / 255) for pixdata in [im_depth.getdata()] for pix in pixdata]
            Img_depth.data = Img_depth_data
            #print(Img_data)

            bag.write('camera/rgb/image_color', Img_rgb, Stamp)

            bag.write('camera/depth_registered/sw_registered/image_rect_raw', Img_depth, Stamp)
    finally:
        bag.close()   

def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    all_imgs, depth_imgs, left_imgs, right_imgs = GetFilesFromDir(args[0])
    if len(all_imgs) <= 0:
        print("No images found in %s" % args[0])
        exit()

    if len(left_imgs) > 0 and len(right_imgs) > 0:
        # create bagfile with stereo camera image pairs
        CreateStereoBag(left_imgs, right_imgs, args[1])
    else:
        # create bagfile with mono camera image stream
        CreateMonoBag(all_imgs, depth_imgs, args[1])        

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        CreateBag( sys.argv[1:])
    else:
        print( "Usage: img2bag imagedir bagfilename")
