#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import rospkg
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import struct
from std_msgs.msg import Header

right_img_c = None
left_img_c = None
cam_info = None
br = CvBridge() 
Q = None
frame_no = None

def right_image_capture(right):
    global right_img_c
    try:
        right_img_c = br.imgmsg_to_cv2(right,"bgr8")
    except CvBridgeError as e:
      print(e)
    # right_img = right

def left_image_capture(left):
    global left_img_c
    try:
        left_img_c = br.imgmsg_to_cv2(left,"bgr8")
    except CvBridgeError as e:
      print(e)
    # left_img = left

def cam_info_capture(info):
    global cam_info,Q,frame_no
    frame_no = info.header.seq
    baseline = 0.12 # in meters
    cam_info = info
    Q = [[1,0,0,-cam_info.K[2]],[0,1,0,-cam_info.K[5]],[0,0,0,cam_info.K[0]],[0,0,-1/baseline,0]]
    # Q = [[1,0,0,-cam_info.K[2]],[0,1,0,-cam_info.K[5]],[0,0,0,cam_info.K[0]],[0,0,-1/baseline,0]]
    Q = np.array(Q,dtype='uint8')

def produce_depth_image():
    global right_img_c,left_img_c,cam_info, Q, frame_no

    # frame number
    param_frame_no = int(rospy.get_param('frame_number'))
    got_pc2 = False
    # define baseline and focal length for calculating depth image
    baseline = 12 # in meters
    focal_len = 28 # in pixel 0.0028 # in meters
    # get current directory
    rospack = rospkg.RosPack()
    pkg_name = rospack.get_path('lab7')

    pub = rospy.Publisher('my_depth_image', Image,queue_size=10)
    mark_pub = rospy.Publisher('zed/zed_node/point_cloud/cloud_registered',PointCloud2,queue_size=10)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        if left_img_c is not None and right_img_c is not None:

            if len(left_img_c.shape) > 2:    
                left_img = cv2.cvtColor(left_img_c,cv2.COLOR_BGR2GRAY)
            else:
                left_img = left_img_c

            if len(right_img_c.shape) > 2:
                right_img = cv2.cvtColor(right_img_c,cv2.COLOR_BGR2GRAY)
            else:
                right_img = right_img_c

            # declare stereo object
            stereo = cv2.StereoSGBM_create(minDisparity=-1,
                                            numDisparities=128, #128 
                                            blockSize=10,#10
                                            uniquenessRatio=5,
                                            speckleWindowSize=3,
                                            speckleRange =1,
                                            disp12MaxDiff=2,
                                            P1 = 8*3*5**2, #8
                                            P2 = 32*3*5**2) #32
            # stereo = cv2.StereoBM_create(numDisparities=48,blockSize=11) #128                                
            # compute stereo image
            disparity = stereo.compute(left_img,right_img)
            
            # bring the disparity image in between 0 and 255
            disparity = cv2.normalize(disparity, disparity, alpha=256,
                                        beta=0, norm_type=cv2.NORM_MINMAX)
            disparity = np.uint8(disparity)

            # publish image so as to be seen in Rviz
            pub.publish(br.cv2_to_imgmsg(disparity))

            # get 3D points
            disparity_float = np.float32(np.divide(disparity,16.0))
            # disparity_float = np.float32(disparity)
            
            if frame_no == param_frame_no and got_pc2 == False:
                points_3D = cv2.reprojectImageTo3D(disparity_float,Q)

                # get color information
                color = cv2.cvtColor(left_img_c,cv2.COLOR_BGR2RGB)

                # create mask
                mask_map = disparity_float > disparity_float.min()

                # mask point and color
                output_points = points_3D[mask_map]
                output_color = color[mask_map]
                output_color = output_color.reshape(-1,3)
                
                # create points to be send to create_point_cloud method
                a = 255
                points_w_c = []
                for i in range(len(output_points)):
                    pnt = output_points[i]
                    color = output_color[i]
                    rgba = struct.unpack('I', struct.pack('BBBB', int(color[2]), int(color[1]), int(color[0]), a))[0]
                    points_w_c.append([pnt[0],pnt[1],pnt[2],rgba]) 

                # header of point cloud
                header = Header()
                header.frame_id = 'base_link'

                # field structure
                fields = [PointField('x', 0, PointField.FLOAT32, 1),
                            PointField('y', 4, PointField.FLOAT32, 1),
                            PointField('z', 8, PointField.FLOAT32, 1),
                            PointField('rgba', 12, PointField.UINT32, 1),
                            ]
                
                pc2 = point_cloud2.create_cloud(header, fields, points_w_c)
                got_pc2 = True
                
            if got_pc2 == True:
                pc2.header.stamp = rospy.Time.now()
                # publish point cloud
                mark_pub.publish(pc2)

        rate.sleep()

if __name__ == '__main__':
    
    # initiate get_intrinsic_parameter node
    rospy.init_node('depth_from_stereo', anonymous=True)
    # rospy.Subscriber("/zed/zed_node/right_raw/image_raw_color",Image,right_image_capture)
    # rospy.Subscriber("/zed/zed_node/left_raw/image_raw_color",Image,left_image_capture)
    rospy.Subscriber("/zed/zed_node/right/image_rect_color",Image,right_image_capture)
    rospy.Subscriber("/zed/zed_node/left/image_rect_color",Image,left_image_capture)
    rospy.Subscriber("/zed/zed_node/left/camera_info",CameraInfo,cam_info_capture)
    
    produce_depth_image()

    rospy.spin()