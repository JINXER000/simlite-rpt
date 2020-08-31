#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import tf
from gazebo_msgs.msg import ModelStates
from message_filters import TimeSynchronizer, Subscriber
from hector_uav_msgs.msg import Model_pose

def quaternion_to_rotation_matrix(quat):
    q = quat.copy()
    n = np.dot(q, q)
    if n < np.finfo(q.dtype).eps:
        return np.identity(4)
    q = q * np.sqrt(2.0 / n)
    q = np.outer(q, q)
    rot_matrix = np.array(
        [[1.0 - q[2, 2] - q[3, 3], q[1, 2] + q[3, 0], q[1, 3] - q[2, 0], 0.0],
         [q[1, 2] - q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] + q[1, 0], 0.0],
         [q[1, 3] + q[2, 0], q[2, 3] - q[1, 0], 1.0 - q[1, 1] - q[2, 2], 0.0],
         [0.0, 0.0, 0.0, 1.0]],
        dtype=q.dtype)
    return rot_matrix[:3,:3]
    

now_time=[0.0,0.0]
T = 0.5

kalman = cv2.KalmanFilter(6, 3)  # 6：状态数，包括（x，y，z, dx，dy, dz）坐标及速度（每次移动的距离）；3：观测量，能看到的是坐标值
kalman.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0],
                                     [0, 1, 0, 0, 0, 0],
                                     [0, 0, 1, 0, 0, 0]], np.float32)  # 系统测量矩阵
kalman.transitionMatrix = np.array([[1, 0, 0, T, 0, 0],
                                    [0, 1, 0, 0, T, 0],
                                    [0, 0, 1, 0, 0, T],
                                    [0, 0, 0, 1, 0, 0],
                                    [0, 0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 0, 1]], np.float32)  # 状态转移矩阵
kalman.processNoiseCov = np.array([[1, 0, 0, 0, 0, 0],
                                   [0, 1, 0, 0, 0, 0],
                                   [0, 0, 1, 0, 0, 0],
                                   [0, 0, 0, 1, 0, 0],
                                   [0, 0, 0, 0, 1, 0],
                                   [0, 0, 0, 0, 0, 1]], np.float32) * 0.01  # 系统过程噪声协方差
kalman.measurementNoiseCov = np.array([[1, 0, 0],
                                       [0, 1, 0],
                                       [0, 0, 1]], np.float32) * 0.001
last_measurement = np.array((3, 1), np.float32)
current_measurement = np.array((3, 1), np.float32)
last_prediction = np.zeros((3, 1), np.float32)
current_prediction = np.zeros((3, 1), np.float32)
current_prediction_v = np.zeros((3, 1), np.float32)

def compute(image_rgb, image_depth,p_c,pose_now):
    # detect
    assert image_rgb.header.stamp == image_depth.header.stamp
    global pose_callback
    pose=pose_now.pose.position
    #pose=pose_callback.position
    #print(pose)
    global last_time,now_time,T
    last_time=now_time
    now_time=[float(image_rgb.header.stamp.secs),float(image_rgb.header.stamp.nsecs)]
    T=(now_time[0]-last_time[0])+(now_time[1]-last_time[1])*1e-9

    x=[None,None,None,None]
    y=[None,None,None,None]
    font = cv2.FONT_HERSHEY_SIMPLEX
    lower_red = np.array([156, 43, 46]) 
    upper_red = np.array([180, 255, 255]) 
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_rgb, desired_encoding='passthrough')

    hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    mask_red = cv2.inRange(hsv_img, lower_red, upper_red) 

    mask_red = cv2.medianBlur(mask_red, 7) 
    mask = mask_red

    mask_red, contours2, hierarchy2 = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for i in range(4):
        if len(contours2)>i:
            x[i],y[i],a,b=cv2.boundingRect(contours2[i]) 
            x[i]+=a/2
            y[i]+=b/2
        else:
            x[i]=None
            y[i]=None

    for cnt2 in contours2:
        (x1, y1, w, h) = cv2.boundingRect(cnt2)
        cv2.rectangle(cv_image, (x1, y1), (x1 + w, y1 + h), (0, 255, 255), 2)
        cv2.putText(cv_image, "Red", (x1, y1 + 5), font, 0.7, (0, 0, 255), 2)
    
    cv2.imshow("dection", cv_image)
    cv2.waitKey(1)


    #depth and point_cloud
    cv_image_depth = bridge.imgmsg_to_cv2(image_depth, desired_encoding='passthrough')
    depth=cv_image_depth
    for i in range(4):
        if x[i]:
            #print "cylinder:",x[i],y[i],depth[y[i]][x[i]]
            p = point_cloud2.read_points_list(p_c, field_names = ("x", "y", "z"), skip_nans=False,uvs=[[x[i], y[i]]])
            #print p[0].z,-p[0].x,-p[0].y
            #print trans[0],trans[1],trans[2]
                #if not np.isnan(p[0].x):
                #print np.dot(quaternion_to_rotation_matrix(np.array(rot)),np.array([p[0].x,p[0].y,p[0].z]).transpose())
                #print np.array([p[0].x,p[0].y,p[0].z])

            #pointcloud 是图像坐标系下的 , 需要转换到相机坐标系
            print "measure: x : %.3f  y: %.3f  z: %.3f" %(-p[0].x+trans[0],-p[0].z+trans[1],-p[0].y+trans[2])
            print "true: x : %.3f  y: %.3f  z: %.3f" %(pose.x,pose.y,pose.z)
            #这里的配置只考虑了图像中出现一个柱子，卡尔曼滤波器只有一个
            x_m=-p[0].x+trans[0]
            y_m=-p[0].z+trans[1]
            z_m=-p[0].y+trans[2]
            kal_p,kal_o=kalman_filter(x_m,y_m,z_m)
            print "kalman_predit: x : %.3f  y: %.3f  z: %.3f" %(kal_p[0],kal_p[1],kal_p[2])
            print "kalman_optimal: x : %.3f  y: %.3f  z: %.3f"%(kal_o[0],kal_o[1],kal_o[2])
            plot(pose,x_m,y_m,z_m,kal_o)

x_true=[]
y_true=[]
z_true=[]
x_mea=[]
y_mea=[]
z_mea=[]
x_opt=[]
y_opt=[]
z_opt=[]
def plot(pose,x_m,y_m,z_m,kal_o):
    global x_true,y_true,z_true,x_mea,y_mea,z_mea,x_opt,y_opt,z_opt
    x_true.append(pose.x)
    y_true.append(pose.y)
    z_true.append(pose.z)
    x_mea.append(x_m)
    y_mea.append(y_m)
    z_mea.append(z_m)
    x_opt.append(kal_o[0][0])
    y_opt.append(kal_o[1][0])
    z_opt.append(kal_o[2][0])
    import matplotlib.pyplot as plt
    if len(x_true)==30:
        plt.scatter(np.array(x_true), np.array(y_true),label="true", c='r')
        plt.scatter(np.array(x_mea), np.array(y_mea),label="measure", c='b')
        plt.scatter(np.array(x_opt), np.array(y_opt),label="kalman estimation", c='y')
        error_mea=(np.array(x_mea[2:])-np.array(x_true[2:]))**2+(np.array(y_mea[2:])-np.array(y_true[2:]))**2
        error_opt=(np.array(x_opt[2:])-np.array(x_true[2:]))**2+(np.array(y_opt[2:])-np.array(y_true[2:]))**2
        print "error_mea:",sum(error_mea)/30
        print "error_opt:",sum(error_opt)/30
        plt.ylabel("y_position")
        plt.xlabel("x_position")
        plt.legend()
        plt.show()
    

def kalman_filter(x_m, y_m, z_m):

    # kalman_filter 
    # input: 测量值xyz
    # output: current_prediction下一时刻位置
    #	      current_prediction_v 下一时刻速度（矢量）
    global kalman, frame, current_measurement, last_measurement, current_prediction, last_prediction, current_prediction_v

    kalman.transitionMatrix = np.array([[1, 0, 0, T, 0, 0],
                                        [0, 1, 0, 0, T, 0],
                                        [0, 0, 1, 0, 0, T],
                                        [0, 0, 0, 1, 0, 0],
                                        [0, 0, 0, 0, 1, 0],
                                        [0, 0, 0, 0, 0, 1]], np.float32)  # 状态转移矩阵
    last_prediction = current_prediction  # 把当前预测存储为上一次预测
    last_measurement = current_measurement  # 把当前测量存储为上一次测量
    current_measurement = np.array([[np.float32(x_m)], [np.float32(y_m)], [np.float32(z_m)]])  # 当前测量
    kalman.correct(current_measurement)  # 用当前测量来校正卡尔曼滤波器

    current_prediction_v = kalman.statePost #记录当前最优位置

    current_prediction = kalman.predict()  # 计算卡尔曼预测值，作为下一时刻预测x,y,z

    #current_prediction_v = current_prediction - current_prediction_v #v = 预测下一时刻位置 - 当前时刻最优位置 （理论上大小为恒定值 方向变化）。

    #msg = Float64MultiArray(data=current_prediction)
    #msg_v = Float64MultiArray(data=current_prediction_v)
    #position_pub.publish(msg)#将预测值pub到/position话题上
    #velocity_pub.publish(msg_v)#将预测值pub到/velocity话题上
    return current_prediction, current_prediction_v
def callback(data):
    global pose_callback
    pose_callback=data.pose[3]

def listener():

    rospy.init_node('red_detect', anonymous=True)
    tss = TimeSynchronizer([Subscriber("/camera/rgb/image_raw", Image),Subscriber("/camera/depth/image_raw", Image),Subscriber("/camera/depth/points", PointCloud2),Subscriber("/model_pose", Model_pose)],60)
    tss.registerCallback(compute)

    #rospy.Subscriber("/camera/rgb/image_raw", Image, callback)
    #rospy.Subscriber("/camera/depth/image_raw", Image, callback1)
    #rospy.Subscriber("/camera/depth/points", PointCloud2 ,callback2)
    #rospy.Subscriber("/gazebo/model_states",ModelStates,callback)

    global trans,rot
    tfcache=rospy.Duration
    tfcache=tfcache.from_sec(5)
    listener = tf.TransformListener(tfcache)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/camera_depth_frame', rospy.Time(0))
            #print(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    listener()
