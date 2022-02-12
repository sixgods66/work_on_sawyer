#!/home/znfs/anaconda3/envs/detectron/bin/python
# coding=utf-8

# 这个文件用来采集数据
import rospy
import os
from sensor_msgs.msg import Image
import cv2
from pynput import keyboard
import sys
# import pyHook

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
count = -1
save = False
path = "/home/znfs/Pictures/IROS2021"
path_name = "/home/znfs/Pictures/IROS2021/"


def on_press(key):
    """按下按键时执行。"""
    try:
        print('alphanumeric key {0} pressed'.format(key.char))
        if key.char == 'q':
            sys.exit()
        global save
        save = True
    except AttributeError:
        print('special key {0} pressed'.format(key))
    # 通过属性判断按键类型。


def cameraCallback(data):
    global count, path_name
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("", cv_image)
    global save
    if save:
        cv2.imwrite(path_name + str(count) + ".png", cv_image)
        print(path_name + str(count) + ".png saved")
        save = False
        count += 1
    cv2.waitKey(10)


def main():
    rospy.init_node('cap_dataset')
    global path
    cnt = -1
    for path, file_dir, files in os.walk(path):
        for file_name in files:
            number = int(file_name.split('.')[0])
            cnt = max(number, cnt)  # 当前循环打印的是当前目录下的所有文件

    global count
    cnt += 1
    count = cnt
    print("count number ", count)

    # rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, cameraCallback, queue_size=1)
    rospy.Subscriber("/camera/color/image_raw", Image, cameraCallback, queue_size=1)
    # rospy.Subscriber("/io/internal_camera/right_hand_camera/image_rect", Image, cameraCallback, queue_size=1)
    rospy.loginfo("Start to subscribe")

    with keyboard.Listener(on_press=on_press,) as listener:
        listener.join()

    rospy.spin()


if __name__ == "__main__":
    main()
