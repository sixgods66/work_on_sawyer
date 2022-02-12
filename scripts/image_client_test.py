#! /home/znfs/anaconda3/envs/detectron/bin/python

import rospy
from iros2021.srv import ImageProc, ImageProcRequest


def image_client():
    rospy.wait_for_service('/image_detection')
    try:
        image_c = rospy.ServiceProxy('/image_detection', ImageProc)
        response = image_c("kinect", True, 0)
        print(response)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


def displayWebcam():
    rospy.init_node('service_client', anonymous=True)
    image_client()


if __name__ == '__main__':
    displayWebcam()
