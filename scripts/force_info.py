import rospy,sys
from numpy import *

import geometry_msgs.msg
import std_msgs.msg
from intera_core_msgs.msg import EndpointState


def mean_filter(kernel_size, data):
    padding_data = []
    mid = kernel_size//2
    for i in range(mid):
        padding_data.append(0)
    padding_data.append(data)
    for i in range(mid):
        padding_data.append(0)
    result = []
    for i in range(0, len(padding_data)-2*mid, 1):
        temp = 0
        for j in range(kernel_size):
            temp += padding_data[i+j]
        temp = temp / kernel_size
        result.append(temp)
    return result

def force_callback(Subdata):
    wrench = Subdata.wrench
    Fx = wrench.force.x
    Fy = wrench.force.y
    Fz = wrench.force.z
    Fx_m = "%.4f" % mean_filter(3, Fx)[0]
    Fy_m = "%.4f" % mean_filter(3, Fy)[0]
    Fz_m = "%.4f" % mean_filter(3, Fz)[0]
    print('Force_x:', Fx_m)
    print('Force_y:', Fy_m)
    print('Force_z:', Fz_m)

def force_info():
    rospy.Subscriber('/robot/limb/right/endpoint_state',EndpointState, force_callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('force_info', anonymous=True)
    force_info()