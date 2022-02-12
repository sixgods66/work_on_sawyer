#! /home/znfs/anaconda3/envs/detectron/bin/python

import os
import cv2
import rospy
import numpy as np
import yaml
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from rospy import ServiceException
import message_filters

from cv_bridge import CvBridge, CvBridgeError
from iros2021.msg import ImageProcMsg
from iros2021.srv import ImageProc, ImageProcRequest, ImageProcResponse
import matplotlib.pyplot as plt

import torch
from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
from detectron2.structures.instances import Instances
from detectron2.utils.visualizer import ColorMode, Visualizer
from detectron2.data.datasets import register_coco_instances
from detectron2.data import MetadataCatalog, DatasetCatalog

project = "workpiece"
yaml_file = "/detectron2/configs/COCO-InstanceSegmentation/mask_rcnn_R_101_FPN_3x.yaml"
weights = "detectron2://COCO-InstanceSegmentation/mask_rcnn_R_101_FPN_3x/138205316/model_final_a3ec72.pkl"
path = "/home/znfs/project/detectron2_learn/newIros2021"
json_path = os.path.join(path, "trainval.json")
image_path = os.path.join(path, "images")
register_coco_instances(project, {}, json_path, image_path)
project_metadata = MetadataCatalog.get(project)
dataset_dicts = DatasetCatalog.get(project)

cfg = get_cfg()
var = os.environ['HOME']
os.path.expandvars('$HOME')
home_path = os.path.expanduser('~')
cfg.merge_from_file(home_path + yaml_file)

cfg.DATASETS.TRAIN = (project,)
cfg.DATASETS.TEST = ()  # no metrics implemented for this dataset
cfg.DATALOADER.NUM_WORKERS = 2
cfg.SOLVER.IMS_PER_BATCH = 2
cfg.SOLVER.BASE_LR = 0.02
cfg.SOLVER.MAX_ITER = (
    300
)  # 300 iterations seems good enough, but you can certainly train longer
cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = (
    128
)  # faster, and good enough for this toy dataset
cfg.MODEL.ROI_HEADS.NUM_CLASSES = 15  # 注意修改这里，对应的是识别的类别数量
cfg.OUTPUT_DIR = os.path.join(path, "output")
cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, "model_final.pth")

cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  # set the testing threshold for this model
cfg.DATASETS.TEST = (project,)
predictor = DefaultPredictor(cfg)
metadata = MetadataCatalog.get(cfg.DATASETS.TEST[0] if len(cfg.DATASETS.TEST) else "__unused")
instance_mode = ColorMode.IMAGE
cpu_device = torch.device("cpu")
parallel = False

# Instances(num_instances=4, image_height=540, image_width=960, fields=[pred_boxes: Boxes(tensor([[255.2790,
# 147.4086, 317.1690, 195.3333], [219.0074, 168.2458, 271.8637, 205.1023], [219.7436, 168.1835, 270.8622, 205.0093],
# [328.8908,   0.0000, 426.4765,  30.3456]])), scores: tensor([0.9937, 0.9078, 0.7405, 0.5464]), pred_classes:
# tensor([6, 6, 7, 0]), pred_masks: tensor([[[False, False, False,  ..., False, False, False],

################# PARAMETERS #################
debug = True

base2ee_topic = '/base2ee'

rs_i = {'width': 1280, 'height': 720, 'px': 639.834, 'py': 374.726, 'fx': 915.539, 'fy': 913.727}
kinct_extrinsic_file = "/home/znfs/project_ws/intera/src/iros2021/files/kinect_calibration.yaml"
debug_dir = "/home/znfs/project_ws/intera/src/iros2021/debug"
direction = {}
map_classify = {}


###############################################

def quaternion_to_trans_matrix(x, y, z, w, trans_x, trans_y, trans_z):
    x_2, y_2, z_2 = x * x, y * y, z * z
    xy, xz, yz, wx, wy, wz = x * y, x * z, y * z, w * x, w * y, w * z
    # origin_rotation_matrix        [1 - 2 * y_2 - 2 * z_2,  2 * xy + 2 * wz,        2 * xz - 2 * wy,
    #                                2 * xy - 2 * wz,        1 - 2 * x_2 - 2 * z_2,  2 * yz + 2 * wx,
    #                                2 * xz + 2 * wy,        2 * yz - 2 * wx,        1 - 2 * x_2 - 2 * y_2]
    translation_matrix = np.array([1 - 2 * y_2 - 2 * z_2, 2 * xy - 2 * wz, 2 * xz + 2 * wy, trans_x,
                                   2 * xy + 2 * wz, 1 - 2 * x_2 - 2 * z_2, 2 * yz - 2 * wx, trans_y,
                                   2 * xz - 2 * wy, 2 * yz + 2 * wx, 1 - 2 * x_2 - 2 * y_2, trans_z,
                                   0, 0, 0, 1
                                   ])
    return translation_matrix.reshape((4, 4))


def resolve_ext(yaml_data):
    orientation = yaml_data['rot']
    trans = yaml_data['trans']
    trans_matrix = quaternion_to_trans_matrix(orientation[0], orientation[1], orientation[2], orientation[3],
                                              trans[0], trans[1], trans[2])
    return trans_matrix


class ImageManage:
    def __init__(self, kinect_info='qhd', kinect=True, rs=True, rh=True):
        self.bridge = CvBridge()
        self.kinect_init = False
        self.rs_init = False
        self.k_info = kinect_info
        file = open(kinct_extrinsic_file, 'r')
        file_data = file.read()
        self.k_extrin = resolve_ext(yaml.safe_load(file_data))

        # we don't use [sd]
        if self.k_info == 'qhd':
            kinect_K = np.array([540.68603515625, 0.0, 479.75, 0.0, 540.68603515625, 269.75, 0.0, 0.0, 1.0])
        else:
            kinect_k = np.array([1081.3720703125, 0.0, 959.5, 0.0, 1081.3720703125, 539.5, 0.0, 0.0, 1.0])
        # realsense_k =

        if kinect:
            self.kinect_init = True
            self.kinect_name = "kinect detection"
            k_depth = message_filters.Subscriber('/kinect2/' + self.k_info + '/image_depth_rect', Image)
            k_image = message_filters.Subscriber('/kinect2/' + self.k_info + '/image_color_rect', Image)
            kinect_ts = message_filters.TimeSynchronizer([k_depth, k_image], 1)
            kinect_ts.registerCallback(self.kinect_callback)
            self.kinect_image = None
            self.kinect_depth = None
        if rs:
            self.rs_init = True
            self.rs_name = "realsense detection"
            rs_depth = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
            rs_image = message_filters.Subscriber('/camera/color/image_rect_color', Image)
            rs_ts = message_filters.TimeSynchronizer([rs_depth, rs_image], 1)
            rs_ts.registerCallback(self.rs_callback)
            self.rs_image = None
            self.rs_depth = None
        if rh:
            self.rh_init = True
            self.rh_name = "right hand detection"
            rospy.Subscriber('/io/internal_camera/right_hand_camera/image_rect', Image, self.rh_callback, queue_size=1)
            self.rh_image = None
        rospy.loginfo("start to subscribe")
        rospy.Service('/image_detection', ImageProc, self.image_callback)
        rospy.loginfo("Detection service start")
        rospy.spin()

    def predict(self, image, window_name, vis=True):
        rospy.loginfo("callback")
        predictions = predictor(image)
        visualizer = Visualizer(image, metadata, instance_mode=instance_mode)
        instances = predictions["instances"].to(cpu_device)

        if vis:
            vis_output = visualizer.draw_instance_predictions(predictions=instances)

            # method 1
            fig = plt.figure(window_name)
            plt.imshow(vis_output.get_image())
            plt.show()
            plt.close(fig)

            if debug:
                cv2.imwrite(os.path.join(debug_dir, "seg_image.jpg"), vis_output.get_image())
            # method 2
            # cv2.imshow(str(r), vis_output.get_image())
            # cv2.waitKey(3000)
            # cv2.destroyAllWindows()

        if debug:
            cv2.imwrite(os.path.join(debug_dir, "image.jpg"), image)

        return instances

    def image_callback(self, req: ImageProc):
        predict = self.predict
        if req.camera == 'rs':
            try:
                # if self.kinect_image is None:
                #     return None
                depth = self.rs_depth
                instance = predict(self.rs_image, self.rs_name, req.visual)
                return self.resolve_instance(depth, instance, req.camera)
            except ServiceException as e:
                rospy.logerr("Realsense is not init, but try to use it. %s", e)
                return None
        if req.camera == 'kinect':
            try:
                # if self.rs_image is None:
                #     return None
                depth = self.kinect_depth
                instance = predict(self.kinect_image, self.kinect_name, req.visual)
                return self.resolve_instance(depth, instance, req.camera)
            except ServiceException as e:
                rospy.logerr("Kinect is not init, but try to use it. %s", e)
                return None
        if req.camera == 'rh':
            try:
                # if self.rh_image is None:
                #     return None
                instance = predict(self.rh_image, self.rh_name, req.visual)
                return self.resolve_instance_rh(instance)
            except ServiceException as e:
                rospy.logerr("Kinect is not init, but try to use it. %s", e)
                return None

    def base_to_object(self, x, y, z):
        trans_b_k = self.k_extrin
        trans_k_o = np.array([x, y, z, 1]).reshape(4, 1)
        trans_b_o = trans_b_k.dot(trans_k_o)

        return trans_b_o

    def convert_pos(self, data, depth_img, ctype):
        camera_factor = 1
        if ctype == 'kinect':
            camera_cx = 479.75
            camera_cy = 269.75
            camera_fx = 540.68603515625
            camera_fy = 540.68603515625
        else:
            camera_cx = 639.8336181640625
            camera_cy = 374.7256164550781
            camera_fx = 915.5391845703125
            camera_fy = 913.7273559570312

        ret, binary = cv2.threshold(data, 0, 255, cv2.THRESH_BINARY)  # 0 or 1?
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        center_x = center_y = center_z = None
        for contour in contours:
            M = cv2.moments(contour)  # 计算第一条轮廓的各阶矩,字典形式
            pixel_center_x = int(M["m10"] / M["m00"])
            pixel_center_y = int(M["m01"] / M["m00"])
            print(pixel_center_x, pixel_center_y)
            # cv2.circle(binary,center=(pixel_center_x,pixel_center_y),radius=2,color=(0,0,0),thickness=-1)
            # cv2.imshow("data",binary)
            # cv2.waitKey(3000)
            # print(binary.shape)
            print(depth_img)
            # print(type(depth_img))

            center_z = depth_img[pixel_center_y, pixel_center_x] * 0.001
            # print(center_z)
            # kinect
            center_x = (pixel_center_x - camera_cx) * center_z / camera_fx
            center_y = (pixel_center_y - camera_cy) * center_z / camera_fy
            rospy.loginfo("x={},y={},z={}".format(center_x, center_y, center_z))
        return center_x, center_y, center_z

    def resolve_instance(self, depth_img, instances: Instances, ctype='kinect', thresh_hold=0.8):
        masks = instances.pred_masks.numpy()
        classes = instances.pred_classes.numpy()
        scores = instances.scores.numpy()
        result_response = ImageProcResponse()
        np.save(os.path.join(debug_dir, "masks.npy"), masks)
        np.save(os.path.join(debug_dir, "classes.npy"), classes)
        np.save(os.path.join(debug_dir, "scores.npy"), scores)
        class_proc = []
        for i in range(len(scores)):
            if scores[i] > thresh_hold:
                binary = np.array(masks[i], dtype=np.uint8)
                class_proc.append(classes[i])
                msg = ImageProcMsg
                x, y, z = self.convert_pos(binary, depth_img, ctype)
                trans_m = self.base_to_object(x, y, z)
                print("target position: ", trans_m[0], trans_m[1], trans_m[2], x, y, z)
                msg.position = [trans_m[0], trans_m[1], trans_m[2]]
                msg.orientation = [0]
                result_response.object.append(msg)
                # TODO 将像素点坐标转换成空间坐标

        result_response.classify = class_proc
        return result_response  # ImageProcResponse([result_msg], [0])

    def convert_pos_rh(self, data):
        ret, binary = cv2.threshold(data, 0, 255, cv2.THRESH_BINARY)  # 0 or 1?
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        center_x = center_y = None
        for contour in contours:
            M = cv2.moments(contour)  # 计算第一条轮廓的各阶矩,字典形式
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])

        return center_x, center_y

    def resolve_instance_rh(self, instances: Instances, thresh_hold=0.8):
        masks = instances.pred_masks.numpy()
        classes = instances.pred_classes.numpy()
        scores = instances.scores.numpy()
        result_response = ImageProcResponse()
        class_proc = []
        for i in range(len(scores)):
            if scores[i] > thresh_hold:
                binary = np.array(masks[i], dtype=np.uint8)
                class_proc.append(classes[i])
                msg = ImageProcMsg
                x, y = self.convert_pos_rh(binary)
                msg.position = [x, y]
                msg.orientation = [0]
                result_response.object.append(msg)

    def kinect_callback(self, depth_data, image_data):
        bridge = self.bridge
        depth = bridge.imgmsg_to_cv2(depth_data, "16UC1")
        image = bridge.imgmsg_to_cv2(image_data, "bgr8")
        self.kinect_image = image
        self.kinect_depth = depth

        # cv2.imshow("depth", depth)
        # cv2.waitKey(10)
        # cv2.imshow("image", image)
        # cv2.waitKey(10)

    def rs_callback(self, depth_data, image_data):
        bridge = self.bridge
        depth = bridge.imgmsg_to_cv2(depth_data, "16UC1")
        image = bridge.imgmsg_to_cv2(image_data, "bgr8")
        self.rs_image = image
        self.rs_depth = depth

    def rh_callback(self, data):
        bridge = self.bridge
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        self.rh_image = image


def displayWebcam():
    rospy.init_node('seg_display', anonymous=True)

    # make a video_object and init the video object
    im = ImageManage()


if __name__ == '__main__':
    displayWebcam()
