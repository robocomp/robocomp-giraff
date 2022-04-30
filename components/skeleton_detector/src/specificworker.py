#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
# from filterpy.common import Q_discrete_white_noise
# from filterpy.kalman import ExtendedKalmanFilter
from numpy import eye, array, asarray
import numpy as np
import os
import time
import cv2
import json
import trt_pose.coco
import trt_pose.models
import copy
import torch
import torch2trt
from torch2trt import TRTModule
import torchvision.transforms as transforms
import PIL.Image
from trt_pose.parse_objects import ParseObjects
device = torch.device('cuda')

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 10
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

        self.model = 'densenet'
        self.IMPUT_WIDTH  = 640
        self.INPUT_HEIGHT = 480
        self.rotate = True        

        self.pipeline = None
        self.profile = None
        self.depth_profile = None
        self.MODEL_WIDTH  = None
        self.MODEL_HEIGHT = None
        self.mean = None
        self.std = None
        self.model_trt = None
        self.parse_objects = None
        self.peoplelist = None
        self.source_img = None
        self.skeleton_img = None
        self.human_pose = None
        self.skeleton_points = []
        self.skeleton_white = []

        # self.v = 1.0/self.Period
        # self.a = 0.5 * (self.v ** 2)
        # self.kalman = cv2.KalmanFilter(9, 3, 0)
        # self.kalman.measurementMatrix = np.array([
        #     [1, 0, 0, self.v, 0, 0, self.a, 0, 0],
        #     [0, 1, 0, 0, self.v, 0, 0, self.a, 0],
        #     [0, 0, 1, 0, 0, self.v, 0, 0, self.a]
        # ], np.float32)
        #
        # self.kalman.transitionMatrix = np.array([
        #     [1, 0, 0, self.v, 0, 0, self.a, 0, 0],
        #     [0, 1, 0, 0, self.v, 0, 0, self.a, 0],
        #     [0, 0, 1, 0, 0, self.v, 0, 0, self.a],
        #     [0, 0, 0, 1, 0, 0, self.v, 0, 0],
        #     [0, 0, 0, 0, 1, 0, 0, self.v, 0],
        #     [0, 0, 0, 0, 0, 1, 0, 0, self.v],
        #     [0, 0, 0, 0, 0, 0, 1, 0, 0],
        #     [0, 0, 0, 0, 0, 0, 0, 1, 0],
        #     [0, 0, 0, 0, 0, 0, 0, 0, 1]
        # ], np.float32)
        #
        # self.kalman.processNoiseCov = np.array([
        #     [1, 0, 0, 0, 0, 0, 0, 0, 0],
        #     [0, 1, 0, 0, 0, 0, 0, 0, 0],
        #     [0, 0, 1, 0, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 1, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 1, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 0, 1, 0, 0, 0],
        #     [0, 0, 0, 0, 0, 0, 1, 0, 0],
        #     [0, 0, 0, 0, 0, 0, 0, 1, 0],
        #     [0, 0, 0, 0, 0, 0, 0, 0, 1]
        # ], np.float32) * 0.007
        #
        # self.kalman.measurementNoiseCov = np.array([
        #     [1, 0, 0],
        #     [0, 1, 0],
        #     [0, 0, 1]
        # ], np.float32) * 0.1
        #
        # self.kalman.statePre = np.array([
        #       [np.float32(0.)], [np.float32(0.)], [np.float32(0.)]
        #     , [np.float32(0.)], [np.float32(0.)], [np.float32(0.)]
        #     , [np.float32(0.)], [np.float32(0.)], [np.float32(0.)]
        # ])

        # self.dt = 0.05
        # self.rk = ExtendedKalmanFilter(dim_x=3, dim_z=1)
        # self.rk.F = eye(3) + array([[0, 1, 0],
        #                        [0, 0, 0],
        #                        [0, 0, 0]]) * dt
        # self.rk.R = np.diag([1 ** 2])
        # self.rk.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=0.1)
        # self.rk.Q[2, 2] = 0.1
        # self.rk.P *= 50

        self.act_people = None

        self.initialize()   

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    def initialize(self):

        with open('human_pose.json', 'r') as f:
            self.human_pose = json.load(f)

        topology = trt_pose.coco.coco_category_to_topology(self.human_pose)
        num_parts = len(self.human_pose['keypoints'])
        num_links = len(self.human_pose['skeleton'])

        if self.model == 'resnet':
            # print('model = densenet')
            MODEL_WEIGHTS = '../resnet18_baseline_att_224x224_A_epoch_249.pth'
            OPTIMIZED_MODEL = 'resnet18_baseline_att_224x224_A_epoch_249_trt.pth'
            model = trt_pose.models.resnet18_baseline_att(num_parts, 2 * num_links).cuda().eval()
            self.MODEL_WIDTH = 224.
            self.MODEL_HEIGHT = 224.
        elif self.model == 'densenet':
            print('model = densenet')
            MODEL_WEIGHTS = 'densenet121_baseline_att_256x256_B_epoch_160.pth'
            OPTIMIZED_MODEL = 'densenet121_baseline_att_256x256_B_epoch_160_trt.pth'
            model = trt_pose.models.densenet121_baseline_att(num_parts, 2 * num_links).cuda().eval()
            self.MODEL_WIDTH = 256.
            self.MODEL_HEIGHT = 256.

        # Preparamos un modelo optimizado si no existe. Si si existe, solo lo cargamos
        data = torch.zeros((1, 3, int(self.MODEL_HEIGHT), int(self.MODEL_WIDTH))).cuda()
        if os.path.exists(OPTIMIZED_MODEL) == False:
            print("Preparando modelo optimizado. Espere...")
            model.load_state_dict(torch.load(MODEL_WEIGHTS))
            self.model_trt = torch2trt.torch2trt(model, [data], fp16_mode=True, max_workspace_size=1 << 25)
            torch.save(self.model_trt.state_dict(), OPTIMIZED_MODEL)

        self.model_trt = TRTModule()
        self.model_trt.load_state_dict(torch.load(OPTIMIZED_MODEL))

        # Popurri extraño quer aun no se sabe que hace :p
        self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
        self.std = torch.Tensor([0.229, 0.224, 0.225]).cuda()

        self.parse_objects = ParseObjects(topology)

    @QtCore.Slot()
    def compute(self):
        color = self.camerargbdsimple_proxy.getImage("camera_top")
        depth = self.camerargbdsimple_proxy.getDepth("camera_top")

        self.act_people = self.peopleObtainer(color, depth)

        return True

    def peopleObtainer(self, color, depth):
        ##random_color = fg(CColor(PMcR))
        ##print(random_color+"=======================  " +fg(15)+  "computePilar"  +random_color+ "  =======================" + G)

        image = np.frombuffer(color.image, np.uint8).reshape(color.height, color.width, color.depth)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_image = np.frombuffer(depth.depth, np.float32).reshape(depth.height, depth.width, 1)
        depth_image = cv2.rotate(depth_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_plot = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

        # self.t1 = time.time()
        # frames = self.pipeline.wait_for_frames()
        # color_frame = frames.get_color_frame()
        # color_image = np.asanyarray(color_frame.get_data())
        # Rotamos
        #

        # depth_frame = frames.get_depth_frame()
        # depth_data = np.asanyarray(depth_frame.get_data())
        # Rotamos
        # depth_data = cv2.rotate(depth_data, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # self.skeleton_image = np.zeros([480,640,1],dtype=np.float32)

        # self.depth_img = copy.deepcopy(depth_data)
        # depth_sensor = self.profile.get_device().first_depth_sensor()
        # depth_scale = depth_sensor.get_depth_scale()

        Center_X = color.width / 2
        Center_Y = color.height / 2


        # depth_intrin = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        # color_intrin = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        # depth_to_color_extrin = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(
        #     self.profile.get_stream(rs.stream.color))
        # color_to_depth_extrin = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(
        #     self.profile.get_stream(rs.stream.depth))

        r_s = 0
        r_w = self.IMPUT_WIDTH
        c_s = 0
        c_w = self.INPUT_HEIGHT

        cropped = image[int(r_s):int(r_s + r_w), int(c_s):int(c_s + c_w)]
        img = cv2.resize(cropped, dsize=(int(self.MODEL_WIDTH), int(self.MODEL_HEIGHT)), interpolation=cv2.INTER_AREA)
        self.source_img = img  # candidata para enviar al componente remoto
        counts, objects, peaks = self.execute(img, image)

        # A_matrix = np.array([[depth.focalx, 0, Center_X],
        #                     [0, depth.focaly, Center_Y],
        #                     [0, 0, 1]])

        keypoints_names = self.human_pose["keypoints"]
        self.skeleton_img = copy.deepcopy(image)
        # color = (0, 255, 0)
        people = RoboCompHumanCameraBody.PeopleData()
        people.timestamp = time.time()
        people.peoplelist = []
        for i in range(counts[0]):
            new_person = RoboCompHumanCameraBody.Person()
            new_person.id = i
            TJoints = {}
            keypoints = self.get_keypoint(objects, i, peaks)

            for j in range(len(keypoints)):
                key_point = RoboCompHumanCameraBody.KeyPoint()
                if keypoints[j][1]:
                    # print(keypoints_names[j])
                    key_point.j = int(keypoints[j][1] * r_w + r_s)
                    key_point.i = int(keypoints[j][2] * c_w + c_s)

                    print(j)
                    # if(keypoints_names[j] == "nose"):

                        # print("COLOR VALUE: ", self.skeleton_img[key_point.j, key_point.i])
                        # print("DEPTH VALUE: ", depth_plot[key_point.j, key_point.i])
                        # print(depth_image[key_point.i, key_point.j])

                    key_point.y = float(depth_image[key_point.j, key_point.i])
                    key_point.z = (key_point.y / depth.focalx) * (key_point.j - Center_X)
                    key_point.x = (key_point.y / depth.focaly) * (key_point.i - Center_Y)
                    print(key_point.x, key_point.y, key_point.z)
                    TJoints[str(j)] = key_point
                        # cv2.circle(depth_plot, (key_point.i, key_point.j), 1, color, 2)
                        # cv2.circle(self.skeleton_img, (key_point.i, key_point.j), 1, color, 2)
                        # cv2.putText(self.skeleton_img, "%d" % int(keypoints[j][0]), (key_point.j + 5, key_point.i),  # 3
                        #                                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)

            # print(x_avg, y_avg, z_avg)
            new_person.joints = TJoints
            # print("skeleton size: ", self.skeleton_img.shape[0], self.skeleton_img.shape[1])
            # cv2.imshow("color", self.skeleton_img)
            # cv2.imshow("depth", depth_plot)
            people.peoplelist.append(new_person)
        return people

    def draw_arrow(self, image, i_w, i_h, ratio, xa, ya, xC, yC, xb, yb):
        tlx = int(ratio * (xa)) + int(i_w / 2)
        tly = int(i_h) - int(ratio * (ya))
        brx = int(ratio * (xb)) + int(i_w / 2)
        bry = int(i_h) - int(ratio * (yb))
        mx = int(ratio * (xC)) + int(i_w / 2)
        my = int(i_h) - int(ratio * (yC))
        cv2.line(image, (tlx, tly), (mx, my), (255, 0, 0), 3)
        cv2.line(image, (mx, my), (brx, bry), (0, 0, 255), 3)

    def preprocess(self, image):
        global device
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = PIL.Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(device)
        image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        return image[None, ...]

    def execute(self, img, src):
        data = self.preprocess(img)
        cmap, paf = self.model_trt(data)
        cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
        counts, objects, peaks = self.parse_objects(cmap, paf)  # , cmap_threshold=0.15, link_threshold=0.15)
        return counts, objects, peaks

    def get_keypoint(self, humans, hnum, peaks):
        kpoint = []
        human = humans[0][hnum]
        C = human.shape[0]
        for j in range(C):
            k = int(human[j])
            if k >= 0:
                peak = peaks[0][j][k]  # peak[1]:width, peak[0]:height
                peak = (j, float(peak[0]), float(peak[1]))
                # print(j, peak)
                kpoint.append(peak)
            else:
                peak = (j, None, None)
                kpoint.append(peak)
        return kpoint

    def cam_matrix_from_intrinsics(self, i):
        return np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]])

    def ekfilter(self, z, updateNumber):
        dt = 1.0
        j = updateNumber
        # Initialize State
        if updateNumber == 0:  # First Update
            # compute position values from measurements
            # x = r*sin(b)
            temp_x = z[0][j] * np.sin(z[1][j] * np.pi / 180)
            # y = r*cos(b)
            temp_y = z[0][j] * np.cos(z[1][j] * np.pi / 180)
            # state vector
            # - initialize position values
            ekfilter.x = np.array([[temp_x],
                                   [temp_y],
                                   [0],
                                   [0]])
            # state covariance matrix
            # - initialized to zero for first update
            ekfilter.P = np.array([[0, 0, 0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0, 0]])
            # state transistion matrix
            # - linear extrapolation assuming constant velocity
            ekfilter.A = np.array([[1, 0, dt, 0],
                                   [0, 1, 0, dt],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])
            # measurement covariance matrix
            ekfilter.R = z[2][j]
            # system error matrix
            # - initialized to zero matrix for first update
            ekfilter.Q = np.array([[0, 0, 0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0, 0]])
            # residual and kalman gain
            # - not computed for first update
            # - but initialized so it could be output
            residual = np.array([[0, 0],
                                 [0, 0]])
            K = np.array([[0, 0],
                          [0, 0],
                          [0, 0],
                          [0, 0]])
        # Reinitialize State
        if updateNumber == 1:  # Second Update
            prev_x = ekfilter.x[0][0]
            prev_y = ekfilter.x[1][0]
            # x = r*sin(b)
            temp_x = z[0][j] * np.sin(z[1][j] * np.pi / 180)
            # y = r*cos(b)
            temp_y = z[0][j] * np.cos(z[1][j] * np.pi / 180)
            temp_xv = (temp_x - prev_x) / dt
            temp_yv = (temp_y - prev_y) / dt
            # state vector
            # - reinitialized with new position and computed velocity
            ekfilter.x = np.array([[temp_x],
                                   [temp_y],
                                   [temp_xv],
                                   [temp_yv]])
            # state covariance matrix
            # - initialized to large values
            # - more accurate position values can be used based on measurement
            #   covariance but this example does not go that far
            ekfilter.P = np.array([[100, 0, 0, 0],
                                   [0, 100, 0, 0],
                                   [0, 0, 250, 0],
                                   [0, 0, 0, 250]])
            # state transistion matrix
            # - linear extrapolation assuming constant velocity
            ekfilter.A = np.array([[1, 0, dt, 0],
                                   [0, 1, 0, dt],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])
            # measurement covariance matrix
            # - provided by the measurment source
            ekfilter.R = z[2][j]
            # system error matrix
            # - adds 4.5 meter std dev in x and y position to state covariance
            # - adds 2 meters per second std dev in x and y velocity to state covariance
            # - these values are not optimized but work for this example
            ekfilter.Q = np.array([[20, 0, 0, 0],
                                   [0, 20, 0, 0],
                                   [0, 0, 4, 0],
                                   [0, 0, 0, 4]])
            # residual and kalman gain
            # - not computed for first update
            # - but initialized so it could be output
            residual = np.array([[0, 0],
                                 [0, 0]])
            K = np.array([[0, 0],
                          [0, 0],
                          [0, 0],
                          [0, 0]])
        if updateNumber > 1:  # Third + Updates
            # Predict State Forward
            x_prime = ekfilter.A.dot(ekfilter.x)
            # Predict Covariance Forward
            P_prime = ekfilter.A.dot(ekfilter.P).dot(ekfilter.A.T) + ekfilter.Q
            # state to measurement transition matrix
            x1 = x_prime[0][0]
            y1 = x_prime[1][0]
            x_sq = x1 * x1
            y_sq = y1 * y1
            den = x_sq + y_sq
            den1 = np.sqrt(den)
            ekfilter.H = np.array([[x1 / den1, y1 / den1, 0, 0],
                                   [y1 / den, -x1 / den, 0, 0]])
            ekfilter.HT = np.array([[x1 / den1, y1 / den],
                                    [y1 / den1, -x1 / den],
                                    [0, 0],
                                    [0, 0]])
            # measurement covariance matrix
            ekfilter.R = z[2][j]
            # Compute Kalman Gain
            S = ekfilter.H.dot(P_prime).dot(ekfilter.HT) + ekfilter.R
            K = P_prime.dot(ekfilter.HT).dot(np.linalg.inv(S))
            # Estimate State
            # temp_z = current measurement in range and azimuth
            temp_z = np.array([[z[0][j]],
                               [z[1][j]]])
            # compute the predicted range and azimuth
            # convert the predicted cartesian state to polar range and azimuth
            pred_x = x_prime[0][0]
            pred_y = x_prime[1][0]
            sumSquares = pred_x * pred_x + pred_y * pred_y
            pred_r = np.sqrt(sumSquares)
            pred_b = np.arctan2(pred_x, pred_y) * 180 / np.pi
            h_small = np.array([[pred_r],
                                [pred_b]])
            # compute the residual
            # - the difference between the state and measurement for that data time
            residual = temp_z - h_small
            # Compute new estimate for state vector using the Kalman Gain
            ekfilter.x = x_prime + K.dot(residual)
            # Compute new estimate for state covariance using the Kalman Gain
            ekfilter.P = P_prime - K.dot(ekfilter.H).dot(P_prime)
        return [ekfilter.x[0], ekfilter.x[1], ekfilter.x[2], ekfilter.x[3], ekfilter.P, K, residual, updateNumber];

    # End of ekfilter

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of newPeopleData method from HumanCameraBody interface
    #
    def HumanCameraBody_newPeopleData(self):
        if self.act_people != None:
            return self.act_people
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompHumanCameraBody you can use this types:
    # RoboCompHumanCameraBody.TImage
    # RoboCompHumanCameraBody.TGroundTruth
    # RoboCompHumanCameraBody.KeyPoint
    # RoboCompHumanCameraBody.Person
    # RoboCompHumanCameraBody.PeopleData


