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
import interfaces as ifaces

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

class FPSCounter():
    def __init__(self):
        self.start = time.time()
        self.counter = 0
    def count(self, period=1):
        self.counter += 1
        if time.time() - self.start > period:
            print("Skeleton Detector - Freq -> ", self.counter, "Hz")
            self.counter = 0
            self.start = time.time()

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 50
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

        self.model = 'densenet'
        self.IMPUT_WIDTH = None
        self.INPUT_HEIGHT = None
        self.pipeline = None
        self.profile = None
        self.depth_profile = None
        self.MODEL_WIDTH = None
        self.MODEL_HEIGHT = None
        self.model_trt = None
        self.parse_objects = None
        self.peoplelist = None
        self.source_img = None
        self.skeleton_img = None
        self.human_pose = None
        self.skeleton_points = []
        self.skeleton_white = []
        self.act_people = None

        self.initialize()   

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        try:
            self.display = params["display"] == "true" or params["display"] == "True"
        except:
        #	traceback.print_exc()
        	print("Error reading config params")
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

        # time
        self.fps = FPSCounter()

    @QtCore.Slot()
    def compute(self):
        rgbd = self.camerargbdsimple_proxy.getAll("camera_top")
        self.act_people = self.people_obtainer(rgbd.image, rgbd.depth)
        if self.display:
            cv2.imshow("color", self.skeleton_img)
        self.fps.count()

################################################################################
    def people_obtainer(self, color, depth):
        image = np.frombuffer(color.image, np.uint8).reshape(color.height, color.width, color.depth)
        depth_image = np.frombuffer(depth.depth, np.float32).reshape(depth.height, depth.width, 1)

        # self.t1 = time.time()
        center_x = color.width / 2
        center_y = color.height / 2

        # copy to a squared image so the resize does not warps the image
        # square_img = np.zeros([color.height, color.height, 3], np.uint8)
        # x_offset = (color.height - color.width)//2
        # square_img[0:image.shape[0], x_offset:x_offset + image.shape[1]] = image
        img = cv2.resize(image, dsize=(int(self.MODEL_WIDTH), int(self.MODEL_HEIGHT)), interpolation=cv2.INTER_AREA)
        counts, objects, peaks = self.execute(img)

        keypoints_names = self.human_pose["keypoints"]
        if self.display:
            self.skeleton_img = copy.deepcopy(image)
        people = ifaces.RoboCompHumanCameraBody.PeopleData()
        people.timestamp = time.time()
        people.peoplelist = []
        for i in range(counts[0]):
            new_person = ifaces.RoboCompHumanCameraBody.Person()
            new_person.id = i
            TJoints = {}
            bounding_list = []
            keypoints = self.get_keypoint(objects, i, peaks)
            for kpoint in range(len(keypoints)):
                key_point = ifaces.RoboCompHumanCameraBody.KeyPoint()
                if keypoints[kpoint][1]:
                    key_point.i = int(keypoints[kpoint][2] * color.width)  # camera is vertical
                    key_point.j = int(keypoints[kpoint][1] * color.height)
                    key_point.y = float(depth_image[key_point.j, key_point.i])
                    key_point.z = -(key_point.y / depth.focalx) * (key_point.j - center_x)
                    key_point.x = (key_point.y / depth.focaly) * (key_point.i - center_y)
                    #print(key_point.x, key_point.y, key_point.z, keypoints_names[kpoint])
                    TJoints[str(kpoint)] = key_point
                    bounding_list.append([key_point.i, key_point.j])
                    if self.display:
                        cv2.circle(self.skeleton_img, (key_point.i, key_point.j), 1, [0, 255, 0], 2)

            # compute ROI
            if len(bounding_list) > 0:
                bx, by, bw, bh = cv2.boundingRect(np.array(bounding_list))
                new_person.roi = ifaces.RoboCompHumanCameraBody.TImage()
                temp_img = image[by:by+bh, bx:bx+bw]
                new_person.roi.width = temp_img.shape[0]
                new_person.roi.height = temp_img.shape[1]
                new_person.roi.image = temp_img.tobytes()
            new_person.joints = TJoints
            people.peoplelist.append(new_person)
        return people

    def preprocess(self, image):
        global device
        image = PIL.Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(device)
        image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        return image[None, ...]

    def execute(self, img):
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

    def draw_arrow(self, image, i_w, i_h, ratio, xa, ya, xC, yC, xb, yb):
        tlx = int(ratio * (xa)) + int(i_w / 2)
        tly = int(i_h) - int(ratio * (ya))
        brx = int(ratio * (xb)) + int(i_w / 2)
        bry = int(i_h) - int(ratio * (yb))
        mx = int(ratio * (xC)) + int(i_w / 2)
        my = int(i_h) - int(ratio * (yC))
        cv2.line(image, (tlx, tly), (mx, my), (255, 0, 0), 3)
        cv2.line(image, (mx, my), (brx, bry), (0, 0, 255), 3)

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

    ###################################################################
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