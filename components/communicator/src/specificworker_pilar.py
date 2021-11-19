#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by Luis J. Manso
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

draw = True
model = 'resnet'
# model = 'densenet'

COLOUR_WIDTH = 848.
COLOUR_HEIGHT = 480.
DEPTH_WIDTH = 848.
DEPTH_HEIGHT = 480.


import time, sys
import cv2
import platform
import sys
import os
import os.path
import copy
import numpy as np



def cam_matrix_from_intrinsics(i):
    return np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]])


import json
import trt_pose.coco
import trt_pose.models
import torch
import torch2trt
from torch2trt import TRTModule
import torchvision.transforms as transforms
import PIL.Image
from trt_pose.parse_objects import ParseObjects
sys.path.append('/usr/local/lib/python3.6/pyrealsense2')
import pyrealsense2 as rs
device = torch.device('cuda')

hostname = platform.node()


if hostname == 'trackera':
    r_s = 23.
    r_w = 352.
    c_s = 176.
    c_w = 526.
elif hostname == 'trackerb':
    r_s = 90.
    r_w = 370.
    c_s = 220.
    c_w = 610.
elif hostname == 'trackerc':
    r_s = 36.
    r_w = 420.
    c_s = 152.
    c_w = 550.
elif hostname == 'trackerd':
    r_s = 15.
    r_w = 400.
    c_s = 68.
    c_w = 590.
else:
    print('Unhandled hostname')
    sys.exit(-1)



from genericworker import *

import time
import platform


def get_keypoint(humans, hnum, peaks):
    kpoint = []
    human = humans[0][hnum]
    C = human.shape[0]
    for j in range(C):
        k = int(human[j])
        if k >= 0:
            peak = peaks[0][j][k]   # peak[1]:width, peak[0]:height
            peak = (j, float(peak[0]), float(peak[1]))
            kpoint.append(peak)
        else:
            peak = (j, None, None)
            kpoint.append(peak)
    return kpoint



with open('../human_pose.json', 'r') as f:
   human_pose = json.load(f)
topology = trt_pose.coco.coco_category_to_topology(human_pose)
num_parts = len(human_pose['keypoints'])
num_links = len(human_pose['skeleton'])

if model == 'resnet':
    MODEL_WEIGHTS = '../resnet18_baseline_att_224x224_A_epoch_249.pth'
    OPTIMIZED_MODEL = 'resnet18_baseline_att_224x224_A_epoch_249_trt.pth'
    model = trt_pose.models.resnet18_baseline_att(num_parts, 2 * num_links).cuda().eval()
    WIDTH = 224.
    HEIGHT = 224.
elif model == 'densenet':
    print('------ model = densenet--------')
    MODEL_WEIGHTS = '../densenet121_baseline_att_256x256_B_epoch_160.pth'
    OPTIMIZED_MODEL = 'densenet121_baseline_att_256x256_B_epoch_160_trt.pth'
    model = trt_pose.models.densenet121_baseline_att(num_parts, 2 * num_links).cuda().eval()
    WIDTH = 256.
    HEIGHT = 256.

data = torch.zeros((1, 3, int(HEIGHT), int(WIDTH))).cuda()
if os.path.exists(OPTIMIZED_MODEL) == False:
    model.load_state_dict(torch.load(MODEL_WEIGHTS))
    model_trt = torch2trt.torch2trt(model, [data], fp16_mode=True, max_workspace_size=1<<25)
    torch.save(model_trt.state_dict(), OPTIMIZED_MODEL)

model_trt = TRTModule()
model_trt.load_state_dict(torch.load(OPTIMIZED_MODEL))


mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
std = torch.Tensor([0.229, 0.224, 0.225]).cuda()


def preprocess(image):
    global device
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = PIL.Image.fromarray(image)
    image = transforms.functional.to_tensor(image).to(device)
    image.sub_(mean[:, None, None]).div_(std[:, None, None])
    return image[None, ...]

parse_objects = ParseObjects(topology)
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, int(COLOUR_WIDTH), int(COLOUR_HEIGHT), rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, int(DEPTH_WIDTH), int(DEPTH_HEIGHT), rs.format.z16, 30)
pc = rs.pointcloud()
points = rs.points()
pipeline.start(config)
profile = pipeline.get_active_profile()
print(profile)
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
print(depth_profile)
depth_intrinsics = depth_profile.get_intrinsics()
print(depth_intrinsics)

def execute(img, src, t):
    data = preprocess(img)
    cmap, paf = model_trt(data)
    cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
    counts, objects, peaks = parse_objects(cmap, paf)#, cmap_threshold=0.15, link_threshold=0.15)
    return counts, objects, peaks

def draw_arrow(image, i_w, i_h, ratio, xa, ya, xC, yC, xb, yb):
    tlx = int(ratio*(xa))+int(i_w/2)
    tly = int(i_h)-int(ratio*(ya))
    brx = int(ratio*(xb))+int(i_w/2)
    bry = int(i_h)-int(ratio*(yb))
    mx = int(ratio*(xC))+int(i_w/2)
    my = int(i_h)-int(ratio*(yC))
    cv2.line(image, (tlx,tly),( mx, my),(255,0,0),3)
    cv2.line(image, (mx, my),(brx,bry),(0,0,255),3)

depth_to_color_extrin =  profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to( profile.get_stream(rs.stream.color))
color_to_depth_extrin =  profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to( profile.get_stream(rs.stream.depth))

last_sent = time.time()

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.host = platform.node()
        while True:
            time.sleep(0.001)
            self.message = self.compute()

            try:
                self.pose_proxy.sendData(self.host, self.message)
            except Ice.ConnectionRefusedException as e:
                print('Cannot connect to host, waiting a few seconds...')
                print(e)
                time.sleep(3)
            except Ice.ConnectionLostException as e:
                print('Cannot connect to host, waiting a few seconds...') 
                print(e)
                time.sleep(3)
            except Ice.ConnectionTimeoutException as e:
                print('timeout')
                print(e)
                time.sleep(3)
            except Ice.UnknownException as e:
                print('Cannot connect to host, waiting a few seconds...')
                print(e)
                time.sleep(3)



    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    def compute(self):
        global last_sent
        data_to_publish = []
        t = time.time()
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_frame = frames.get_depth_frame()
        depth_data = np.asanyarray(depth_frame.get_data())
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        color_intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        depth_to_color_extrin =  profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(profile.get_stream(rs.stream.color))
        color_to_depth_extrin =  profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(profile.get_stream(rs.stream.depth))

        #r_s = 0
        #r_w = COLOUR_HEIGHT
        #c_s = 0
        #c_w = COLOUR_WIDTH

        cropped = color_image[int(r_s):int(r_s+r_w), int(c_s):int(c_s+c_w)]
        img = cv2.resize(cropped, dsize=(int(WIDTH), int(HEIGHT)), interpolation=cv2.INTER_AREA)
        counts, objects, peaks = execute(img, color_image, t)

        if draw:
            ret = copy.deepcopy(color_image)
            color = (0,255,0)
            for i in range(counts[0]):
                keypoints = get_keypoint(objects, i, peaks)
                for j in range(len(keypoints)):
                    if keypoints[j][1]:
                        x = keypoints[j][2]*c_w + c_s
                        y = keypoints[j][1]*r_w + r_s
                        if draw:
                            cv2.circle(ret, (int(x), int(y)), 1, color, 2)
                            cv2.putText(ret , "%d" % int(keypoints[j][0]), (int(x) + 5, int(y)),  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1)

            cv2.imshow("cropped", cropped)

        def c2d(p):
            x,y = p
            depth_min = 0.1
            depth_max = 10.
            depth_point = rs.rs2_project_color_pixel_to_depth_pixel(depth_frame.get_data(), depth_scale, depth_min, depth_max, depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, [x,y])
            depth_point[0] = int(depth_point[0]+0.5)
            depth_point[1] = int(depth_point[1]+0.5)
            return depth_point

        def d2xyz(depth_point):
            depth_min = 0.01
            depth_max = 2.
            if np.any(depth_point == None):
                return False, [0,0,0]
            try:
                ret = True, rs.rs2_deproject_pixel_to_point(depth_intrin, depth_point, depth_scale*depth_data[depth_point[1], depth_point[0]])
                return ret
            except IndexError:
                return False, [0,0,0]


        for i in range(counts[0]):
            keypoints = get_keypoint(objects, i, peaks)
            kps = dict()
            for kp in range(len(keypoints)):
                centre = keypoints[kp]
                if centre[1] and centre[2]:
                    cx = centre[2]*c_w + c_s
                    cy = centre[1]*r_w + r_s
                    #cx = round(centre[2]* COLOUR_WIDTH) # + c_s
                    #cy = round(centre[1]* COLOUR_HEIGHT)# + r_s
                    got3d, c = d2xyz(c2d([int(round(cx-3)),int(round(cy+2))]))
                    kps[kp] = [kp, [cx, cy], c + [float(got3d)] ]
                    if draw:
                        vector1 = np.array([c]).transpose()
                        K = cam_matrix_from_intrinsics(color_intrin)
                        vector2 = K @ vector1
                        vector2 /= vector2[2][0]
                        try:
                            cv2.circle(ret, (int(vector2[0][0])-1, int(vector2[1][0])-1), 1, (0,100,255), 2)
                        except:
                            pass
            data_to_publish.append(kps)

        if draw:
            cv2.imshow("test", ret)
            if cv2.waitKey(1)%256 == 27:
                print("Escape hit, closing...")
                cv2.destroyAllWindows()
                sys.exit(0)

        return json.dumps(data_to_publish)





