#!/usr/bin/env python3.5
from __future__ import division
from function import *
from std_msgs.msg import String
import rospy
import threading
import os
import numpy as np
import pickle
import time
import tensorflow as tf
import function
from keras import backend as K
from keras.layers import Input
from keras.models import Model

import sys
rospath = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if rospath in sys.path:
	sys.path.remove(rospath)
import cv2
sys.path.append(rospath)

import argparse
import os

#=====Function=======
def format_img_size(img, cfg):
    """ formats the image size based on config """
    img_min_side = float(cfg.im_size)
    (height, width, _) = img.shape

    if width <= height:
        ratio = img_min_side / width
        new_height = int(ratio * height)
        new_width = int(img_min_side)
    else:
        ratio = img_min_side / height
        new_width = int(ratio * width)
        new_height = int(img_min_side)
    img = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
    return img, ratio


def format_img_channels(img, cfg):
    """ formats the image channels based on config """
    img = img[:, :, (2, 1, 0)]
    img = img.astype(np.float32)
    img[:, :, 0] -= cfg.img_channel_mean[0]
    img[:, :, 1] -= cfg.img_channel_mean[1]
    img[:, :, 2] -= cfg.img_channel_mean[2]
    img /= cfg.img_scaling_factor
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    return img


def format_img(img, C):
    """ formats an image for model prediction based on config """
    img, ratio = format_img_size(img, C)
    img = format_img_channels(img, C)
    return img, ratio


# Method to transform the coordinates of the bounding box to its original size
def get_real_coordinates(ratio, x1, y1, x2, y2):
    real_x1 = int(round(x1 // ratio))
    real_y1 = int(round(y1 // ratio))
    real_x2 = int(round(x2 // ratio))
    real_y2 = int(round(y2 // ratio))

    return real_x1, real_y1, real_x2, real_y2


####################################
global mybox
global Move_obj
####################################
def predict_single_image(img_path, model_rpn, model_classifier_only, cfg, class_mapping):
    st = time.time()
    img = img_path.copy()
    if img is None:
        print('reading image failed.')
        exit(0)

    X, ratio = format_img(img, cfg)
    if K.image_dim_ordering() == 'tf':
        X = np.transpose(X, (0, 2, 3, 1))
    # get the feature maps and output from the RPN
    [Y1, Y2, F] = model_rpn.predict(X)

    # this is result contains all boxes, which is [x1, y1, x2, y2]
    result = rpn_to_roi(Y1, Y2, cfg, K.image_dim_ordering(), overlap_thresh=0.9)
    
    ## test
    global mybox
    mybox = []
    # convert from (x1,y1,x2,y2) to (x,y,w,h)
    result[:, 2] -= result[:, 0]
    result[:, 3] -= result[:, 1]
    bbox_threshold = 0.8

    # apply the spatial pyramid pooling to the proposed regions
    boxes = dict()
    for jk in range(result.shape[0] // cfg.num_rois + 1):
        rois = np.expand_dims(result[cfg.num_rois * jk:cfg.num_rois * (jk + 1), :], axis=0)
        if rois.shape[1] == 0:
            break
        if jk == result.shape[0] // cfg.num_rois:
            # pad R
            curr_shape = rois.shape
            target_shape = (curr_shape[0], cfg.num_rois, curr_shape[2])
            rois_padded = np.zeros(target_shape).astype(rois.dtype)
            rois_padded[:, :curr_shape[1], :] = rois
            rois_padded[0, curr_shape[1]:, :] = rois[0, 0, :]
            rois = rois_padded

        [p_cls, p_regr] = model_classifier_only.predict([F, rois])

        for ii in range(p_cls.shape[1]):
            if np.max(p_cls[0, ii, :]) < bbox_threshold or np.argmax(p_cls[0, ii, :]) == (p_cls.shape[2] - 1):
                continue

            cls_num = np.argmax(p_cls[0, ii, :])
            if cls_num not in boxes.keys():
                boxes[cls_num] = []
            (x, y, w, h) = rois[0, ii, :]
            try:
                (tx, ty, tw, th) = p_regr[0, ii, 4 * cls_num:4 * (cls_num + 1)]
                tx /= cfg.classifier_regr_std[0]
                ty /= cfg.classifier_regr_std[1]
                tw /= cfg.classifier_regr_std[2]
                th /= cfg.classifier_regr_std[3]
                x, y, w, h = apply_regr(x, y, w, h, tx, ty, tw, th)
            except Exception as e:
                print(e)
                pass
            boxes[cls_num].append(
                [cfg.rpn_stride * x, cfg.rpn_stride * y, cfg.rpn_stride * (x + w), cfg.rpn_stride * (y + h),
                 np.max(p_cls[0, ii, :])])
    # add some nms to reduce many boxes
    for cls_num, box in boxes.items():
        boxes_nms = non_max_suppression_fast(box, overlap_thresh=0.5)
        boxes[cls_num] = boxes_nms
        print(class_mapping[cls_num] + ":")
        for b in boxes_nms:
            b[0], b[1], b[2], b[3] = get_real_coordinates(ratio, b[0], b[1], b[2], b[3])
            mybox.append([cls_num,b[-1],b[0], b[1], b[2], b[3]])
            print('{} prob: {}'.format(b[0: 4], b[-1]))
    
    # Draw
    img = draw_boxes_and_label_on_image_cv2(img, class_mapping, boxes)
    print('Elapsed time = {}'.format(time.time() - st))
    
    
    # Save
    result_path = './results_images/result.jpg'
    print('result saved into ', result_path)
    cv2.imwrite(result_path, img)
    cv2.waitKey(0)
    
    # Output
    global img_kinect
    global detect_or_not
    global Move_obj
    detect_or_not = False
    
    for i in mybox:
        if i[0]==0:
            Move_obj = i
            detect_or_not = True
            img_kinect = img_path[int(i[3]):int(i[5]),int(i[2]):int(i[4])]
            break
# ============================ Imgae Processing =======================
def Image_process(img_in):
	img = img_in.copy()
	#img = cv2.resize(img, (256, 256), interpolation=cv2.INTER_CUBIC)
	img = cv2.Canny(img,50,250,apertureSize = 3)
	#img = cv2.GaussianBlur(img, (3, 3), 0)
	ret1,img = cv2.threshold(img,10,255,cv2.THRESH_BINARY)
	cv2.imwrite("./image_canny/Canny.jpg",img)
	minLineLength = 3
	maxLineGap = 1
	lines = cv2.HoughLinesP(img,1,np.pi/180,5,minLineLength,maxLineGap)

	for x1,y1,x2,y2 in lines[0]:
    		degree = math.atan((x2-x1)/(y2-y1))
    		return (degree)

#==================== FRCNN Initialize =================
global mybox

global img_kinect
global detect_or_not

#####################
input_layer_num = 1024
#####################
with open('config.pickle', 'rb') as f_in:
    cfg = pickle.load(f_in)
cfg.use_horizontal_flips = False
cfg.use_vertical_flips = False
cfg.rot_90 = False

class_mapping = cfg.class_mapping
if 'bg' not in class_mapping:
    class_mapping['bg'] = len(class_mapping)

class_mapping = {v: k for k, v in class_mapping.items()}
input_shape_img = (None, None, 3)
input_shape_features = (None, None, input_layer_num)

img_input = Input(shape=input_shape_img)
roi_input = Input(shape=(cfg.num_rois, 4))
feature_map_input = Input(shape=input_shape_features)

shared_layers = nn_base(img_input, trainable=True)

# define the RPN, built on the base layers
num_anchors = len(cfg.anchor_box_scales) * len(cfg.anchor_box_ratios)
rpn_layers = rpn(shared_layers, num_anchors)
classifier1 = classifier(feature_map_input, roi_input, cfg.num_rois, nb_classes=len(class_mapping),
	                       trainable=True)
model_rpn = Model(img_input, rpn_layers)
model_classifier_only = Model([feature_map_input, roi_input], classifier1)

model_classifier = Model([feature_map_input, roi_input], classifier1)

print('Loading weights from {}'.format(cfg.model_path))
model_rpn.load_weights(cfg.model_path, by_name=True)
model_classifier.load_weights(cfg.model_path, by_name=True)

model_rpn.compile(optimizer='sgd', loss='mse')
model_classifier.compile(optimizer='sgd', loss='mse')


#==================== Main =================
rospy.init_node('Object_detect',anonymous=True)
pub = rospy.Publisher('Object_cordinate', String, queue_size=0)

while True:
	img = cv2.imread("./tmp_img/capture.jpg")
	predict_single_image(img , model_rpn, model_classifier_only, cfg, class_mapping)
	
	#pkg = ",{pos_x},{pos_y},{angle},".format(pos_x = 23.2/1000, pos_y = 281/1000,angle=-3.14*10/180)
	#pub.publish(pkg)
	#rospy.sleep(5)
	

	if detect_or_not == True:
		print("found something!")
		y = 2.54*((Move_obj[3]+Move_obj[5])/2 - 240)
		x = 2.74*(320 -(Move_obj[2]+Move_obj[4])/2)
		theta = Image_process(img_kinect)

		pkg = [x/1000,y/1000,-theta]
		print("Position at ({pos_x},{pos_y}), and angle = {angle}".format(pos_x = pkg[0], pos_y = pkg[1],angle=pkg[2]) )
		pkg = ",{pos_x},{pos_y},{angle},".format(pos_x = pkg[0], pos_y = pkg[1],angle=pkg[2])
		pub.publish(pkg)

	else:
		print("found nothing!")
		False

	
