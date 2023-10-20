#!/usr/bin/env python3.5
import rospy

import threading
from std_msgs.msg import String

from __future__ import division
from function import *
import os
import cv2
import numpy as np
import pickle
import time
import tensorflow as tf
import function
from keras import backend as K
from keras.layers import Input
from keras.models import Model
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
####################################
def predict_single_image(img_path, model_rpn, model_classifier_only, cfg, class_mapping):
    st = time.time()
    img = img_path
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
    result_path = './results_images/{}.png'.format(os.path.basename(img_path).split('.')[0])
    print('result saved into ', result_path)
    cv2.imwrite(result_path, img)
    cv2.waitKey(0)
    
    # Output
    global img_kinect
    global detect_or_not
    detect_or_not = False
    
    for i in mybox:
        if i[0]==3:
            detect_or_not = True
            img_kinect = img_path[int(i[3]):int(i[5]),int(i[2]):int(i[4])]
            break

#==================== FRCNN Initialize =================

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

# ============================ Imgae Processing =======================
def Image_process(img_in):
	img = img_in.copy()
	img = cv2.resize(img, (256, 256), interpolation=cv2.INTER_CUBIC)
	img = cv2.Canny(img,50,250,apertureSize = 3)
	img = cv2.GaussianBlur(img, (3, 3), 0)
	ret1,img = cv2.threshold(img,10,255,cv2.THRESH_BINARY)
	cv2.imwrite("./image_canny/Canny.jpg",img)
	minLineLength = 200
	maxLineGap = 50
	lines = cv2.HoughLinesP(img,1,np.pi/180,100,minLineLength,maxLineGap)

	for x1,y1,x2,y2 in lines[0]:
    		degree = math.atan((x2-x1)/(y2-y1))
    		return (degree/np.pi*180)

# ============================ Main Program ================================
global mybox

global img_kinect
global detect_or_not

def kinect_callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def kinect_listener():
	rospy.init_node('Project_ai', anonymous=True)
	kinect_rgb = rospy.Subscriber("/camera/rgb/image_color", String, kinect_callback)

if __name__ == '__main__':
	print("Starting program...")
	kinect_listener()
	kinect_rgb = cv2.imread("./image/20191230_211413.jpg")
	predict_single_image(kinect_rgb , model_rpn, model_classifier_only, cfg, class_mapping)
	if detect_or_not == True:
		print("Angle={angle}".format(angle=Image_process(img_kinect)))
	
