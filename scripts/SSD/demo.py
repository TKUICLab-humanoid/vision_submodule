#!/usr/bin/python3
import sys
sys.path.insert(1, '/home/user_name/.local/lib/python3.6/site-packages/')
sys.path.insert(0, '/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/')

import glob
import os
import time
from numpy.core.numeric import True_

import torch
from PIL import Image
from vizer.draw import draw_boxes

from ssd.config import cfg
from ssd.data.datasets import COCODataset, VOCDataset
import argparse
import numpy as np

from ssd.data.transforms import build_transforms
from ssd.modeling.detector import build_detection_model
from ssd.utils import mkdir
from ssd.utils.checkpoint import CheckPointer
import cv2

import rospy
from tku_msgs.msg import ObjectData
from tku_msgs.msg import DemoData
from sensor_msgs.msg import Image as getImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

global cv_image
global getImageFlag

getImageFlag = False

def imageCallBackFunction(msg):
    global cv_image
    global getImageFlag

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        getImageFlag = True
    except CvBridgeError as e:
        print(e)
        getImageFlag = False

@torch.no_grad()
def run_demo(cfg, ckpt, score_threshold, images_dir, output_dir, dataset_type):
    global cv_image
    global getImageFlag

    if dataset_type == "voc":
        class_names = VOCDataset.class_names
    elif dataset_type == 'coco':
        class_names = COCODataset.class_names
    else:
        raise NotImplementedError('Not implemented now.')
    device = torch.device(cfg.MODEL.DEVICE)

    model = build_detection_model(cfg)
    # model = torch.load(ckpt)
    # model = torch.load('outputs/pruning/320/pruning_rate_0.25/strategy/pretraining_model.pth') # no load_state_dict
    model = model.to(device)
    checkpointer = CheckPointer(model, save_dir=cfg.OUTPUT_DIR)
    checkpointer.load(ckpt, use_latest=ckpt is None)
    
    weight_file = ckpt if ckpt else checkpointer.get_checkpoint_file()
    print('Loaded weights from {}'.format(weight_file))



    cpu_device = torch.device("cpu")
    transforms = build_transforms(cfg, is_train=False)
    model.eval()

    # cap = cv2.VideoCapture(0)
    fps_count = 0
    text = '0'
    text_sum=0
    text_sum_index = 0

    rospy.init_node('demo', anonymous=True)
    demodata_pub = rospy.Publisher("/vision/demodata", DemoData, queue_size=1000)
    rospy.Subscriber("/camera/color/image_raw", getImage, imageCallBackFunction)

    objectdata = ObjectData()
    demodata = DemoData()

    while not rospy.is_shutdown(): # and cap.isOpened():
        if getImageFlag == True:
            # _, frame = cap.read()

            start = time.time()
        
            # image = Image.fromarray(frame)
            # height, width = frame.shape[:2]
            # images = transforms(frame)[0].unsqueeze(0)
            
            image = Image.fromarray(cv_image)
            height, width = cv_image.shape[:2]
            images = transforms(cv_image)[0].unsqueeze(0)
            load_time = time.time() - start

            start = time.time()
            result = model(images.to(device))[0]
            inference_time = time.time() - start

            result = result.resize((width, height)).to(cpu_device).numpy()
            boxes, labels, scores = result['boxes'], result['labels'], result['scores']

            indices = scores > score_threshold
            boxes = boxes[indices]
            labels = labels[indices]
            scores = scores[indices]
            
            if boxes.shape[0] > 0:
                for i in range(0, boxes.shape[0]):
                    if boxes[i][0] < 0:
                        boxes[i][0] = 0.0
                    if boxes[i][1] < 0:
                        boxes[i][1] = 0.0
                    if boxes[i][2] >= width:
                        boxes[i][2] = width - 1.0
                    if boxes[i][3] >= height:
                        boxes[i][3] = height - 1.0
                    
                    objectdata.x = np.int16(boxes[i][0])
                    objectdata.y = np.int16(boxes[i][1])
                    objectdata.height = np.int16(boxes[i][3] - boxes[i][1])
                    objectdata.width = np.int16(boxes[i][2] - boxes[i][0])
                    objectdata.mode = "partner"
                    demodata.object.append(objectdata)
                
            demodata_pub.publish(demodata)
            demodata.object = []
            
            meters = 'FPS {}'.format(round(1.0 / inference_time))
            print('{}'.format( meters))
            # drawn_image = draw_boxes(image, boxes, labels, scores, class_names).astype(np.uint8)
            # text_sum_index = text_sum_index +1
            # if text_sum_index <=1000:
            #     text_sum = text_sum + round(1.0 / inference_time)
            # else:
            #     text_sum = text_sum/1000
            #     print("avg_fps=",text_sum)
            #     return 0
            # if fps_count == 10:
            #     fps_count = 0
            #     text = str(round(1.0 / inference_time))
            # else:
            #     fps_count = fps_count+1
            # cv2.putText(drawn_image,text, (10, 25), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1, 4)
            # cv2.imshow("aa",drawn_image)
            # cv2.waitKey(1)

def main():
    parser = argparse.ArgumentParser(description="SSD Demo.")
    # parser.add_argument(
    #     "--config-file",
    #     default="",
    #     metavar="FILE",
    #     help="path to config file",
    #     type=str,
    # )
    # parser.add_argument("--ckpt", type=str, default=None, help="Trained weights.")
    parser.add_argument(
        "--config-file",
        default="src/vision/scripts/SSD/configs/mobilenet_v2_prune_320.yaml",
        metavar="FILE",
        help="path to config file",
        type=str,
    )
    parser.add_argument("--ckpt", type=str, default="src/vision/scripts/SSD/model_final.pth", help="Trained weights.")
    parser.add_argument("--score_threshold", type=float, default=0.7)
    parser.add_argument("--images_dir", default='demo', type=str, help='Specify a image dir to do prediction.')
    parser.add_argument("--output_dir", default='demo/result', type=str, help='Specify a image dir to save predicted images.')
    parser.add_argument("--dataset_type", default="voc", type=str, help='Specify dataset type. Currently support voc and coco.')

    parser.add_argument(
        "opts",
        help="Modify config options using the command-line",
        default=None,
        nargs=argparse.REMAINDER,
    )
    args = parser.parse_args()
    print(args)

    cfg.merge_from_file(args.config_file)
    cfg.merge_from_list(args.opts)
    cfg.freeze()

    print("Loaded configuration file {}".format(args.config_file))
    with open(args.config_file, "r") as cf:
        config_str = "\n" + cf.read()
        print(config_str)
    print("Running with config:\n{}".format(cfg))

    run_demo(cfg=cfg,
             ckpt=args.ckpt,
             score_threshold=args.score_threshold,
             images_dir=args.images_dir,
             output_dir=args.output_dir,
             dataset_type=args.dataset_type)


if __name__ == '__main__':
    main()
