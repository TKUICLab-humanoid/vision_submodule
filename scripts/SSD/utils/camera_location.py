import cv2
import numpy as np 
from . import paremeter


def box_center(bounding_box):
    center = [(bounding_box[0]+bounding_box[2])/2,(bounding_box[1]+bounding_box[3])/2]
    print("This box's center is :",center)
    return center

def object_sita(bounding_box,distance):
    x =  paremeter.camera_field[0]/2- box_center(bounding_box)[0]
    y =  paremeter.camera_field[1] - box_center(bounding_box)[1]
    sita = np.arctan(x/y)
    print("x is :",x)
    print("sita is :",round(sita*180/paremeter.pi,3))
    return round(sita*180/paremeter.pi,3)


if __name__ == '__main__':
    bounding_box = [50,50,250,250]
    distance = 200
    object_sita(bounding_box,distance)