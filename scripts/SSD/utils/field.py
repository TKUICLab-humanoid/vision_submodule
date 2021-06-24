import cv2
import numpy as np
from . import field_location


def field():
    field = np.zeros((800, 1100, 3), np.uint8)
    cv2.rectangle(field, (102,102), (998,698),  (255, 255, 255), 3, cv2.LINE_AA)
    cv2.rectangle(field, (102,152), (298,648),  (255, 255, 255), 3, cv2.LINE_AA)
    cv2.rectangle(field, (802,152), (998,648),  (255, 255, 255), 3, cv2.LINE_AA)
    cv2.rectangle(field, (102,252), (198,548),  (255, 255, 255), 3, cv2.LINE_AA)
    cv2.rectangle(field, (902,252), (998,548),  (255, 255, 255), 3, cv2.LINE_AA)
    cv2.circle(field,(550,400),73,(255, 255, 255), 3)
    cv2.line(field,(550,102),(550,698), (255, 255, 255), 3)
    cv2.line(field,(542,400),(558,400), (255, 255, 255), 3)

    cv2.line(field,(250,392),(250,408), (255, 255, 255), 3)
    cv2.line(field,(242,400),(258,400), (255, 255, 255), 3)
    cv2.line(field,(850,392),(850,408), (255, 255, 255), 3)
    cv2.line(field,(842,400),(858,400), (255, 255, 255), 3)
    return field

def locate_myself(field,my_location):
    cv2.circle(field,my_location,5,(0, 255, 255), 10)
    return field

def locate_others(field,my_location,sita,distance):
    x,y = field_location.other_location(my_location,sita,distance)
    cv2.circle(field,(x,y),5,(0, 0, 255), 10)
    return field

