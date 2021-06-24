import cv2
import numpy as np
from . import paremeter



def other_location(my_location,sita,distance):
    print(sita)
    x_value = int(my_location[0] + distance* np.cos(sita*paremeter.pi/180))
    y_value = int(my_location[1] - distance* np.sin(sita*paremeter.pi/180))
    print("other robot's (x,y) = ",x_value,y_value)
    return x_value,y_value

if __name__ == '__main__':
    other_location(30)
