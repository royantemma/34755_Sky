import cv2 as cv
import numpy as np
from aruco import get_processor

ap = get_processor()
print("mtx:", ap.mtx)
print("R_cam2rob:\n", ap.R_cam2rob)
print("T_cam2rob:\n", ap.T_cam2rob)

