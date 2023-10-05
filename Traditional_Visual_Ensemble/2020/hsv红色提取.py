import cv2
import numpy as np


def extract_red(pic):
    """method1:使用inRange方法,拼接mask0,mask1"""

    img = cv2.imdecode(np.fromfile(pic, dtype=np.uint8), -1)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    rows, cols, channels = img.shape
    # 区间1
    lower_red = np.array([0, 43, 46])
    upper_red = np.array([10, 255, 255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    # 区间2
    lower_red = np.array([156, 43, 46])
    upper_red = np.array([180, 255, 255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    # 拼接两个区间
    mask = mask0 + mask1
    # 保存图片
    cv2.imencode(".png", mask)[1].tofile(pic)
