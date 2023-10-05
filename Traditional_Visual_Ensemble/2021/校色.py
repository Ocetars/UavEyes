import cv2
import numpy as np
import argparse

# add命令行参数
parser = argparse.ArgumentParser()
# colors
parser.add_argument("--B", type=int, nargs=1, default=0, help="lower green threshold")
parser.add_argument("--G", type=int, nargs=1, default=30, help="upper green threshold")
parser.add_argument("--R", type=int, nargs=1, default=45, help="lower green threshold")
# 解析命令行参数
args = parser.parse_args()

# 读取图片
IMGwrong = cv2.imread(".\\2021\\cut.jpg")
IMGgoal = cv2.imread(".\\2021\\color426.jpg")

cap = cv2.VideoCapture("D:\\Gitworkspace\\UavEyes\\2021\\WIN_20230729_16_24_46_Pro.mp4")
while True:
    ret, frame = cap.read()
    B = args.B
    G = args.G
    R = args.R
    # 自定义LUT表
    LUT = np.zeros((256, 1, 3), dtype=np.uint8)

    LUT[:, 0, 0] = np.clip(np.arange(256) - B, 0, 255)
    LUT[:, 0, 1] = np.clip(np.arange(256) - G, 0, 255)
    LUT[:, 0, 2] = np.clip(np.arange(256) - R, 0, 255)

    # 色温调整
    result = cv2.LUT(IMGwrong, LUT)
    # result = cv2.LUT(frame, LUT)

    # 显示调整前后的图片
    # cv2.imshow("Original Image", IMGwrong)
    cv2.imshow("228_after", result)
    cv2.imshow("426", IMGgoal)

    # 等待按键事件
    key = cv2.waitKey(25)
    # 处理按键事件
    if key & 0xFF == ord('q'):
        break
    elif key & 0xFF == ord('B'):
        args.B += 5
    elif key & 0xFF == ord('b'):
        args.B -= 5
    elif key & 0xFF == ord('G'):
        args.G += 5
    elif key & 0xFF == ord('g'):
        args.G -= 5
    elif key & 0xFF == ord('R'):
        args.R += 5
    elif key & 0xFF == ord('r'):
        args.R -= 5
    elif key & 0xFF == ord('p'):
        print('--------')
        print("B:", B)
        print("G:", args.G)
        print("R:", args.R)
        print('--------')
cv2.destroyAllWindows()