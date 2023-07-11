#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np
import argparse

# add命令行参数
parser = argparse.ArgumentParser()
# colors
parser.add_argument("--lower_green", type=int, nargs=3, default=[100, 50, 35], help="lower green threshold")
parser.add_argument("--upper_green", type=int, nargs=3, default=[117, 255, 220], help="upper green threshold")
parser.add_argument("--lower_red", type=int, nargs=3, default=[0, 50, 50], help="lower red threshold")
parser.add_argument("--upper_red", type=int, nargs=3, default=[5, 255, 255], help="upper red threshold")
parser.add_argument("--lower_red2", type=int, nargs=3, default=[245, 50, 50], help="lower red threshold 2")
parser.add_argument("--upper_red2", type=int, nargs=3, default=[260, 255, 255], help="upper red threshold 2")

parser.add_argument("--erode_iterations", type=int, default=2, help="number of erode iterations")
parser.add_argument("--green_dilate_iterations", type=int, default=3, help="number of green dilate iterations")
parser.add_argument("--red_dilate_iterations", type=int, default=2, help="number of red dilate iterations")
parser.add_argument("--green_morph_kernel_size", type=int, default=5, help="green morph kernel size")
parser.add_argument("--red_morph_kernel_size", type=int, default=5, help="red morph kernel size")
parser.add_argument("--areamin", type=int, default=2000, help="minimum area of contours")
parser.add_argument("--areamax", type=int, default=100000, help="maximum area of contours")
# 解析命令行参数
args = parser.parse_args()

# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
xoutVideo = pipeline.create(dai.node.XLinkOut)

xoutVideo.setStreamName("video")

# Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setVideoSize(1280, 720)

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

# Linking
camRgb.video.link(xoutVideo.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    # Print device information
    print('Device information:')
    print('Name:', device.getDeviceName())
    print('USB speed:', device.getUsbSpeed().name)
    print('Bootloader version:', device.getBootloaderVersion())
    print('Connected cameras:', device.getConnectedCameras())

    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

    while True:
        videoIn = video.get()
        frame = videoIn.getCvFrame()
        # 将图像转换为HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
        
        # 使用解析后的参数
        lower_green = np.array(args.lower_green)
        upper_green = np.array(args.upper_green)
        lower_red = np.array(args.lower_red)
        upper_red = np.array(args.upper_red)
        lower_red2 = np.array(args.lower_red2)
        upper_red2 = np.array(args.upper_red2)
        erode_iterations = args.erode_iterations
        green_dilate_iterations = args.green_dilate_iterations
        red_dilate_iterations = args.red_dilate_iterations
        green_morph_kernel_size = args.green_morph_kernel_size
        red_morph_kernel_size = args.red_morph_kernel_size
        areamin = args.areamin
        areamax = args.areamax
        
        # 提取绿色和红色杆子的区域
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_red1 = cv2.inRange(hsv, lower_red, upper_red)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        # mask_red = mask_red1

        # 对二值图像进行形态学操作，去除干扰的白色网格
        kernel = np.ones((args.green_morph_kernel_size, args.green_morph_kernel_size), np.uint8)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        kernel = np.ones((args.red_morph_kernel_size, args.red_morph_kernel_size), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        # 对二值图像进行腐蚀操作
        kernel = np.ones((5, 5), np.uint8)
        mask_green = cv2.erode(mask_green, kernel, iterations=args.erode_iterations)
        mask_red = cv2.erode(mask_red, kernel, iterations=args.erode_iterations)

        # 对二值图像进行膨胀操作
        kernel = np.ones((5, 5), np.uint8)
        mask_green = cv2.dilate(mask_green, kernel, iterations=args.green_dilate_iterations)
        mask_red = cv2.dilate(mask_red, kernel, iterations=args.red_dilate_iterations)

        # 对二值图像进行形态学操作，填充轮廓内部的空洞
        kernel = np.ones((args.green_morph_kernel_size, args.green_morph_kernel_size), np.uint8)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        kernel = np.ones((args.red_morph_kernel_size, args.red_morph_kernel_size), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

        # 查找绿色和红色杆子的轮廓
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 绘制绿色和红色杆子的轮廓
        for cnt in contours_green:
            area = cv2.contourArea(cnt)
            if area > areamin and area < areamax:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # cv2.drawContours(frame, [cnt], 0, (0, 255, 0), 2)
        for cnt in contours_red:
            area = cv2.contourArea(cnt)
            if area > areamin and area < areamax:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                # cv2.drawContours(frame, [cnt], 0, (0, 0, 255), 2)

        # 显示图像
        cv2.imshow('frame', frame)
        cv2.imshow("dilated_green", mask_green)
        cv2.imshow("dilated_red", mask_red)

        # 等待按键事件
        key = cv2.waitKey(1)

        # 处理按键事件
        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('g'):
            args.green_dilate_iterations += 1
        elif key & 0xFF == ord('G'):
            args.green_dilate_iterations -= 1
        elif key & 0xFF == ord('r'):
            args.red_dilate_iterations += 1
        elif key & 0xFF == ord('R'):
            args.red_dilate_iterations -= 1
        elif key & 0xFF == ord('A'):
            args.areamin -= 1000
        elif key & 0xFF == ord('a'):
            args.areamin += 1000
        elif key & 0xFF == ord('W'):
            args.areamax -= 1000
        elif key & 0xFF == ord('w'):
            args.areamax += 1000
    #################    
        elif key & 0xFF == ord('h'):
            args.lower_green[0] += 5
        elif key & 0xFF == ord('H'):
            args.lower_green[0] -= 5
        elif key & 0xFF == ord('j'):
            args.upper_green[0] += 5
        elif key & 0xFF == ord('J'):
            args.upper_green[0] -= 5
            
        elif key & 0xFF == ord('z'):
            args.lower_red2[0] += 5
        elif key & 0xFF == ord('Z'):
            args.lower_red2[0] -= 5
        elif key & 0xFF == ord('x'):
            args.upper_red2[0] += 5
        elif key & 0xFF == ord('X'):
            args.upper_red2[0] -= 5
    ##################
        elif key & 0xFF == ord('p'):
            print('--------')
            print("lower_green:", args.lower_green)
            print("upper_green:", args.upper_green)
            # print("lower_red:", args.lower_red)
            # print("upper_red:", args.upper_red)
            # print("lower_red2:", args.lower_red2)
            # print("upper_red2:", args.upper_red2)
            print("erode_iterations:", args.erode_iterations)
            print("green_dilate_iterations:", args.green_dilate_iterations)
            # print("red_dilate_iterations:", args.red_dilate_iterations)
            # print("green_morph_kernel_size:", args.green_morph_kernel_size)
            # print("red_morph_kernel_size:", args.red_morph_kernel_size)
            print("areamin:", args.areamin)
            print("areamax:", args.areamax)
            print('--------')

        if cv2.waitKey(1) == ord('q'):
            break