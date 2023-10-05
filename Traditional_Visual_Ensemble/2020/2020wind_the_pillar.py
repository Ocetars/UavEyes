#!/usr/bin/env python3
# coding=utf-8
import argparse
import cv2
import depthai as dai
import numpy as np
import math
import rospy
from geometry_msgs.msg import Point

display = True

parser = argparse.ArgumentParser()
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


# 创建 ROS 节点
rospy.init_node('talker', anonymous=True)

def create_pipeline(device):
    monoResolution = dai.MonoCameraProperties.SensorResolution.THE_480_P
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

    rgbOut = pipeline.create(dai.node.XLinkOut)
    disparityOut = pipeline.create(dai.node.XLinkOut)

    xoutSpatialData = pipeline.create(dai.node.XLinkOut)
    xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

    rgbOut.setStreamName("rgb")
    disparityOut.setStreamName("disp")

    xoutSpatialData.setStreamName("spatialData")
    xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

    # Properties
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setFps(30)
    camRgb.setIspScale(2, 3)
    # For now, RGB needs fixed focus to properly align with depth.
    # This value was used during calibration
    try:
        calibData = device.readCalibration2()
        lensPosition = calibData.getLensPosition(dai.CameraBoardSocket.CAM_A)
        if lensPosition:
            camRgb.initialControl.setManualFocus(lensPosition)
    except:
        raise
    left.setResolution(monoResolution)
    left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    left.setFps(30)
    right.setResolution(monoResolution)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    right.setFps(30)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    # LR-check is required for depth alignment
    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

    # Config

    config = dai.SpatialLocationCalculatorConfigData()
    config.depthThresholds.lowerThreshold = 100
    config.depthThresholds.upperThreshold = 10000
    config.roi = dai.Rect(topLeft, bottomRight)

    spatialLocationCalculator.inputConfig.setWaitForMessage(False)
    spatialLocationCalculator.initialConfig.addROI(config)

    # Linking
    camRgb.isp.link(rgbOut.input)

    left.out.link(stereo.left)
    right.out.link(stereo.right)

    stereo.disparity.link(disparityOut.input)
    stereo.depth.link(spatialLocationCalculator.inputDepth)

    spatialLocationCalculator.out.link(xoutSpatialData.input)

    xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

    return pipeline, stereo.initialConfig.getMaxDisparity()

def check_input(roi, frame, DELTA=5):
    """
    Check if input is ROI or point. If point, convert to ROI
    """
    # Limit the point so ROI won't be outside the frame
    if len(roi) == 2:
        if len(roi[0]) == 2:
            roi = np.array(roi) + [[-DELTA, -DELTA], [DELTA, DELTA]]
        else:
            roi = np.array([roi, roi]) + [[-DELTA, -DELTA], [DELTA, DELTA]]
    elif len(roi) == 4:
        roi = np.array(roi) + [[-DELTA, -DELTA], [DELTA, DELTA]]

    roi.clip([DELTA, DELTA], [frame.shape[1] - DELTA, frame.shape[0] - DELTA])

    return roi / frame.shape[1::-1]

def drawText(frame, text, org, color=(255, 255, 255)):
            cv2.putText(
                frame,
                text,
                org,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA
            )
def drawSpatialLocations(frame, spatialLocations):
    results = []
    for depthData in spatialLocations:
        roi = depthData.config.roi
        roi = roi.denormalize(width=frame.shape[1], height=frame.shape[0])
        xmin = int(roi.topLeft().x)
        ymin = int(roi.topLeft().y)
        xmax = int(roi.bottomRight().x)
        ymax = int(roi.bottomRight().y)

        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 0), 4)
        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (255, 255, 255), 1)
        drawText(
            frame,
            f"X: {int(depthData.spatialCoordinates.x)/10} cm",
            (xmin + 10, ymin + 20),
        )
        drawText(
            frame,
            f"Y: {int(depthData.spatialCoordinates.y)/10} cm",
            (xmin + 10, ymin + 35),
        )
        drawText(
            frame,
            f"Z: {int(depthData.spatialCoordinates.z)/10} cm",
            (xmin + 10, ymin + 50),
        )
        X = int(depthData.spatialCoordinates.x)/10
        Y = int(depthData.spatialCoordinates.y)/10
        Z = int(depthData.spatialCoordinates.z)/10
        results.append((X, Y, Z))
    return results

def run():
    # 初始化一些变量
    topLeft = dai.Point2f(0, 0)
    bottomRight = dai.Point2f(0, 0)
    newConfig = True
    # Connect to device and start pipeline
    with dai.Device() as device:
        print('Connected cameras:', device.getConnectedCameraFeatures())
        print('Usb speed:', device.getUsbSpeed().name)
        if device.getBootloaderVersion() is not None:
            print('Bootloader version:', device.getBootloaderVersion())
        print('Device name:', device.getDeviceName())
        
        pipeline,  maxDisparity = create_pipeline(device)
        device.startPipeline(pipeline)
        frameRgb = None
        frameDisp = None
        depthDatas = []
        
    # Configure windows; trackbar adjusts blending ratio of rgb/depth
        rgbWindowName = "rgb"
        depthWindowName = "depth"
        cv2.namedWindow(rgbWindowName)
        cv2.namedWindow(depthWindowName)
        rgbWidth, rgbHeight = cv2.getWindowImageRect(rgbWindowName)[2:]
        depthWidth, depthHeight = cv2.getWindowImageRect(depthWindowName)[2:]
        
        spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig") # type: ignore
        imageQueue = device.getOutputQueue("rgb") # type: ignore
        dispQueue = device.getOutputQueue("disp") # type: ignore
        spatialDataQueue = device.getOutputQueue("spatialData") # type: ignore
        
        while not device.isClosed():
            # 解析命令行参数
            args = parser.parse_args()
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
            
            imageData = imageQueue.tryGet()
            dispData = dispQueue.tryGet()
            spatialData = spatialDataQueue.tryGet()
            
            if spatialData is not None:
                depthDatas = spatialData.getSpatialLocations()

            if imageData is not None:
                frameRgb = imageData.getCvFrame()
                # 将图像转换为HSV颜色空间
                hsv = cv2.cvtColor(frameRgb, cv2.COLOR_BGR2HSV_FULL)
                # 提取绿色和红色杆子的区域
                mask_green = cv2.inRange(hsv, lower_green, upper_green)
                mask_red1 = cv2.inRange(hsv, lower_red, upper_red)
                mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
                mask_red = cv2.bitwise_or(mask_red1, mask_red2)
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
                
                # 绘制红色和绿色杆子的轮廓
                cam_x = int(rgbWidth / 2)
                cam_y = int(rgbHeight / 2)
                min_dist = float('inf')
                min_cnt = None
                for cnt in contours_red:
                    x, y, w, h = cv2.boundingRect(cnt)
                    threshold = 2.5  # 宽高比阈值
                    if w / h > threshold or h / w > threshold:  
                        area = cv2.contourArea(cnt)
                        if area > areamin and area < areamax:
                            M = cv2.moments(cnt)
                            cx = int(M['m10'] / M['m00'])
                            cy = int(M['m01'] / M['m00'])
                            # 计算中心点坐标与摄像头中心点坐标之间的距离
                            dist = math.sqrt((cx - cam_x) ** 2 + (cy - cam_y) ** 2)
                            # 更新距离摄像头中心点最近的轮廓
                            if dist < min_dist:
                                min_dist = dist
                                min_cnt = cnt
                # 绘制距离摄像头中心点最近的轮廓
                if min_cnt is not None:
                        x, y, w, h = cv2.boundingRect(min_cnt)
                        cv2.rectangle(frameRgb, (x, y), (x + w, y + h), (71,130,255), 2)
                        Mred = cv2.moments(min_cnt)
                        global cxmin,cymin
                        cxmin = int(Mred['m10'] / Mred['m00'])
                        cymin = int(Mred['m01'] / Mred['m00'])
                        topLeft = dai.Point2f(cxmin - 15, cymin - 30)
                        bottomRight = dai.Point2f(cxmin + 15, cymin + 30)
                        newConfig = True

                        pub = rospy.Publisher('point_topic_2', Point, queue_size=10)
                        # 获取矩形框中心点坐标
                        cxmin = int(Mred['m10'] / Mred['m00'])
                        cymin = int(Mred['m01'] / Mred['m00'])
                        # 创建 Point 消息
                        point_msg = Point()
                        point_msg.x = cxmin
                        point_msg.y = cymin
                        point_msg.z = 0
                        # 发布 Point 消息
                        pub.publish(point_msg)
                            
                # min_distG = float('inf')
                # min_cntG = None
                # for cntG in contours_green:
                #     areaG = cv2.contourArea(cntG)
                #     if areaG > areamin and areaG < areamax:
                #         M = cv2.moments(cntG)
                #         cxG = int(M['m10'] / M['m00'])
                #         cyG = int(M['m01'] / M['m00'])
                #         # 计算中心点坐标与摄像头中心点坐标之间的距离
                #         distG = math.sqrt((cxG - cam_x) ** 2 + (cyG - cam_y) ** 2)
                #         # 更新距离摄像头中心点最近的轮廓
                #         if distG < min_distG:
                #             min_distG = distG
                #             min_cntG = cntG
                # # 绘制距离摄像头中心点最近的轮廓
                # if min_cntG is not None:
                #         x, y, w, h = cv2.boundingRect(min_cntG)
                #         cv2.rectangle(frameRgb, (x, y), (x + w, y + h), (212, 255, 127), 2)
                #         newConfig = True
                                
                # 调用 drawSpatialLocations 函数
                results = drawSpatialLocations(frameRgb, depthDatas)
                
                # 创建 ROS 发布者
                publisher = rospy.Publisher('point_topic', Point, queue_size=10)
                
                # 创建 Point 消息
                point_msg = Point()

                # 提取所有的值
                X_values = [float(result[0]) for result in results]
                Y_values = [float(result[1]) for result in results]
                Z_values = [float(result[2]) for result in results]

                # 将 X_values 中的值分别赋值给 point_msg.x
                for x in X_values:
                    point_msg.x = x
                    # publisher.publish(point_msg)

                # 将 Y_values 中的值分别赋值给 point_msg.y
                for y in Y_values:
                    point_msg.y = y
                    # publisher.publish(point_msg)

                # 将 Z_values 中的值分别赋值给 point_msg.z
                for z in Z_values:
                    point_msg.z = z
                    
                publisher.publish(point_msg)
                        
                if display:
                    # IM SHOW图像
                    cv2.imshow(rgbWindowName, frameRgb)
                
            if dispData is not None:
                frameDisp = dispData.getFrame()
                frameDisp = (frameDisp * (255 / maxDisparity)).astype(np.uint8)
                frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_JET)
                frameDisp = np.ascontiguousarray(frameDisp)
                drawSpatialLocations(frameDisp, depthDatas)
                # if display:
                    # cv2.imshow(depthWindowName, frameDisp)

                key = cv2.waitKey(1)
                if key == ord("q"):
                    break
            
            # 更新 SpatialLocationCalculatorConfigData
            if newConfig:
                config = dai.SpatialLocationCalculatorConfigData()
                config.depthThresholds.lowerThreshold = 100
                config.depthThresholds.upperThreshold = 10000
                config.roi = dai.Rect(topLeft, bottomRight)
                config.calculationAlgorithm = (
                    dai.SpatialLocationCalculatorAlgorithm.MEAN
                )
                cfg = dai.SpatialLocationCalculatorConfig()
                cfg.addROI(config)
                spatialCalcConfigInQueue.send(cfg)
                # newConfig = False
            
if __name__ == "__main__":
    topLeft = dai.Point2f(0, 0)
    bottomRight = dai.Point2f(0,0)
    run()