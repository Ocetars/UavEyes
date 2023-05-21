import cv2
import numpy as np

def detect_qrcode(frame):
    # 设置检测器
    qrcoder = cv2.QRCodeDetector()
    # 检测识别二维码
    codeinfo, points, straight_qrcode = qrcoder.detectAndDecode(frame)

    # 如果识别到二维码
    if codeinfo:
        return codeinfo
    else:
        return None

def detect_barcode(frame):
    # 创建条形码检测器
    detect_obj = cv2.barcode_BarcodeDetector()

    # 检测条形码
    is_ok, bar_info, bar_type, points = detect_obj.detectAndDecode(frame)

    # 如果检测到条形码，返回条形码信息
    if is_ok:
        return bar_info
    else:
        return None

def detect_allcode(frame):
    # 设置二维码检测器
    qrcoder = cv2.QRCodeDetector()
    # 创建条形码检测器
    detect_obj = cv2.barcode_BarcodeDetector()

    # 检测识别二维码
    codeinfo, QRpoints, straight_qrcode = qrcoder.detectAndDecode(frame)
    # 检测识别条形码
    is_ok, bar_info, bar_type, BARpoints = detect_obj.detectAndDecode(frame)

    # 如果识别到二维码
    if codeinfo:
        return codeinfo
    # 如果识别到条形码
    elif is_ok:
        return bar_info
    else:
        return None
    
'''
只返回检测到的信息
如果没检测到则返回None
'''