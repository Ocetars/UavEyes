import cv2

# 创建条形码检测器
detect_obj = cv2.barcode_BarcodeDetector()

# 打开摄像头
cap = cv2.VideoCapture(0)

while True:
    # 读取视频帧
    ret, frame = cap.read()
    if not ret:
        break

    # 检测条形码
    is_ok, bar_info, bar_type, points = detect_obj.detectAndDecode(frame)

    # 如果检测到条形码，绘制矩形框并显示条形码信息
    if is_ok:
        x1 = int(points[0][1][0]) # 左上x
        y1 = int(points[0][1][1]) # 左上y
        x2 = int(points[0][3][0]) # 右下x
        y2 = int(points[0][3][1]) # 右下y
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, str(bar_info), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # 显示视频帧
    cv2.imshow("Barcode Detection", frame)

    # 等待按键
    key = cv2.waitKey(1)
    if key == ord("q"):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()