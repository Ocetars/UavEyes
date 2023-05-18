import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# 设置二维码检测器
qrcoder = cv2.QRCodeDetector()
# 创建条形码检测器
detect_obj = cv2.barcode_BarcodeDetector()

while True:
    ret, frame = cap.read()
    if not ret:
        print("can't open camera!")
        break

    # 检测识别二维码
    codeinfo, QRpoints, straight_qrcode = qrcoder.detectAndDecode(frame)
    # 检测识别条形码
    is_ok, bar_info, bar_type, BARpoints = detect_obj.detectAndDecode(frame)

    # 如果识别到二维码
    if codeinfo:
        result = np.copy(frame)
        cv2.drawContours(result, [np.int32(QRpoints)], 0, (0, 0, 255), 2)
        # 显示
        print("qrcode information is : \n%s" % codeinfo)
        cv2.imshow("result", result)
        # cv2.imshow("qrcode roi", np.uint8(straight_qrcode))
    # 如果识别到条形码
    elif is_ok:
        x1 = int(BARpoints[0][1][0])  # 左上x
        y1 = int(BARpoints[0][1][1])  # 左上y
        x2 = int(BARpoints[0][3][0])  # 右下x
        y2 = int(BARpoints[0][3][1])  # 右下y
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            frame,
            str(bar_info),
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2,
        )
        # 显示
        cv2.imshow("result", frame)
    else:
        # 显示原始的摄像头画面
        cv2.imshow("result", frame)
    # 按下 q 键退出循环
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()
