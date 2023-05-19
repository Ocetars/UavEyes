import cv2
import numpy as np

# 打开摄像头
cap = cv2.VideoCapture(0)

while True:
    # 读取视频帧
    ret, frame = cap.read()
    if not ret:
        break

    # 设置检测器
    qrcoder = cv2.QRCodeDetector()
    # 检测识别二维码
    codeinfo, points, straight_qrcode = qrcoder.detectAndDecode(frame)

    # 如果识别到二维码
    if codeinfo:
        result = np.copy(frame)
        cv2.drawContours(result, [np.int32(points)], 0, (0, 0, 255), 2)
        # 输出识别二维码的信息
        print("qrcode information is : \n%s"% codeinfo)
        # 显示图片
        cv2.imshow("result", result)
        cv2.imshow("qrcode roi", np.uint8(straight_qrcode))
    else:
        # 显示原始的摄像头画面
        cv2.imshow("result", frame)

    # 按下 q 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()