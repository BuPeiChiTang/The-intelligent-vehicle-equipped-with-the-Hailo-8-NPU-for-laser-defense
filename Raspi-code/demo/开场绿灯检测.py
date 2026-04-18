import cv2
import numpy as np

def detect_green_light(frame, area_thresh=(10, 200), brightness_thresh=220):
    """
    检测图像中是否有小而亮的绿色区域（绿灯），返回mask和检测结果
    area_thresh: (min_area, max_area)
    brightness_thresh: 只认非常亮的绿色
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([40, 80, 80])
    upper_green = np.array([80, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detected = False
    max_brightness = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < area_thresh[0] or area > area_thresh[1]:
            continue
        x1, y1, w1, h1 = cv2.boundingRect(cnt)
        roi_mask = mask[y1:y1+h1, x1:x1+w1]
        roi_v = hsv[y1:y1+h1, x1:x1+w1, 2]
        if np.any(roi_mask > 0):
            roi_max_brightness = roi_v[roi_mask > 0].max()
            max_brightness = max(max_brightness, roi_max_brightness)
            if roi_max_brightness > brightness_thresh:
                detected = True
                break
    return detected, mask, max_brightness

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    system_status = "等待"
    print("系统状态：", system_status)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头帧")
            break

        detected, mask, max_brightness = detect_green_light(frame)
        display_frame = frame.copy()

        # 标出亮度足够高的绿色区域
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 10:
                x, y, w, h = cv2.boundingRect(cnt)
                roi = mask[y:y+h, x:x+w]
                roi_v = cv2.cvtColor(frame[y:y+h, x:x+w], cv2.COLOR_BGR2HSV)[:, :, 2]
                roi_max_brightness = roi_v[roi > 0].max() if np.any(roi > 0) else 0
                if roi_max_brightness > 250:
                    cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(display_frame, "Green LED", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if detected:
            system_status = "开始"
            cv2.putText(display_frame, "Green Detected: START", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            system_status = "等待"
            cv2.putText(display_frame, "Waiting for Green...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.putText(display_frame, f"Max Green Brightness: {max_brightness:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Camera", display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()