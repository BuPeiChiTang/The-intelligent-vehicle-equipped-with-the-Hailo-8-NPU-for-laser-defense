'''
实现功能：
    检测绿灯
    串口通信
    模型检测
    打击时间检测
'''
#!/usr/bin/env python3
import os
import sys
from pathlib import Path
import numpy as np
#from loguru import logger
import queue
import threading
import cv2
#from typing import List
from object_detection_utils_final import ObjectDetectionUtils
import time


# Add the parent directory to the system path to access utils module
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import HailoAsyncInference
import serial
# 定义全局退出标志,防止终端运行卡死
exit_event = threading.Event()
# 定义全局绿灯检测事件
def detect_green_light_fast(frame, area_thresh=(10, 200), brightness_thresh=220):
    # 缩小图片，加快处理
    small = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
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
#串口通信1部分舵机
def serial_communication(serial_port: str, baud_rate: int, data_queue: queue.Queue, send_queue: queue.Queue, exit_event: threading.Event) -> None:
    """
    Handle serial communication with the lower machine.

    Args:
        serial_port (str): Serial port name (e.g., '/dev/ttyUSB0').
        baud_rate (int): Baud rate for the serial communication.
        data_queue (queue.Queue): Queue for receiving data from the lower machine.
        send_queue (queue.Queue): Queue for sending data to the lower machine.
        exit_event (threading.Event): Event to signal thread exit.
    """
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"串口 1{serial_port} 已打开，波特率 {baud_rate}")
    except Exception as e:
        print(f"无法打开串口 {serial_port}: {e}")
        return

    last_send_time = 0
    

    while not exit_event.is_set():
        try:
            # 接收数据
            if ser.in_waiting > 0:
                received_bytes = ser.readline()
                try:
                    received_str = received_bytes.decode('utf-8').strip()
                    print(f"{received_str}")
                    
                    
                    
                except Exception as e:
                    print(f"解码串口数据出错: {e}")
                #bits = ''.join(f'{byte:08b}' for byte in received_bytes)
                #print(bits)

            # 每50ms尝试发送一次
            now = time.time()
            if (now - last_send_time) >= 0.05:
                latest_data = None
                #latest_data=send_queue.get()
                
                
                # 取出send_queue中最新的数据并清空队列
                try:
                    while True:
                        #latest_data1=latest_data
                        
                        latest_data = send_queue.get_nowait()
                        
                except queue.Empty:
                    pass
                    
                # 如果队列中没有数据，发送默认值
                if latest_data is None:
                    latest_data = "304,282,0,0\n"

                ser.write(f"A{latest_data}\n".encode('utf-8'))
                #print(f"发送数据: A{latest_data}")
                '''
                # 新增：计算并打印两次发送的时间间隔
                if last_send_time is not None:
                    interval = now -last_send_time
                    print(f"串口1两次发送间隔: {interval:.6f} 秒")
                last_send_time = now
                '''
                last_send_time = now

                time.sleep(0.001)
        except Exception as e:
            print(f"串口通信错误: {e}")
            break
    ser.close()
    print("串口已关闭")
#串口通信2部分
def serial_communication2(serial_port: str, baud_rate: int, data_queue: queue.Queue, send_queue: queue.Queue, exit_event: threading.Event) -> None:
    """
    第二路串口通信线程，支持收发数据
    """
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"串口2 {serial_port} 已打开，波特率 {baud_rate}")
    except Exception as e:
        print(f"无法打开串口2 {serial_port}: {e}")
        return

    last_send_time = 0
    bb_received = False  # 是否收到过线信号标志

    while not exit_event.is_set():
        try:
            # 只处理一次BB
            if ser.in_waiting > 0 and not bb_received:
                received_bytes = ser.readline()
                try:
                    received_str = received_bytes.decode('utf-8').strip()
                    print(f"[串口2] 接收到字符串: {received_str}")
                    data_queue.put(received_str)  # 放入接收队列
                    if received_str == "BB":
                        print("[串口2] 检测到过线信号！")
                        ser.write(b"OK")  # 发送OK
                        print("[串口2] 发送数据: OK")
                        bb_received = True
                except Exception as e:
                    print(f"[串口2] 解码串口数据出错: {e}")

            # 发送部分不受影响，持续发送CC/DD等信号
            now = time.time()
            if (now - last_send_time) >= 0.025 :
                latest_data = None
                try:
                    while True:
                        latest_data = send_queue.get_nowait()
                except queue.Empty:
                    pass

                if latest_data is not None:
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    ser.write(f"{latest_data}\n".encode('utf-8'))
                    print(f"[串口2] 发送数据: {latest_data}")
                last_send_time = now

            time.sleep(0.01)
        except Exception as e:
            print(f"[串口2] 串口通信错误: {e}")
            break

    ser.close()
    print("串口2已关闭")


    
#预处理部分
def preprocess_from_cap(cap: cv2.VideoCapture, 
                        batch_size: int, 
                        input_queue: queue.Queue, 
                        model_width: int, 
                        model_height: int, 
                        utils: ObjectDetectionUtils) -> None:
    """
    Process frames from the camera stream and enqueue them.

    Args:
        batch_size (int): Number of images per batch.
        input_queue (queue.Queue): Queue for input images.
        width (int): Model input width.
        height (int): Model input height.
        utils (ObjectDetectionUtils): Utility class for object detection preprocessing.
    """
    frames = []
    processed_frames = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        '''
        # ====== 中心区域放大功能 ======
        h, w = frame.shape[:2]
        crop_size = min(h, w) * 3 // 4  # 取中心4/3区域
        cx, cy = w // 2, h // 2
        x1 = max(cx - crop_size // 2, 0)
        y1 = max(cy - crop_size // 2, 0)
        x2 = min(cx + crop_size // 2, w)
        y2 = min(cy + crop_size // 2, h)
        center_crop = frame[y1:y2, x1:x2]
        # 放大到原始分辨率
        zoomed_frame = cv2.resize(center_crop, (640, 480), interpolation=cv2.INTER_LINEAR)
        # ===========================
        '''
        frames.append(frame)
        processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        processed_frame = utils.preprocess(processed_frame, model_width, model_height)
        processed_frames.append(processed_frame)

        if len(frames) == batch_size:
            input_queue.put((frames, processed_frames))#在ｑｕｅｕｅ里加入原始帧和预处理帧
            processed_frames, frames = [], []
#后处理函数
def postprocess(output_queue: queue.Queue,
                cap: cv2.VideoCapture,
                save_stream_output: bool,
                utils: ObjectDetectionUtils,
                send_queue: queue.Queue,
                send_queue2: queue.Queue,
                exit_event: threading.Event,
                data_queue2: queue.Queue
                ) -> None:
    """
    Process inference results and display or save output.

    Args:
        output_queue (queue.Queue): Queue for inference results.
        cap (cv2.VideoCapture): Video capture object.
        save_stream_output (bool): Whether to save the output stream.
        utils (ObjectDetectionUtils): Utility class for postprocessing.
        send_queue (queue.Queue): Queue for sending data to the lower machine.
        exit_event (threading.Event): Event to signal thread exit.
    """
    image_id = 0
    out = None
    output_path = Path("output_images")  # Directory to save output images
    img_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    img_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # 新增：初始化VideoWriter
    out = None
    

    prev_time = time.time()
    fps = 0
    hit_start_time = None
    hit_duration = 0
    WIN_TIME =2.0  # 2秒
    last_valid_hit_time = None
    TOLERANCE = 0.2  # 容忍时间（秒）
    frame_count = 0
    crossed_line = True  # 新增：过线信号标志
    
    green_light_detected =  True# 绿灯检测状态
    green_sent = False      # 是否已发送CC
    win_sent = False        # 是否已发送DD
    S =2.0                 # 持续时间阈值（秒）
    while not exit_event.is_set():  # 检查退出标志
        try:
            try:
                while True:
                    msg = data_queue2.get_nowait()
                    if msg.strip() == "BB":
                        crossed_line = True
                        print("收到BB过线信号，允许计时！")
            except queue.Empty:
                pass
            result = output_queue.get(timeout=1)  # 设置超时时间
            if result is None:
                break  # Exit the loop if sentinel value is received

            original_frame, infer_results = result

            if len(infer_results) == 1:
                infer_results = infer_results[0]

            detections = utils.extract_detections(infer_results, threshold=0.5)
            
            # 1. 绿灯检测（只检测一次，检测到后不再检测）
            if not green_light_detected:
                frame_count += 1
                if frame_count % 3 == 0:
                    detected, mask, max_brightness = detect_green_light_fast(original_frame)
                    if detected:
                        green_light_detected = True
                        print(f"检测到绿灯，最大亮度: {max_brightness}")
            # 检测到绿灯，串口发C
            if green_light_detected and not green_sent:
                send_queue2.put("CC")    # 串口2
                green_sent = True
            
            detections = utils.extract_detections(infer_results, threshold=0.1)#调整置信度阈值
            # 默认无有效打击
            valid_hit = False
            
            # 获取打击区框的中心坐标和宽高的一半
            for idx, cls in enumerate(detections['detection_classes']):
                if cls == 1:  # 假设类别 1 是打击区
                    box = detections['detection_boxes'][idx]
                    # 将归一化坐标转换为像素坐标
                    '''
                    y_min = int(box[0] * img_height)  
                    x_min = int(box[1] * img_width)
                    y_max = int(box[2] * img_height)
                    x_max = int(box[3] * img_width)
                    '''
                    y_min = int(box[0] * img_height* 0.8)  
                    x_min = int(box[1] * img_width*0.8)
                    y_max = int(box[2] * img_height*0.8)
                    x_max = int(box[3] * img_width*0.8)
                    
                    center_x = (x_min + x_max) // 2
                    center_y = (y_min + y_max) // 2
                    half_width = (x_max - x_min) // 2
                    half_height = (y_max - y_min) // 2
                    # 判断点(308, 280)是否在打击区框内
                    #print(f"检测框: x=({center_x - half_width},{center_x + half_width}), y=({center_y - half_height},{center_y + half_height})")
                    #print(f"判断点: (308, 280)")
                    if (center_x - half_width <= 308 <= center_x + half_width) and \
                       (center_y - half_height <= 280 <= center_y + half_height):
                        valid_hit = True
                          # 只要有一个goal框包含该点即可
   
                    # 打印或保存结果
                    #print(f"打击区中心坐标: ({center_x}, {center_y}), 半宽: {half_width}, 半高: {half_height}")

                    # 将数据放入发送队列
                    send_queue.put(f"{center_x},{center_y},{half_width},{half_height}")
            # 有效打击计时
            now = time.time()
            # 只有绿灯信号为True时才允许计时
            if green_light_detected and crossed_line:
                if valid_hit:
                    if hit_start_time is None:
                        hit_start_time = now
                    last_valid_hit_time = now
                    hit_duration = now - hit_start_time
                    if hit_duration >= S:
                        send_queue2.put("DD")    # 串口2
                        win_sent = True
                else:
                    if last_valid_hit_time is not None and (now - last_valid_hit_time) <= TOLERANCE:
                        hit_duration = now - hit_start_time
                    else:
                        hit_start_time = None
                        hit_duration = 0
                        last_valid_hit_time = None
            

            frame_with_detections = utils.draw_detections(detections, original_frame)
            
            # 2. 绿灯检测提示
            if not green_light_detected:
                cv2.putText(
                    frame_with_detections,
                    "Waiting for GREEN LIGHT...",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 255, 255),
                    3,
                    cv2.LINE_AA
                )
                
            else:
                cv2.putText(
                    frame_with_detections,
                    "GREEN LIGHT DETECTED! START!",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 255, 0),
                    3,
                    cv2.LINE_AA
                )
                
            # 绘制打击区计时
            cv2.putText(
                frame_with_detections,
                f"Hit Time: {hit_duration:.2f}s",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255) if not valid_hit else (0, 255, 0),
                2,
                cv2.LINE_AA
            )

            if hit_duration >= WIN_TIME:
                cv2.putText(
                    frame_with_detections,
                    "WIN!",
                    (200, 200),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    2,
                    (0, 255, 0),
                    4,
                    cv2.LINE_AA
                )
                
            # 计算帧率
            
            current_time = time.time()
            fps = 1 / (current_time - prev_time)
            prev_time = current_time
            
            cv2.putText(
                frame_with_detections,
                f"FPS: {fps:.2f}",
                (10, 30),  # 显示位置
                cv2.FONT_HERSHEY_SIMPLEX,
                1,  # 字体大小
                (0, 255, 0),  # 字体颜色
                2,  # 字体粗细
                cv2.LINE_AA
            )

            if cap is not None:
                # Display output
                cv2.imshow("Output", frame_with_detections)
                if save_stream_output:
                    out.write(frame_with_detections)
            else:
                cv2.imwrite(str(output_path / f"output_{image_id}.png"), frame_with_detections)

            # Wait for key press "q"
            image_id += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                exit_event.set()  # 设置退出标志
                break
        except queue.Empty:
            continue

    if cap is not None and save_stream_output:
        out.release()  # Release the VideoWriter object
    output_queue.task_done()  # Indicate that processing is complete


def main():
    CAMERA_CAP_WIDTH = 800
    CAMERA_CAP_HEIGHT = 600
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        sys.exit(1)
    #cap.set(cv2.CAP_PROP_FPS, 60)  # Set FPS to 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_CAP_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_CAP_HEIGHT)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    #cap.set(cv2.CAP_PROP_FPS, 60)  # Set FPS to 
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"实际帧率: {actual_fps}")
    
    HEF_PATH = "/home/pi/hailo-rpi5-examples/Hailo-Application-Code-Examples/runtime/hailo-8/python/object_detection/goal_car/yolov11n.hef"
    labels_path = "/home/pi/hailo-rpi5-examples/Hailo-Application-Code-Examples/runtime/hailo-8/python/object_detection/goal_car/label.txt"
    utils = ObjectDetectionUtils(labels_path)
    batch_size = 1

    input_queue = queue.Queue()
    output_queue = queue.Queue()
    send_queue = queue.Queue()
    data_queue = queue.Queue()
    exit_event = threading.Event()
    send_queue2 = queue.Queue()
    data_queue2 = queue.Queue()
    

    preprocess_thread = threading.Thread(target=preprocess_from_cap, args=(cap, batch_size, input_queue, 640, 640, utils))
    infer_thread = HailoAsyncInference(hef_path=HEF_PATH, input_queue=input_queue, output_queue=output_queue, batch_size=batch_size, send_original_frame=True)
    post_thread = threading.Thread(target=postprocess, args=(output_queue, cap, False, utils, send_queue,send_queue2, exit_event,data_queue2))
    serial_thread = threading.Thread(target=serial_communication, args=('/dev/ttyUSB0', 115200, data_queue, send_queue, exit_event))
    serial_thread2 = threading.Thread(target=serial_communication2, args=('/dev/ttyUSB1', 115200, data_queue2, send_queue2, exit_event))
    try:
        print("Starting threads...")
        preprocess_thread.start()
        post_thread.start()
        serial_thread.start()
        serial_thread2.start()
        infer_thread.run()
        
        
        preprocess_thread.join()
        output_queue.put(None)  # Signal process thread to exit
        post_thread.join()
        serial_thread.join()
         
    except Exception as e:
        print(f"发生异常: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("程序已正常退出")


if __name__ == "__main__":
        main()