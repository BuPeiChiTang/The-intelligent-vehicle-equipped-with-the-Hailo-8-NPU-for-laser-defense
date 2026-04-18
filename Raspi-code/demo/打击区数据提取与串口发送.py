#!/usr/bin/env python3
import os
import sys
from pathlib import Path
import numpy as np
from loguru import logger
import queue
import threading
import cv2
from typing import List
import supervision as sv
from object_detection_utils_final import ObjectDetectionUtils
import time

# Add the parent directory to the system path to access utils module
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import HailoAsyncInference, load_images_opencv, validate_images, divide_list_to_batches
import serial
# 定义全局退出标志,防止终端运行卡死
exit_event = threading.Event()
#串口通信部分
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
        print(f"串口 {serial_port} 已打开，波特率 {baud_rate}")
    except Exception as e:
        print(f"无法打开串口 {serial_port}: {e}")
        return

    last_send_time = 0
    send_interval = None

    while not exit_event.is_set():
        try:
            # 接收数据
            if ser.in_waiting > 0:
                received_bytes = ser.readline()
                try:
                    received_str = received_bytes.decode('utf-8').strip()
                    print(f"接收到字符串: {received_str}")
                    if received_str.startswith('B'):
                        print("检测到过线信号！")
                except Exception as e:
                    print(f"解码串口数据出错: {e}")
                #bits = ''.join(f'{byte:08b}' for byte in received_bytes)
                #print(bits)

            # 每50ms尝试发送一次
            now = time.time()
            if (now - last_send_time) >= 0.05:
                latest_data = None
                # 取出send_queue中最新的数据并清空队列
                try:
                    while True:
                        latest_data = send_queue.get_nowait()
                except queue.Empty:
                    pass

                # 如果队列中没有数据，发送默认值
                if latest_data is None:
                    latest_data = "320,240,0,0\n"

                ser.write(f"A{latest_data}\n".encode('utf-8'))
                if last_send_time != 0:
                    send_interval = now - last_send_time
                    #print(f"本次有效数据发送间隔: {send_interval:.4f} 秒")
                print(f"发送数据: A{latest_data}")
                last_send_time = now

            time.sleep(0.01)
        except Exception as e:
            print(f"串口通信错误: {e}")
            break

   

    ser.close()
    print("串口已关闭")
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

        frames.append(frame)
        processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        processed_frame = utils.preprocess(processed_frame, model_width, model_height)
        processed_frames.append(processed_frame)

        if len(frames) == batch_size:
            input_queue.put((frames, processed_frames))#在ｑｕｅｕｅ里加入原始帧和预处理帧
            processed_frames, frames = [], []
#简单后处理
def postprocess(output_queue: queue.Queue,
                cap: cv2.VideoCapture,
                save_stream_output: bool,
                utils: ObjectDetectionUtils,
                send_queue: queue.Queue,
                exit_event: threading.Event) -> None:
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

    prev_time = time.time()
    fps = 0

    while not exit_event.is_set():  # 检查退出标志
        try:
            result = output_queue.get(timeout=1)  # 设置超时时间
            if result is None:
                break  # Exit the loop if sentinel value is received

            original_frame, infer_results = result

            if len(infer_results) == 1:
                infer_results = infer_results[0]

            detections = utils.extract_detections(infer_results, threshold=0.5)
            
            # 获取打击区框的中心坐标和宽高的一半
            for idx, cls in enumerate(detections['detection_classes']):
                if cls == 0:  # 假设类别 1 是打击区
                    box = detections['detection_boxes'][idx]
                    # 将归一化坐标转换为像素坐标
                    y_min = int(box[0] * img_height)
                    x_min = int(box[1] * img_width)
                    y_max = int(box[2] * img_height)
                    x_max = int(box[3] * img_width)

                    center_x = (x_min + x_max) // 2
                    center_y = (y_min + y_max) // 2
                    half_width = (x_max - x_min) // 2
                    half_height = (y_max - y_min) // 2
                    
                    
                    
                    # 打印或保存结果
                    #print(f"打击区中心坐标: ({center_x}, {center_y}), 半宽: {half_width}, 半高: {half_height}")

                    # 将数据放入发送队列
                    send_queue.put(f"{center_x},{center_y},{half_width},{half_height}")

            frame_with_detections = utils.draw_detections(
                detections, original_frame,
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

    preprocess_thread = threading.Thread(target=preprocess_from_cap, args=(cap, batch_size, input_queue, 640, 640, utils))
    infer_thread = HailoAsyncInference(hef_path=HEF_PATH, input_queue=input_queue, output_queue=output_queue, batch_size=batch_size, send_original_frame=True)
    post_thread = threading.Thread(target=postprocess, args=(output_queue, cap, False, utils, send_queue, exit_event))
    serial_thread = threading.Thread(target=serial_communication, args=('/dev/ttyUSB0', 115200, data_queue, send_queue, exit_event))

    try:
        print("Starting threads...")
        preprocess_thread.start()
        post_thread.start()
        serial_thread.start()
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