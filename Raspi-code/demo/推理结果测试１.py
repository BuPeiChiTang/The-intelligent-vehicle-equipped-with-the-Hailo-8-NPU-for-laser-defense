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
# 定义全局退出标志
exit_event = threading.Event()
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

    while not exit_event.is_set():  # 检查退出标志
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头帧")
            break

        
        frames.append(frame)
        processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        processed_frame = utils.preprocess(processed_frame, model_width, model_height)
        processed_frames.append(processed_frame)

        if len(frames) == batch_size:
            
            input_queue.put((frames, processed_frames))#在ｑｕｅｕｅ里加入原始帧和预处理帧
            processed_frames, frames = [], []
#简单后处理
def postprocess(
    output_queue: queue.Queue,
    cap: cv2.VideoCapture,
    save_stream_output: bool,
    utils: ObjectDetectionUtils
    ) -> None:
    image_id = 0
    out = None
    output_path = Path("output_images")
    prev_time = time.time()  # 初始化上一帧时间

    while not exit_event.is_set():
        try:
            result = output_queue.get(timeout=1)
            if result is None:
                break

            original_frame, infer_results = result
            if len(infer_results) == 1:
                infer_results = infer_results[0]

            detections = utils.extract_detections(infer_results, threshold=0.3)
            frame_with_detections = utils.draw_detections(detections, original_frame)

            # 计算帧率
            current_time = time.time()
            fps = 1.0 / (current_time - prev_time)
            prev_time = current_time

            # 在图像上绘制FPS
            cv2.putText(
                frame_with_detections,
                f"FPS: {fps:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

            cv2.imshow("Output", frame_with_detections)
            if save_stream_output:
                if out is not None:
                    out.write(frame_with_detections)

            image_id += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                exit_event.set()
                break
        except queue.Empty:
            continue

    if cap is not None and save_stream_output:
        if out is not None:
            out.release()
    output_queue.task_done()
     


def main():
    CAMERA_CAP_WIDTH = 800
    CAMERA_CAP_HEIGHT = 600
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_CAP_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_CAP_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 60)  # Set FPS to 
    # 设置视频格式为 MJPG

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    cap.set(cv2.CAP_PROP_FPS, 60)  # Set FPS to 
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"实际帧率: {actual_fps}")

    HEF_PATH = "/home/pi/hailo-rpi5-examples/Hailo-Application-Code-Examples/runtime/hailo-8/python/object_detection/goal_car/yolov11n.hef"  # Path to your HEF file
    labels_path = "/home/pi/hailo-rpi5-examples/Hailo-Application-Code-Examples/runtime/hailo-8/python/object_detection/goal_car/label.txt"  # Path to your labels file
    utils = ObjectDetectionUtils(labels_path)
    batch_size = 1  # Define your batch size

    # Example usage of preprocess_from_cap
    input_queue = queue.Queue()
    output_queue = queue.Queue()
    preprocess_thread = threading.Thread(target=preprocess_from_cap, args=(cap, batch_size, input_queue, 640, 640, utils))
    print(f"加载模型: {HEF_PATH}")
    infer_thread = HailoAsyncInference( hef_path=HEF_PATH,
                                       input_queue=input_queue,
                                       output_queue=output_queue,
                                       batch_size=batch_size,
                                       send_original_frame= True
                                       )
    print("模型加载完成")
    post_thread = threading.Thread(target=postprocess, args=(output_queue,cap,0, utils))
    
    print("Starting preprocess thread...")
    preprocess_thread.start()

    print("Starting postprocess thread...")
    post_thread.start()

    print("Starting inference thread...")
    infer_thread.run()

    print("All threads started.")
    
    # Start the postprocessing thread
    


    # Wait for the thread to finish
    try:
        preprocess_thread.start()
        post_thread.start()
        infer_thread.run()

        preprocess_thread.join()
        output_queue.put(None)  # Signal process thread to exit
        post_thread.join()
    except Exception as e:
        print(f"发生异常: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("程序已正常退出")

if __name__ == "__main__":
        main()