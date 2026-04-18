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
# 定义全局退出标志,防止终端运行卡死
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
    )    -> None:
    image_id = 0
    out = None
    output_path = Path("output_images")  # Directory to save output images
    
    while not exit_event.is_set():  # 检查退出标志
        try:
            result = output_queue.get(timeout=1)  # 设置超时时间
            if result is None:
                break  # Exit the loop if sentinel value is received

            original_frame, infer_results = result

            if len(infer_results) == 1:
                infer_results = infer_results[0]

            detections = utils.extract_detections(infer_results, threshold=0.3)

            frame_with_detections = utils.draw_detections(
                detections, original_frame,
            )

            # 显示结果
            cv2.imshow("Output", frame_with_detections)

            # 按下 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                exit_event.set()  # 设置退出标志
                break
        except queue.Empty:
            continue

    # 如果有摄像头输入
    
    if cap is not None:
        
         # 初始化帧率计算变量
        prev_time = time.time()
        fps = 0
        # Create a named window
        cv2.namedWindow("Output", cv2.WND_PROP_AUTOSIZE)

        # Set the window to fullscreen
        cv2.setWindowProperty("Output", cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_AUTOSIZE)
        '''
        if save_stream_output:
            output_path.mkdir(exist_ok=True)
            frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
             # Define the codec and create VideoWriter object
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            # Save the output video in the output path
            fps = cap.get(cv2.CAP_PROP_FPS)
            if fps == 0:  # If FPS is not available, set a default value
                print(f"fps: {fps}")
                fps = 20.0
            out = cv2.VideoWriter(str(output_path / 'output_video.avi'), fourcc, fps, (frame_width, frame_height))
        
    if (cap is None):
        # Create output directory if it doesn't exist
        output_path.mkdir(exist_ok=True)
        '''
    while True:
        result = output_queue.get()
        if result is None:
            break  # Exit the loop if sentinel value is received

        original_frame, infer_results= result

       
       

        # Deals with the expanded results from hailort versions < 4.19.0
        if len(infer_results) == 1:
            infer_results = infer_results[0]

        detections = utils.extract_detections(infer_results,threshold=0.4)
        
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
            # Close the window and release the camera
            if save_stream_output:
                out.release()  # Release the VideoWriter object
            cap.release()
            cv2.destroyAllWindows()
            break

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
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_CAP_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_CAP_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 60)  # Set FPS to 
    # 设置视频格式为 MJPG

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    cap.set(cv2.CAP_PROP_FPS, 60)  # Set FPS to 
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"实际帧率: {actual_fps}")

    HEF_PATH = "/home/pi/hailo-rpi5-examples/Hailo-Application-Code-Examples/runtime/hailo-8/python/object_detection/goal_car/yolov8nv2.hef"  # Path to your HEF file
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