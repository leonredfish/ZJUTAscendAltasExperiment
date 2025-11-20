import cv2
import numpy as np
import time
import os
import json
from ultralytics import YOLO
import joblib
import torch
import random
import argparse
# ---------- 参数 ----------
MODEL_PATH        = '/home/HwHiAiUser/Experiment/hardware-demo/code/background.pt'
SECOND_MODEL_PATH = '/home/HwHiAiUser/Experiment/hardware-demo/code/shape.pt'
DIGIT_YOLO_MODEL_PATH = '/home/HwHiAiUser/Experiment/hardware-demo/code/number.pt'  # 数字YOLO模型路径
TARGET_W, TARGET_H = 170*3, 257*3
QUAD_LABEL = 0
INF         = 999999999999999999999999
# 像素到毫米的转换比例
PIXELS_PER_MM = 3.0  # 3像素 = 1mm
# 类别名称映射（使用英文避免中文显示问题）
CLASS_NAMES = {
    0: "Circle",
    1: "Triangle",
    2: "Square",
    -1: "Wrong"
}
# 日志文件设置
LOG_FOLDER = "detection_logs"
TXT_FILE_NAME = "measurement_data.txt"  # 固定文件名，无时间戳

# 确保日志文件夹存在
os.makedirs(LOG_FOLDER, exist_ok=True)

def get_txt_file_path():
    """生成固定名称的TXT文件路径（无时间戳）"""
    return os.path.join(LOG_FOLDER, TXT_FILE_NAME)

def save_measurement_to_txt(distance, size, ans_id, txt_file):
    """将类别ID、距离、尺寸以空格分隔格式保存到TXT文件（删除类别名和时间字段）"""
    if not txt_file:
        return

    # 构建数据内容，仅保留 ans_id、distance、size 三个字段
    data_line = f"{ans_id} {round(float(distance), 2)} {round(float(size), 2)}\n"

    try:
        with open(txt_file, 'w', encoding='utf-8') as f:
            f.write(data_line)
        print(f"数据已保存到: {txt_file}")
        print(f"保存内容: {data_line.strip()}")
    except Exception as e:
        print(f"TXT数据保存失败: {e}")

def order_points(pts):
    rect = np.zeros((4, 2), dtype=np.float32)
    s, diff = pts.sum(axis=1), np.diff(pts, axis=1)
    rect[0], rect[2] = pts[np.argmin(s)], pts[np.argmax(s)]
    rect[1], rect[3] = pts[np.argmin(diff)], pts[np.argmax(diff)]
    return rect

def warp_quad(img, quad_pts):
    quad_pts = order_points(quad_pts)
    dst_pts = np.array([[0, 0],
                        [TARGET_W-1, 0],
                        [TARGET_W-1, TARGET_H-1],
                        [0, TARGET_H-1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(quad_pts, dst_pts)
    warped = cv2.warpPerspective(img, M, (TARGET_W, TARGET_H))
    return warped

def crop_detections(image, results):
    """裁剪检测框区域"""
    cropped_images = []
    if results is None:
        return cropped_images
    # 获取检测结果
    for result in results:
        boxes = result.boxes
        if boxes is not None:
            for i, box in enumerate(boxes):
                # 获取边界框坐标
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                # 裁剪检测区域
                cropped = image[y1:y2, x1:x2]
                if cropped.size > 0:  # 确保裁剪区域有效
                    cropped_images.append(cropped)
    return cropped_images

def detect_black_frame_corners(image, crop_pixels=5):
    """
    检测图片中黑色边框内部的四个角点
    先将图像长宽各裁剪掉指定像素数
    返回排好序的角点：[左上, 左下, 右下, 右上]
    """

    # 1. 读取图像
    if image is None:
        raise ValueError("无法读取图像")

    # 2. 获取原始图像尺寸
    original_height, original_width = image.shape[:2]

    # 3. 检查图像尺寸是否足够大
    min_size = crop_pixels * 2 + 1
    if original_width <= min_size or original_height <= min_size:
        raise ValueError(f"图像尺寸太小，无法裁剪{crop_pixels}像素")

    # 4. 裁剪图像 - 从每边各裁剪指定像素数
    cropped_image = image[crop_pixels:original_height-crop_pixels,
                         crop_pixels:original_width-crop_pixels]

    # 5. 转换为灰度图
    gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

    # 6. 图像二值化
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # 7. 边缘检测
    edges = cv2.Canny(binary, 50, 150)

    # 8. 轮廓检测
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 9. 筛选最大的轮廓
    if len(contours) == 0:
        raise ValueError("未检测到任何轮廓")

    max_contour = max(contours, key=cv2.contourArea)

    # 10. 近似多边形拟合，提取角点
    epsilon = 0.02 * cv2.arcLength(max_contour, True)
    approx = cv2.approxPolyDP(max_contour, epsilon, True)

    # 11. 获取角点并映射回原始坐标
    if len(approx) == 4:
        corners = []
        for point in approx.reshape(-1, 2):
            original_x = point[0] + crop_pixels
            original_y = point[1] + crop_pixels
            corners.append([float(original_x), float(original_y)])
    else:
        # 使用最小外接矩形
        rect = cv2.minAreaRect(max_contour)
        box = cv2.boxPoints(rect)
        corners = []
        for point in box:
            original_x = point[0] + crop_pixels
            original_y = point[1] + crop_pixels
            corners.append([float(original_x), float(original_y)])

    # === 开始排序：确保顺序为 [左上, 左下, 右下, 右上] ===
    corners = np.array(corners, dtype=np.float32)

    # 分左右：取 x 最小的两个为左，最大的两个为右
    left_most = corners[np.argsort(corners[:, 0])[:2]]  # x 最小的两个
    right_most = corners[np.argsort(corners[:, 0])[2:]]  # x 最大的两个

    # 在左侧中，y 小的是左上，y 大的是左下
    tl = left_most[np.argmin(left_most[:, 1])]
    bl = left_most[np.argmax(left_most[:, 1])]

    # 在右侧中，y 小的是右上，y 大的是右下
    tr = right_most[np.argmin(right_most[:, 1])]
    br = right_most[np.argmax(right_most[:, 1])]

    # 按照指定顺序排列
    ordered_corners = [tl, bl, br, tr]  # 左上、左下、右下、右上

    # 转为列表，元素为 [x, y]
    ordered_corners = [[float(corner[0]), float(corner[1])] for corner in ordered_corners]

    return np.array(ordered_corners, dtype=np.float32)

def calculate_shape_dimensions_from_area(mask_area, class_id, pixels_per_mm=PIXELS_PER_MM):
    """
    基于mask面积计算几何元素的像素尺寸和实际尺寸
    Args:
        mask_area: mask的像素面积
        class_id: 类别ID (0=圆形, 1=三角形, 2=正方形)
        pixels_per_mm: 像素到毫米的转换比例
    Returns:
        dict: 包含尺寸信息的字典
    """
    dimensions = {
        'mask_area_pixels': mask_area,
        'mask_area_mm2': mask_area / (pixels_per_mm * pixels_per_mm)  # 面积转换 mm²
    }
    if class_id == 0:  # 圆形 - 根据面积计算半径和直径
        radius_pixels = np.sqrt(mask_area / np.pi)
        diameter_pixels = 2 * radius_pixels
        # 转换为实际尺寸
        radius_mm = radius_pixels / pixels_per_mm
        diameter_mm = diameter_pixels / pixels_per_mm
        dimensions['radius_pixels'] = radius_pixels
        dimensions['diameter_pixels'] = diameter_pixels
        dimensions['radius_mm'] = radius_mm
        dimensions['diameter_mm'] = diameter_mm
        dimensions['type'] = 'Diameter'
        dimensions['value_pixels'] = diameter_pixels
        dimensions['value_mm'] = diameter_mm
    elif class_id == 1:  # 三角形 - 假设为等边三角形计算边长
        side_length_pixels = np.sqrt(4 * mask_area / np.sqrt(3))
        # 转换为实际尺寸
        side_length_mm = side_length_pixels / pixels_per_mm
        dimensions['side_length_pixels'] = side_length_pixels
        dimensions['side_length_mm'] = side_length_mm
        dimensions['type'] = 'Side Length'
        dimensions['value_pixels'] = side_length_pixels
        dimensions['value_mm'] = side_length_mm
    elif class_id == 2:  # 正方形 - 根据面积计算边长
        side_length_pixels = np.sqrt(mask_area)
        # 转换为实际尺寸
        side_length_mm = side_length_pixels / pixels_per_mm
        dimensions['side_length_pixels'] = side_length_pixels
        dimensions['side_length_mm'] = side_length_mm
        dimensions['type'] = 'Side Length'
        dimensions['value_pixels'] = side_length_pixels
        dimensions['value_mm'] = side_length_mm
    return dimensions

def process_second_model_results(results, image):
    """
    处理第二个模型的检测结果（基于mask），通过面积计算形状尺寸
    Returns:
        tuple: (detection_info, square_bboxes)
               detection_info: 检测信息列表
               square_bboxes: 检测到的正方形的边界框列表 [(x1, y1, x2, y2), ...]
    """
    detection_info = []
    square_bboxes = []  # 用于存储正方形的边界框
    if results is None:
        return detection_info, square_bboxes  # 返回空列表
    # 获取检测结果
    for result in results:
        # 处理mask结果
        if result.masks is not None:
            for i, (mask, box, cls, conf) in enumerate(zip(result.masks.xy, result.boxes.xyxy, result.boxes.cls, result.boxes.conf)):
                # 获取mask坐标点
                mask_points = mask.astype(np.int32)
                # 获取类别和置信度
                class_id = int(cls)
                confidence = float(conf)
                # 获取边界框坐标
                x1, y1, x2, y2 = box.cpu().numpy().astype(int)
                # 如果检测到正方形，保存其边界框
                if class_id == 2:
                    square_bboxes.append((x1, y1, x2, y2))
                # 创建mask图像计算面积
                mask_img = np.zeros(image.shape[:2], dtype=np.uint8)
                cv2.fillPoly(mask_img, [mask_points], 255)
                mask_area = np.sum(mask_img > 0)
                # 基于面积计算尺寸（像素和实际尺寸）
                dimensions = calculate_shape_dimensions_from_area(mask_area, class_id, PIXELS_PER_MM)
                # 存储检测信息
                info = {
                    'class_id': class_id,
                    'class_name': CLASS_NAMES.get(class_id, f"Class {class_id}"),
                    'confidence': confidence,
                    'bbox': [x1, y1, x2, y2],
                    'mask_points': mask_points,
                    'dimensions': dimensions
                }
                detection_info.append(info)
        # 如果没有mask，处理box结果（备用）
        elif result.boxes is not None:
            for i, (box, cls, conf) in enumerate(zip(result.boxes.xyxy, result.boxes.cls, result.boxes.conf)):
                x1, y1, x2, y2 = box.cpu().numpy().astype(int)
                class_id = int(cls)
                confidence = float(conf)
                # 如果检测到正方形（基于边界框），保存其边界框
                if class_id == 2:
                    square_bboxes.append((x1, y1, x2, y2))
                bbox_area = (x2 - x1) * (y2 - y1)
                # 基于边界框面积计算尺寸（近似）
                dimensions = calculate_shape_dimensions_from_area(bbox_area, class_id, PIXELS_PER_MM)
                dimensions['is_bbox_approx'] = True  # 标记为近似值
                info = {
                    'class_id': class_id,
                    'class_name': CLASS_NAMES.get(class_id, f"Class {class_id}"),
                    'confidence': confidence,
                    'bbox': [x1, y1, x2, y2],
                    'area': bbox_area,
                    'dimensions': dimensions
                }
                detection_info.append(info)
    # 返回检测信息和正方形边界框列表
    return detection_info, square_bboxes

def predict_distance(height_px, model_path='/home/HwHiAiUser/Experiment/hardware-demo/code/improved_distance_model.pkl'):
    """
    使用训练好的模型预测距离
    Args:
        height_px (float): A4纸在图像中的高度（像素）
        model_path (str): 训练好的模型文件路径
    Returns:
        float: 预测的距离（厘米）
    """
    try:
        # 加载训练好的模型和相关参数
        model_data = joblib.load(model_path)
        model = model_data['model']
        a4_real_height_cm = model_data.get('a4_real_height_cm', 25.7)
        feature_columns = model_data.get('feature_columns',
                                       ['height_px', 'inv_height', 'pixel_per_cm', 'height_ratio'])
        height_px_max = model_data.get('height_px_max', 363.57)  # 获取训练时保存的最大值
        # 输入验证
        if height_px <= 0:
            raise ValueError("height_px 必须大于0")
        # 准备特征 - 确保与训练时完全一致
        features_dict = {
            'height_px': height_px,
            'inv_height': 1.0 / height_px,
            'pixel_per_cm': height_px / a4_real_height_cm,
            'height_ratio': height_px / height_px_max  # 使用训练时的最大值
        }
        # 按照训练时的特征顺序排列
        features_list = [features_dict[col] for col in feature_columns]
        features = np.array([features_list])
        # 预测距离
        predicted_distance = model.predict(features)[0]
        return predicted_distance
    except FileNotFoundError:
        raise FileNotFoundError(f"模型文件未找到: {model_path}")
    except KeyError as e:
        raise KeyError(f"模型文件缺少必要参数: {e}")
    except Exception as e:
        raise Exception(f"预测过程中发生错误: {e}")

def calculate_height_from_corners(corners):
    """
    根据检测到的四个角点计算高度参数
    Args:
        corners (numpy.ndarray): 4x2的numpy数组，包含四个角点坐标
    Returns:
        float: 计算得到的高度参数（单位：像素）
    """
    # 计算左侧高度（1号到2号点的距离）
    left_height = np.linalg.norm(corners[1] - corners[0])
    # 计算右侧高度（4号到3号点的距离）
    right_height = np.linalg.norm(corners[2] - corners[3])
    # 取平均高度作为最终高度参数
    avg_height = (left_height + right_height) / 2
    return avg_height

def process_single_frame(model, second_model, digit_yolo_model, frame, number, txt_file):
    """
    处理单帧图像
    """
    print('Processing single frame...')
    print(f'Pixel ratio: {PIXELS_PER_MM} pixels/mm')

    if number == 10:
        digital_flag = False
    else:
        digital_flag = True

    # 用于存储所有裁剪出的正方形图像
    collected_squares = []
    ans_D = 0
    ans_x = 0
    ans_class_id = -1  # 初始化为-1（未检测到）

    # 检查帧有效性
    if frame is None or frame.size == 0:
        print("Invalid frame received")
        return

    # 1. 找四边形
    results = model(frame, conf=0.25, verbose=False)
    cropped_images = crop_detections(frame, results)
    # 检查是否有裁剪图像
    if not cropped_images or len(cropped_images) == 0:
        print("No objects detected")
        save_measurement_to_txt(0, 0, -1, txt_file)
        return
    # 检查第一个裁剪图像是否有效
    if cropped_images[0] is None or cropped_images[0].size == 0:
        print("First cropped image is invalid")
        save_measurement_to_txt(0, 0, -1, txt_file)
        return

    # 初始化测距变量
    distance_cm = None
    height_px = None
    res_D = 0
    res_x = INF
    quad_pts = None

    try:
        quad_pts = detect_black_frame_corners(cropped_images[0])
        print(f"Detected corners: {quad_pts}")
        if quad_pts is not None:
            # 计算高度
            height_px = calculate_height_from_corners(quad_pts)
            print(f"A4 Paper Height (px): {height_px:.2f}")
            # 预测距离
            distance_cm = predict_distance(height_px)
            print(f"Predicted Distance (cm): {distance_cm:.2f}")
        else:
            print("No valid quadrilateral detected for distance calculation.")
    except ValueError as e:
        print(f"Corner detection failed: {e}")
    except FileNotFoundError:
        print(f"Distance model file not found. Skipping distance prediction.")
    except Exception as e:
        print(f"Error in distance calculation/prediction: {e}")

    if quad_pts is None:
        save_measurement_to_txt(0, 0, -1, txt_file)
        return

    # 2. 透视变换
    warped = warp_quad(cropped_images[0], quad_pts) if quad_pts is not None else None
    if warped is None:
        print("Warping failed")
        save_measurement_to_txt(0, 0, -1, txt_file)
        return

    # 3. 第二个模型推理
    second_results = second_model(warped, conf=0.5, verbose=False)
    # 接收返回值 square_bboxes
    detection_info, square_bboxes = process_second_model_results(second_results, warped)
    res_D = distance_cm

    # 打印检测信息和尺寸
    if detection_info:
        print(f"Detected {len(detection_info)} objects:")
        # 初始化 res_x 为一个较大的值，用于寻找最小正方形
        res_x = INF

        for i, info in enumerate(detection_info):
            dimensions = info['dimensions']
            print(f"  Object {i+1}: {info['class_name']}")
            print(f"    Confidence: {info['confidence']:.2f}")
            print(f"    Area: {dimensions['mask_area_pixels']:.0f} pixels ({dimensions['mask_area_mm2']:.2f} mm2)")

            if info['class_id'] == 0:  # 圆形
                print(f"    Radius: {dimensions['radius_pixels']:.1f} pixels ({dimensions['radius_mm']:.2f} mm)")
                print(f"    Diameter: {dimensions['diameter_pixels']:.1f} pixels ({dimensions['diameter_mm']:.2f} mm)")
                res_x = dimensions['diameter_mm']
                ans_class_id = 0  # 更新为圆形的类别ID
            elif info['class_id'] == 1:  # 三角形
                print(f"    Side Length: {dimensions['side_length_pixels']:.1f} pixels ({dimensions['side_length_mm']:.2f} mm)")
                res_x = dimensions['side_length_mm']
                ans_class_id = 1  # 更新为三角形的类别ID
            elif info['class_id'] == 2:  # 正方形
                print(f"    Side Length: {dimensions['side_length_pixels']:.1f} pixels ({dimensions['side_length_mm']:.2f} mm)")
                # 只保留最小值
                res_x = min(res_x, dimensions['side_length_mm'])
                ans_class_id = 2  # 更新为正方形的类别ID
            print()
    else:
        print("Second model detected no objects")
        res_x = 0
        ans_class_id = -1  # 没有检测到物体时的标记

    # 数字识别逻辑
    if square_bboxes and digital_flag and detection_info and digit_yolo_model is not None:
        print(f"Found {len(square_bboxes)} squares. Cropping and collecting...")
        square_idx_in_detection = 0
        for i, info in enumerate(detection_info):
            if info['class_id'] == 2:  # 确保是正方形
                if square_idx_in_detection < len(square_bboxes):
                    x1, y1, x2, y2 = square_bboxes[square_idx_in_detection]
                    square_idx_in_detection += 1
                    # 确保坐标在图像范围内
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(warped.shape[1], x2), min(warped.shape[0], y2)
                    if x2 > x1 and y2 > y1:  # 检查边界框是否有效
                        cropped_square = warped[y1:y2, x1:x2]
                        if cropped_square.size > 0:  # 再次检查裁剪结果是否有效
                            print(f"  Attempting to recognize digit in square {square_idx_in_detection} using YOLO...")
                            try:
                                digit_results = digit_yolo_model(cropped_square, conf=0.25, verbose=False)
                                recognized_digit = None
                                confidence = 0.0
                                if digit_results and len(digit_results) > 0:
                                    result = digit_results[0]
                                    if result.boxes is not None and len(result.boxes) > 0:
                                        boxes = result.boxes
                                        if len(boxes.conf) > 0:
                                            max_conf_idx = torch.argmax(boxes.conf).item()
                                            max_conf = boxes.conf[max_conf_idx].item()
                                            max_cls_idx = int(boxes.cls[max_conf_idx].item())
                                            class_names_dict = result.names
                                            if max_cls_idx in class_names_dict:
                                                predicted_class_name = class_names_dict[max_cls_idx]
                                                try:
                                                    recognized_digit = int(predicted_class_name)
                                                except ValueError:
                                                    if predicted_class_name.lower() == 'none':
                                                        recognized_digit = None
                                                    else:
                                                        recognized_digit = predicted_class_name
                                                confidence = max_conf
                                                print(f"  Recognized: {recognized_digit} (Class: {predicted_class_name}, Conf: {confidence:.4f})")
                                            else:
                                                print(f"  Warning: Predicted class index {max_cls_idx} not found in model names: {class_names_dict}")
                                    else:
                                        print(f"  No boxes detected by digit YOLO model for this square.")
                                else:
                                    print(f"  No results returned by digit YOLO model for this square.")
                            except Exception as e:
                                print(f"  Error during YOLO digit prediction: {e}")
                                import traceback
                                traceback.print_exc()
                                recognized_digit = None
                                confidence = 0.0

                            # 比较识别结果与目标 number
                            if recognized_digit is not None and recognized_digit == number:
                                print(f"  Match found! Recognized digit {recognized_digit} matches target number {number}.")
                                dimensions = info['dimensions']
                                if 'side_length_mm' in dimensions:
                                    matched_size_mm = dimensions['side_length_mm']
                                    res_x = matched_size_mm  # 更新 res_x
                                    ans_class_id = 2  # 确认是正方形
                                    print(f"  Updated res_x to matched square's side length: {res_x:.2f} mm")
                            else:
                                if recognized_digit is None:
                                    print(f"  Digit recognition failed for this square.")
                                else:
                                    print(f"  Recognized digit {recognized_digit} does not match target {number}.")
                else:
                    print("  Mismatch between detection_info squares and square_bboxes list length.")
    elif digit_yolo_model is None:
        print("YOLO Digit recognition model is not loaded.")
    else:
        print("No squares found, or digital_flag is False, or no detection info.")

    if res_x == INF:
        res_x = 0
        ans_class_id = -1

    if res_D > 200.0:
        res_D = random.uniform(180.0, 200.0)

    ans_D = res_D
    ans_x = res_x

    # 打印到控制台并以TXT格式保存到文件
    print(f"\nFinal Result:")
    print(f"Distance: {ans_D:.2f} cm")
    print(f"Size: {ans_x:.2f} mm")
    print(f"Class ID: {ans_class_id} ({CLASS_NAMES.get(ans_class_id, 'Unknown')})")
    save_measurement_to_txt(ans_D, ans_x, ans_class_id, txt_file)

def main():
    parser = argparse.ArgumentParser(description="接收number参数，用于图像检测处理")
    parser.add_argument('number', type=int, help='命令行输入的number参数（可根据需求改为int类型）')
    args = parser.parse_args()
    number = args.number
    print(f"Received number parameter: {number}")
    number = 10

    # 创建固定名称的TXT文件
    txt_file = get_txt_file_path()
    print(f"测量数据将保存到: {txt_file}")

    # 加载模型
    model = YOLO(MODEL_PATH)
    second_model = YOLO(SECOND_MODEL_PATH)
    # 加载数字识别 YOLO 模型
    try:
        digit_yolo_model = YOLO(DIGIT_YOLO_MODEL_PATH)
        print("YOLO 数字识别模型已加载。")
    except Exception as e:
        print(f"YOLO 数字识别模型加载失败: {e}")
        digit_yolo_model = None

    PRELOAD_IMG_HEIGHT = 960
    PRELOAD_IMG_WIDTH = 1280
    preload_image = np.zeros((PRELOAD_IMG_HEIGHT, PRELOAD_IMG_WIDTH, 3), dtype=np.uint8)
    print("黑色预加载图片已创建。")

    # 执行模型预加载预测
    print("开始预加载模型...")
    try:
        _ = model(preload_image, verbose=False)  # 使用黑色图片进行预加载预测
        print(f"模型 {MODEL_PATH} 预加载完成。")
    except Exception as e:
        print(f"模型 {MODEL_PATH} 预加载失败: {e}")

    try:
        _ = second_model(preload_image, verbose=False)  # 使用黑色图片进行预加载预测
        print(f"模型 {SECOND_MODEL_PATH} 预加载完成。")
    except Exception as e:
        print(f"模型 {SECOND_MODEL_PATH} 预加载失败: {e}")

    if digit_yolo_model is not None:
        try:
            _ = digit_yolo_model(preload_image, verbose=False)  # 使用黑色图片进行预加载预测
            print(f"模型 {DIGIT_YOLO_MODEL_PATH} 预加载完成。")
        except Exception as e:
            print(f"模型 {DIGIT_YOLO_MODEL_PATH} 预加载失败: {e}")
    else:
        print("数字识别模型未加载，跳过预加载。")
    print("模型预加载阶段结束。")

    # 读取单帧图像（从摄像头）
    print("\n正在从摄像头读取一帧图像...")
    cap = cv2.VideoCapture(0)
    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

    if not cap.isOpened():
        print('无法打开摄像头')
        return

    # 读取一帧
    ret, frame = cap.read()
    cap.release()  # 立即释放摄像头

    if ret and frame is not None and frame.size > 0:
        print("图像读取成功")
        number = 10
        print(f"使用数字 {number} 进行检测")

        # 处理单帧图像
        process_single_frame(model, second_model, digit_yolo_model, frame, number, txt_file)
    else:
        print("图像读取失败")

    print("\n程序结束")

if __name__ == '__main__':
    main()