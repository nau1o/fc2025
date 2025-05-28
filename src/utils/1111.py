from ultralytics import YOLO
import cv2
import os
from pathlib import Path
from tqdm import tqdm
from paddleocr import PaddleOCR
import logging
from collections import deque
import numpy as np
import re
import time
from try_pnp import get_key_points, get_objectpoints, pnp2depth
from mask2coords import get_pentagon_coordinates
import math
from torch import Tensor
"""该版本的yolo模型为pose"""

"""目前的主要程序，对视频流进行识别"""

"""回溯了一下，因为五点识别还没做好，先用这个版本融入一下pnp"""

"""感觉这个版本很可能会夺舍"""

"""夺舍成功"""

"""准度很高啊"""

def calculate_centroid(polygon):
    """
    计算多边形的质心，输入为 [(x1, y1), (x2, y2), ..., (xn, yn)]
    使用面积加权公式
    """
    area = 0.0
    cx = 0.0
    cy = 0.0
    n = len(polygon)
    
    for i in range(n):
        x0, y0 = polygon[i]
        x1, y1 = polygon[(i + 1) % n]
        cross = x0 * y1 - x1 * y0
        area += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    
    area *= 0.5
    if area == 0:
        raise ValueError("多边形面积为0，无法计算质心")
    cx /= (6 * area)
    cy /= (6 * area)
    
    return (cx, cy)

def angle_to_upward_line(points):
    """
    计算将图片旋转多少度，使得第一个点与中心点的连线竖直向上。
    
    参数：
        points (list): 长度为5的列表，包含五个(x, y)坐标点。
        
    返回：
        angle (float): 顺时针旋转角度，单位为度。
    """
   

    # 计算中心点（质心）
    center_x = sum(p[0] for p in points) / 5
    center_y = sum(p[1] for p in points) / 5
    center = (center_x, center_y)

    # 构造向量：中心 - 第一个点
    dx = center[0] - points[0][0]
    dy = center[1] - points[0][1]

    # 参考方向：竖直向上（0, -1）
    ref_dx, ref_dy = 0, -1

    # 向量夹角公式
    dot = dx * ref_dx + dy * ref_dy
    det = dx * ref_dy - dy * ref_dx
    angle_rad = math.atan2(det, dot)
    angle_deg = math.degrees(angle_rad)

    # 返回顺时针旋转角度（正值表示顺时针）
    return -angle_deg

def rotate_image(image, angle):
    """
    顺时针旋转图像指定角度，并用黑色填充空白区域。

    参数:
        image: 输入图像（numpy 数组）
        angle: 旋转角度（单位：度，顺时针为正）

    返回:
        旋转后的图像
    """
    # 获取图像尺寸
    (h, w) = image.shape[:2]
    center = (w // 2, h // 2)

    # 构建旋转矩阵（注意 angle 为正时顺时针）
    M = cv2.getRotationMatrix2D(center, -angle, 1.0)

    # 计算旋转后的图像尺寸，避免裁剪
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])

    new_w = int((h * sin) + (w * cos))
    new_h = int((h * cos) + (w * sin))

    # 调整旋转矩阵中的平移项
    M[0, 2] += (new_w / 2) - center[0]
    M[1, 2] += (new_h / 2) - center[1]

    # 执行仿射变换，填充颜色为黑色（0, 0, 0）
    rotated = cv2.warpAffine(image, M, (new_w, new_h), borderValue=(0, 0, 0))

    return rotated
# 设置日志级别
logging.getLogger("ppocr").setLevel(logging.WARNING)
numbers=[]
num_of_tar1=[]
num_of_tar2=[]
num_of_tar3=[]  ##预定义三个数组，储存识别到的数字
best1=[]
best2=[]
best3=[] #初始化三个标靶对应的最大可能的数字


# 标靶坐标
measured_points = [[0.25, 0.25, 0.75],
                   [0, -0.25, 0.75],
                   [-0.25, 0, 0.75]]
def process_ocr_result(ocr_text, numbers,i):#统计每一个标靶对应的数字，并进行概率输出
    """
    处理OCR识别结果并更新统计数组
    :param ocr_text: OCR识别出的文本
    :param numbers: 统计数组，格式为 [[digit, count], ...]
    """
    digit = ocr_text
       
    found = False
    
    # 遍历第一维度查找匹配项
    for i in range(len(numbers)):
        if numbers[i][0] == digit:
            numbers[i][1] += 1
            found = True
            break
    
    if not found:
        numbers.append([digit, 1])
    total_count=sum(count for _,count in numbers)
    if total_count > 0:
        for digit,count in numbers :
            probablity = count/total_count
            print(f"{i}号标靶中数字为{digit}的概率为{probablity:.2f}\n")
    max_pro_num = max(numbers,key=lambda x: x[1])
    probablity = 0
    if total_count >0 :
        probablity = max_pro_num[1]/total_count
    #max_pro_num.append(probablity)

    return [max_pro_num[0],probablity]


def calculate_distance(point_a, point_b):# 计算两点距离

    return np.linalg.norm(np.array(point_a) - np.array(point_b))


def find_closest_point(measurements, target):# 比较并得出目标标靶
    # 计算目标点到所有测量点的距离
    distances = [np.linalg.norm(np.array(target) - np.array(point)) 
                for point in measurements]
    
    # 直接返回最小距离的索引
    return np.argmin(distances)


def pixel2camera(x, y, target_depth):# 转换为相机坐标
    target_pixel = np.array([x, y])
    #淦，我说为什么pnp解出来都是0.75，原来问题在这
    camera_intrinsics = np.array([
        [241,0.0,327],
        [0.0,322,230],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]                                                                             
    ])
    fx, fy, cx, cy = camera_intrinsics[0, 0], camera_intrinsics[1, 1], camera_intrinsics[0, 2], camera_intrinsics[1, 2]
    normalized_x = (target_pixel[0] - cx) / fx
    normalized_y = (target_pixel[1] - cy) / fy
    target_camera_coords = np.array([normalized_x * target_depth, normalized_y *target_depth, target_depth])
    return target_camera_coords                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            

# 转换为世界坐标
"""def camera2enu(camera, enu, distance, pitch=90, yaw=0, roll=0):
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    R_yaw = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    R_pitch = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    R_roll = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R = R_yaw @ R_pitch @ R_roll
    T_cam_model = np.array([distance * np.cos(pitch) * np.cos(yaw),
                            distance * np.cos(pitch) * np.sin(yaw),
                            -distance * np.sin(pitch)])
    point_model = R @ np.array(camera) + T_cam_model
    enu_coord = point_model + np.array(enu)
    return enu_coord.round(4).tolist()"""

class WindowPositionManager:
    """预定义窗口位置管理类"""
    def __init__(self):
        self.positions = [(x, y) for x in range(0, 1680, 60) for y in range(20, 1000, 60)]
        self.position_queue = deque(maxlen=len(self.positions))
        
    def get_next_position(self):
        """获取下一个可用的显示位置"""
        if len(self.position_queue) < len(self.positions):
            pos = self.positions[len(self.position_queue)]
            self.position_queue.append(pos)
            return pos
        else:
            old_pos = self.position_queue.popleft()
            self.position_queue.append(old_pos)
            return old_pos

def video_crop_with_ocr(
    model_path: str = r"C:\Users\qixin\Desktop\weights\516pose900.pt",
    video_path: str = 1,
    output_dir: str = r"C:\Users\qixin\Desktop\test-dinwg",
    conf_threshold: float = 0.6,
    ocr_lang: str = 'en',
    process_interval: int = 1
):
    # 初始化模型
    model = YOLO(model_path)
    ocr_engine = PaddleOCR(use_angle_cls=True,det_db_box_thresh=0.1, lang=ocr_lang,cls_model_dir=r"C:\Users\qixin\PaddleOCR\output\v3_en_mobile\best_model")
    
    # 初始化窗口管理器
    window_mgr = WindowPositionManager()
    active_windows = set()  # 当前活跃窗口
    window_size = 30  # 统一窗口尺寸

    # 创建输出目录
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    # 颜色代码定义
    COLOR = {
        'blue': '\033[94m',
        'green': '\033[92m',
        'yellow': '\033[93m',
        'red': '\033[91m',
        'end': '\033[0m'
    }

    # 打开视频文件
    cap = cv2.VideoCapture(video_path)   #和采集卡一样1080p,cv2.CAP_DSHOW
    """录制视频代码块"""
    # 获取输入视频的帧率和分辨率（用于设置输出视频参数）
    fps = int(cap.get(cv2.CAP_PROP_FPS))  # 帧率
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))  # 宽度
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))  # 高度
    output_video= r"C:\Users\qixin\Desktop\111.mp4"  #输出路径
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 编码格式（MP4）
    out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    if not cap.isOpened():
        print(f"{COLOR['red']}错误: 无法打开视频文件 {video_path}{COLOR['end']}")
        return

    # 获取视频信息
    pbar = tqdm(desc="实时处理帧", unit='frame')
    
    frame_count = 0

    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                break
            
            if frame_count % process_interval == 0:
                results = model.predict(source=frame, conf=conf_threshold)
                """图片保存"""
                out.write(frame)
                save_path = r"C:\Users\qixin\Desktop\111"  # 替换为你的路径
                filename = f"output_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
                cv2.imwrite(f"{save_path}\\{filename}", frame)  # Windows 用双反斜杠

                for result in results:
                    if result.boxes is None:
                        continue
                    
                    for obj_idx, (box, cls_id) in enumerate(zip(
                        result.boxes. xyxy.cpu().numpy(),
                        result.boxes.cls.cpu().numpy()
                    )):
                        
                        x1, y1, x2, y2 = map(int, box)
                        
                        keypoints = result.keypoints  # Keypoints object for pose outputs
                        points = np.array(Tensor.cpu(keypoints.xy[0]))
                        #fenu=[0,0,1.72]
                        #enu = camera2enu(target_camera_coords,fenu,0,pitch=90, yaw=0, roll=0)
                        crop = frame[y1:y2, x1:x2]
                        #pentagons = get_pentagon_coordinates(result,0,15,0.3)
                        angle = angle_to_upward_line(points)
                        rotated = rotate_image(crop,180-angle)



                        # 获取显示位置并创建窗口
                        pos = window_mgr.get_next_position()
                        win_name = f"rotated_{pos[0]}_{pos[1]}"
                        
                        # 调整显示尺寸并显示
                        display_img = cv2.resize(rotated, (window_size, window_size))
                        cv2.imshow(win_name, display_img)
                        cv2.moveWindow(win_name, pos[0], pos[1])
                        active_windows.add(win_name)

                        # OCR处理流程
                        global best1
                        global best2
                        global best3
                        processed = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
                        
                        processed = cv2.resize(processed, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC) #一个resize函数，将原图放大三倍，有助于ocr
                        ocr_result = ocr_engine.ocr(processed, cls=False)#ocr识别
                        if ocr_result[0] is not None:#确保结果非空再进行下一步，否则会报错
                            for line in ocr_result :
                                if line is not None: #确保非空
                                    text = line[0][1][0] #此处注意line数组的内部结构

                                    """以下一段为pnp解算相机深度
                                    imagePoints = get_key_points(frame, (x1,y1,x2,y2))#
                                    objectPoints = get_objectpoints(0.5)
                                    depth = 20 
                                        #这里还需要加一个判断，如果imagepoints没有解算出来，就用gps高度
                                        #我先加了个预设高度20
                                    if imagePoints is not None:  #加一个验证，确保imagepoints非空
                                        depth = pnp2depth(objectPoints, imagePoints)


                                    #######enu = pixel2camera((x1+x2)/2, (y1+y2)/2 , depth) 
                                    best_pro_tar = find_closest_point(measured_points,enu)+1
                                    if line[0][1][1]>0.8:#添加ocr的置信度阈值，确保识别的准确度
                                        with open(r"C:\Users\qixin\Desktop\test-results\1.txt", "a", encoding="utf-8") as file: 
                                            file.write(f"识别结果: {line[0][1]},坐标：{enu},是{best_pro_tar}号标靶"+"\n") 
                                        

                                            print(enu,best_pro_tar)"""
                                        #根据索引，将每个标靶最大可能的元素和概率的数组输出
                                        #很粗暴的方法
                                        #可以试一下更高维的数组
                                        #但是貌似没什么必要
                                        if best_pro_tar == 1 :
                                            best1=process_ocr_result(text,num_of_tar1,1)
                                        elif best_pro_tar == 2 :
                                            best2=process_ocr_result(text,num_of_tar2,2)
                                        elif best_pro_tar == 3 :
                                            best3=process_ocr_result(text,num_of_tar3,3)  
                                    
                        else:
                            continue
                        

                        
                # 实时更新统计显示
                os.system('cls' if os.name == 'nt' else 'clear')

                print("=== 实时数字统计 ===")
           
                
            if best1  :
                print(f"1号标靶最可能的数字为{best1[0]}概率为{best1[1]}\n")
            if best2  :
                print(f"2号标靶最可能的数字为{best2[0]}概率为{best2[1]}\n")
            if  best3  :
                print(f"3号标靶最可能的数字为{best3[0]}概率为{best3[1]}\n")
                              
            # 保持窗口响应
            if cv2.waitKey(1) & 0xFF == 27:  # ESC退出
                break
            
            frame_count += 1
            pbar.update(1)
            
    finally:
        # 释放资源
        cap.release()
        out.release()
        pbar.close()

if __name__ == "__main__":
    video_crop_with_ocr(video_path=r"C:\Users\qixin\Desktop\DJI_0011.MP4")