import os
import json
import numpy as np
from PIL import Image
import shutil
from tqdm import tqdm
import re
import argparse

def is_valid_demo_dir(dirname):
    """检查目录名是否为数字和下划线的组合"""
    return bool(re.match(r'^[\d_]+$', dirname))

def merge_consecutive_actions(action_sequence):
    """合并连续相同的动作，计算累积动作值，同时记录位置和yaw信息"""
    if not action_sequence:
        return []
    
    merged = []
    current_action = action_sequence[0][0]
    count = 1
    start_pos = action_sequence[0][1]  # 起始位置
    start_yaw = action_sequence[0][2]  # 起始yaw角
    
    for i in range(1, len(action_sequence)):
        if action_sequence[i][0] == current_action:
            count += 1
        else:
            # 保存当前动作及其累积值，以及位置和yaw信息
            merged.append({
                'action': current_action,
                'value': round(0.02 * count, 2),  # 基础动作值0.02乘以重复次数，保留2位小数
                'frames': count,
                'start_pos': start_pos,  # 动作开始时的位置
                'end_pos': action_sequence[i][1],  # 动作结束时的位置
                'start_yaw': start_yaw,  # 动作开始时的yaw角
                'end_yaw': action_sequence[i][2]  # 动作结束时的yaw角
            })
            # 更新下一个动作的起始信息
            current_action = action_sequence[i][0]
            count = 1
            start_pos = action_sequence[i][1]
            start_yaw = action_sequence[i][2]
    
    # 处理最后一组动作
    merged.append({
        'action': current_action,
        'value': round(0.02 * count, 2),
        'frames': count,
        'start_pos': start_pos,
        'end_pos': action_sequence[-1][1],
        'start_yaw': start_yaw,
        'end_yaw': action_sequence[-1][2]
    })
    
    return merged

def process_demo(demo_dir):
    """处理单个demo目录"""
    # 获取demo目录名（用于构建图像子目录路径）
    demo_name = os.path.basename(demo_dir)
    
    # 读取任务指令
    with open(os.path.join(demo_dir, 'task.json'), 'r', encoding='utf-8') as f:
        task_data = json.load(f)
        if "Task instruction" in task_data:
            instruction = task_data['Task instruction']
        else:
            instruction = task_data['instruction']
    
    # 读取动作序列
    with open(os.path.join(demo_dir, 'output.json'), 'r', encoding='utf-8') as f:
        output_data = json.load(f)
    
    # 提取动作序列并按帧号排序
    action_sequence = []
    frame_keys = sorted(output_data.keys(), key=lambda x: int(x.split('_')[1]))
    for frame_key in frame_keys:
        frame_data = output_data[frame_key]
        action_sequence.append(frame_data)
    
    # 合并连续动作
    merged_actions = merge_consecutive_actions(action_sequence)
    
    # 构建数据样本
    samples = []
    current_frame = 1  # 从1开始，因为帧号从frame_0001开始
    
    for action_data in merged_actions:
        # 获取当前动作对应的图像帧
        frame_idx = f"frame_{str(current_frame).zfill(4)}"  # 格式化为frame_XXXX
        
        # 构建图像路径（包含子目录）
        left_img = os.path.join(demo_dir,f"{demo_name}_l", f"{frame_idx}_l.png")
        center_img = os.path.join(demo_dir,f"{demo_name}_m", f"{frame_idx}_m.png")
        right_img = os.path.join(demo_dir,f"{demo_name}_r", f"{frame_idx}_r.png")
        
        img_exist = [os.path.exists(img) for img in [left_img, center_img, right_img]]
        # 检查图像文件是否存在
        if not all(img_exist):
            print(f"Warning: Missing images for frame !")
            continue
        
        # 构建样本
        sample = {
            "left": left_img,
            "center": center_img,
            "right": right_img,
            "instruction": instruction,
            "action": action_data['action'],
            "value": action_data['value'],
            "start_pos": action_data['start_pos'],  # 添加位置信息
            "end_pos": action_data['end_pos'],
            "start_yaw": action_data['start_yaw'],  # 添加yaw角信息
            "end_yaw": action_data['end_yaw']
        }
        samples.append(sample)
        
        current_frame += action_data['frames']
    
    return samples

def build_dataset(data_root, output_dir, train_ratio=0.8):
    """构建数据集"""
    os.makedirs(output_dir, exist_ok=True)
    
    # 获取所有demo目录，只保留数字和下划线组合的目录
    demo_dirs = [d for d in os.listdir(data_root) 
                if os.path.isdir(os.path.join(data_root, d)) and is_valid_demo_dir(d)]
    only_hssd_demo_dirs = []
    for demo_dir in demo_dirs:
        if len(demo_dir.split("_"))==2:
            only_hssd_demo_dirs.append(demo_dir)
    print(only_hssd_demo_dirs)
    # demo_dirs = only_hssd_demo_dirs
    print(demo_dirs)
    print(f"Found {len(demo_dirs)} valid demo directories")
    
    # 随机打乱目录顺序
    np.random.shuffle(demo_dirs)
    
    # 按目录划分训练集和验证集
    split_idx = int(len(demo_dirs) * train_ratio)
    train_dirs = demo_dirs
    val_dirs = demo_dirs[split_idx:]
    
    # 处理训练集目录
    train_samples = []
    for demo_dir in tqdm(train_dirs, desc="Processing training demos"):
        demo_path = os.path.join(data_root, demo_dir)
        samples = process_demo(demo_path)
        train_samples.append(samples)
    
    # 处理验证集目录
    val_samples = []
    for demo_dir in tqdm(val_dirs, desc="Processing validation demos"):
        demo_path = os.path.join(data_root, demo_dir)
        samples = process_demo(demo_path)
        val_samples.append(samples)
    
    # 保存数据集
    with open(os.path.join(output_dir, 'train.json'), 'w', encoding='utf-8') as f:
        json.dump(train_samples, f, ensure_ascii=False, indent=2)
    
    with open(os.path.join(output_dir, 'val.json'), 'w', encoding='utf-8') as f:
        json.dump(val_samples, f, ensure_ascii=False, indent=2)
    
    print(f"Dataset built successfully:")
    print(f"Total demo directories: {len(demo_dirs)}")
    print(f"Training demo directories: {len(train_dirs)}")
    print(f"Validation demo directories: {len(val_dirs)}")
    print(f"Training samples: {sum(len(s) for s in train_samples)}")
    print(f"Validation samples: {sum(len(s) for s in val_samples)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build imitation learning dataset.")
    parser.add_argument('--data_root', type=str, required=True, help='Root directory of the data')
    parser.add_argument('--output_dir', type=str, required=True, help='Directory to save the output dataset')
    args = parser.parse_args()
    build_dataset(args.data_root, args.output_dir) 