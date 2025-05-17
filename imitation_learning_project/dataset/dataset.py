import os
import json
import torch
from torch.utils.data import Dataset
from PIL import Image
import torchvision.transforms as transforms
import numpy as np
from transformers import AutoTokenizer
import re

class ImitationDataset(Dataset):
    def __init__(self, data_root, json_file, transform=None):
        self.data_root = data_root
        self.transform = transform or transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        
        # 初始化文本编码器
        self.tokenizer = AutoTokenizer.from_pretrained("bert-base-chinese")
        
        # 加载JSON数据
        with open(os.path.join(data_root, json_file), 'r', encoding='utf-8') as f:
            self.data = json.load(f)
        
        # 展平数据列表
        self.samples = []
        for demo_samples in self.data:
            self.samples.extend(demo_samples)
        
        # 动作映射
        self.action_map = {
            'move forward': 0,
            'turn left': 1,
            'turn right': 2
        }
    
    def preprocess_instruction(self, instruction):
        """预处理指令文本"""
        # 移除"-数字"模式
        instruction = re.sub(r'-\d+', '', instruction)
        # 移除"_"和数字
        instruction = re.sub(r'_\d+', '', instruction)
        # 移除多余的空格
        instruction = ' '.join(instruction.split())
        return instruction
    
    def __len__(self):
        return len(self.samples)
    
    def __getitem__(self, idx):
        sample = self.samples[idx]
        
        # 加载图像
        left_img = Image.open(os.path.join(self.data_root, sample['left'])).convert('RGB')
        center_img = Image.open(os.path.join(self.data_root, sample['center'])).convert('RGB')
        right_img = Image.open(os.path.join(self.data_root, sample['right'])).convert('RGB')
        
        # 应用变换
        if self.transform:
            left_img = self.transform(left_img)
            center_img = self.transform(center_img)
            right_img = self.transform(right_img)
        
        # 处理位置和yaw角数据
        start_pos = torch.tensor(sample['start_pos'], dtype=torch.float32)
        end_pos = torch.tensor(sample['end_pos'], dtype=torch.float32)
        start_yaw = torch.tensor(sample['start_yaw'], dtype=torch.float32)
        end_yaw = torch.tensor(sample['end_yaw'], dtype=torch.float32)
        
        # 预处理指令文本
        instruction = self.preprocess_instruction(sample['instruction'])
        
        # 编码指令文本
        encoded = self.tokenizer(
            instruction,
            padding='max_length',
            truncation=True,
            max_length=512,
            return_tensors='pt'
        )
        
        # 动作标签
        action = torch.tensor(self.action_map[sample['action']], dtype=torch.long)
        value = torch.tensor(sample['value'], dtype=torch.float32)
        
        return {
            'left_img': left_img,
            'center_img': center_img,
            'right_img': right_img,
            'instruction': instruction,
            'input_ids': encoded['input_ids'].squeeze(0),
            'attention_mask': encoded['attention_mask'].squeeze(0),
            'action': action,
            'value': value,
            'start_pos': start_pos,
            'end_pos': end_pos,
            'start_yaw': start_yaw,
            'end_yaw': end_yaw
        }

def get_dataloader(data_root, json_file, batch_size=32, num_workers=4, shuffle=True):
    dataset = ImitationDataset(data_root, json_file)
    dataloader = torch.utils.data.DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=num_workers,
        pin_memory=True
    )
    return dataloader 