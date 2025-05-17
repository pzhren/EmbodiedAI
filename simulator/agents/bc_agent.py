import torch
import torch.nn as nn
from simulator.core.agent import BaseAgent
from gym import spaces
import os
import sys
import re
from PIL import Image
import torchvision.transforms as transforms
# Add the imitation learning project path to system path
sys.path.append('./imitation_learning_project')
from transformers import AutoTokenizer
from models.bc_transformer import BCTransformer
from configs.model_config import ModelConfig

class BCAgent(BaseAgent):
    def __init__(self, config):
        super().__init__(config)
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        # Initialize model
        self.model_config = ModelConfig()
        self.model = BCTransformer(self.model_config).to(self.device)
        
        # Load checkpoint if specified in config
        if hasattr(config, 'checkpoint_path') and config.checkpoint_path:
            self._load_checkpoint(config.checkpoint_path)
        else:
            raise ValueError("Checkpoint path must be specified in config for BCAgent")
            
        self.model.eval()
        self.tokenizer = AutoTokenizer.from_pretrained("bert-base-uncased")
        # Define action space
        self.action_space = spaces.Tuple((spaces.Discrete(4), spaces.Box(low=0, high=1, shape=(1,))))

    def _load_checkpoint(self, checkpoint_path):
        """Load model checkpoint"""
        if not os.path.exists(checkpoint_path):
            raise FileNotFoundError(f"Checkpoint not found at {checkpoint_path}")
            
        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        self.model.load_state_dict(checkpoint['model_state_dict'])
    
    def preprocess_instruction(self, instruction):
        """预处理指令文本"""
        # 移除"-数字"模式
        instruction = re.sub(r'-\d+', '', instruction)
        # 移除"_"和数字
        instruction = re.sub(r'_\d+', '', instruction)
        # 移除多余的空格
        instruction = ' '.join(instruction.split())
        return instruction
    def act(self, observation):
        """Generate action based on observation using the trained policy"""
        # Prepare input data
        rgb_image_m = Image.fromarray(observation["robot0_front_camera"]["rgb"])
        rgb_image_l = Image.fromarray(observation["robot0_left_camera"]["rgb"])
        rgb_image_r = Image.fromarray(observation["robot0_right_camera"]["rgb"])
        left_img = self.transform(rgb_image_l)
        center_img = self.transform(rgb_image_m)
        right_img = self.transform(rgb_image_r)
        instruction = self.preprocess_instruction(observation['instruction'])
        encoded = self.tokenizer(
            instruction,
            padding='max_length',
            truncation=True,
            max_length=512,
            return_tensors='pt'
        )
        
        batch = {
            'left_img': left_img.unsqueeze(0).to(self.device),
            'center_img': center_img.unsqueeze(0).to(self.device),
            'right_img': right_img.unsqueeze(0).to(self.device),
            'input_ids': encoded['input_ids'].to(self.device),
            'attention_mask': encoded['attention_mask'].to(self.device),
            # 'position': torch.from_numpy(observation['position']).unsqueeze(0).to(self.device),
            # 'yaw': torch.from_numpy(observation['yaw']).unsqueeze(0).to(self.device)
        }
        
        # Use model to predict action
        with torch.no_grad():
            outputs = self.model(batch)
            act_id = int(outputs["action_logits"].argmin().item())
            value = outputs["value_pred"].squeeze(-1).item()
            action = [act_id, value]
            
        return action

    def reset(self):
        """Reset agent state if needed"""
        pass 