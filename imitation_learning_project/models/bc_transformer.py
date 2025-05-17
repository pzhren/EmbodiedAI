import torch
import torch.nn as nn
from transformers import AutoModel, AutoTokenizer
import torch.nn.functional as F
from torchvision.models import resnet50, ResNet50_Weights

class BCTransformer(nn.Module):
    def __init__(self, config):
        super().__init__()
        self.config = config
        
        # 图像编码器 (ResNet50)
        self.image_encoder = resnet50(weights=ResNet50_Weights.IMAGENET1K_V2)
        # 移除最后的全连接层
        self.image_encoder.fc = nn.Identity()
        # 投影到transformer的隐藏维度
        self.image_proj = nn.Linear(2048, config.hidden_size)
        
        # 文本编码器
        self.text_encoder = AutoModel.from_pretrained(config.text_encoder_name)
        self.text_proj = nn.Linear(self.text_encoder.config.hidden_size, config.hidden_size)
        
        # # 位置和yaw角编码器
        # self.pos_encoder = nn.Sequential(
        #     nn.Linear(3, config.hidden_size),  # 3维位置向量
        #     nn.LayerNorm(config.hidden_size),
        #     nn.ReLU()
        # )
        
        # self.yaw_encoder = nn.Sequential(
        #     nn.Linear(1, config.hidden_size),  # 1维yaw角
        #     nn.LayerNorm(config.hidden_size),
        #     nn.ReLU()
        # )
        
        # Transformer编码器
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=config.hidden_size,
            nhead=config.num_heads,
            dim_feedforward=config.ff_dim,
            dropout=config.dropout,
            batch_first=True
        )
        self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=config.num_layers)
        
        # 动作预测头
        self.action_head = nn.Sequential(
            nn.Linear(config.hidden_size, config.hidden_size),
            nn.ReLU(),
            nn.Dropout(config.dropout),
            nn.Linear(config.hidden_size, config.num_actions)
        )
        
        # 值预测头
        self.value_head = nn.Sequential(
            nn.Linear(config.hidden_size, config.hidden_size),
            nn.ReLU(),
            nn.Dropout(config.dropout),
            nn.Linear(config.hidden_size, 1)
        )
        
        # # 位置预测头
        # self.pos_head = nn.Sequential(
        #     nn.Linear(config.hidden_size, config.hidden_size),
        #     nn.ReLU(),
        #     nn.Dropout(config.dropout),
        #     nn.Linear(config.hidden_size, 3)  # 预测3维位置
        # )
        
        # # yaw角预测头
        # self.yaw_head = nn.Sequential(
        #     nn.Linear(config.hidden_size, config.hidden_size),
        #     nn.ReLU(),
        #     nn.Dropout(config.dropout),
        #     nn.Linear(config.hidden_size, 1)  # 预测yaw角
        # )
        
    def forward(self, batch):
        # 处理图像输入
        left_img = self.image_encoder(batch['left_img'])  # (B, 2048)
        center_img = self.image_encoder(batch['center_img'])
        right_img = self.image_encoder(batch['right_img'])
        
        # 投影图像特征
        left_img = self.image_proj(left_img).unsqueeze(1)  # (B, 1, hidden_size)
        center_img = self.image_proj(center_img).unsqueeze(1)
        right_img = self.image_proj(right_img).unsqueeze(1)
        
        # 处理文本输入
        text = self.text_encoder(input_ids=batch['input_ids'], attention_mask=batch['attention_mask']).last_hidden_state
        text = self.text_proj(text)
        
        # 处理位置和yaw角输入
        # start_pos = self.pos_encoder(batch['start_pos']).unsqueeze(1)
        # end_pos = self.pos_encoder(batch['end_pos']).unsqueeze(1)
        # start_yaw = self.yaw_encoder(batch['start_yaw'].unsqueeze(-1)).unsqueeze(1)
        # end_yaw = self.yaw_encoder(batch['end_yaw'].unsqueeze(-1)).unsqueeze(1)
        
        # 合并所有特征
        features = torch.cat([
            left_img, center_img, right_img,
            text,
            # start_pos, end_pos,
            # start_yaw, end_yaw
        ], dim=1)
        
        # Transformer编码
        features = self.transformer(features)
        
        # 使用[CLS]token的表示进行预测
        cls_token = features[:, 0]
        
        # 预测动作、值和位置
        action_logits = self.action_head(cls_token)
        value_pred = self.value_head(cls_token)
        # pos_pred = self.pos_head(cls_token)
        # yaw_pred = self.yaw_head(cls_token)
        
        return {
            'action_logits': action_logits,
            'value_pred': value_pred,
            # 'pos_pred': pos_pred,
            # 'yaw_pred': yaw_pred
        }
    
    def compute_loss(self, outputs, batch):
        # 动作分类损失
        action_loss = F.cross_entropy(outputs['action_logits'], batch['action'])
        
        # 值回归损失
        value_loss = F.mse_loss(outputs['value_pred'].squeeze(-1), batch['value'])
        
        # # 位置预测损失
        # pos_loss = F.mse_loss(outputs['pos_pred'], batch['end_pos'])
        
        # # yaw角预测损失
        # yaw_loss = F.mse_loss(outputs['yaw_pred'].squeeze(-1), batch['end_yaw'])
        
        # 总损失
        total_loss = (
            self.config.action_loss_weight * action_loss +
            self.config.value_loss_weight * value_loss
            # self.config.pos_loss_weight * pos_loss +
            # self.config.yaw_loss_weight * yaw_loss
        )
        
        return {
            'total_loss': total_loss,
            'action_loss': action_loss,
            'value_loss': value_loss,
            # 'pos_loss': pos_loss,
            # 'yaw_loss': yaw_loss
        } 