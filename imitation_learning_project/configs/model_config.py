from dataclasses import dataclass

@dataclass
class ModelConfig:
    # 模型基本配置
    hidden_size: int = 768
    num_heads: int = 12
    num_layers: int = 6
    ff_dim: int = 3072
    dropout: float = 0.1
    
    # 编码器配置
    # text_encoder_name: str = "bert-base-chinese"  # 或其他预训练文本模型
    text_encoder_name: str = "bert-base-uncased"
    
    # 输出配置
    num_actions: int = 3 # forward, backward, left, right, turn_left, turn_right
    
    # 损失权重
    action_loss_weight: float = 1.0
    value_loss_weight: float = 0.5
    pos_loss_weight: float = 0.3
    yaw_loss_weight: float = 0.3
    
    # 训练配置
    learning_rate: float = 1e-4
    weight_decay: float = 0.01
    warmup_steps: int = 1000
    max_epochs: int = 100
    batch_size: int = 32
    
    # 数据配置
    image_size: int = 224
    max_text_length: int = 512 