import os
import torch
import torch.nn as nn
from torch.optim import AdamW
from torch.optim.lr_scheduler import CosineAnnealingLR
from transformers import AutoTokenizer
from tqdm import tqdm
import logging
import json
from datetime import datetime
import wandb
import yaml
import gc
from torch.cuda.amp import autocast, GradScaler
import argparse

from models.bc_transformer import BCTransformer
from dataset.dataset import get_dataloader
from config.model_config import ModelConfig

def setup_logging():
    """设置日志"""
    log_dir = "logs"
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"train_{timestamp}.log")
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger(__name__)

def setup_wandb(config):
    """设置wandb离线模式"""
    # 创建wandb目录
    wandb_dir = "wandb"
    os.makedirs(wandb_dir, exist_ok=True)
    
    # 设置环境变量为离线模式
    os.environ["WANDB_MODE"] = "offline"
    os.environ["WANDB_DIR"] = wandb_dir
    
    # 初始化wandb
    wandb.init(
        project="imitation_learning",
        config=vars(config),
        dir=wandb_dir
    )
    
    # 保存配置到wandb目录
    config_path = os.path.join(wandb_dir, "config.yaml")
    with open(config_path, 'w') as f:
        yaml.dump(vars(config), f)
    
    return wandb

def save_checkpoint(model, optimizer, scheduler, epoch, loss, save_dir):
    """保存检查点"""
    os.makedirs(save_dir, exist_ok=True)
    checkpoint = {
        'epoch': epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
        'scheduler_state_dict': scheduler.state_dict(),
        'loss': loss
    }
    path = os.path.join(save_dir, f'checkpoint_epoch_{epoch}.pt')
    torch.save(checkpoint, path)
    logging.info(f"Saved checkpoint to {path}")
    
    # 保存到wandb
    wandb.save(path)

def train(config, train_loader, val_loader, model, optimizer, scheduler, device, save_dir):
    """训练循环"""
    best_val_loss = float('inf')
    scaler = GradScaler()  # 用于混合精度训练
    accumulation_steps = 4  # 梯度累积步数
    
    for epoch in range(config.max_epochs):
        # 训练阶段
        model.train()
        train_losses = []
        train_action_losses = []
        train_value_losses = []
        train_pos_losses = []
        train_yaw_losses = []
        
        train_pbar = tqdm(train_loader, desc=f'Epoch {epoch+1}/{config.max_epochs} [Train]')
        optimizer.zero_grad()  # 在每个epoch开始时清零梯度
        
        for batch_idx, batch in enumerate(train_pbar):
            # 清理GPU缓存
            if batch_idx % 5 == 0:  # 更频繁地清理缓存
                torch.cuda.empty_cache()
                gc.collect()
            
            # 将数据移动到设备
            batch = {k: v.to(device) if isinstance(v, torch.Tensor) else v for k, v in batch.items()}
            
            # 使用混合精度训练
            with autocast():
                # 前向传播
                outputs = model(batch)
                losses = model.compute_loss(outputs, batch)
                # 缩放损失以适应梯度累积
                scaled_loss = losses['total_loss'] / accumulation_steps
            
            # 反向传播
            scaler.scale(scaled_loss).backward()
            
            # 梯度累积
            if (batch_idx + 1) % accumulation_steps == 0:
                # 梯度裁剪
                scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                
                scaler.step(optimizer)
                scaler.update()
                optimizer.zero_grad()
            
            # 记录损失
            train_losses.append(losses['total_loss'].item())
            train_action_losses.append(losses['action_loss'].item())
            train_value_losses.append(losses['value_loss'].item())
            # train_pos_losses.append(losses['pos_loss'].item())
            # train_yaw_losses.append(losses['yaw_loss'].item())
            
            # 更新进度条
            train_pbar.set_postfix({
                'loss': f"{losses['total_loss'].item():.4f}",
                'action_loss': f"{losses['action_loss'].item():.4f}",
                'value_loss': f"{losses['value_loss'].item():.4f}",
                # 'pos_loss': f"{losses['pos_loss'].item():.4f}",
                # 'yaw_loss': f"{losses['yaw_loss'].item():.4f}"
            })
            
            # 记录到wandb
            if batch_idx % 10 == 0:  # 每10个batch记录一次
                wandb.log({
                    'train/total_loss': losses['total_loss'].item(),
                    'train/action_loss': losses['action_loss'].item(),
                    'train/value_loss': losses['value_loss'].item(),
                    'train/pos_loss': losses['pos_loss'].item(),
                    'train/yaw_loss': losses['yaw_loss'].item(),
                    'train/learning_rate': scheduler.get_last_lr()[0],
                    'train/batch': epoch * len(train_loader) + batch_idx
                })
        
        # 更新学习率
        scheduler.step()
        
        # 验证阶段
        model.eval()
        val_losses = []
        val_action_losses = []
        val_value_losses = []
        val_pos_losses = []
        val_yaw_losses = []
        
        with torch.no_grad():
            val_pbar = tqdm(val_loader, desc=f'Epoch {epoch+1}/{config.max_epochs} [Val]')
            for batch in val_pbar:
                # 清理GPU缓存
                torch.cuda.empty_cache()
                gc.collect()
                
                batch = {k: v.to(device) if isinstance(v, torch.Tensor) else v for k, v in batch.items()}
                
                # 使用混合精度推理
                with autocast():
                    outputs = model(batch)
                    losses = model.compute_loss(outputs, batch)
                
                val_losses.append(losses['total_loss'].item())
                val_action_losses.append(losses['action_loss'].item())
                val_value_losses.append(losses['value_loss'].item())
                val_pos_losses.append(losses['pos_loss'].item())
                val_yaw_losses.append(losses['yaw_loss'].item())
                
                val_pbar.set_postfix({
                    'loss': f"{losses['total_loss'].item():.4f}",
                    'action_loss': f"{losses['action_loss'].item():.4f}",
                    'value_loss': f"{losses['value_loss'].item():.4f}",
                    'pos_loss': f"{losses['pos_loss'].item():.4f}",
                    'yaw_loss': f"{losses['yaw_loss'].item():.4f}"
                })
        
        # 计算平均损失
        avg_train_loss = sum(train_losses) / len(train_losses)
        avg_val_loss = sum(val_losses) / len(val_losses)
        
        # 记录日志
        logging.info(f"Epoch {epoch+1}/{config.max_epochs}")
        logging.info(f"Train Loss: {avg_train_loss:.4f}")
        logging.info(f"Val Loss: {avg_val_loss:.4f}")
        logging.info(f"Train Action Loss: {sum(train_action_losses)/len(train_action_losses):.4f}")
        logging.info(f"Train Value Loss: {sum(train_value_losses)/len(train_value_losses):.4f}")
        logging.info(f"Train Position Loss: {sum(train_pos_losses)/len(train_pos_losses):.4f}")
        logging.info(f"Train Yaw Loss: {sum(train_yaw_losses)/len(train_yaw_losses):.4f}")
        logging.info(f"Val Action Loss: {sum(val_action_losses)/len(val_action_losses):.4f}")
        logging.info(f"Val Value Loss: {sum(val_value_losses)/len(val_value_losses):.4f}")
        logging.info(f"Val Position Loss: {sum(val_pos_losses)/len(val_pos_losses):.4f}")
        logging.info(f"Val Yaw Loss: {sum(val_yaw_losses)/len(val_yaw_losses):.4f}")
        
        # 记录到wandb
        wandb.log({
            'epoch': epoch + 1,
            'val/total_loss': avg_val_loss,
            'val/action_loss': sum(val_action_losses)/len(val_action_losses),
            'val/value_loss': sum(val_value_losses)/len(val_value_losses),
            'val/pos_loss': sum(val_pos_losses)/len(val_pos_losses),
            'val/yaw_loss': sum(val_yaw_losses)/len(val_yaw_losses),
            'train/total_loss_epoch': avg_train_loss,
            'train/action_loss_epoch': sum(train_action_losses)/len(train_action_losses),
            'train/value_loss_epoch': sum(train_value_losses)/len(train_value_losses),
            'train/pos_loss_epoch': sum(train_pos_losses)/len(train_pos_losses),
            'train/yaw_loss_epoch': sum(train_yaw_losses)/len(train_yaw_losses)
        })
        
        # 保存最佳模型
        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            save_checkpoint(model, optimizer, scheduler, epoch, avg_val_loss, save_dir)
            logging.info(f"New best model saved with validation loss: {best_val_loss:.4f}")
            
            # 记录最佳模型到wandb
            wandb.run.summary['best_val_loss'] = best_val_loss
            wandb.run.summary['best_epoch'] = epoch + 1

def main(data_root, save_dir):
    # 设置设备
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logging.info(f"Using device: {device}")
    
    # 设置CUDA内存分配器
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.backends.cudnn.benchmark = True
        torch.backends.cudnn.deterministic = True
        # 设置内存分配器
        torch.cuda.set_per_process_memory_fraction(0.8)  # 限制GPU内存使用比例
    
    # 加载配置
    config = ModelConfig()
    
    # 设置保存目录
    os.makedirs(save_dir, exist_ok=True)
    
    # 设置wandb
    wandb = setup_wandb(config)
    
    # 加载数据
    train_loader = get_dataloader(data_root, "train.json", batch_size=config.batch_size)
    val_loader = get_dataloader(data_root, "val.json", batch_size=config.batch_size, shuffle=False)
    
    # 初始化模型
    model = BCTransformer(config).to(device)
    
    # 设置优化器和学习率调度器
    optimizer = AdamW(
        model.parameters(),
        lr=config.learning_rate,
        weight_decay=config.weight_decay
    )
    
    scheduler = CosineAnnealingLR(
        optimizer,
        T_max=config.max_epochs,
        eta_min=config.learning_rate * 0.1
    )
    
    # 训练模型
    train(config, train_loader, val_loader, model, optimizer, scheduler, device, save_dir)
    
    # 关闭wandb
    wandb.finish()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train imitation learning model.")
    parser.add_argument('--data_root', type=str, default="/home/wangy/renpengzhen/linmin/imitation_learning_project/only_hssd_data", help='Root directory of the data')
    parser.add_argument('--save_dir', type=str, default="no_gen_scene_checkpoints", help='Directory to save checkpoints')
    args = parser.parse_args()
    logger = setup_logging()
    main(args.data_root, args.save_dir) 