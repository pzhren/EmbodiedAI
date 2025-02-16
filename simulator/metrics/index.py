import numpy as np

class Metrics():
    def __init__(self, task):
        self.task = task
        
    def NE(self, pos1, pos2):
        '''
        世界坐标系下的两点之间的欧氏距离
        '''
        return np.linalg.norm(np.array(pos1) - np.array(pos2))
    
    
    def SPL(self, path_length, optimal_path_length, is_success):
        '''
        成功路径长度比例, 成功路径长度/最优路径长度，若失败则返回0
        '''
        return optimal_path_length / path_length if is_success else 0
    
    
    def nav_success(self, pos1, pos2, threshold=0.8):
        '''
        世界坐标系下的两点之间的欧氏距离是否小于阈值
        '''
        return self.NE(pos1, pos2) < threshold
    
    