import numpy as np

def point_to_plane_distance(p1, vec3d, p2):
    # 提取点和法向量的坐标
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    a, b, c = vec3d
    
    # 计算分子
    numerator = abs(a * (x2 - x1) + b * (y2 - y1) + c * (z2 - z1))
    
    # 计算分母
    denominator = np.sqrt(a**2 + b**2 + c**2)
    
    # 计算距离
    distance = numerator / denominator
    
    return distance
