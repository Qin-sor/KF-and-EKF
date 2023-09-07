import numpy as np
import os

# 设置随机种子以保证结果可复现
np.random.seed(42)

# 生成时间序列
time = np.linspace(0, 10, 300)

# 生成x和y的真实运动轨迹
y_true = np.sin(time)  # 示例中使用了正弦函数
x_true = np.float16(time)

# 添加高斯噪声
mean = 0  # 噪声的均值
std_dev = 0.1  # 噪声的标准差
x_noisy = x_true + np.random.normal(mean, std_dev, size=len(time))
y_noisy = y_true + np.random.normal(mean, std_dev, size=len(time))

# 构建数据矩阵
data_matrix = np.column_stack((x_noisy, y_noisy, time))

# 将数据输出到文件
filename = "file.txt"

# 如果文件存在，则删除文件
if os.path.exists(filename):
    os.remove(filename)

# 将数据写入文件
with open(filename, 'w') as file:
    for row in data_matrix:
        formatted_row = [format(value, '.3f') for value in row]
        file.write(','.join(formatted_row) + ', \n')

print("数据已成功写入文件：", filename)