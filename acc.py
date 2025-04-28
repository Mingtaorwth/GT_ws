import numpy as np
import matplotlib.pyplot as plt

# 从txt文件读取数据
file_path = 'data_2/gt_acc.txt'  # 请替换为实际路径
data = np.genfromtxt(file_path, delimiter=',', dtype=float)

# 提取时间戳和加速度分量
timestamps = data[:, 0]
ax, ay, az = data[:, 1], data[:, 2], data[:, 3]

# 计算每帧的合成加速度
acceleration = np.sqrt(ax**2 + ay**2 + az**2)

# 使用IQR方法去除离群值
Q1 = np.percentile(acceleration, 25)
Q3 = np.percentile(acceleration, 75)
IQR = Q3 - Q1
outlier_threshold_low = Q1 - 1.5 * IQR
outlier_threshold_high = Q3 + 1.5 * IQR

# 去除离群值
acceleration_cleaned = acceleration[(acceleration >= outlier_threshold_low) & (acceleration <= outlier_threshold_high)]
timestamps_cleaned = timestamps[(acceleration >= outlier_threshold_low) & (acceleration <= outlier_threshold_high)]

# 绘制清理后的加速度图像
plt.figure(figsize=(10, 6))
plt.plot(timestamps_cleaned, acceleration_cleaned, label="Cleaned Acceleration", color='blue')
plt.xlabel('Timestamp')
plt.ylabel('Acceleration (m/s²)')
plt.title('Cleaned Acceleration vs Time')
plt.grid(True)
plt.legend()
plt.show()

# 找出最大加速度值
max_acceleration = np.max(acceleration_cleaned)
mean_acc = np.mean(acceleration_cleaned)
print("mean_acc", mean_acc)
max_acceleration_idx = np.argmax(acceleration_cleaned)
max_acceleration_time = timestamps_cleaned[max_acceleration_idx]

# 打印最大加速度和对应的时间戳
print(f"Maximum Acceleration (after cleaning): {max_acceleration:.4f} m/s² at timestamp {max_acceleration_time}")
