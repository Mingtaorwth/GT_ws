import numpy as np
import matplotlib.pyplot as plt

# 从txt文件读取数据，跳过非数字字符（比如逗号）
file_path = 'data_5/gt_twist.txt'  # 请替换为实际路径
data = np.genfromtxt(file_path, delimiter=',', dtype=float)

# 提取时间戳和速度分量
timestamps = data[:, 0]
vx, vy, vz = data[:, 1], data[:, 2], data[:, 3]

# 计算每帧的合速度
speeds = np.sqrt(vx**2 + vy**2 + vz**2)

# 使用IQR方法去除离群值
Q1 = np.percentile(speeds, 25)
Q3 = np.percentile(speeds, 75)
IQR = Q3 - Q1
outlier_threshold_low = Q1 - 1.5 * IQR
outlier_threshold_high = Q3 + 1.5 * IQR

# 去除离群值
speeds_cleaned = speeds[(speeds >= outlier_threshold_low) & (speeds <= outlier_threshold_high)]
timestamps_cleaned = timestamps[(speeds >= outlier_threshold_low) & (speeds <= outlier_threshold_high)]

# 绘制清理后的合速度随时间变化的图像
plt.figure(figsize=(10, 6))
plt.plot(timestamps_cleaned, speeds_cleaned, label="Cleaned Speed", color='blue')
plt.xlabel('Timestamp')
plt.ylabel('Speed (m/s)')
plt.title('Cleaned Speed vs Time')
plt.grid(True)
plt.legend()
plt.show()

# 找出最大速度值（去除离群值后的数据）
max_speed = np.max(speeds_cleaned)
mean_speed = np.mean(speeds_cleaned)
print("mean_v", mean_speed)
max_speed_idx = np.argmax(speeds_cleaned)
max_speed_time = timestamps_cleaned[max_speed_idx]

# 打印最大速度和对应的时间戳
print(f"Maximum Speed (after cleaning): {max_speed:.4f} m/s at timestamp {max_speed_time}")
