import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# 从txt文件读取数据，跳过非数字字符（比如逗号）
file_path = 'data_5/gt_pose.txt'  # 请替换为实际路径
data = np.genfromtxt(file_path, delimiter=',', dtype=float)

# 提取时间戳、位置和四元数
timestamps = data[:, 0]
x, y, z = data[:, 1], data[:, 2], data[:, 3]
qx, qy, qz, qw = data[:, 4], data[:, 5], data[:, 6], data[:, 7]

# 计算每帧的角速度
angular_velocities = []

for i in range(1, len(timestamps)):
    delta_t = timestamps[i] - timestamps[i - 1]
    
    # 计算当前和前一帧的四元数之间的差异
    q1 = R.from_quat([qx[i-1], qy[i-1], qz[i-1], qw[i-1]])
    q2 = R.from_quat([qx[i], qy[i], qz[i], qw[i]])

    # 计算旋转角度（弧度）
    delta_q = q2 * q1.inv()  # q2 到 q1 的旋转差
    angle = delta_q.magnitude()  # 得到旋转角度（弧度）

    # 角速度计算：angle / delta_t
    angular_velocity = angle / delta_t * np.pi / 180

    angular_velocities.append(angular_velocity)

# 转换为numpy数组
angular_velocities = np.array(angular_velocities)

# 去除噪音：使用滑动平均
window_size = 5  # 设置窗口大小，越大平滑效果越强
angular_velocities_smoothed = np.convolve(angular_velocities, np.ones(window_size)/window_size, mode='valid')

# 计算最大角速度和平均角速度
max_angular_velocity = np.max(angular_velocities_smoothed)
mean_angular_velocity = np.mean(angular_velocities_smoothed)

# 打印最大角速度和平均角速度
print(f"Maximum Angular Velocity: {max_angular_velocity:.4f} rad/s")
print(f"Mean Angular Velocity: {mean_angular_velocity:.4f} rad/s")

# 绘制平滑后的角速度随时间变化的图像
plt.figure(figsize=(10, 6))
plt.plot(timestamps[1:][window_size-1:], angular_velocities_smoothed, label="Smoothed Angular Velocity", color='red')
plt.xlabel('Timestamp')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Smoothed Angular Velocity vs Time')
plt.grid(True)
plt.legend()
plt.show()
