import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from scipy import interpolate
from scipy.spatial.transform import Slerp
import os

# 读取并转换为变换矩阵的函数
def read_and_convert_to_transformation_matrix(filename):
    transformation_matrices = []

    with open(filename, 'r') as file:
        for line in file:
            data = line.strip().split(" ")
            
            timestamp = float(data[0])  # 时间戳
            x, y, z = float(data[1]), float(data[2]), float(data[3])  # 位置 (x, y, z)
            qx, qy, qz, qw = float(data[4]), float(data[5]), float(data[6]), float(data[7])  # 四元数

            quat = [qx, qy, qz, qw]
            rotation = R.from_quat(quat)  # 创建旋转对象
            R_matrix = rotation.as_matrix()  # 旋转矩阵 R
            
            # 生成变换矩阵 T
            T = np.hstack((R_matrix, np.array([[x], [y], [z]])))  # 4x3
            T = np.vstack((T, [0, 0, 0, 1]))  # 添加最后一行 [0, 0, 0, 1]，形成 4x4 矩阵

            transformation_matrices.append((timestamp, T))  # 存储时间戳和变换矩阵

    return transformation_matrices

# 对变换矩阵进行插值的函数
def interpolate_transformations(transformation_matrices, new_time_stamps):
    # 提取时间戳、位置和四元数
    timestamps = np.array([ts for ts, _ in transformation_matrices])
    positions = np.array([T[:3, 3] for _, T in transformation_matrices])  # 提取位置
    quaternions = np.array([R.from_matrix(T[:3, :3]).as_quat() for _, T in transformation_matrices])  # 提取四元数

    # 插值位置数据：线性插值
    pos_interpolator = interpolate.interp1d(timestamps, positions, axis=0, kind='nearest')
    interpolated_positions = pos_interpolator(new_time_stamps)

    # 使用Slerp插值四元数数据
    rotations = R.from_quat(quaternions)  # 创建旋转对象
    slerp = Slerp(timestamps, rotations)  # 创建Slerp插值器
    interpolated_rotations = slerp(new_time_stamps)  # 插值四元数
    interpolated_quaternions = interpolated_rotations.as_quat()  # 转回四元数

    # 获取第一个变换矩阵 T0
    _, T0 = transformation_matrices[0]
    T0_inv = np.linalg.inv(T0)

    # 创建插值后的变换矩阵
    interpolated_transformations = []
    for t, pos, quat in zip(new_time_stamps, interpolated_positions, interpolated_quaternions):
        rotation = R.from_quat(quat)
        R_matrix = rotation.as_matrix()  # 得到旋转矩阵
        T = np.hstack((R_matrix, np.array([[pos[0]], [pos[1]], [pos[2]]])))  # 拼接成 4x3 矩阵
        T = np.vstack((T, [0, 0, 0, 1]))  # 补充为 4x4 齐次变换矩阵
        
        # 归一化：将当前变换矩阵与第一个变换矩阵相对
        T_relative = np.dot(T0_inv, T)  # T_relative = T0_inv * T
        interpolated_transformations.append((t, T_relative))

    return interpolated_transformations

# 绘制轨迹和刚体朝向的函数
def plot_trajectory_with_orientation(transformation_matrices):
    # 提取所有的位置坐标
    positions = np.array([T[:3, 3] for _, T in transformation_matrices])  # 获取每个变换矩阵的平移向量 [x, y, z]
    
    # 提取 x, y, z
    x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]
    
    # 绘制 3D 轨迹
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, label='Trajectory')

    # 设置标签和标题
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Trajectory with Orientation')

    # 设置相同的比例尺度
    max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0
    mid_x = (x.max() + x.min()) / 2
    mid_y = (y.max() + y.min()) / 2
    mid_z = (z.max() + z.min()) / 2
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # 绘制每20步的RGB坐标系
    for i in range(0, len(transformation_matrices), 60):
        timestamp, T = transformation_matrices[i]
        
        # 提取旋转矩阵
        R_matrix = T[:3, :3]

        # 定义局部坐标系的三个基向量（红色、绿色、蓝色）
        origin = T[:3, 3]  # 变换矩阵的平移部分，表示坐标系的原点
        x_axis = origin + R_matrix[:, 0] * 0.3  # x方向，红色
        y_axis = origin + R_matrix[:, 1] * 0.3  # y方向，绿色
        z_axis = origin + R_matrix[:, 2] * 0.3  # z方向，蓝色

        # 绘制坐标系轴
        ax.quiver(origin[0], origin[1], origin[2], x_axis[0] - origin[0], x_axis[1] - origin[1], x_axis[2] - origin[2], color='r', length=0.1)
        ax.quiver(origin[0], origin[1], origin[2], y_axis[0] - origin[0], y_axis[1] - origin[1], y_axis[2] - origin[2], color='g', length=0.1)
        ax.quiver(origin[0], origin[1], origin[2], z_axis[0] - origin[0], z_axis[1] - origin[1], z_axis[2] - origin[2], color='b', length=0.1)

    # 显示图例
    ax.legend()

    # 显示轨迹图
    plt.show()

def save_to_kitti_format(transformation_matrices, output_filename):
    with open(output_filename, 'w') as file:
        for _, T in transformation_matrices:
            # 提取旋转矩阵和位移向量
            R_matrix = T[:3, :3] # 旋转矩阵展平为一维数组
            translation = T[:3, 3]           # 平移向量
            
            # 拼接为一行：r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
            line = np.hstack((R_matrix[0], translation[0], R_matrix[1], translation[1], R_matrix[2], translation[2])).tolist()
            
            # 写入文件，值用空格分隔
            file.write(" ".join(map(str, line)) + "\n")

def count_lines_in_file(file_path):
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
            return len(lines)
    except FileNotFoundError:
        print(f"文件 {file_path} 未找到。")
        return 0
    except Exception as e:
        print(f"读取文件时出错: {e}")
        return 0
    
def count_images_in_folder(folder_path, image_extensions=None):
    if image_extensions is None:
        image_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.gif', '.tiff'}

    try:
        files = os.listdir(folder_path)
        image_count = sum(1 for file in files if any(file.lower().endswith(ext) for ext in image_extensions))
        return image_count
    except FileNotFoundError:
        print(f"文件夹 {folder_path} 未找到。")
        return 0
    except Exception as e:
        print(f"读取文件夹时出错: {e}")
        return 0


if __name__ == "__main__":
    # 文件路径
    step = 1
    len_img_data = int(count_images_in_folder("/home/mingtao/Gt_ws/data_vio/rgbl") / step)

    
    filename = "/home/mingtao/Gt_ws/data_vio/gt_pose.txt"

    camere_t_path = "/home/mingtao/Gt_ws/data_vio/left_rgb_timestamps.txt"

    inter_num = count_lines_in_file(camere_t_path)

    lag = len_img_data - inter_num
    
    # 读取并转换为变换矩阵
    transformation_matrices = read_and_convert_to_transformation_matrix(filename)
    transformation_matrices = transformation_matrices[lag*step:]

    # 提取时间戳并生成新的均匀时间戳
    timestamps = np.array([ts for ts, _ in transformation_matrices])
    new_time_stamps = np.linspace(timestamps.min(), timestamps.max(), num=inter_num)

    # 对变换矩阵进行插值
    interpolated_transformations = interpolate_transformations(transformation_matrices, new_time_stamps)

    save_to_kitti_format(interpolated_transformations, "data_vio/ground_truth_rgb.txt",)

    # 绘制均匀化后的轨迹和朝向
    plot_trajectory_with_orientation(interpolated_transformations)
