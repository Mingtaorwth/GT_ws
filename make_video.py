import cv2
import os

# 设置图片文件夹路径和输出视频文件路径
folder_path = 'data_1/rgbr'  # 请替换为实际的文件夹路径
output_video_path = 'rgb1.avi'  # 输出视频的路径和名称

# 获取文件夹中的所有图片文件
image_files = [f for f in os.listdir(folder_path) if f.endswith(('.jpg', '.png', '.jpeg', 'bmp'))]
# 自定义排序函数，按文件名中的数字进行排序
image_files.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))

# 读取第一张图片来获取图片的尺寸
first_image_path = os.path.join(folder_path, image_files[0])
first_image = cv2.imread(first_image_path)
height, width, _ = first_image.shape

# 设置视频写入器（编码器和FPS）
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 视频编码器，使用XVID
fps = 30  # 设置帧率为30FPS
out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

# 逐张读取图片并写入视频
for image_file in image_files:
    image_path = os.path.join(folder_path, image_file)
    image = cv2.imread(image_path)
    out.write(image)  # 将图片写入视频

# 释放资源
out.release()
cv2.destroyAllWindows()

print("视频已保存到:", output_video_path)
