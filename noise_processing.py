import cv2
import os
import numpy as np

# 图像所在的文件夹路径
folder_path = "sdr"

# 获取文件夹中的所有 XML 文件
xml_files = [f for f in os.listdir(folder_path) if f.endswith(".xml")]

# 创建一个窗口，用于显示图像
cv2.namedWindow("Image Playback", cv2.WINDOW_NORMAL)

xml_path1 = os.path.join(folder_path, xml_files[0])
fs1 = cv2.FileStorage(xml_path1, cv2.FILE_STORAGE_READ)
image1 = fs1.getNode("image").mat()  # 假设图像保存在名为 "image" 的节点中
fs1.release()
# 遍历每个 XML 文件并读取图像
sum_image = np.zeros((160, 160))
for xml_file in xml_files[:1000]:
    # 生成完整的文件路径
    xml_path = os.path.join(folder_path, xml_file)

    # 读取图像
    fs = cv2.FileStorage(xml_path, cv2.FILE_STORAGE_READ)
    image = fs.getNode("image").mat()  # 假设图像保存在名为 "image" 的节点中
    fs.release()

    sum_image += image

    # 显示图像
    cv2.imshow("Image Playback", image - image1)

    # 等待按键，1ms 后继续（也可以在这里处理其他事情）
    key = cv2.waitKey(1)

    # 如果按下 'q' 键，退出
    
    if key == ord('q'):
        break

ave_noise = sum_image / len(xml_files)

for xml_file in xml_files[:1000]:
    # 生成完整的文件路径
    xml_path = os.path.join(folder_path, xml_file)

    # 读取图像
    fs = cv2.FileStorage(xml_path, cv2.FILE_STORAGE_READ)
    image = fs.getNode("image").mat()  # 假设图像保存在名为 "image" 的节点中
    fs.release()

    sum_image += image

    # 显示图像
    cv2.imshow("Image Playback", image - ave_noise)

    # 等待按键，1ms 后继续（也可以在这里处理其他事情）
    key = cv2.waitKey(1)

    # 如果按下 'q' 键，退出
    
    if key == ord('q'):
        break


# 关闭所有窗口
cv2.destroyAllWindows()
