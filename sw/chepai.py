# 导入所需模块
import tempfile

import cv2
from matplotlib import pyplot as plt
import os
import numpy as np
from PIL import ImageFont, ImageDraw, Image
import glob

def binarize_and_resize_image(splite_image, output_dir, num=1, size=(24, 50), threshold=128):

    # 定义输入和输出路径
    input_image_path = os.path.join(str(splite_image), f"image_0.png")
    output_directory = output_dir

    output_hex_path = os.path.join(output_directory, f"{num}.hex")
    output_path = os.path.join(output_directory, f"{num}.txt")

    # 创建目录
    # os.makedirs(output_directory, exist_ok=True)

    # 打开图像并转换为灰度图
    # img = Image.open(input_image_path).convert('L')
    def save_image_array_as_tempfile(image_array):
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.png')
        img = Image.fromarray(image_array)
        img.save(temp_file.name)
        return temp_file.name

    input_image_path = save_image_array_as_tempfile(splite_image)
    img = Image.open(input_image_path).convert('L')
    # 二值化处理
    binary_img = img.point(lambda p: 1 if p > threshold else 0, '1')
    
    # 调整图像大小
    resized_img = binary_img.resize(size, Image.NEAREST)

    # 将二值化图像转换为像素数组
    width, height = resized_img.size
    pixels = list(resized_img.getdata())

    # 保存为二值化文件
    with open(output_path, 'w') as hex_file:
        for y in range(height):
            for x in range(width):
                pixel_value = pixels[y * width + x]
                hex_file.write(f"{pixel_value:X}")  # 以十六进制形式写入
            hex_file.write("\n")  # 换行

    print(f"二值化并调整大小的图像已保存为 {output_path}")

    # 保存为 HEX 文件
    with open(output_hex_path, 'w') as hex_file:
        for y in range(height):
            hex_values = []
            for x in range(0, width, 4):  # 每4个像素处理一次
                # 取4个像素值
                group = pixels[y * width + x:y * width + x + 4]
                # 计算该组的16进制值
                hex_value = sum(val << (3 - i) for i, val in enumerate(group))  # 计算每4个的十六进制值
                hex_values.append(f"{hex_value:X}")  # 转换为十六进制字符串

            # 写入文件，每行6个十六进制数
            hex_file.write("".join(hex_values) + "\n")

    print(f"二值化并调整大小的图像已保存为 {output_hex_path}")


# plt显示灰度图片
def plt_show(img):
    plt.imshow(img, cmap='gray')
    plt.show()


# 图像去噪灰度处理
def gray_guss(image):
    if image is not None:
        if len(image.shape) >= 3:
            # Convert from RGB to grayscale
            image = cv2.GaussianBlur(image, (3, 3), 0)
            gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        else:
            # The image is already grayscale
            gray_image = image
    else:
        # rise ValueError
        ValueError("Error: Image not loaded correctly.")

    return gray_image

# input dir
input_dir = 'input'
# output dir
output_base_dir = 'output'

#error count
error_count = 0

#total count
total_count = 0
#遍历input文件夹下的所有图片
image_paths = glob.glob(os.path.join(input_dir, '*.jpg'))
#remove input dir from image_paths
image_paths = [os.path.basename(image_path) for image_path in image_paths]
for files in image_paths:
    total_count += 1
    #print("\nWorking on "+ files+ "...")
    # 读取待检测图片
    origin_image = cv2.imread(os.path.join(input_dir, files))
    # 复制一张图片，在复制图上进行图像操作，保留原图
    image = origin_image.copy()
    #plt_show(image)
    # 图像去噪灰度处理
    gray_image = gray_guss(image)
    # x方向上的边缘检测（增强边缘信息）
    Sobel_x = cv2.Sobel(gray_image, cv2.CV_16S, 1, 0)
    absX = cv2.convertScaleAbs(Sobel_x)
    image = absX

    # 图像阈值化操作——获得二值化图
    ret, image = cv2.threshold(image, 0, 255, cv2.THRESH_OTSU)
    # 显示灰度图像
    # plt_show(image)
    # 形态学（从图像中提取对表达和描绘区域形状有意义的图像分量）——闭操作
    kernelX = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 10))
    image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernelX, iterations=1)
    # 显示灰度图像
    #plt_show(image)

    # 腐蚀（erode）和膨胀（dilate）
    kernelX = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 1))
    kernelY = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 20))
    # x方向进行闭操作（抑制暗细节）
    image = cv2.dilate(image, kernelX)
    image = cv2.erode(image, kernelX)
    # y方向的开操作
    image = cv2.erode(image, kernelY)
    image = cv2.dilate(image, kernelY)
    # 中值滤波（去噪）
    image = cv2.medianBlur(image, 21)
    # 显示灰度图像
    #plt_show(image)

    # 获得轮廓
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for item in contours:
        rect = cv2.boundingRect(item)
        x = rect[0]
        y = rect[1]
        weight = rect[2]
        height = rect[3]
        # 根据轮廓的形状特点，确定车牌的轮廓位置并截取图像
        if (weight > (height * 3)) and (weight < (height * 4.5)):
            image = origin_image[y:y + height, x:x + weight]
            #plt_show(image)

    # 车牌字符分割
    # 图像去噪灰度处理
    gray_image = gray_guss(image)

    # 图像阈值化操作——获得二值化图
    ret, image = cv2.threshold(gray_image, 0, 255, cv2.THRESH_OTSU)
    # plt_show(image)

    # 膨胀操作
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
    image = cv2.dilate(image, kernel)
    #plt_show(image)

    # 查找轮廓
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    words = []
    word_images = []
    # 对所有轮廓逐一操作
    for item in contours:
        word = []
        rect = cv2.boundingRect(item)
        x = rect[0]
        y = rect[1]
        weight = rect[2]
        height = rect[3]
        word.append(x)
        word.append(y)
        word.append(weight)
        word.append(height)
        words.append(word)
    # 排序，车牌号有顺序。words是一个嵌套列表
    words = sorted(words, key=lambda s: s[0], reverse=False)
    i = 0
    size=(24, 50)
    # word中存放轮廓的起始点和宽高
    for word in words:
        # 筛选字符的轮廓
        if (word[3] > (word[2] * 1.5)) and (word[3] < (word[2] * 5.5)) and (word[2] > 10):
            i = i + 1
            if word[2] < 15:
                splite_image = image[word[1]:word[1] + word[3], word[0] - word[2]:word[0] + word[2] * 2]
            else:
                splite_image = image[word[1]:word[1] + word[3], word[0]:word[0] + word[2]]
            # binary_img = splite_image.point(lambda p: 1 if p > threshold else 0, '1')
            # resized_img = binary_img.resize(size, Image.NEAREST)
            # if all word is 0/255, then it is not a word
            if np.all(splite_image == 0) or np.all(splite_image == 255):
                continue
            else:
                word_images.append(splite_image)
            # print(i)
    
    if len(word_images) !=7:
        error_count += 1
        continue

    print(words)

    output_dir = os.path.join(output_base_dir, files)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    fig, axes = plt.subplots(1, min(7, len(word_images)), figsize=(14, 2))  # 创建一个包含最多7个子图的图形

    for i, (splite_image, ax) in enumerate(zip(word_images, axes)):
        # 显示字符图像
        # ax.imshow(splite_image, cmap='gray')
        # ax.axis('off')  # 关闭坐标轴

        # 保存字符图像
        file_name = f"{files}_{i}.png"
        destination_path = os.path.join(output_dir, file_name)

        if isinstance(splite_image, np.ndarray):
            cv2.imwrite(destination_path, splite_image)

        else:
            # 如果 splite_image 不是 NumPy 数组，则可能需要先将其转换为数组
            # 例如，如果它是 PIL 图像，则可以使用 np.array(splite_image)
            splite_image_array = np.array(splite_image)
            cv2.imwrite(destination_path, splite_image_array)
        print(f"Saved image array to {destination_path}")
        binarize_and_resize_image(splite_image,output_dir, i)

    # # 调整布局以避免重叠
    # plt.tight_layout()
    # # 显示所有字符图像
    # plt.show()
# print('error count:',error_count)
# print('total count:',total_count)
# print('accuracy:',(total_count-error_count)/total_count)
