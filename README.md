## 环境依赖

- OS：Ubuntu 22.04
- C++ 标准：C++17
- 依赖库：
  - OpenCV4（图像处理、视频读写）
  - Eigen3（矩阵线性代数，头文件库）
  - Ceres‑Solver（非线性最小二乘优化）

安装命令：

```
sudo apt update
sudo apt install build-essential cmake libopencv-dev libeigen3-dev libceres-dev
```

## 工程目录

```
vision_training/
├── CMakeLists.txt          # 顶层构建脚本
├── README.md               # 本说明文档
├── include/                # 公共头文件
├── src/
│   ├── common/             # 公共工具代码
│   ├── task1_image/        # Task1 郁金香图像处理 main.cpp
│   ├── task2_fit/          # Task2 合成视频角速度拟合 main.cpp
│   └── task3_windmill/     # Task3 能量机关跟踪（待完成）
├── resources/
│   ├── test_image.jpg      # Task1输入郁金香图片
│   ├── task_2.mp4          # Task2合成旋转视频
│   ├── task_3.mp4
│   └── task_4.mp4
└── result/
    ├── task1_images/       # Task1全部输出图片(16张)
    ├── task2_fit/          # Task2输出视频、曲线图、md报告
    │   ├── tracking_overlay.mp4
    │   ├── fit_comparison.png
    │   ├── angular_velocity.png
    │   └── residuals.png
    ├── task3_windmill/     # Task3输出视频    
    |    ├── task_3/
    |    │   ├── binary_process.mp4
    |    │   └── recognition_overplay.mp4
    |    └── task_4/
    |        ├── binary_process.mp4
    |        └── recognition_overplay.mp4
    ├── task2_fit_result.md
    └── task3_fit_result.md
```

### 素材说明
所有素材均为课程发放，统一存放于项目根目录下的 `resources/`：
- `test_image.jpg`：`resources/test_image.jpg`
- `task_2.mp4`：`resources/task_2.mp4`
- `task_3.mp4`：`resources/task_3.mp4`
- `task_4.mp4`：`resources/task_4.mp4`

## 编译与运行

```
# 创建构建目录
cmake -S . -B build

# 运行任务1
cmake --build build -j4 --target task1
./build/task1

# 运行任务2
cmake --build build -j4 --target task2
./build/task2

# 运行任务3
cmake --build build -j4 --target task3
# ./build/task3
```

---

# Task1：OpenCV 郁金香图像处理

## 任务简述

对`resources/test_image.jpg`郁金香图片完成：读图颜色转换、滤波对比、HSV 红色提取、形态学操作、轮廓筛选、绘图几何变换，输出共 16 张结果图片保存至`result/task1_images/`。

### 使用关键参数记录

1. **灰度 & HSV 通道**

- `cvtColor(img, gray, COLOR_BGR2GRAY)`；
- `cvtColor(img, hsv, COLOR_BGR2HSV)`，分离 H/S/V 单通道输出。

2. **滤波参数**

表格

| 滤波类型 | 核大小 | 其他参数 | 说明 |
| --- | --- | --- | --- |
| 均值滤波 | Size(5,5) | ‑ | 邻域平均平滑，边缘模糊明显 |
| 高斯滤波 | Size(5,5) | sigmaX=1.5 | 平滑同时保留部分边缘细节 |
| 中值滤波 | Size(5,5) | 核 = 5 | 抑制椒盐噪声，花瓣边缘保留优于均值滤波 |

> 
> 核越大平滑越强，细节丢失越严重；本任务统一使用 5×5 核。

3. **红色 HSV 阈值**

> 
> OpenCV HSV：H∈[0‑179], S∈[0‑255], V∈[0‑255]
> 红色跨越色相两端，两段区间合并：

- 区间 1：`Scalar(0,100,100)` ~ `Scalar(10,255,255)`
- 区间 2：`Scalar(170,100,100)` ~ `Scalar(179,255,255)`
- 使用`bitwise_or`合并两段 mask 得到红色掩膜。

**效果说明**

- 红色花瓣：可以较好提取出红色连通区域；
- 黄色花瓣：色相不在红色区间，不会进入红色 mask；
- 阴影区域：V 明度低，会被阈值过滤，阴影部分不会被识别为红色。

4. **形态学操作参数**

- 结构元：`MORPH_RECT`，Size(5,5)
- 腐蚀 erode：收缩红色前景，细小连通区域消失；
- 膨胀 dilate：扩张前景，填补小缝隙；
- 开运算 MORPH_OPEN：先腐蚀后膨胀，去除白色小噪点；
- 闭运算 MORPH_CLOSE：先膨胀后腐蚀，填补区域内部小黑洞。

5. **轮廓筛选参数**

- 轮廓来源：闭运算之后二值掩膜；
- 检索模式：`RETR_EXTERNAL`只取最外层轮廓；
- 压缩：`CHAIN_APPROX_SIMPLE`；
- 面积阈值：**最小面积 500**，过滤微小噪声轮廓；
- 外接矩形长宽比过滤：`0.2 < ratio <5.0`；

> 
> 在原图绘制筛选后轮廓、红色外接矩形；输出图片中标注每个筛选后轮廓的`contourArea()`面积。

6. **几何变换与绘制**
7. 在原图副本绘制：圆形、矩形、文字；
8. 绕图像**中心点旋转 35°**输出图片；
9. 裁剪原图左上角 1/4 区域（宽 / 2，高 / 2）输出图片。

### Task1 输出文件清单（result/task1_images/）

```
gray.png                 #灰度图
hsv_h.png                #H通道
hsv_s.png                #S通道
hsv_v.png                #V通道
mean_filter.png          #均值滤波
gaussian_filter.png      #高斯滤波
median_filter.png        #中值滤波
red_mask.png             #红色HSV掩膜
erode.png                #腐蚀结果
dilate.png               #膨胀结果
open.png                 #开运算
close.png                #闭运算
contours_boxes.png       #筛选轮廓+外接矩形，标注轮廓面积
drawing.png              #绘制圆、矩形、文字
rotated_35deg.png        #图像旋转35°
crop_top_left.png        #左上角1/4裁剪
```

---

# Task2：合成旋转视频参数拟合

## 任务简述

读取`resources/task_2.mp4`合成旋转视频，识别青色小球目标；计算**并展开角度（unwrap 解缠绕）**；采用拟合角速度路线，模型：
$$\omega(t)=b+A\sin(\Omega t+\varphi)$$
使用 Ceres Solver 自动求导非线性最小二乘拟合；输出带标记视频、拟合对比图、角速度曲线、残差图；生成`task2_fit_result.md`报告。


### 关键要点

1. 旋转中心直接使用题目给定 $(c_x=480,\ c_y=360)$；
2. 角度计算公式严格遵循作业定义：
$$\theta_{wrapped}=\text{atan2}(c_y-y_i,\ x_i-c_x)$$
3. 处理：`atan2`得到包裹角度，执行 unwrap 解缠绕，**内存保存连续无跳变展开角度序列，不存储包裹角度数组**；
4. 角速度：间隔 kSample=5 帧差分基于展开角度序列计算观测角速度；
5. 拟合选择：**直接拟合角速度，因此不需要估计初始角度$\theta_0$**；
6. Ceres 约束：$`A>0, \Omega>0`$；求解结束相位$`\varphi`$归一化映射到 $`(-\pi,\pi)`$；
7. 线性求解器：`DENSE_QR`适合小规模稠密问题；
8. 评估指标：角速度 RMSE；记录有效样本数量、参与拟合帧区间。

### Task2 输出产物（result/task2_fit/）

- `tracking_overlay.mp4`：原视频叠加小球识别标记，画面打印展开角度与瞬时角速度；
- `fit_comparison.png`：原始观测角速度与 Ceres 拟合角速度对比曲线；
- `angular_velocity.png`：模型估计角速度曲线；
- `residuals.png`：拟合残差曲线；
- `task2_fit_result.md`：完整报告：识别流程、模型、初值、约束、求解状态、参数表、误差指标。

> 
> 详细参数数值见 `result/task2_fit/task2_fit_result.md`。

# Task3：真实能量机关视频识别与稳定跟踪

## 任务简述

读取`resources/task_3.mp4`（小能量机关，同时最多点亮 1 个扇叶）、`resources/task_4.mp4`（大能量机关，同时最多点亮 2 个扇叶）。
HSV 红色阈值分割 + 形态学闭运算 +`RETR_TREE`轮廓层级解析，区分 R 标与点亮扇叶；基于扇叶相对 R 标的角度特征实现跨帧跟踪，完成身份保持、丢失标记、条件重选。

> 
> 本任务**不做角速度拟合与预测**，仅完成视觉检测跟踪。

### 关键要点

1. 颜色分割：红色跨越 HSV 色相两端，两段`inRange`结果`bitwise_or`合并得到红色二值掩码；
2. 轮廓解析：`RETR_TREE`获取轮廓父子层级，依靠外轮廓子轮廓数量`childCount`区分点亮扇叶与 R 标候选；
3. R 标检测：统计有效圆环圆心求平均得到集群`hub`；引入**帧间记忆**，优先匹配上一帧 R 标位置抑制单帧噪声；连续丢失达到阈值清空历史记忆；
4. 目标关联：以 R 标为参考中心，计算扇叶相对 R 标的角度；跨帧采用角度差做匹配，**不直接使用轮廓数组下标作为目标 ID**；
5. 锁定重选：目标可见时维持原有`trackId`；短暂漏检标记`LOST`保留 ID；连续丢失超限置为非活跃，再次出现分配新 ID；R 标失效直接关闭扇叶跟踪逻辑；
6. 可视化：输出原图叠加标注视频与二值掩码中间视频。

### Task3 参数说明

表格

| 参数项 | 数值 | 说明 |
| --- | --- | --- |
| HSV 红色区间 1 | `Scalar(0,45,50)~Scalar(12,255,255)` | 红色低色相区间 |
| HSV 红色区间 2 | `Scalar(168,45,50)~Scalar(179,255,255)` | 红色高色相区间 |
| 形态学闭运算核 | `Size(9,9)` | 填补掩码内部断裂 |
| R 标 blob 面积 | `180~400` | R 标候选面积过滤 |
| R 标 hub 最大距离 | `180像素` | 仅有效圆环≥2 个时生效 |
| R 标连续丢失清空记忆 | `40帧` | R 标长时间消失重置帧记忆 |
| 扇叶角度匹配阈值 | `50°` | 跨帧匹配允许最大角度差 |
| 扇叶允许最大丢失帧数 | `30帧` | `LOST`状态容忍上限，超限置 inactive |
| 扇叶‑R 标最大距离 | `330像素` | 过滤距离中心过远扇叶 |

### Task3 输出产物（`result/task3_windmill/`）

- `task_3/recognition_overlay.mp4`：小能量机关原图叠加标注视频，绘制 R 标、扇叶轮廓、中心连线、ID 与跟踪状态；
- `task_3/binary_process.mp4`：小能量机关二值掩码中间过程视频；
- `task_4/recognition_overlay.mp4`：大能量机关原图叠加标注视频；
- `task_4/binary_process.mp4`：大能量机关二值掩码中间过程视频；
- `result/task3_tracking_result.md`：完整报告：检测流程、锁定重选规则、全部参数、已知局限。

> 
> 详细算法、规则与问题说明见 `result/task3_tracking_result.md`。