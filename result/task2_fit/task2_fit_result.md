# Task2 合成旋转视频参数拟合报告
## 任务1：识别青色目标
- HSV颜色分割 + 形态学开运算，选取最大面积轮廓，minEnclosingCircle获取小球中心
- atan2得到原始包裹角度，执行unwrap角度解缠绕，内存保存展开后的连续角度序列
- 使用间隔kSample帧差分（基于展开角度序列）计算观测角速度

## 任务2：参数估计
- 模型 $\omega(t)=b+A\sin(\Omega t+\varphi)$，直接拟合角速度，不需要估计初始角度$\theta_0$
- 求解：Ceres AutoDiff自动求导，DENSE_QR
- 初值：b=1.35, A=0.549406, Ω=1.64984, φ=0.633697
- 参数约束：A>0, Ω>0；相位φ输出归一化至 (-π, π)
- Ceres求解状态：可用

|参数|数值|单位|
|---|---|---|
|b(平均角速度)|1.35|rad/s|
|A(振幅)|0.549406|rad/s|
|Ω(频率参数)|1.64984|rad/s|
|φ(相位)|0.633697|rad|

## 任务3误差
- 角速度RMSE：0.0123356 rad/s
- 有效拟合样本：287
- 帧采样间隔：kSample=5帧
- 参与拟合帧范围：第5帧 ~ 第1439帧

## 任务4输出
- tracking_overlay.mp4 带标记视频
- fit_comparison.png 观测‑拟合对比
- residuals.png 残差图
- angular_velocity.png 角速度曲线
