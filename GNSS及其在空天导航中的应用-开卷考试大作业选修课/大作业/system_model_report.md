
# 系统建模与滤波算法说明报告（含变量释义）
**文件名：system_model_report.md**

---

## 一、总体概述

本系统基于 IMU（惯性测量单元）与 GPS 数据融合，通过无迹卡尔曼滤波（UKF）实现对象在 NED 坐标系（北-东-下）下的位置、姿态与速度估计。  
以下公式均使用 LaTeX 语法表示，并在每处给出公式中出现的变量含义。

---

## 二、坐标系定义与符号约定

常用符号（列向量均为列）：
\[
\begin{aligned}
\mathbf{p} &= [p_N, p_E, p_D]^T && \text{位置（NED）} \\
\mathbf{v}_b &= [v_x, v_y, v_z]^T && \text{机体/手机坐标系下的速度} \\
\mathbf{q} &= [q_0, q_1, q_2, q_3]^T && \text{单位四元数表示的姿态} \\
\boldsymbol{\omega}_b &= [P, Q, R]^T && \text{机体角速度（陀螺仪），单位：rad/s} \\
\mathbf{a}_b &= [a_x, a_y, a_z]^T && \text{机体坐标系下的加速度（加速度计测量），单位：m/s}^{2} \\
\mathbf{y}_{gps} &= [\text{lat}, \text{lon}, \text{alt}]^T && \text{GPS 测量（纬度/经度：deg，海拔：m）} \\
Y_0 &= [\text{lat}_0, \text{lon}_0, \text{alt}_0]^T && \text{参考初始 GPS 点}
\end{aligned}
\]

---

## 三、IMU（陀螺仪与加速度计）数学建模

### 3.1 陀螺仪测量模型

\[
\boldsymbol{\omega}_m = \boldsymbol{\omega}_b + \mathbf{b}_g + \mathbf{n}_g
\]

变量含义：
- \(\boldsymbol{\omega}_m\)：陀螺仪的测量输出向量 \([P_m,Q_m,R_m]^T\)（rad/s）。
- \(\boldsymbol{\omega}_b\)：系统真实的机体角速度 \([P,Q,R]^T\)（rad/s）。
- \(\mathbf{b}_g\)：陀螺仪零偏（bias），通常随时间缓慢漂移（rad/s）。
- \(\mathbf{n}_g\)：陀螺仪测量噪声，常假设为零均值高斯白噪声，协方差 \( \mathbb{E}[\mathbf{n}_g\mathbf{n}_g^T]=R_g \)。

注：若把 \(\mathbf{b}_g\) 视为未知常数或随机游走过程，可将其加入状态向量并估计。

---

### 3.2 加速度计测量模型

\[
\mathbf{a}_m = \mathbf{a}_b + \mathbf{b}_a + \mathbf{n}_a
\]

变量含义：
- \(\mathbf{a}_m\)：加速度计测量（机体坐标系），单位 m/s\(^2\)。
- \(\mathbf{a}_b\)：真实机体加速度（含惯性加速度和重力在某种表示下）。
- \(\mathbf{b}_a\)：加速度计零偏（bias），单位 m/s\(^2\)。
- \(\mathbf{n}_a\)：加速度计测量噪声，通常为零均值高斯噪声，协方差 \(R_a\)。

工程实践：
- 通常需把重力以合适方式从测量中分离或在状态方程中显式建模。代码中在预处理阶段对加速度做了常数校正来近似移除重力分量。

---

## 四、GPS 数学建模与坐标转换

GPS 原始输出：
\[
\mathbf{y}_{gps} = \begin{bmatrix}\text{lat}\\ \text{lon}\\ \text{alt}\end{bmatrix}
\]
变量含义：
- \(\text{lat}, \text{lon}\)：纬度、经度（单位：度）。
- \(\text{alt}\)：海拔高度（单位：米）。

### 4.1 GPS→NED（局部增量近似）

设参考点为 \(Y_0 = [\text{lat}_0,\text{lon}_0,\text{alt}_0]^T\)，定义
\[
\begin{aligned}
dlat &= (\text{lat}-\text{lat}_0)\cdot\frac{\pi}{180},\\
dlon &= (\text{lon}-\text{lon}_0)\cdot\frac{\pi}{180},\\
R(\phi) &\approx \text{地球在纬度 }\phi\text{ 处的曲率半径（m）},\\
dN &= R(\text{lat}_0)\cdot dlat,\\
dE &= R(\text{lat}_0)\cdot dlon\cdot\cos(\text{lat}_0),\\
dD &= -(\text{alt}-\text{alt}_0).
\end{aligned}
\]

变量含义补充：
- \(dN,dE,dD\)：相对于参考点的北向、东向、下向增量（单位：m）。
- \(R(\phi)\)：地球半径或局部曲率半径函数，常用 WGS84 近似公式计算。

**重要修正**：高度 \( \text{alt} \) 是米，不应进行角度转换（即不要 `deg2rad(alt)`）。

### 4.2 NED→GPS（逆变换）

\[
\begin{aligned}
\text{lat} &= \text{lat}_0 + \frac{dN}{R(\text{lat}_0)}\cdot\frac{180}{\pi},\\
\text{lon} &= \text{lon}_0 + \frac{dE}{R(\text{lat}_0)\cos(\text{lat}_0)}\cdot\frac{180}{\pi},\\
\text{alt} &= \text{alt}_0 - dD.
\end{aligned}
\]

变量含义同上。

GPS 测量噪声模型：
\[
\mathbf{v}_k \sim \mathcal{N}(0, R)
\]
- \(\mathbf{v}_k\)：GPS 测量噪声（经度/纬度/高度的噪声），与 \(R\) 的单位一致（注意角度 vs 米）。

---

## 五、状态空间模型

### 5.1 状态向量

定义状态向量：
\[
\mathbf{x} =
\begin{bmatrix}
p_N \\ p_E \\ p_D \\ q_0 \\ q_1 \\ q_2 \\ q_3 \\ v_x \\ v_y \\ v_z
\end{bmatrix}
\]
变量含义：
- \(p_N,p_E,p_D\)：在 NED 框下的位置分量（m）。
- \(q_0,q_1,q_2,q_3\)：四元数分量，单位四元数表示旋转（体到惯性或惯性到体需约定）。
- \(v_x,v_y,v_z\)：机体坐标系下的速度分量（m/s）。

### 5.2 连续时间过程方程

位置传播：
\[
\dot{\mathbf{p}} = \mathbf{C}_{ib}^T \mathbf{v}_b
\]
变量含义：
- \(\mathbf{C}_{ib}\)：从机体坐标系（body）到惯性/NED 坐标系（inertial）的方向余弦矩阵（DCM），由四元数 \(\mathbf{q}\) 计算得到；若 \( \mathbf{v}_b \) 在机体系，则通过 \( \mathbf{C}_{ib}^T \) 转为惯性系下的位置变化。

四元数传播：
\[
\dot{\mathbf{q}} = \frac{1}{2}\boldsymbol{\Omega}(\boldsymbol{\omega}_b)\mathbf{q}
\]
其中
\[
\boldsymbol{\Omega}(\boldsymbol{\omega}_b) =
\begin{bmatrix}
0 & -P & -Q & -R \\
P & 0 & R & -Q \\
Q & -R & 0 & P \\
R & Q & -P & 0
\end{bmatrix}
\]
变量含义：
- \(\boldsymbol{\omega}_b = [P,Q,R]^T\)：机体角速度。
- \(\dot{\mathbf{q}}\)：四元数时间导数。

速度传播（机体框）：
\[
\dot{\mathbf{v}}_b = \mathbf{a}_m - (\boldsymbol{\omega}_b \times \mathbf{r}_{sensor})
\]
变量含义：
- \(\mathbf{a}_m\)：加速度计测量（通常已去重力或在测量中保留重力，需一致）。
- \(\mathbf{r}_{sensor}\)：传感器相对于机体参考点（如车体质心或车轮坐标）的位移向量（m）。
- \(\boldsymbol{\omega}_b \times \mathbf{r}_{sensor}\)：角速度引起的线速度项（Coriolis/旋转相关项）。

### 5.3 离散化（显式欧拉）

\[
\mathbf{x}_{k+1} = \mathbf{x}_k + f(\mathbf{x}_k, \mathbf{u}_k)\Delta t + \mathbf{w}_k,
\quad \mathbf{w}_k \sim \mathcal{N}(0, Q)
\]
变量含义：
- \(\mathbf{u}_k\)：输入（传感器测量，如 \(\mathbf{a}_m,\boldsymbol{\omega}_m\)）。
- \(\Delta t\)：采样周期（代码中为 0.1 s）。
- \(Q\)：过程噪声协方差矩阵。

四元数归一化（数值稳定性）：
\[
\mathbf{q} \leftarrow \frac{\mathbf{q}}{\|\mathbf{q}\|}
\]

---

## 六、观测模型（GPS）

观测方程：
\[
\mathbf{y}_k = h(\mathbf{x}_k) + \mathbf{v}_k
\]
\[
h(\mathbf{x}) = Y_0 + \text{NED2GPS}(\mathbf{p})
\]
变量含义：
- \(\mathbf{y}_k\)：GPS 测量向量 \([\text{lat},\text{lon},\text{alt}]^T\)。
- \(h(\mathbf{x})\)：从状态映射到 GPS 空间的非线性函数（先取 NED 位置后转换为经纬度增量并加回基准点）。
- \(\mathbf{v}_k\)：测量噪声，协方差矩阵为 \(R\)。

---

## 七、无迹卡尔曼滤波（UKF）数学原理（含变量释义）

设状态维度为 \(n\)，后验估计为 \(\mathbf{x}\)，协方差为 \(P\)。

### 7.1 Sigma 点生成

Cholesky 分解：
\[
\mathbf{S} = \mathrm{chol}(nP)
\]
\[
\chi_i =
\begin{cases}
\mathbf{x} + \mathbf{S}_i, & i=1,\dots,n\\
\mathbf{x} - \mathbf{S}_{i-n}, & i=n+1,\dots,2n
\end{cases}
\]
变量含义：
- \(\mathbf{S}_i\)：Cholesky 根矩阵 \(\mathbf{S}\) 的第 \(i\) 行（或列，取决实现）。
- \(\chi_i\)：第 \(i\) 个 sigma 点（状态空间向量）。

权重（等权实现）：
\[
W_i = \frac{1}{2n}
\]

### 7.2 时间更新（预测）

\[
\chi_i^- = f(\chi_i, \mathbf{u}_k)
\]
\[
\hat{\mathbf{x}}^- = \sum_{i=1}^{2n} W_i \chi_i^-
\]
\[
P^- = \sum_{i=1}^{2n} W_i (\chi_i^- - \hat{\mathbf{x}}^-)(\chi_i^- - \hat{\mathbf{x}}^-)^T + Q
\]
变量含义：
- \(\chi_i^-\)：经过过程模型 \(f\) 传播后的 sigma 点。
- \(\hat{\mathbf{x}}^-\)、\(P^-\)：先验均值与先验协方差。
- \(Q\)：过程噪声协方差。

### 7.3 测量更新

将先验 sigma 点映射到观测空间：
\[
\gamma_i = h(\chi_i^-)
\]
\[
\hat{\mathbf{y}} = \sum_{i=1}^{2n} W_i \gamma_i
\]
\[
P_y = \sum_{i=1}^{2n} W_i (\gamma_i - \hat{\mathbf{y}})(\gamma_i - \hat{\mathbf{y}})^T + R
\]
\[
P_{xy} = \sum_{i=1}^{2n} W_i (\chi_i^- - \hat{\mathbf{x}}^-)(\gamma_i - \hat{\mathbf{y}})^T
\]
\[
K = P_{xy} P_y^{-1}
\]
\[
\mathbf{x}^+ = \hat{\mathbf{x}}^- + K(\mathbf{y}_{meas} - \hat{\mathbf{y}})
\]
\[
P^+ = P^- - K P_y K^T
\]
变量含义：
- \(\gamma_i\)：第 \(i\) 个 sigma 点在观测空间的映射。
- \(\hat{\mathbf{y}}\)：预测观测均值。
- \(P_y\)：预测观测协方差矩阵（包含测量噪声 \(R\)）。
- \(P_{xy}\)：状态-观测交叉协方差。
- \(K\)：卡尔曼增益。
- \(\mathbf{y}_{meas}\)：实际测量值（GPS）。

注：代码中为数值稳定使用 \( \text{pinv}(P_y) \) 或伪逆替代逆矩阵。

---

## 八、噪声与协方差（变量释义）

示例（代码内给定）：
\[
P_0 = \mathrm{diag}(100,100,100,10^{-4},10^{-5},10^{-5},10^{-5},0.1,0.1,0.1)
\]
\[
R = \mathrm{diag}(1.1964\times10^{-12},\;1.1964\times10^{-12},\;0.0947)
\]

含义：
- \(P_0\)：初始状态估计的协方差矩阵（对角给出各分量方差）。
- \(R\)：测量噪声协方差矩阵。前两项对应经度/纬度的方差（需注意单位：度或弧度），第三项对应高度（米）的方差。

---

## 九、重要变量总表（方便查阅）

- \(\mathbf{p}=[p_N,p_E,p_D]^T\)：位置（m）
- \(\mathbf{v}_b=[v_x,v_y,v_z]^T\)：机体速度（m/s）
- \(\mathbf{q}=[q_0,q_1,q_2,q_3]^T\)：四元数（单位四元数）
- \(\boldsymbol{\omega}_b=[P,Q,R]^T\)：角速度（rad/s）
- \(\mathbf{a}_m\)、\(\boldsymbol{\omega}_m\)：IMU 测量（加速度、角速度）
- \(\mathbf{b}_a, \mathbf{b}_g\)：加速度计 / 陀螺仪偏置
- \(Q\)：过程噪声协方差
- \(R\)：观测噪声协方差
- \(n\)：状态维度（本例为 10）
- \(\chi_i\)、\(\gamma_i\)：UKF 中的 sigma 点（状态空间与观测空间）

---

## 十、改进建议

1. 四元数每步归一化。  
2. 将 IMU 偏置加入状态向量。  
3. 使用带权参数的 UKF。  
4. 校验 GPS 单位一致性。  
5. 修复 `gps2ned` 高度角度错误。  
6. 对 P 做正定修正以避免数值不稳定。

---

## 十一、总结

本文档对每个主要公式中的变量都加上了详细释义，便于在实现和验证算法时逐项核对单位与意义。如需我把此 Markdown 导出为可下载文件（system_model_report.md），我可以立即将其写入并给出下载链接。
