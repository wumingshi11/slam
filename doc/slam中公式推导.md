# slam中公式推导

# 优化

任何（一层）优化问题都可以归结为最小二乘法，线性优化为只需要一次运算即可求解。非线性优化一般求误差最小，不可简单表述为Hx = g, 然后求x. 使Hx与g的距离最小。 但利用泰勒级数，任何问题都可以通过一阶导数和二阶导数转化为线性问题。但转化后，与原问题不同，需要不断的迭代。同时，某些情况下非线性问题的二阶导较难求，一般使用拟牛顿法减少计算量。

## 线性优化

$$
Ax = b \\
A^TAx = A^Tb \\ 
x = (A^TA)^{-1}A^Tb
$$

## 非线性优化

非线性优化一般求误差最小

### 高斯牛顿法

$$
f(x + \Delta{x}) = f(x) + J^T(x) + ... \\
令f^2(x)最小 , 展开对x求导\\
即 J(x)J^T(x)\Delta{x} = -J(x)f(x)  \\ H(x) =  J(x)J^T(x)  \\ g(x) = -J(x)f(x)  \\ H\Delta{x} = g
$$

H有时半正定。

# 四元数

# 相机投影模型

$$
\begin{bmatrix}
u \\
v \\
1
\end{bmatrix}  = \begin{bmatrix}
f_x&0&c_x \\
0&f_y&c_y \\
0&0&1
\end{bmatrix} \begin{bmatrix}
X \\
Y \\
Z
\end{bmatrix} /Z
$$

注：

1. fx，fy为焦距，其实际含义为在深度为单位距离的投影平面中，一米的长宽在照片中投影为多少个像素。（双目相机中根据投影的像素位置，可以推算出深度，进一步计算出XYZ）
2. cx，cy为偏移量。
3. X,Y,Z为相机坐标系下坐标。
4. 设双目相机中视差为d, 双目之间距离为base，则 $Z = base * f_x / d$。

# 李代数

李代数用于位姿优化。由于无法直接对旋转矩阵R进行求导（R没有加法（不能作差）,且R为正交矩阵），转而对李代数求导。李代数与旋转矩阵一一对应。

## 李代数与李群的关系

公式含义

$$
R = exp(\hat{\phi}) \\
\phi = \theta a \\
a = Ra \\
\theta = arccos\frac{tr(R)-1}{2}\\
R^` = \hat{\phi}R
$$

李代数在为李群的切线

## 扰动模型

左扰动

$$
\frac{\partial{(Rp)}}{\partial\delta} = \lim_{\delta\to0}\frac{exp(\hat{\delta})exp(\hat{\phi})p - exp({\hat\phi})p}{\delta} \\ 
= \lim_{\delta\to0}\frac{(I + \hat{\delta})exp(\hat{\phi})p - exp({\hat\phi})p}{\delta} \\
= \lim_{\delta\to0}\frac{\hat\delta{Rp}}{\delta}= -\lim_{\delta\to0}\frac{\widehat{Rp}\delta}{\delta} = -\widehat{Rp}

$$

右扰动

$$
\frac{\partial{(Rp)}}{\partial\delta} = \lim_{\delta\to0}\frac{exp(\hat{\phi})exp(\hat{\delta})p - exp({\hat\phi})p}{\delta} \\ = \lim_{\delta\to0}\frac{exp(\hat{\phi})(I + \hat{\delta})p - exp({\hat\phi})p}{\delta} \\= \lim_{\delta\to0}\frac{R\hat\delta{p}}{\delta}= -\lim_{\delta\to0}\frac{R\hat{p}\delta}{\delta} = -R\hat{p}
$$

一般使用左扰动模型，直接利用旋转后点。

# 拆分

$R_1R_2,李代数的变化$，两个李代数相加，旋转矩阵的变化。

# 重投影误差

重投影误差为相机模型和扰动模型的结合。

$$
\begin{bmatrix}
u^` \\
v^` \\
1
\end{bmatrix}  = \begin{bmatrix}
f_x&0&c_x \\
0&f_y&c_y \\
0&0&1
\end{bmatrix} \begin{bmatrix}
X^` \\
Y^` \\
Z^`
\end{bmatrix} / \hat{Z} \\

\begin{bmatrix}
X^` \\
Y^` \\
Z^`
\end{bmatrix} = R \begin{bmatrix}
X\\
Y \\
Z
\end{bmatrix} = exp(\hat{\phi})\begin{bmatrix}
X\\
Y \\
Z \\
1
\end{bmatrix} \\
e =\begin{bmatrix}
u_s \\
v_s 
\end{bmatrix} - \begin{bmatrix}
u^` \\
v^` 
\end{bmatrix} = \begin{bmatrix}
u \\
v 
\end{bmatrix}
$$

## 位姿误差

位姿误差为对R的导数
$$
\frac{\partial e}{\partial T} = \frac{\partial e}{\partial p^`}\frac{\partial p^`}{\partial T}
$$

先求 e 对 P_的导数

$$
\frac{\partial e}{\partial P^`} = -\begin{bmatrix}
\frac{\partial u}{\partial X^`}&\frac{\partial u}{\partial Y^`}&\frac{\partial u}{\partial Z^`} \\
\frac{\partial v}{\partial X^`}&\frac{\partial v}{\partial Y^`}&\frac{\partial v}{\partial Z^`} \\
\end{bmatrix} = - \begin{bmatrix}   f_x/Z^` & 0 & -f_x/Z^{`2} \\
0 & f_y/Z^` & -f_y/Z^{`2}
\end{bmatrix} \\
\frac{\partial p^`}{\partial T} = \begin{bmatrix} I & -\hat{p^`}  \\
0 & 0
\end{bmatrix}

\begin{bmatrix} 1&0&0&0&Z^`&-Y^` \\
0&1&0&-Z^`&0&X^` \\
0&0&1&Y^`&-X^`&0\end{bmatrix} \\ 
注意:前面是对t的倒数，后面是对R的倒数，g2o中顺序相反\\
\frac{\partial e}{\partial T}  = \frac{\partial e}{\partial P^`}\frac{\partial p^`}{\partial T}
$$

## 观察点误差

观察点误差为对X,Y,Z的导数。

$$
\frac{\partial e}{\partial p}  = \frac{\partial e}{\partial P^`}\frac{\partial p^`}{\partial p} = \frac{\partial e}{\partial P^`}R = - \begin{bmatrix}   f_x/Z^` & 0 & -f_x/Z^{`2} \\
0 & f_y/Z^` & -f_y/Z^{`2}
\end{bmatrix}R
$$

## 重投影误差中上述公式的利用。基于高斯牛顿法

# BA

BA为大规模重投影误差。设有n个观察点，m个目标。对整体进行优化。 其本质是个最小二乘法问题 。 最优化问题一般通过拟牛顿法求解。 需要做n次重投影误差优化。与重投影误差有区别的时，n次重投影误差中互为约束。所以将其整合为一个巨大的重投影误差。直观上，该矩阵很大，但利用其稀疏性，可以进行简化。  有以下几个问题需要注意：

$$
H\Delta{x} = g \\
H = J^TJ
$$

## 待优化向量的雅可比矩阵

位姿T的维数为6， 观察点的维数为3， 所以待优化变量的为 6*n + 3*m.

样本量最大为[m,n*m], 其取值每个观察点可以观察到多少个特征点。

考虑第一个样本，即第一行向量。他应该至于第一个位姿和第一个目标点有关，其余都有0.

## 待优化向量的海塞矩阵

由于雅可比矩阵的稀疏性，H也是稀疏矩阵

## 矩阵是否可化解

利用稀疏性和消元法，可以分两步进行求解，先求解位姿，然后求解目标点。

# 对极约束

## 约束

对于不同图像上匹配两个特征点，有如下约束

$$
x_2^T * \hat{t}R x_1 = 0 \\
p_2^t * K^{-T}\hat{t}RK^{-1}p_1 = 0 \\
本质矩阵 E = \hat{t}R     \\
 基础矩阵 F =  K^{-T}\hat{t}RK^{-1}
$$

## 求解

通过8个特征点即可求解E，不过通常，使用多个特侦点的最小二乘解。

$$
E = U\Sigma V^T \\
\hat{t_1} = UR_Z(\pi/2)\Sigma U^T  \\ R_1 = UR_Z(\pi/2)\Sigma V^T \\
\hat{t_2} = UR_Z(-\pi/2)\Sigma U^T  \\ R_2 = UR_Z(-\pi/2)\Sigma V^T \\

$$

将上述四组解代入验证，即可解的。

对极约束由于具有尺度不变性。所以t的值实际应该*s(未知)。一般情况下将t归一化。固定尺寸。以后以这个尺寸为标准。 

## 作用

对极约束用于初始化，除了固定尺寸，还要对特征点的深度进行估计（以t为标准），之后就可以进行pnp问题求解了。 

初始化时注意t不能为0。

# 三角化

三角化是为了求深度。s1,s2根据下式可解。

$$
s_2x_2 = s_1Rx_1 + t \\
s_2\hat{x_2}x_2 = 0 = s_1\hat{x_2}Rx_1+ \hat{x_2}t
$$

# EPNP
EPNP算法在解决3D-2D的位姿求解中有较好的效果，目前普遍使用。一般先使用EPNP + ransac求出一个初始值，然后使用g2o进行优化。

EPNP的思想是先求解地图点在相机坐标系下的位置，然后进行3D-3D的ICP求解。

利用条件：
1. 旋转前后旋转后点的相对关系不变。
$$
世界坐标系下4个点 \\
a1 = [X_1,Y_1,Z_1,1]\\
a2 = [X_2,Y_2,Z_2,1]\\
a3 = [X_3,Y_3,Z_3,1]\\
a4 = [X_4,Y_4,Z_4,1] \\
对于世界坐标系下任一点 b \\
则存在 \alpha_1 a_1 + \alpha_2 a_2 + \alpha_3 a_3 + \alpha_4 a_4 = b \\
且 \alpha_1 + \alpha_2 + \alpha_3 + \alpha_4 = 1 \\
在相机坐标系下 \\
a_1^` = Ra_1 + t \\
a_2^` = Ra_2 + t \\
a_3^` = Ra_3 + t \\
a_4^` = Ra_4 + t \\
b^` = Rb + t = \alpha_1 R a_1^` + \alpha_2 R a_2^` + \alpha_3 R a_3^` + \alpha_4 R a_4^` + t \\
在归一化平面 \\
[x_1,y_1,1] = a_1^1/Z_1^` \\
[x_2,y_2,1] = a_2^1/Z_2^`  \\
[x_3,y_3,1] = a_3^1/Z_3^`  \\
[x_4,y_4,1] = a_4^1/Z_4^`  \\


目前 \alpha_1 ,  \alpha_2 , \alpha_3,  \alpha_4可以通过矩阵乘法求出。\\
问题在于 求解 a_1^`,a_2^`,a_3^`,a_4^` 共12个未知量\\

约束关系为
b^` = Rb + t = \alpha_1 R a_1^` + \alpha_2 R a_2^` + \alpha_3 R a_3^` + \alpha_4 R a_4^` + t = s[x,y,1]\\
s = \alpha_1 Z_1^` + \alpha_2 Z_2^` + \alpha_3 Z_3^` + \alpha_4 Z_4^` + t \\

上式构造在了两个约束等式，6个点即可构造所有所有约束条件，进行求解。之后所有点转为3D点，进行ICP求解。

$$

# ICP
ICP问题基于SVD求解，不加证明地给出结论
$$
设两组点的质心为 p（变换后）,p^`(变换前) \\
q_i = p_i - p, q_i^` = p^`_i - p^` \\
R^* = argmin \sum _{i=1} ^{n} ||q_i - R q_i^`||^2 \\
W = \sum _{i=1} ^{n} q_iq_i^{`T} = U \Sigma V^T \\
R = UV^T \\
t^* = p - Rp^`
$$

# 卡尔曼滤波器

$$
x_k = A_kx_{k-1} + u_k +w_k  ~~~~[1] 
 \\ w_k 服从 N(0,R) \\
z_k = C_kx_k + v_k ~~~~~[2] \\
v_k 服从 N(0,Q)
$$

上述（1）（2）都可以独立对$x_k$进行计算，但通常情况下，二者不相同。最优估计需要综合考虑二者，计算出最佳$x_k^+$。 卡尔曼滤波器就是为了解答上述问题。

$$
x_k^+ = x_k^- + K(z_k - z_k^-) ~~~~~[3]
\\ x_k^- = A_kx_{k-1} +u_k \\
z_k^- = C_kx_k^-
$$

式3中如何求取K是关键。 下面使用最大似然估计法进行计算。

$$
N(x_k^+,P_k^+) = \theta N(C_kx_k，Q) N(x_k^-,P_k^-) ~~~[4]\\
P_K^- = A_kP_{k-1}^+A_k^T + R \\
P_K为x向量的不确定度（只有预测方程的情况下不断增大）\\
\theta N(C_kx_k，Q) 似然函数 \\
N(x_k^-,P_k^-) 先验分布
$$

求解方法是将【4】的指数部分展开，分别计算P_k和x_k。

可计算出

$$
K = P_K^-C_K^T(C_kP_K^-C_K^T+Q_k)^{-1} \\
x_k^+ = x_k^- + K(z_k - C_kx_k^-) \\ P_K^+ = (I - KC_k)P_k^-
$$

# 李代数
## 李代数的引出
$$
R(t)R(t)^T = I \\
对t 求导，得到 \\
R(t)^{'}R(t)^T + R(t)R(t)^{T'} = 0 \\
R(t)^{'}R(t)^T = - R(t)R(t)^{T'} = -(R(t)^{'}R(t)^T)^{T}\\
即上面是个反对称矩阵 \\
令 R(t)^{'}R(t)^T = {\Phi(t)}^{\Lambda} \\
R(t)^{'} = {\Phi(t)}^{\Lambda} * R(t)
解此方程，可得：\\
R(t) = e^{t{\Phi(t)}^{\Lambda}} \\
$$
由上式可推导出：任何一个R矩阵都可以表示为$e^{t{\Phi(t)}^{\Lambda}}，其中t为实数，t{\Phi(t)}^{\Lambda}为李代数$。（一对多）

## 旋转矩阵和李代数的对应关系
不加证明给出以下转换：
$$
\\ 李代数转为旋转矩阵 \\
R = exp(\theta \hat{a}) = cos(\theta)I + (1-cos(\theta))aa^T + sin(\theta)a^{\Lambda}  （1.1）\\
旋转矩阵转为李代数 \\
\theta = arccos((trace(R)-I)/2) \\
Ra = a \\

当 \theta  = 0 时，R = I \\
当 \theta -> 0 时，R = I + \theta a^{\Lambda}  根据此公式，进行扰动求导 （1.2）
$$
### 进一步，R随$\Phi$的变换（一阶近似）
$$
J_l = \frac{sin\theta}{\theta} + (1 - \frac{sin\theta}{\theta})aa^T +  \frac{1 - cos\theta}{2}a^{\Lambda} \\
J_r = J_l(-\Phi)  \\

R2 = exp({(\Phi + \Delta \Phi)}^{\Lambda}) \\
= exp({\Phi }^{\Lambda})exp({(J_r\Delta \Phi)}^{\Lambda} ) \\
= exp({(J_l\Delta \Phi)}^{\Lambda}) exp({\Phi }^{\Lambda})  （2）\\ 
（省略反算）
根据此公式进行李代数求导
$$
## 求导
引入李代数的原因就是为了方便求导。R是一个3*3的矩阵，求导比较复杂。转而求李代数的倒数。分别为李代数求导和扰动求导
### 李代数求导
根据 式2，可求出李代数加一个扰动的导数。
$$
\frac{\partial exp((\Phi + \Delta \Phi)^{\Lambda})p}{\partial \Delta\Phi} = -(Rp)^{\Lambda}J_l
$$
### 李代数伴随性质
$$
RExp(\Phi )R^T = exp(R\Phi^{\Lambda}R^T) = Exp(R\Phi) \\
 Exp(\Phi)R=  R Exp(R^T\Phi) = R exp((R^T\Phi)^{\Lambda}) 
$$
### 扰动求导
根据式1.2，可求出李代数的一个扰动的导数。
$$
\frac{\partial exp((\Delta \Phi)^{\Lambda})exp((\Phi )^{\Lambda})p}{\partial \Delta\Phi} = - (Rp)^{\Lambda} \\
\frac{\partial exp((\Phi )^{\Lambda}) exp((\Delta \Phi)^{\Lambda}) p}{\partial \Delta\Phi} = - Rp^{\Lambda} \\

$$
对R的导数
$$
\frac{\partial log(R_1R_2))}{\partial R_1} = \lim_{\phi->0} \frac{log(R_1exp(\phi^{\Lambda})R_2) - log(R_1R_2)}{\phi} \\
= \lim_{\phi->0} \frac{log(R_1R_2exp((R_2^T\phi)^{\Lambda})) - log(R_1R_2)}{\phi} \\
= \lim_{\phi->0} \frac{log(exp(log(R_1R_2) + J^{-1}(log(R_1R_2))(R_2^T\phi)) - log(R_1R_2)}{\phi} \\
= J^{-1}(log(R_1R_2))R_2^T \\
第二个 \\
\frac{\partial log(R_1R_2))}{\partial R_1} = J^{-1}(log(R_1R_2))
$$

# 运动学
运动学这一章中R，t为相机坐标系到世界坐标系的变换。不同于14讲中的。
## 李群视角下的运动学
### 旋转矩阵，角速度
$$
\dot{R} = Rw^{\Lambda} (w瞬时速度) \\
\Delta{R} = Exp(w^{\Lambda} \Delta t)) (瞬时变换，有上式解的)\\
R = R_0 * Exp(w^{\Lambda} \Delta t) 
$$
### 线速度
先考虑一个问题，相机坐标系下的速度如何转为世界坐标系下的速度。如果没有旋转：$ v_w = Rv_c$
如果有旋转呢？ 根据$p_w = R_{wc}p_c + t$，可以得到：$v_w = Rv_c + w \times (Rp_0 + t)$
$$
p_1 = R_{12}p_2 + t  \\
分别对R和p求导 关于时间的导数\\
v_1 = \dot{p_1} = R_{12}v_c + R_{12}w^{\Lambda} p_2 = R_{12} (v_2 + w^{\Lambda}p_2)\\
$$
补充：
p1，p2指什么，v1，v2指什么？ 上述公式有什么物理意义？ 与我们关注的车的速度有什么关系？
- 如果车辆不发生转动，则v1和v2是同一个向量在不同坐标系的转换。如果车辆
发生转动，则不是同一个向量。
- 采用微分的思想，将p当作车辆的位置，v_2车辆的t时刻的瞬时速度（相对于***上一个时刻***车辆坐标系下，该坐标系在一小段时间内是静止的）v2由轮速记提供或者通过IMU计算。
- 实时情况下，在车辆坐标系下，车辆速度始终为0。 
- 下一个车辆坐标系的变换可通过车辆速度计算。
- $w^{\Lambda}p$与v_1垂直，线速度一般不计算。

### 线性加速度
对v求导：
$$
a_1 = \dot{v_1} = \dot{R}_{12}（v_2 + w^{\Lambda} p_2） + R_{12} (\dot{v_2} + \dot{w}^{\Lambda} p_2 + w^{\Lambda}\dot{p_2})  \\  = R_{12}w^{\Lambda}(v_2 + w^{\Lambda} p_2) + R_{12}(\dot{v_2} + \dot{w}^{\Lambda} p_2 +  w^{\Lambda}\dot{p_2}) \\ = R_{12} (\dot{v_2} + 2w^{\Lambda} v_2 + \dot{w}^{\Lambda} p_2 + w^{\Lambda} w^{\Lambda} p_2) 
$$
运动中一般舍弃后3项。

