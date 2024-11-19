# ICP
ICP中主要有基于几何，基于统计的，基于特征的。三者的相似点为都需要最近邻。最近邻的实现分为体素法和kd-tree,oct-tree。
## 基于几何的ICP
基于几何的ICP主要有3个问题需要解决
1. 拟合。基于最近邻的点进行拟合。假设该点在拟合对象上。
2. 残差。每个点云（通过滤波过滤过的) 到拟合平面的距离即为残差。
3. 雅可比。残差对R和t的雅可比。
基于残差和雅可比，利用高斯牛顿法或者LM迭代方法，不断减少残差，直到收敛。得到R和t。
### 面的拟合
1. 面的公式推导
 $$ 
 nx+d=0   (四个未知数)  \\
 n=[n,d],x=[x,1] (求解n) \\
 x^Tn=0  (对于任意面上x)\\
 令 X = [x_1,x_2,x_3,...,x_n]^T \\
 求最小二乘解  min(n^TXX^Tn) \\
 XX^T为对称矩阵，对其进行SVD分解。当n=v^T(col.4)时取得最小值
 $$
 2. 求解出面之后，可对其进行验证，每个点到面的距离。剔除效果差的点和面。（可以迭代优化）
### 线的拟合
线的公式推导
直观上，对一堆点进行PCL变换，则最大值对奇异应的轴在最小二乘法的意义下为最优轴。
$$
线的公式: dt + p = x , d为线的方向 ||d||=1，t为尺度标量，x为线上的点，p为线上任一点。\\
不妨设 p = X_{mean},则dt = x-p （理想情况下） \\
对于拟合问题，则是求（x-p）到 dt的距离最小。即d^Tx最大。\\
同面的拟合，此处取d=v^T(col.1)
$$
### 基于点的ICP
1. 残差公式
$$
e_i = Rp_i + t - q_i
$$
2. 雅可比
$$
\frac{\partial e_i}{\partial R} = -R\hat p_i \\
\frac{\partial e_i}{\partial t} = I \\

J_R = \sum_{i=1}^n \frac{\partial e_i}{\partial R}  \\
J_t = \sum_{i=1}^n \frac{\partial e_i}{\partial t} \\
e = \sum_{i=1}^n e_i
$$
3. 补充：基于SDV的ICP
更具最小二乘法构建其解
### 基于线的ICP
1. 残差公式
残差利用点到直线的距离
$$
e_i = \hat d(Rp_i + t - p)
$$
2.雅可比
$$
\frac{\partial e_i}{\partial R} = - \hat d(R\hat p_i) \\
\frac{\partial e_i}{\partial t} = \hat d(I)
$$
### 基于面的ICP
1. 残差公式
残差利用点到面的距离(一维)
$$
    e_i =  n^T(Rp_i + t) - d
$$
2.雅可比
$$
\frac{\partial e_i}{\partial R} = - n^T d(R\hat p_i) \\
\frac{\partial e_i}{\partial R} = - n^T \\
$$
## 基于统计的ICP NDT
### 基本原理
1. 假设体素内点云的分布为高斯分布，基于最大似然法估计R和t。
$$
第i个所在体素为的高斯分布为(u_i,\Sigma_i) \\
下式中第一个i表示体素 \\
u_i= \sum_j x_{ij} \\
\Sigma_i = E(x_{ii}x_{x_{ij}}) - E(x_{ii})E(x_{ij}) \\

其概率为 G_i(Rp_i + t - u_i | 0, \Sigma_i) \\
似然函数为：\Pi G_i \\
直接求解过于复杂，很多指数运算，采用迭代法
$$
2. 残差
$$
e_i = Rp_i + t - u_i
$$
3. 雅可比
$$
\frac{\partial e_i}{\partial R} = - R\hat p_i \\
\frac{\partial e_i}{\partial t} = I
$$
雅可比基于点的与统计相同，信息矩阵是体现其差别的地方。
4. 信息矩阵（协方差之逆)
$$
\sum_i (J_i^T\Sigma_i^{-1}J_i)\Delta x = - \sum_i e_iJ_i^T\Sigma_i^{-1}
$$
基于点的ICP是没有信息矩阵的，可能导致异常值影响过大。
### 增量NDT
增量NDT关键点在重新计算 $u_i,\Sigma_i$，\\
 设两个点云的均值和协方差分别为 $u_{ia}$ 和 $\Sigma_{ia}$,$ u_{ib} $,$\Sigma_{ib}$。\\
 ,数量分别为n,m则
 $$
 u_i = (n * u_{ia} + m * u_{ib})/(n+m)) \\
 \Sigma_i = (n*(\Sigma_{ia} + (u_{ia} - u_i)(u_{ia} - u_i)^T) \\ 
 + m*(\Sigma_{ib} + (u_{ib}-u_i)(u_{ib}-u_{i})^T)) / (n + m) 
 $$