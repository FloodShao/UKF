#滤波器进行状态估计
任何传感器，要解决的问题就只有一个：**如何通过数据来估计自身状态**。每种传感器的测量模型不一样，他们的精度也不一样。换句话说，状态估计问题，也就是“如何最好地使用传感器数据”。

##离散时间系统的状态估计
记机器人在各时刻的状态为$x_1, x_2, ..., x_k$, 其中k是离散时间下标。在SLAM中，我们通常要估计机器人的位置，那么系统的状态就指的是机器人的位姿。用两个方程来描述状态估计问题：
$$\begin{cases}
x_k = f(x_{k-1}, u_k, w_k) \\
y_k = g(x_k, n_k) \\
\end{cases}
$$
其中：
* f为状态转移方程（也称为运动方程，可以是线性的也可以是非线性的）描述状态从$x_{k-1}$转换到$x_k$
* u为状态输入
* w为状态输入噪声，一般认为是高斯分布
* g为观测方程，描述从$x_k$状态观察到数据$y_k$
* y为观测数据
* n为观测噪声

我们会根据f, g是否线性，将他们分为线性/非线性系统。同时对于噪声w和n，根据他们是否为高斯分布，分为高斯/非高斯噪声系统。最一般，也是最困难的问题就是**非线性-非高斯（NLNG）**的状态估计

##Bayes Filter
定义状态估计$p(x|z, u)$，即根据此时的观测z和之前的状态输入，来估计现在的状态。

###递归贝叶斯滤波器
首先定义$bel(...)$为贝叶斯模型
From Bayes Rules:
$$ \begin{align}
bel(x_t) = p(x_t|z_{1:t}, u_{1:t})\\
= \eta * p(z_t | x_t, z_{1:t-1}, u_{1:t}) * p(x_t|z_{1:t-1}, u_{1:t})
\label{Bayes Rules}
\end{align}
$$

推导Bayes公式
$$
p(x_t | z_{1:t}, u_{1:t}) = p(x_t | z_t, z_{1:t-1}, u_{1:t}) = 
\frac{p(z_t | x_t, z_{1:t-1}, u_{1:t}) * p(x_t | z_{1:t-1}, u_{1:t})}{p(z_t | z_{1:t-1}, u_{1:t})}
$$
其中$$\frac{1}{p(z_t | z_{1:t-1}, u_{1:t})}$$将会是一个常数确定的值 $\eta$

对$\eqref{Bayes Rules}$进行Markov Assumption 假设，可以推得：
$$
\begin{align}
bel(x_t) = \eta * p(z_t|x_t) * p(x_t | z_{1:t-1}, u_{1:t})
\label{Markov Assumption}
\end{align} 
$$

根据全概率公式，$\eqref{Markov Assumption}$又可写成：
$$
\begin{align}
bel(x_t) = \eta * p(z_t | x_t) * \int p(x_t|x_{t-1}, z_{1:t-1}, u_{1:t}) p(x_{t-1}|z_{1:t-1}, u_{1:t}) dx_{t-1}
\end{align}
$$

进行Markov Assumption，上式又表示为
$$
\begin{align}
bel(x_t) = \eta * p(z_t | x_t) * \int p(x_t|x_{t-1}, u_t) p(x_{t-1}|z_{1:t-1}, u_{1:t-1}) dx_{t-1} \\
 = \eta * p(z_t | x_t) * \int p(x_t|x_{t-1}, u_t) bel(x_{t-1}) dx_{t-1}
\end{align}
$$

至此我们得到了一个迭代模型，由上一时刻的贝叶斯模型来估计当前时刻位姿的贝叶斯模型。

**Prediction Step**
$$
\overline{bel}(x_t) = \int p(x_t|x_{t-1}, u_t)* bel(x_{t-1}) dx_{t-1}
$$
in which p(x_t|x_{t-1}, u_t) is the motion model, AKA f() in previous section
**Correction Step**
$$
bel(x_t) = \eta * p(z_t | x_t) * \overline{bel}(x_t)
$$
in which, p(z_t | x_t) is the sensor and observation model, AKA g() in previous section
前边的$\eta$为正则化系数，所有的正则化系数相加为1
之后我们提出的所有的滤波器模型，都是基于Bayes Filter模型进行假设。

## 线性高斯系统 Kalman Filter是最优估计
在线性高斯系统中，运动方程、观测方程是线性的，且两个噪声项均服从零均值高斯分布。**高斯分布经过线性变换之后仍为高斯分布**，对于一个高斯分布，只要计算出它的一阶和二阶矩就可以描述他。
```
In mathematics, a moment is a specific quantitative measure, used in both mechanics and statistics, of the shape of a set of points
矩是一组点组成的模型的特定的数量测度。
如果这些点在力学中代表“质量”：
* 零阶矩代表所有点的质量
* 一阶矩代表质心
* 二阶矩表示转动惯量
如果这些点代表概率密度：
* 零阶矩表示这些点的总概率 （1）
* 一阶矩代表期望
* 二阶（中心）矩代表方差
* 三阶（中心）矩表示偏斜度
* 四阶（中心）矩表示峰度
```
在数学上，k阶原点矩表示随机变量x“偏离”原点的“距离”的k次方的期望值。k阶中心距表示随机变量x“偏离”其中心的“距离”的k次方的期望值。一般以其平均数为中心。一般的，对于正整数k,如果$E[(X-0)^k] = E[X^k] < \infty$,那么就称$E[X^k]$为随机变量X的k阶原点矩。

线性系统的状态形式如下：
$$\begin{cases}
x_k = A_{k-1}x_{k-1} + u_k + w_k\\ 
y_k = C_{k}x_{k} + n_k\\
w_k \sim N(0, Q_k) \\
n_k \sim N(0, R_k) \\
\end{cases}
$$
通过计算x的贝叶斯估计的MAP (Maximum A Posterior)分布，可以直接得出Kalman Filter。Kalman Filter是线性递推系统的无偏最优估计。

```
需要后续增加两种推导过程，MAP推导和求导归零推导
```

Kalman Filter的五个公式
（1）Prediction
State transformation model (Use the mean as the prediction)
$$
\hat{x_k} = A * x_{k-1}
$$ 
State Covariance update (Motion Model Noise)
$$ 
\hat{P_k} = A * P_{k-1} * A^T + Q 
$$

(2) Correction
Observation Model
$$ y_k = H * \hat{x_k} $$
Actual Observation $Z_k$
Kalman Gain
$$ K = \frac{\hat{P_k}*H^T}{H*\hat{P_k}*H^T + R}$$
Final Prediction
$$ x_k = \hat{x_k} + K*(Z_k-y_k) $$
$$ P_k = (I - K*H)\hat{P_k}$$

可以得知，Kalman Gain越小，根据运动模型得出的估计值越准，也就是传感器对运动输入量的测量越准确。

##非线性高斯系统 Extended Kalman Filter 
对于非线性高斯系统，EKF做了两点：
（1）在工作点附近$\hat{x_{k-1}}, \hat{x_k}$对非线性系统进行一阶泰勒展开进行线性近似化，计算状态转换方程和测量方程的Jacobian矩阵：
$$f(x_{k-1}, u_k, w_k) \approx f(\hat{x_{k-1}}, u_k, 0) + \frac{\partial f}{\partial x_{k-1}}(x_{k-1} - \hat{x_{k-1}}) + w_k$$
$$h(x_k, n_k) \approx h(\tilde x_k, 0) + \frac{\partial h}{\partial x_k}(\tilde{x_k} - x_k) + n_k$$
(2)在线性系统近似下，将噪声项和状态都当成了**高斯分布**。这样只要估计他们的均值和协方差矩阵就可以描述状态了。经过这样的近似，后续工作和卡尔曼滤波是一样的。
5个相对应的估计方程：
**Prediction**
利用状态输入得到的均值来估计
$$\tilde{x_k} = f(x_{k-1}, u_k, 0)$$
$$\tilde{P_k} = J_A * P_{k-1} * J_A^T + Q$$
**计算Kalman Gain**
$$y_k = h(x_k)$$
$$K = \frac{\tilde{P_k}J_H^T}{J_H \tilde{P_k} J_H^T + R}$$
**Correction**
$$x_k = \tilde{x_k} + K(Z_k - y_k)$$
$$P_k = (I - K J_H)\tilde{P_k}$$
```
EKF面临的问题：
(1) 即使是高斯分布，经过一个非线性变换后也不再是高斯分布。EKF只能通过计算均值和协方差，是在用高斯近似这个非线性变换后的结果。而实际中这个近似可能很差。
(2) 系统本身线性化的过程中，丢掉了高阶项。
(3) 线性化的工作点往往不是输入状态真实的均值，而是一个估计的均值。于是在这个工作点下计算得到的F和H的Jacobian矩阵可能也是不好的
(4) 在估计非线性输出的均值时，EKF计算的是上一个状态的均值经过非线性变换的均值。这个结果几乎不会是输出分布的真实期望值，协方差也是同理。 
```
那么如何克服以上缺点？
（1） 为了克服上述（3）中工作点的问题，我们以EKF估计的结果作为工作点展开，重新计算一遍EKF，直到这个工作点变化足够小。这就会推出IEKF(Iterative EKF)。 EKF进行近似可能离真实估计较远，通过IEKF可以通过优化的方式得到local MAP，离真实的估计比较近。
（2）为了克服上述（4）中的缺点，我们除了$\mu_y = f(\mu_x)$这个形式，我们再计算几个精心挑选的采样点，然后用这几个点估计输出的高斯分布。这就为Sigma Point KF (SPEKF or UKF)。
或者**我们不要进行高斯分布假设**。于是问题变为，丢掉高斯假设之后，怎么描述输出函数的分布。一种暴力的方式是：**用足够多的采样点，来表达输出的分布**。这种蒙特卡洛的方式就是粒子滤波的思路。
再进一步，**丢弃滤波器思路**。为什么要用前一时刻的值来估计下一时刻，我们可以把所有状态看成变量，把运动方程和观测方程看成变量间的约束，构造误差函数，然后最小化这个误差的二次型，就得到了非线性优化的方法。这就是SLAM中**图优化**的思路。

##UKF无迹卡尔曼滤波
UKF通过取多个点的方式来近似线性模型，因为没有使用Jacobian和偏导数，让计算变得更加简单并且也没有忽略高阶导数项。相对于EKF有更少的计算量，并且效果也更好。

UKF生成了一些sigma points来决定实际的分布的均值与方差。与粒子滤波器不同的是，UKF的sigma points生成与概率分布无关。

###生成sigma points
根据$N(\mu_x, \Sigma_{xx})$生成$2L+1$个sigma points，同时保证：
$$\boldsymbol{L}\boldsymbol{L}^T = \Sigma_{xx} \\
\boldsymbol{x_0} = \boldsymbol{\mu_x} \\
\boldsymbol{x_i} = \boldsymbol{\mu_x} + \sqrt{L+\kappa} col_i \boldsymbol{L} \\
\boldsymbol{x_{i+L}} = \boldsymbol{\mu_x} - \sqrt{L+\kappa} col_i \boldsymbol{L}$$ 
其中 $L = dim(\boldsymbol{\mu_x})$,并且
$$
\boldsymbol{\mu_x} = \sum_{i_0}^{2L+1} \alpha_i\boldsymbol{x_i} \\
\boldsymbol{\Sigma_{xx}} = \sum_{i_0}^{2L+1} \alpha_i(\boldsymbol{x_i} - \boldsymbol{\mu_x}) (\boldsymbol{x_i} - \boldsymbol{\mu_x})^T \\
\alpha_i = 
\begin{cases}
\frac{\kappa}{L+\kappa} , i = 0\\
\frac{1}{2}\frac{1}{L+\kappa}, others\\
\end{cases}
$$
###将上述sigma points依次通过非线性系统
$$\boldsymbol{y_i} = g(\boldsymbol{x_i}), i = 0, ..., 2L $$
###计算均值与方差
$$\boldsymbol{\mu_y} = \sum_{i_0}^{2L} \alpha_i\boldsymbol{y_i} \\
\boldsymbol{\Sigma_{yy}} = \sum_{i_0}^{2L} \alpha_i(\boldsymbol{y_i} - \boldsymbol{\mu_y}) (\boldsymbol{y_i} - \boldsymbol{\mu_y})^T  + \boldsymbol{R}\\$$
其中R为measurement noise covariance
### UKF的优点
（1）通过用sigma points去近似状态输入的分布，不用计算线性化的Jacobian矩阵
（2）仅仅使用了初等线性代数的计算 Cholesky分解，外卷积，和矩阵加减
（3）计算的成本也是线性的，（相比于要数值化计算Jacobian矩阵）
（4）对非线性系统没有光滑和线性化的必要，可以保留高阶项的信息
















