# Introduction
This project implementing a UKF to fuse the lidar and radar data provided from Udacity.

# Environment
The project is complied under macOS High Sierra 10.13.6.
Dependencies are: 
* cmake 3.14.1 (brew install cmake)
* GNU Make 3.81
* gcc 6.5.0

Install Xcode for IDE developing.

* eigen 3.3.7 For matrix calculation (brew install eigen)

# Sensor Measurements:
Lidar: (px, py)
Radar: (px, py, rho, phi, drho) where drho is the radius velocity
Ground Truths: (px, py, vx, vy)
**Measurement dataset.txt format**

| Sensor Type | Measurements | TimeStamp | Ground Truth |
| --- | --- | --- | --- |
| L(lidar) | (px, py) | timestamp | (px, py, vx, vy) |
| R(radar) | (rho, phi, drho) | timestamp | (px, py, vx, vy) |

# Generalized Gaussian Filter
Reference: State Estimation for Robotics (Timothy D. Barfoot)
Filter based state estimation are all constructed based on the Bayes Filter.
By assuming the **Process Noise** and **Measurement Noise** as Gaussian, we have:
(1) Kalman Filter, the **Motion Model** and the **Observation Model** are linear, such as constant velocity (CV) motion model, and lidar observation.

(2) Extended Kalman Filter, the motion model and observation model are non-linear, so we use the first order Taylor expansion to linearise the motion model and the observation model. We will calculate the Jacobian matrix. 

We can do this because when a Gaussian process pass through a linear system the outcome would still be Gaussian. So the predicted state, we use the approximated linear model (Jacobian matrix) to transfer the Gaussian process.
However, when a Gaussian process pass through a non-linear system, we can not predict the distribution of the outcome. That's the problem.

(3) UKF still assume the process of the non-linear system is Gaussian, however, instead of directly estimating the process distribution from the system, UKF samples the outcome, and calculate the mean and covariance of the outcome process. (sigma points)

The generalized gaussian filter can be formatted as:

1. Begin at 
$$
p(x_{k-1} | \bar{x_{0}}, u_{1:k-1}, z_{1:k-1}) = N(\hat{x}_{k-1}, \hat{P}_{k-1})
$$
Every state has the probability, we use the mean as the estimation of the current state. 
2. We pass through the non-linear motion model, and we get
$$
p(x_k | \bar{x_{0}}, u_{1:k}, z_{1:k-1}) = N(\bar{x}_k, \bar{P}_k)
$$
3. For the correction step, we add the observation at k-th step in the procee:
$$
p(x_k, z_k | \bar{x_{0}}, u_{1:k}, z_{1:k-1}) = N \left( 
\left[\begin{matrix}
\mu_{x, k}\\
\mu_{x, k}\\
\end{matrix} \right], 
\left[\begin{matrix}
\Sigma_{xx,k} & \Sigma_{xz,k}\\
\Sigma_{zx,k} & \Sigma_{zz,k}\\
\end{matrix}\right]
\right)
$$
4. Then we write the conditional Gaussian density for $x_k$：
$$
p(x_k| \bar{x_{0}}, u_{1:k}, z_{1:k}) = \\
N(\mu_{x, k} + \Sigma_{xz,k}\Sigma_{zz,k}^{-1}(z_k-\mu_{z,k}), \Sigma_{xx,k} - \Sigma_{xz,k}\Sigma_{zz,k}^{-1}\Sigma_{zx,k})
$$
where you can see that
$$
\hat{x}_{k} = \mu_{x, k} + \Sigma_{xz,k}\Sigma_{zz,k}^{-1}(z_k-\mu_{z,k}) \\
\hat{P}_{k} = \Sigma_{xx,k} - \Sigma_{xz,k}\Sigma_{zz,k}^{-1}\Sigma_{zx,k}
$$
Comparing with the Kalman Filter equation, we have:
$$
K = \Sigma_{xz,k}\Sigma_{zz,k}^{-1} \\
S = HPH^T+R = \Sigma_{zz,k}^{-1} \\
PH^T = \Sigma_{xz,k} \\
$$

All the filter based method are calculating the above three probability process.

# UKF
1. Prediction: 
**We engaging the process noise in here**

Given the k-1 th state, we generated an augmented vector and the augmented matrix to engage the process noise. 

$$
\mu = \left[ 
\begin{matrix}
\hat{x}_{k-1} \\ 
0 \\
\end{matrix}\right],
\Sigma = \left[
\begin{matrix}
\hat{P}_{k-1} & 0 \\
0 & Q_k\\
\end{matrix}
\right]
$$

We convert the {$\mu, \Sigma$} to a sigma point representation:
$$
LL^T = \Sigma,\\
sp_0 = \mu,\\
sp_i = \mu + \sqrt{l+\kappa} L.col(i),\\
sp_{i+l} = \mu - \sqrt{l+\kappa} L.col(i)\\
$$

Note that, this version we added the process noise directly to the sigma point rather than using the cholesky decomposition to construct the sigma point.

And then we recombine the sigmapoints into the predicted belief, according to

$$
\bar{x}_k = \sum_{i=0}^{2l}\alpha_i sp_i \\
\bar{P}_k = \sum_{i=0}^{2l}\alpha_i (sp_i - \bar{x}_k) \\
\alpha_i = 
\begin{cases}
\frac{\kappa}{l+\kappa}, i=0 \\
\frac{0.5}{l+\kappa}, otherwise\\
\end{cases}
$$

2. Measurement Sigma Point:

Use each sigma point to generate the observation sigma point to estimate the $\mu_Z$ and $\Sigma_zz$.
We pass all the sigma points through the observation model, adding the Measurement noise R, and recombine the observation sigma points using the same weight $\alpha_i$.

$$
\mu_z=\sum_{i=0}^{2l}\alpha_i spz_i \\
\Sigma_{zz} = \sum_{i=0}^{2l} \alpha_i (spz_i - \mu_z)(spz_i - \mu_z)^T
$$
Be sure to engaging the measurement noise. And the S matrix is 
$$
S = \Sigma_{zz} + R
$$

3. Correction step:
Use the sigma points of prediction belief and observation to calculate the
$$
\Sigma_{xz} = \sum_{i=0}^{2l} \alpha_i (sp_i - \mu_x) (spz_i - \mu_z)^T
$$

and the Kalman Gain can be derived as $K = \Sigma_{xz} \Sigma_{zz}^{-1}$

and estimation is:
$$
\hat{x}_k = \bar{x}_k + K(z - \mu_z),\\
\hat{P}_k = \bar{P}_k - K\Sigma_{zx}\\
$$




 

 







