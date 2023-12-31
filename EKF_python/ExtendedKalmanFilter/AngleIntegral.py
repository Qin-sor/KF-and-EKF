import numpy as np
np.set_printoptions(suppress=True)
from matplotlib import pyplot as plt
np.random.seed(0)
# -----------------------------------------
'''
例子：假设有一辆小车从远点出发，在平面上进行运动。每个DeltaT时间，小车上的传感器
可以返回小车的速度speed和角速度omega。且每一时刻可以通过探测得到小车距离原点的距离。
希望得到小车每一时刻的x坐标和y坐标。

状态向量：将小车的x坐标、y坐标、小车车身的角度theta、速度speed和角速度omega作为系统状态向量（5维度）
观测向量：将小车距离原点的距离的平方，速度speed和角速度omega作为系统观测向量

状态方程：小车的x坐标等于上一时刻的x坐标，加上这一个时间段内行使的距离speed*Delta乘以cos(theta)
'''

'''
数据生成
定义状态向量为：[x,y,theta,speed,omega]
观测向量为：[dist, speed,omega]
则f(x_{k-1},u_k,0) = [[x_{k-1}+cos(theta)*speed*deltaT],
                      [y_{k-1}+sin(theta)*speed*deltaT],
                      [theta_{k-1}+omega*deltaT],
                      [speed_{k-1}],
                      [omega_{k-1}]]
f的雅各比矩阵为：A = [[1 0 -sin(theta_{k-1}*speed_{k-1}*deltaT cos(theta_{k-1})*deltaT 0]
                     [0 1 cos(theta_{k-1})*speed_{k-1}*deltaT sin(theta_{k-1})*deltaT 0]
                     [0 0 1 0 deltaT]
                     [0 0 0 1 0]
                     [0 0 0 0 1]]

观测方程h(x_k,0) = [[(x_k)**2+(y_k)**2],
                    [speed],
                    [omega]]

h的雅各比矩阵为：H = [[2*x, 2*y, 0, 0, 0],
                     [0, 0, 0, 1, 0],
                     [0, 0, 0, 0, 1]]
'''

N_steps = 1000  # 1000 steps to estimate
Delta_t = 0.1  # 0.1s
timeline = np.arange(N_steps) * Delta_t
omega = np.ones(N_steps) * 0.01  # 角速度，rad/s
speed_c = 0.1
speed = np.ones(N_steps) * speed_c  # 速度，m/s
zs_gt = np.stack([speed, omega], axis=0)
speed_noise_var = 0.1
omega_noise_var = 0.01
speed_noise = np.random.rand(N_steps) * speed_noise_var
omega_noise = np.random.randn(N_steps) * omega_noise_var
speed_w_noise = speed + speed_noise
omega_w_noise = omega + omega_noise
zs_w_noise = np.stack((speed_w_noise, omega_w_noise), axis=0)
x0 = [0, 0, 0, 0, 0]  # x,y,theta,speed,omega
xs = []
xt = x0
xt_noise = x0
xs_w_noise = []
# xs.append(x0)
for i in range(N_steps):
    # perfect data
    cur_x_0 = xt[0] + np.cos(xt[2])*speed[i]*Delta_t
    cur_x_1 = xt[1] + np.sin(xt[2])*speed[i]*Delta_t
    cur_x_2 = xt[2] + omega[i]*Delta_t
    print(omega[i])
    xt = [cur_x_0, cur_x_1, cur_x_2, speed[i], omega[i]]
    xs.append(xt)
    #noise data 
    cur_x_0 = xt_noise[0] + np.cos(xt_noise[2]) * speed_w_noise[i] * Delta_t
    cur_x_1 = xt_noise[1] + np.sin(xt_noise[2]) * speed_w_noise[i] * Delta_t
    cur_x_2 = xt_noise[2] + omega_w_noise[i] * Delta_t
    xt_noise = [cur_x_0, cur_x_1, cur_x_2, speed_w_noise[i], omega_w_noise[i]]
    xs_w_noise.append(xt_noise)
xs = np.array(xs).reshape(-1, 5)
xs_w_noise = np.array(xs_w_noise).reshape(-1, 5)

dist_to_origin_gt = xs[:,0]**2 + xs[:, 1]**2
dist_noise_var = 0.001
dist_noise = np.random.randn(N_steps) * dist_noise_var
dist_w_noise = dist_to_origin_gt + dist_noise
#---------------------------------

def state_forward(x_t):
    """状态方程，将上一时刻状态映射为当前时刻状态
    """
    a, b, theta, s, omega = x_t
    xt_next = np.array([
        a+np.cos(theta)*s*Delta_t,
        b+np.sin(theta)*s*Delta_t,
        (theta + omega*Delta_t),
        s,
        omega
    ]).reshape(-1, 1)
    return xt_next

def get_A(x_t):
    """对状态方程相对于状态向量求雅各比矩阵
    """
    a, b, theta, s, omega = x_t.reshape(-1)
    A = np.array([
        [1., 0, -np.sin(theta)*s*Delta_t, np.cos(theta)*Delta_t, 0],
        [0, 1, np.cos(theta)*s*Delta_t, np.sin(theta)*Delta_t, 0],
        [0, 0, 1, 0, Delta_t],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]
    ]).astype(float)
    # print(x_t[0], "  ", x_t[1], "  ", x_t[2], "  ", x_t[3], "  ", x_t[4])
    return A

def get_H(x_t):
    """对观测方程相对于状态向量求雅各比矩阵
    """
    a, b, theta, s, omega = x_t.reshape(-1)
    H = np.array([
        [2*a, 2*b, 0, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]
    ]).astype(float)
    return H

# ------------------------------------------------------------------------------------------
# 设置初始值
x0 = np.array([0., 0., 0, 0, 0]).reshape(-1, 1)  # 系统状态向量初始值，假设未知，随便设为0
# 系统状态初始协方差，由于x0是不准确的，因此P0不能为0
# P0如果设置为只有对角线有值，似乎对结果影响较大
P0 = np.array([  
    [10, 10, 1, 0, 0],
    [10, 10, 1, 0, 0],
    [1, 1, 0.1, 0, 0],
    [0, 0, 0, 1e-8, 0],
    [0, 0, 0, 0, 1e-8]
])

# 过程噪声的协方差，需要调整的参数
Q = np.array([
    [0.01, 0, 0, 0, 0],
    [0, 0.01, 0, 0, 0],
    [0, 0, 0.0001, 0, 0],
    [0, 0, 0, 0.01, 0],
    [0, 0, 0, 0, 0.01]
])

# 观测噪声协方差矩阵
R = np.array([
    [0.0001, 0, 0],
    [0., 0.0001, 0],
    [0, 0, 0.0001]
]) # 观测噪声的协方差，需要调整的参数

x_t_ = None  # predicted system state vector
x_t = None  # corrected system state vector
P_t_ = None  # covariance matrix of predicted state vector
P_t = None  # covariance matrix of corrected state vector
K = None

kf_result = []
x_t = x0
P_t = P0
# 以下是卡尔曼滤波的过程
for i in range(N_steps):

    x_t_ = state_forward(x_t)  # 预测方程
    A = get_A(x_t)
    P_t_ = A@P_t@A.T + Q  # 预测状态向量的协方差矩阵
    
    zt = np.array([dist_w_noise[i], zs_w_noise[0][i], zs_w_noise[1][i]]).reshape(-1, 1)  # 当前时刻的观测值
    zth = np.array([x_t_[0]**2+x_t_[1]**2, x_t_[3], x_t_[4]]).reshape(-1, 1)  # 当前时刻测量值的预测值

    H = get_H(x_t_)
    S_t = H@P_t_@H.T+R
    S_t_ = np.linalg.inv(S_t)
    K = P_t_@H.T@S_t_ # 卡尔曼增益
    x_t = x_t_ + K@(zt-zth)  # 更新方程
    P_t = P_t_ - K@H@P_t_  # 更新状态向量协方差矩阵
    kf_result.append(x_t)

kf_result = np.concatenate(kf_result, axis=-1).T  # N x 5
# print(kf_result[:,1])

# # ------------------------------------------------------
# # error analysis
# error_integral = xs_w_noise - xs
# error_integral_var = np.linalg.norm(error_integral, axis=0)
# error_kf = kf_result - xs
# error_kf_var = np.linalg.norm(error_kf, axis=0)
# print(f"Error covariance for kf is {error_kf_var}")
# print(f"Error covariance for simple integral is {error_integral_var}")
# # vis result 
# VIS_DATA = True
# if VIS_DATA:
#     fig = plt.figure()
#     fig.subplots_adjust(hspace=0.4,wspace=0.5)

#     ax = fig.add_subplot(231)
#     ax.plot(xs[:, 0], xs[:, 1])
#     ax.set_title("Trajectory")

#     ax2 = fig.add_subplot(234)
#     ax2.plot(timeline, xs[:, 2], 'r')
#     ax2.plot(timeline, xs[:, 3], 'g')
#     ax2.plot(timeline, xs[:, 4], 'b')

#     ax3 = fig.add_subplot(232)
#     ax3.plot(kf_result[:,0], kf_result[:, 1])
#     ax3.set_title("Trajectory with KF")

#     ax4 = fig.add_subplot(235)
#     ax4.plot(timeline, kf_result[:, 2], 'r')
#     ax4.plot(timeline, kf_result[:, 3], 'g')
#     ax4.plot(timeline, kf_result[:, 4], 'b')
#     ax4.set_title("(KF) Red for theta; Green for speed; Blue for omega")

#     ax5 = fig.add_subplot(233)
#     ax5.plot(xs_w_noise[:,0], xs_w_noise[:, 1])
#     ax5.set_title("Trajectory with noise")

#     ax6 = fig.add_subplot(236)
#     ax6.plot(timeline, xs_w_noise[:, 2], 'r')
#     ax6.plot(timeline, xs_w_noise[:, 3], 'g')
#     ax6.plot(timeline, xs_w_noise[:, 4], 'b')

#     fig2 = plt.figure()
#     ax = fig2.add_subplot(111)
#     ax.plot(xs[:, 0], xs[:, 1], 'r')
#     ax.plot(kf_result[:, 0], kf_result[:, 1], 'g')
#     ax.plot(xs_w_noise[:, 0], xs_w_noise[:, 1], 'b')
#     ax.set_title("Traj, red for gt; green for kf; blue for noise")
#     plt.show()
