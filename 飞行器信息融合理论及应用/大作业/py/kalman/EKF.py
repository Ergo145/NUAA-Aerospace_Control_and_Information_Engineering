import matplotlib
matplotlib.use('TkAgg')  # 避免 PyCharm 后端冲突，允许正常显示或保存图像

import numpy as np
import math
import matplotlib.pyplot as plt
plt.rcParams["font.sans-serif"] = ["SimHei"]   # 黑体支持全符号
plt.rcParams["axes.unicode_minus"] = False

# ==============================
#  EKF 参数设置
# ==============================
Q = np.diag([0.1, 0.1, np.deg2rad(1.0), 1.0])**2
R = np.diag([1.0, np.deg2rad(40.0)])**2

# 仿真参数
Qsim = np.diag([0.5, 0.5])**2
Rsim = np.diag([1.0, np.deg2rad(30.0)])**2

DT = 0.1  # 时间步长 [s]
SIM_TIME = 50.0  # 仿真总时长 [s]

show_animation = True


def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.matrix([v, yawrate]).T
    return u


def observation(xTrue, xd, u):
    xTrue = motion_model(xTrue, u)

    # 添加 GPS 噪声
    zx = xTrue[0, 0] + np.random.randn() * Qsim[0, 0]
    zy = xTrue[1, 0] + np.random.randn() * Qsim[1, 1]
    z = np.matrix([zx, zy])

    # 添加输入噪声
    ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
    ud = np.matrix([ud1, ud2]).T

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.matrix([[1.0, 0, 0, 0],
                   [0, 1.0, 0, 0],
                   [0, 0, 1.0, 0],
                   [0, 0, 0, 1.0]])

    B = np.matrix([[DT * math.cos(x[2, 0]), 0],
                   [DT * math.sin(x[2, 0]), 0],
                   [0.0, DT],
                   [0.0, 0.0]])

    x = F * x + B * u
    return x


def observation_model(x):
    H = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    z = H * x
    return z


def jacobF(x, u):
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.matrix([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    return jF


def jacobH(x):
    jH = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    return jH


def ekf_estimation(xEst, PEst, z, u):
    # 预测
    xPred = motion_model(xEst, u)
    jF = jacobF(xPred, u)
    PPred = jF * PEst * jF.T + Q

    # 更新
    jH = jacobH(xPred)
    zPred = observation_model(xPred)
    y = z.T - zPred
    S = jH * PPred * jH.T + R
    K = PPred * jH.T * np.linalg.inv(S)
    xEst = xPred + K * y
    PEst = (np.eye(len(xEst)) - K * jH) * PPred

    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)
    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    #n_sigma = 2.4477  # 约对应 95% 置信区间
    #a = n_sigma * math.sqrt(eigval[bigind])
    #b = n_sigma * math.sqrt(eigval[smallind])
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.matrix([[math.cos(angle), math.sin(angle)],
                   [-math.sin(angle), math.cos(angle)]])
    fx = R * np.matrix([x, y])
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r", linewidth=1)


def main():
    print("EKF Simulation start!!")

    time = 0.0

    # 状态初始化 [x, y, yaw, v]'
    xEst = np.matrix(np.zeros((4, 1)))
    xTrue = np.matrix(np.zeros((4, 1)))
    PEst = np.eye(4)

    xDR = np.matrix(np.zeros((4, 1)))  # 航位推算初值

    # 历史记录
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((1, 2))

    while SIM_TIME >= time:
        time += DT
        u = calc_input()
        xTrue, z, xDR, ud = observation(xTrue, xDR, u)
        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)

        # 保存历史
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.vstack((hz, z))

        if show_animation:
            plt.cla()
            plt.plot(hz[:, 0], hz[:, 1], ".g", label="GPS 观测")
            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b", label="真实轨迹")
            plt.plot(np.array(hxDR[0, :]).flatten(),
                     np.array(hxDR[1, :]).flatten(), "-k", label="航位推算轨迹")
            plt.plot(np.array(hxEst[0, :]).flatten(),
                     np.array(hxEst[1, :]).flatten(), "-r", label="EKF 估计轨迹")
            plot_covariance_ellipse(xEst, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.legend(loc="upper left")
            plt.title("扩展卡尔曼滤波（EKF）定位仿真")
            plt.pause(0.001)

    # ==============================
    # 仿真结束后保存与显示最终结果
    # ==============================
    plt.figure()
    plt.plot(hz[:, 0], hz[:, 1], ".g", label="GPS 观测")
    plt.plot(np.array(hxTrue[0, :]).flatten(),
             np.array(hxTrue[1, :]).flatten(), "-b", label="真实轨迹")
    plt.plot(np.array(hxDR[0, :]).flatten(),
             np.array(hxDR[1, :]).flatten(), "-k", label="航位推算轨迹")
    plt.plot(np.array(hxEst[0, :]).flatten(),
             np.array(hxEst[1, :]).flatten(), "-r", label="EKF 估计轨迹")
    plot_covariance_ellipse(xEst, PEst)
    plt.axis("equal")
    plt.grid(True)
    plt.legend(loc="upper left")
    plt.title("扩展卡尔曼滤波（EKF）定位结果")
    plt.savefig("ekf_result.png", dpi=300)

    # ==============================
    # 新增：EKF与航位推算误差对比分析
    # ==============================

    # 时间序列
    t = np.arange(0, hxTrue.shape[1]) * DT

    # 提取数据
    x_true = np.array(hxTrue[0, :]).flatten()
    y_true = np.array(hxTrue[1, :]).flatten()

    x_est = np.array(hxEst[0, :]).flatten()
    y_est = np.array(hxEst[1, :]).flatten()

    x_dr = np.array(hxDR[0, :]).flatten()
    y_dr = np.array(hxDR[1, :]).flatten()

    # -------- 1. EKF位置误差 --------
    ekf_pos_err = np.sqrt((x_est - x_true) ** 2 + (y_est - y_true) ** 2)

    plt.figure()
    plt.plot(t, ekf_pos_err, '-b', linewidth=1.5, label='EKF位置误差')
    plt.xlabel('时间 [s]')
    plt.ylabel('误差 [m]')
    plt.title('EKF 预估轨迹与真实轨迹位置误差')
    plt.grid(True)
    plt.legend()
    plt.savefig("ekf_position_error.png", dpi=300)

    # -------- 2. EKF 与 航位推算误差对比 --------
    dr_pos_err = np.sqrt((x_dr - x_true) ** 2 + (y_dr - y_true) ** 2)

    plt.figure()
    plt.plot(t, ekf_pos_err, '-r', linewidth=1.5, label='EKF 位置误差')
    plt.plot(t, dr_pos_err, '--k', linewidth=1.5, label='航位推算位置误差')
    plt.xlabel('时间 [s]')
    plt.ylabel('误差 [m]')
    plt.title('EKF 与航位推算轨迹误差对比')
    plt.legend()
    plt.grid(True)
    plt.savefig("ekf_dr_error_compare.png", dpi=300)

    plt.show()

    plt.show(block=False)   # 非阻塞显示
    plt.pause(5)            # 显示 5 秒
    plt.close('all')         # 关闭所有图窗，防止阻塞
    print("✅ 仿真结束，结果已保存为 ekf_result.png")


if __name__ == '__main__':
    main()
