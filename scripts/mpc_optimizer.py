#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
import numpy as np
import cvxpy
import rospy
from mpc_track.srv import *
from geometry_msgs.msg import *
from mpc_track.msg import *
import math
import time

global prediction_horizon, freq, L, MAX_VEL_delta, MAX_DSTEER, MAX_ANGULAR_delta


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def state_space(control_ref, ref_yaw):
    v_d = control_ref[0]
    ref_delta = control_ref[1]
    A = np.matrix([
        [1.0, 0.0, -v_d * dt * math.sin(ref_yaw)],
        [0.0, 1.0, v_d * dt * math.cos(ref_yaw)],
        [0.0, 0.0, 1.0]])
    B = np.matrix([
        [dt * math.cos(ref_yaw), 0],
        [dt * math.sin(ref_yaw), 0],
        [0, dt]
    ])
    return A, B


def mpc_optimize_function(car_state, state_reference, control_reference):
    dsteer_limit = min(abs(math.atan(MAX_ANGULAR_delta*L/(control_reference[0, 0] + MAX_VEL_delta))), MAX_DSTEER)
    # print(dsteer_limit)
    x = cvxpy.Variable((3, prediction_horizon + 1))
    u = cvxpy.Variable((2, prediction_horizon))
    cost = 0  # 代价函数
    constraints = []  # 约束条件
    t = 0
    while t < prediction_horizon:
        A, B = state_space(control_reference[:, t], state_reference[2, t])
        # U成本函数
        cost += cvxpy.quad_form(u[:, t], R)
        # X成本函数
        cost += cvxpy.quad_form(x[:, t] - state_reference[:, t], Q)
        # 状态方程约束
        constraints += [x[:, t + 1] - state_reference[:, t] == A @ (x[:, t] - state_reference[:, t]) + B @ u[:, t]]
        t += 1

    # 状态量初值为当前小车位姿
    constraints += [(x[:, 0]) == car_state]
    constraints += [u[0, :] <= MAX_VEL_delta, -MAX_VEL_delta <= u[0, :]]
    constraints += [u[1, :] <= dsteer_limit, -dsteer_limit <= u[1, :]]
    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    s_l = time.time()
    prob.solve(verbose=False, warm_start=True, solver=cvxpy.SCS)
    solve_time = time.time() - s_l
    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        opt_x = get_nparray_from_matrix(x.value[0, :])
        opt_y = get_nparray_from_matrix(x.value[1, :])
        opt_yaw = get_nparray_from_matrix(x.value[2, :])
        opt_v_dd = get_nparray_from_matrix(u.value[0, :])
        opt_kesi_dd = get_nparray_from_matrix(u.value[1, :])

    else:
        rospy.logerr("Error: Cannot solve mpc..")
        rospy.signal_shutdown("Error: Cannot solve mpc..")
        opt_v_dd, opt_kesi_dd, opt_x, opt_y, opt_yaw = None, None, None, None, None,
    opt_v = opt_v_dd[0] + control_reference[0, 0]
    opt_kesi = opt_kesi_dd[0] + control_reference[1, 0]
    return opt_v, opt_kesi, opt_x, opt_y, opt_yaw, solve_time

def callback(req):
    # 数据预处理
    state_reference = np.zeros((3, prediction_horizon))
    control_reference = np.zeros((2, prediction_horizon))
    car_state = np.zeros(3)
    car_state[0] = req.state.x
    car_state[1] = req.state.y
    car_state[2] = req.state.theta
    i = 0
    while i < len(req.state_ref):
        state_reference[0, i] = req.state_ref[i].x
        state_reference[1, i] = req.state_ref[i].y
        state_reference[2, i] = req.state_ref[i].theta
        control_reference[0, i] = req.control_ref[i].v
        control_reference[1, i] = req.control_ref[i].kesi
        i += 1

    # 向优化函数传参解算
    v, kesi, opt_x, opt_y, opt_yaw, time = mpc_optimize_function(car_state, state_reference, control_reference)

    # 数据打包
    opt_control = control()
    opt_control.v = v
    opt_control.kesi = kesi
    solve_time = time
    state_predictive = []
    j = 0
    while j < prediction_horizon+1:
        state_pre = Pose2D()
        state_pre.x = opt_x[j]
        state_pre.y = opt_y[j]
        state_pre.theta = opt_yaw[j]
        state_predictive.append(state_pre)
        j += 1
    return mpcResponse(opt_control, state_predictive, solve_time)


if __name__ == "__main__":
    rospy.init_node("mpc_optimizer")
    server = rospy.Service("mpc_optimization", mpc, callback)
    prediction_horizon = rospy.get_param('~prediction_horizon')
    freq = rospy.get_param('~freq')
    L = rospy.get_param('~L')
    MAX_VEL_delta = rospy.get_param('~MAX_VEL_delta')
    MAX_ANGULAR_delta = rospy.get_param('~MAX_ANGULAR_delta')
    MAX_DSTEER = rospy.get_param('~MAX_DSTEER')
    MAX_DSTEER = np.deg2rad(MAX_DSTEER)
    R_set = rospy.get_param('~R_set')
    Q_set = rospy.get_param('~Q_set')
    dt = 1/freq
    Q = np.diag(Q_set)  # state cost matrix
    R = np.diag(R_set)  # input cost matrix
    rospy.loginfo("ready to optimize")
    rospy.spin()

