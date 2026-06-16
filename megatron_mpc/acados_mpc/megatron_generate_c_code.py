from acados_template import AcadosOcp, AcadosOcpSolver
from megatron_model import export_robot_model
import numpy as np
import time
import os
from casadi import diagcat, vertcat


Tf = 1  # Define the prediction horizon
N = 20  # Define the number of discretization steps

# create ocp object to formulate the OCP
ocp = AcadosOcp()

model = export_robot_model()
ocp.model = model

# set dimensions
nx = model.x.rows()
nu = model.u.rows()
ny_0 = nu
ny = nx + nu

# set cost
Q = np.diag([400, 400, 13000, 1, 0.1])  # [x, y, theta]
R = np.diag([10, 500])  # [v_cmd , w_cmd]

Q_e = 50 * Q  # terminal cost

# path cost
ocp.cost.cost_type = 'NONLINEAR_LS'
ocp.model.cost_y_expr = vertcat(model.x, model.u)
ocp.cost.yref = np.zeros((nx+nu,))
ocp.cost.W = diagcat(Q, R).full()

# terminal cost
ocp.cost.cost_type_e = 'NONLINEAR_LS'
ocp.cost.yref_e = np.zeros((nx,))
ocp.model.cost_y_expr_e = model.x
ocp.cost.W_e = Q_e

ocp.constraints.lbx_0 = np.zeros(nx)
ocp.constraints.ubx_0 = np.zeros(nx)
ocp.constraints.idxbx_0 = np.array([0, 1, 2, 3, 4])

# State constraints
# lower bounds, the difference in control and state bounds are due to the gain difference
ocp.constraints.lbx = np.array([-0.8, -3])
ocp.constraints.ubx = np.array([0.8, 3])  # higher bounds
# Index of states in the state vector matrix
ocp.constraints.idxbx = np.array([3, 4])  # [v, w]


# Control input constraints
ocp.constraints.lbu = np.array([-0.8, -5])
ocp.constraints.ubu = np.array([0.8, 5])
# Index of states in the control vector matrix
ocp.constraints.idxbu = np.array([0, 1])  # [v_cmd, w_cmd]

# set prediction horizon
ocp.solver_options.tf = Tf
ocp.solver_options.N_horizon = N

# set options
ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # "FULL_CONDENSING_QPOASES"
ocp.solver_options.nlp_solver_type = "SQP"
ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
ocp.solver_options.integrator_type = "ERK"
ocp.solver_options.globalization = 'MERIT_BACKTRACKING' 
ocp.solver_options.tol = 1e-6       # general solver tolerance
ocp.solver_options.qp_tol = 1e-6    # QP solver tolerance
ocp.solver_options.max_iter = 100


acados_ocp_solver = AcadosOcpSolver(ocp)
