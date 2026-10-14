#!/usr/bin/env python3
"""Generate the fixed prototype solver and its shared numeric configuration."""
import argparse
import json
from pathlib import Path

import casadi as ca
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver


def generate(config_path, output):
    cfg = json.loads(Path(config_path).read_text())
    if any(not np.isfinite(v) for v in cfg.values()):
        raise ValueError('Configuration must be finite')
    if cfg['horizon'] < 2 or any(cfg[k] <= 0 for k in (
            'dt', 'wheelbase', 'front_extent', 'rear_extent', 'half_width',
            'speed_cap', 'speed_rate', 'steering_rate')):
        raise ValueError('Invalid positive configuration value')
    if cfg['clearance'] < 0:
        raise ValueError('Invalid clearance')
    if 'steering_min' in cfg:
        low, high = cfg['steering_min'], cfg['steering_max']
    else:
        if not cfg['servo_gain'] < 0:
            raise ValueError('Invalid servo conversion')
        low = (cfg['servo_max'] - cfg['servo_offset']) / cfg['servo_gain']
        high = (cfg['servo_min'] - cfg['servo_offset']) / cfg['servo_gain']
    if not -1.5 < low < 0 < high < 1.5:
        raise ValueError('Steering interval must straddle zero')
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    constants = {**cfg, 'steering_min': low, 'steering_max': high}
    profile = Path(config_path).stem
    lines = ['#pragma once', 'namespace mpc {',
             f'inline constexpr const char * model_profile = {json.dumps(profile)};']
    for name, value in constants.items():
        kind = 'int' if name == 'horizon' else 'double'
        lines.append(f'inline constexpr {kind} {name} = {value};')
    (output / 'mpc_config.hpp').write_text('\n'.join(lines + ['}']) + '\n')

    ocp = AcadosOcp()
    ocp.name = 'mpc_proto'
    ocp.model.name = 'mpc_proto'
    x, u, p = ca.SX.sym('x', 5), ca.SX.sym('u', 2), ca.SX.sym('p', 17)
    ocp.model.x, ocp.model.u, ocp.model.p = x, u, p

    def derivative(z):
        return ca.vertcat(u[0] * ca.cos(z[2]), u[0] * ca.sin(z[2]),
                          u[0] * ca.tan(u[1]) / cfg['wheelbase'])

    dt, z = cfg['dt'], x[:3]
    k1 = derivative(z)
    k2 = derivative(z + dt * k1 / 2)
    k3 = derivative(z + dt * k2 / 2)
    k4 = derivative(z + dt * k3)
    ocp.model.disc_dyn_expr = ca.vertcat(z + dt * (k1 + 2*k2 + 2*k3 + k4)/6, u)
    change = (u - x[3:5]) / p[4]
    offset = (cfg['front_extent'] - cfg['rear_extent']) / 2
    center = x[:2] + offset * ca.vertcat(ca.cos(x[2]), ca.sin(x[2]))
    faces = ca.vertcat(*[p[5+3*i]*center[0] + p[6+3*i]*center[1] - p[7+3*i]
                          for i in range(4)])
    ocp.model.con_h_expr = ca.vertcat(change, faces)
    ocp.model.con_h_expr_0 = ocp.model.con_h_expr
    ocp.model.con_h_expr_e = faces
    ocp.model.cost_y_expr = ca.vertcat(x[:3] - p[:3], u[0] - p[3], change, u[1])
    ocp.model.cost_y_expr_e = x[:3] - p[:3]
    ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.W = np.diag([20., 20., 2., 3., .05, .03, .01])
    ocp.cost.W_e = np.diag([20., 20., 2.])
    ocp.cost.yref, ocp.cost.yref_e = np.zeros(7), np.zeros(3)
    ocp.constraints.x0 = np.zeros(5)
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([0., low])
    ocp.constraints.ubu = np.array([cfg['speed_cap'], high])
    ocp.constraints.lh = np.array([-cfg['speed_rate'], -cfg['steering_rate']] + [-1e4]*4)
    ocp.constraints.uh = np.array([cfg['speed_rate'], cfg['steering_rate']] + [0.]*4)
    ocp.constraints.lh_0 = ocp.constraints.lh.copy()
    ocp.constraints.uh_0 = ocp.constraints.uh.copy()
    ocp.constraints.lh_e, ocp.constraints.uh_e = np.full(4, -1e4), np.zeros(4)
    ocp.parameter_values = np.array([0., 0., 0., 0., dt] + [0., 0., 100.]*4)
    ocp.solver_options.N_horizon = int(cfg['horizon'])
    ocp.solver_options.tf = dt * cfg['horizon']
    ocp.solver_options.integrator_type = 'DISCRETE'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver_cond_N = 10
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.print_level = 0
    ocp.code_gen_options.code_export_directory = str(output)
    ocp.code_gen_options.json_file = 'mpc_proto.json'
    AcadosOcpSolver.generate(ocp)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    generate(args.config, args.output)
