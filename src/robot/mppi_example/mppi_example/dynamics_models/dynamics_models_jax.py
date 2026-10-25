import jax.numpy as jnp
import jax
try:
    from . import tire_models as tireModel
    from .mb_model_params import params_real, params_f1tenth
except ImportError:
    import dynamics_models.tire_models as tireModel
    from dynamics_models.mb_model_params import params_real, params_f1tenth
params = params_f1tenth
import numpy as np
from functools import partial


def accl_constraints(vel, accl, v_switch, a_max, v_min, v_max):
    """
    Acceleration constraints, adjusts the acceleration based on constraints

        Args:
            vel (float): current velocity of the vehicle
            accl (float): unconstraint desired acceleration
            v_switch (float): switching velocity (velocity at which the acceleration is no longer able to create wheel spin)
            a_max (float): maximum allowed acceleration
            v_min (float): minimum allowed velocity
            v_max (float): maximum allowed velocity

        Returns:
            accl (float): adjusted acceleration
    """

    # positive accl limit
    pos_limit = jax.lax.select(vel > v_switch, a_max*v_switch/vel, a_max)

    # accl limit reached?
    accl = jax.lax.select((vel <= v_min) & (accl <= 0), 0., accl)
    accl = jax.lax.select((vel >= v_max) & (accl >= 0), 0., accl)
    
    accl = jax.lax.select(accl <= -a_max, -a_max, accl)
    accl = jax.lax.select(accl >= pos_limit, pos_limit, accl)

    return accl

def steering_constraint(steering_angle, steering_velocity, s_min, s_max, sv_min, sv_max):
    """
    Steering constraints, adjusts the steering velocity based on constraints

        Args:
            steering_angle (float): current steering_angle of the vehicle
            steering_velocity (float): unconstraint desired steering_velocity
            s_min (float): minimum steering angle
            s_max (float): maximum steering angle
            sv_min (float): minimum steering velocity
            sv_max (float): maximum steering velocity

        Returns:
            steering_velocity (float): adjusted steering velocity
    """

    # constraint steering velocity
    steering_velocity = jax.lax.select((steering_angle <= s_min) & (steering_velocity <= 0), 0., steering_velocity)
    steering_velocity = jax.lax.select((steering_angle >= s_max) & (steering_velocity >= 0), 0., steering_velocity)
    steering_velocity = jax.lax.select(steering_velocity <= sv_min, sv_min, steering_velocity)
    steering_velocity = jax.lax.select(steering_velocity >= sv_max, sv_max, steering_velocity)
    
    return steering_velocity

@jax.jit
def point_mass_dynamics_jax(x, u, a_max=1.):
    """
    Uses a point-mass model to update the state of the vehicle
    Dynamics reference: https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/vehicleModels_commonRoad.pdf?ref_type=heads
    x: current state of the system [x, y, vx, vy]
    u: current input to the system [ax, ay]
    """
    # accl_constraints
    a_max = params['a_max']
    [u_0, u_1] = u
    # u_0 = jax.lax.select(u[0] > a_max, a_max, u[0])
    # u_0 = jax.lax.select(u[0] < -a_max, -a_max, u[0])
    # u_1 = jax.lax.select(u[1] > a_max, a_max, u[1])
    # u_1 = jax.lax.select(u[1] < -a_max, -a_max, u[1])
    
    A = jnp.array([[0, 0, 1, 0], 
                   [0, 0, 0, 1], 
                   [0, 0, 0, 0], 
                   [0, 0, 0, 0]])
    B = jnp.array([[0, 0],
                   [0, 0],
                   [1, 0],
                   [0, 1]])
    u = jnp.array([u_0, u_1])
    f = A @ x + B @ u
    
    return f

@jax.jit
def vehicle_dynamics_ks(x, u_init, C_Sf=20.898, C_Sr=20.898, 
                        lf=0.88392, lr=1.50876, h=0.59436, m=1225.887, I=1538.853371):
    """
    Single Track Kinematic Vehicle Dynamics.

        Args:
            x (numpy.ndarray (3, )): vehicle state vector (x1, x2, x3, x4, x5)
                x1: x position in global coordinates
                x2: y position in global coordinates
                x3: steering angle of front wheels
                x4: velocity in x direction
                x5: yaw angle
            u (numpy.ndarray (2, )): control input vector (u1, u2)
                u1: steering angle velocity of front wheels
                u2: longitudinal acceleration

        Returns:
            f (numpy.ndarray): right hand side of differential equations
    """
    # wheelbase
    lf = params['lf']  # distance from spring mass center of gravity to front axle [m]  LENA
    lr = params['lr']  # distance from spring mass center of gravity to rear axle [m]  LENB
    lwb = lf + lr
    # steering constraints
    s_min = params['s_min']  # minimum steering angle [rad]
    s_max = params['s_max']  # maximum steering angle [rad]
    # longitudinal constraints
    v_min = params['v_min']  # minimum velocity [m/s]
    v_max = params['v_max'] # minimum velocity [m/s]
    sv_min = params['sv_min'] # minimum steering velocity [rad/s]
    sv_max = params['sv_max'] # maximum steering velocity [rad/s]
    v_switch = params['v_switch']  # switching velocity [m/s]
    a_max = params['a_max'] # maximum absolute acceleration [m/s^2]
    
    
    # constraints
    u = jnp.array([steering_constraint(x[2], u_init[0], s_min, s_max, sv_min, sv_max), accl_constraints(x[3], u_init[1], v_switch, a_max, v_min, v_max)])

    # system dynamics
    f = jnp.array([x[3]*jnp.cos(x[4]),
         x[3]*jnp.sin(x[4]), 
         u[0],
         u[1],
         x[3]/lwb*jnp.tan(x[2])])
    return f

@jax.jit
def vehicle_dynamics_st_pacjeka_frenet(x, u_init, curvature):
    """
    Single Track Dynamic Vehicle Dynamics.

        Args:
            x (numpy.ndarray (7, )): vehicle state vector (x1, x2, x3, x4, x5, x6, x7)
                x1: curvilinear s coordinate
                x2: lateral error ey
                x3: Steering angle (delta)
                x4: velocity in x direction
                x5: yaw error (epsi)
                x6: yaw rate (wz)
                x7: velocity in y direction

            u (numpy.ndarray (2, )): control input vector (u1, u2)
                u1: steering angle velocity of front wheels
                u2: longitudinal acceleration

        Returns:
            f (numpy.ndarray): right hand side of differential equations
    """
    # gravity constant m/s^2
    # g = 9.81
    
    # steering constraints
    mu = params['mu']  # friction coefficient
    lf = params['lf']  # distance from spring mass center of gravity to front axle [m]  LENA
    lr = params['lr']  # distance from spring mass center of gravity to rear axle [m]  LENB
    h = params['h']  # M_s center of gravity above ground [m]  HS
    m = params['m']  # vehicle mass [kg]  MASS
    I = params['I']  # moment of inertia for sprung mass in yaw [kg m^2]  IZZ
    C_Sf = params['C_Sf']  # front tire cornering stiffness [N/rad]  CF
    C_Sr = params['C_Sr']  # rear tire cornering stiffness [N/rad]  CR
    s_min = params['s_min']  # minimum steering angle [rad]
    s_max = params['s_max']  # maximum steering angle [rad]

    # longitudinal constraints
    v_min = params['v_min']  # minimum velocity [m/s]
    v_max = params['v_max'] # minimum velocity [m/s]
    sv_min = params['sv_min'] # minimum steering velocity [rad/s]
    sv_max = params['sv_max'] # maximum steering velocity [rad/s]
    v_switch = params['v_switch']  # switching velocity [m/s]
    a_max = params['a_max'] # maximum absolute acceleration [m/s^2]

    # constraints
    # u = u_init
    # steer_v = jax.lax.select(u_init[0] > sv_max, sv_max, u_init[0])
    # steer_v = jax.lax.select(steer_v < sv_min, sv_min, steer_v)

    # acc = jax.lax.select(u_init[1] > a_max, a_max, u_init[1])
    # acc = jax.lax.select(acc < -a_max, -a_max, acc)

    u = jnp.array([steering_constraint(x[2], u_init[0], s_min, s_max, sv_min, sv_max), accl_constraints(x[3], u_init[1], v_switch, a_max, v_min, v_max)])
    # u = jnp.array([steer_v, acc])
    
    delta_v = u[0]
    a = u[1]
    
    C_Sf, C_Sr = 1.7, 1.7
    # C_Sf, C_Sr = 1.3507, 1.3507
    Fzf = (m * 9.81) * (lr / (lf + lr))
    Fzr = (m * 9.81) * (lf / (lf + lr)) 
    # ky1 = 21.92 # Lateral slip stiffness Kfy/Fz at Fznom normal car
    ky1 = 35. # Lateral slip stiffness Kfy/Fz at Fznom sports car
    Df = mu * Fzf
    Kf = Fzf * ky1
    Bf = Kf / (C_Sf * Df)
    Dr = mu * Fzr
    Kr = Fzr * ky1
    Br = Kr / (C_Sr * Dr)
    
    s     = x[0]
    ey    = x[1]
    delta = x[2]
    vx    = x[3]
    epsi  = x[4]
    wz    = x[5]
    vy    = x[6]

    # Compute tire split angle
    alpha_f = delta - jnp.arctan2( vy + lf * wz, vx )
    alpha_r = - jnp.arctan2( vy - lf * wz , vx)

    # Compute lateral force at front and rear tire
    Fyf = Df * jnp.sin( C_Sf * jnp.arctan(Bf * alpha_f ) )
    Fyr = Dr * jnp.sin( C_Sr * jnp.arctan(Br * alpha_r ) )

    # system dynamics
    f = jnp.asarray([ ( (vx * jnp.cos(epsi) - vy * jnp.sin(epsi)) / (1 - curvature * ey) ), # s
                   ( vx * jnp.sin(epsi) + vy * jnp.cos(epsi) ), # ey
                   delta_v, # delta
                   (a - 1 / m * Fyf * jnp.sin(delta) + wz * vy), # vx
                   ( wz - (vx * jnp.cos(epsi) - vy * jnp.sin(epsi)) / (1 - curvature * ey) * curvature ), # epsi
                   (1 / I *(lf * Fyf * jnp.cos(delta) - lr * Fyr) ), # wz
                   (1 / m * (Fyf * jnp.cos(delta) + Fyr) - wz * vx), # vy
                   ]) 
    return f

@jax.jit
def vehicle_dynamics_ks_frenet(x, u_init, curvature, lf=0.88392, lr=1.50876):
    """
    Uses the kinematic bicycle model in frenet frame to update the state of the vehicle
    Dynamics reference: https://arxiv.org/pdf/2005.07691.pdf
    x: current state of the vehicle [s, ey, delta, v, epsi]
    u: delta: steering speed, a: acceleration, 
    """
    # input check
    # if delta >= self.config.MAX_STEER:
    #     delta = self.config.MAX_STEER
    # elif delta <= self.config.MIN_STEER:
    #     delta = self.config.MIN_STEER
    lf = params['lf']  # distance from spring mass center of gravity to front axle [m]  LENA
    lr = params['lr']  # distance from spring mass center of gravity to rear axle [m]  LENB
    h = params['h']  # M_s center of gravity above ground [m]  HS
    m = params['m']  # vehicle mass [kg]  MASS
    I = params['I']  # moment of inertia for sprung mass in yaw [kg m^2]  IZZ
    C_Sf = params['C_Sf']  # front tire cornering stiffness [N/rad]  CF
    C_Sr = params['C_Sr']  # rear tire cornering stiffness [N/rad]  CR
    s_min = params['s_min']  # minimum steering angle [rad]
    s_max = params['s_max']  # maximum steering angle [rad]
    # longitudinal constraints
    v_min = params['v_min']  # minimum velocity [m/s]
    v_max = params['v_max'] # minimum velocity [m/s]
    sv_min = params['sv_min'] # minimum steering velocity [rad/s]
    sv_max = params['sv_max'] # maximum steering velocity [rad/s]
    v_switch = params['v_switch']  # switching velocity [m/s]
    a_max = params['a_max'] # maximum absolute acceleration [m/s^2]

    # constraints
    u = jnp.array([steering_constraint(x[2], u_init[0], s_min, s_max, sv_min, sv_max), accl_constraints(x[3], u_init[1], v_switch, a_max, v_min, v_max)])
    
    # curvature = self.centerline.spline.calc_curvature(x[0])
    f = jnp.array([ ((x[3] * jnp.cos(x[4])) / (1 - x[1] * curvature)),
        (x[3] * jnp.sin(x[4])),
        u[0],
        u[1],
        (x[3] * jnp.tan(x[2]) / (lf + lr) - curvature * ((x[3] * jnp.cos(x[4]))/(1 - curvature * x[1])))])
    
    return f


@jax.jit
def vehicle_dynamics_st(x, u_init, mu=1.0, C_Sf=20.898, C_Sr=20.898, 
                        lf=0.88392, lr=1.50876, h=0.59436, m=1225.887, I=1538.853371):
    """
    Single Track Dynamic Vehicle Dynamics.

        Args:
            x (numpy.ndarray (3, )): vehicle state vector (x1, x2, x3, x4, x5, x6, x7)
                x1: x position in global coordinates
                x2: y position in global coordinates
                x3: steering angle of front wheels
                x4: velocity in x direction
                x5: yaw angle
                x6: yaw rate
                x7: slip angle at vehicle center
            u (numpy.ndarray (2, )): control input vector (u1, u2)
                u1: steering angle velocity of front wheels
                u2: longitudinal acceleration

        Returns:
            f (numpy.ndarray): right hand side of differential equations
    """
    # gravity constant m/s^2
    g = 9.81
    
    
    # steering constraints
    lf = params['lf']  # distance from spring mass center of gravity to front axle [m]  LENA
    lr = params['lr']  # distance from spring mass center of gravity to rear axle [m]  LENB
    h = params['h']  # M_s center of gravity above ground [m]  HS
    m = params['m']  # vehicle mass [kg]  MASS
    I = params['I']  # moment of inertia for sprung mass in yaw [kg m^2]  IZZ
    C_Sf = params['C_Sf']  # front tire cornering stiffness [N/rad]  CF
    C_Sr = params['C_Sr']  # rear tire cornering stiffness [N/rad]  CR
    s_min = params['s_min']  # minimum steering angle [rad]
    s_max = params['s_max']  # maximum steering angle [rad]
    # longitudinal constraints
    v_min = params['v_min']  # minimum velocity [m/s]
    v_max = params['v_max'] # minimum velocity [m/s]
    sv_min = params['sv_min'] # minimum steering velocity [rad/s]
    sv_max = params['sv_max'] # maximum steering velocity [rad/s]
    v_switch = params['v_switch']  # switching velocity [m/s]
    a_max = params['a_max'] # maximum absolute acceleration [m/s^2]

    # constraints
    u = jnp.array([steering_constraint(x[2], u_init[0], s_min, s_max, sv_min, sv_max), accl_constraints(x[3], u_init[1], v_switch, a_max, v_min, v_max)])
    # print('u model', u)
    
    # u = u_init
    # steer = jax.lax.select(u_init[0] > 0.4, 0.4, u_init[0])
    # steer = jax.lax.select(u_init[0] < -0.4, -0.4, u_init[0])
    # u = jnp.array([steer, u_init[1]])
    
    # switch to kinematic model for small velocities
    # if abs(x[3]) < 0.5:
    #     # wheelbase
    lwb = lf + lr

    # u = jax.lax.select(jnp.abs(x[3]) < 1, jnp.array([0., 1.]), u)
    #     # system dynamics
    x_ks = x[0:5]
    f_ks = vehicle_dynamics_ks(x_ks, u)
    f_ks = jnp.hstack((f_ks, jnp.array([u[1]/lwb*jnp.tan(x[2])+x[3]/(lwb*jnp.cos(x[2])**2)*u[0],0])))

    # else:
    # system dynamics
    f = jnp.array([x[3]*jnp.cos(x[6] + x[4]),
        x[3]*jnp.sin(x[6] + x[4]),
        u[0],
        u[1],
        x[5],
        -mu*m/(x[3]*I*(lr+lf))*(lf**2*C_Sf*(g*lr-u[1]*h) + lr**2*C_Sr*(g*lf + u[1]*h))*x[5] \
            +mu*m/(I*(lr+lf))*(lr*C_Sr*(g*lf + u[1]*h) - lf*C_Sf*(g*lr - u[1]*h))*x[6] \
            +mu*m/(I*(lr+lf))*lf*C_Sf*(g*lr - u[1]*h)*x[2],
        (mu/(x[3]**2*(lr+lf))*(C_Sr*(g*lf + u[1]*h)*lr - C_Sf*(g*lr - u[1]*h)*lf)-1)*x[5] \
            -mu/(x[3]*(lr+lf))*(C_Sr*(g*lf + u[1]*h) + C_Sf*(g*lr-u[1]*h))*x[6] \
            +mu/(x[3]*(lr+lf))*(C_Sf*(g*lr-u[1]*h))*x[2]])
    # return f
    return jax.lax.select(jnp.abs(x[3]) < 1, f_ks, f)


def init_mb(init_state, params=params):
    params = jnp.array(list(params.values()))
    # init_MB - generates the initial state vector for the multi-body model
    #
    # Syntax:
    #     x0 = init_MB(init_state, p)
    #
    # Inputs:
    #     init_state - core initial states
    #     p - parameter vector
    #
    # Outputs:
    #     x0 - initial state vector
    #
    # Example:
    #
    # See also: ---

    # Author:       Matthias Althoff
    # Written:      11-January-2017
    # Last update:  ---
    # Last revision:---

    # Parameters
    # steering constraints
    s_min = params[2]  # minimum steering angle [rad]
    s_max = params[3]  # maximum steering angle [rad]
    # longitudinal constraints
    v_min = params[6]  # minimum velocity [m/s]
    v_max = params[7] # minimum velocity [m/s]
    # masses
    m_s = params[11] # sprung mass [kg]  SMASS
    m_uf = params[12] # unsprung mass front [kg]  UMASSF
    m_ur = params[13] # unsprung mass rear [kg]  UMASSR
    # axes distances
    lf = params[14] # distance from spring mass center of gravity to front axle [m]  LENA
    lr = params[15]  # distance from spring mass center of gravity to rear axle [m]  LENB
    # geometric parameters
    K_zt = params[30] # vertical spring rate of tire [N/m]  TSPRINGR
    R_w = params[39]  # effective wheel/tire radius  chosen as tire rolling radius RR  taken from ADAMS documentation [m]
    # create equivalent bicycle parameters
    g = 9.81  # [m/s^2]

    # obtain initial states from vector
    sx0 = init_state[0] # x-position in a global coordinate system
    sy0 = init_state[1] # y-position in a global coordinate system
    delta0 = init_state[2] # steering angle of front wheels
    vel0 = init_state[3] # speed of the car
    Psi0 = init_state[4] # yaw angle
    dotPsi0 = init_state[5] # yaw rate
    beta0 = init_state[6] # slip angle

    if delta0 > s_max:
        delta0 = s_max

    if delta0 < s_min:
        delta0 = s_min

    if vel0 > v_max:
        vel0 = v_max

    if vel0 < v_min:
        vel0 = v_min

    # auxiliary initial states
    F0_z_f = m_s * g * lr / ((lf + lr)) + m_uf * g
    F0_z_r = m_s * g * lf / ((lf + lr)) + m_ur * g

    # sprung mass states
    x0 = np.zeros((29,))  # init initial state vector
    x0[0] = sx0  # x-position in a global coordinate system
    x0[1] = sy0  # y-position in a global coordinate system
    x0[2] = delta0  # steering angle of front wheels
    x0[3] = jnp.cos(beta0) * vel0  # velocity in x-direction
    x0[4] = Psi0  # yaw angle
    x0[5] = dotPsi0  # yaw rate
    x0[6] = 0  # roll angle
    x0[7] = 0  # roll rate
    x0[8] = 0  # pitch angle
    x0[9] = 0  # pitch rate
    x0[10] = jnp.sin(beta0) * vel0  # velocity in y-direction
    x0[11] = 0  # z-position (zero height corresponds to steady state solution)
    x0[12] = 0  # velocity in z-direction

    # unsprung mass states (front)
    x0[13] = 0  # roll angle front
    x0[14] = 0  # roll rate front
    x0[15] = jnp.sin(beta0) * vel0 + lf * dotPsi0  # velocity in y-direction front
    x0[16] = (F0_z_f) / (2 * K_zt)  # z-position front
    x0[17] = 0  # velocity in z-direction front

    # unsprung mass states (rear)
    x0[18] = 0  # roll angle rear
    x0[19] = 0  # roll rate rear
    x0[20] = jnp.sin(beta0) * vel0 - lr * dotPsi0  # velocity in y-direction rear
    x0[21] = (F0_z_r) / (2 * K_zt)  # z-position rear
    x0[22] = 0  # velocity in z-direction rear

    # wheel states
    x0[23] = x0[3] / (R_w)  # left front wheel angular speed
    x0[24] = x0[3] / (R_w)  # right front wheel angular speed
    x0[25] = x0[3] / (R_w)  # left rear wheel angular speed
    x0[26] = x0[3] / (R_w)  # right rear wheel angular speed

    x0[27] = 0  # delta_y_f
    x0[28] = 0  # delta_y_r

    return jnp.array(x0)


def reset_mb(st_state, params=params):
    params = jnp.array(list(params.values()))
    # Parameters
    # steering constraints
    s_min = params[2]  # minimum steering angle [rad]
    s_max = params[3]  # maximum steering angle [rad]
    # longitudinal constraints
    v_min = params[6]  # minimum velocity [m/s]
    v_max = params[7] # minimum velocity [m/s]
    # masses
    m_s = params[11] # sprung mass [kg]  SMASS
    m_uf = params[12] # unsprung mass front [kg]  UMASSF
    m_ur = params[13] # unsprung mass rear [kg]  UMASSR
    # axes distances
    lf = params[14] # distance from spring mass center of gravity to front axle [m]  LENA
    lr = params[15]  # distance from spring mass center of gravity to rear axle [m]  LENB
    # geometric parameters
    K_zt = params[30] # vertical spring rate of tire [N/m]  TSPRINGR
    R_w = params[39]  # effective wheel/tire radius  chosen as tire rolling radius RR  taken from ADAMS documentation [m]
    # create equivalent bicycle parameters
    g = 9.81  # [m/s^2]

    # obtain initial states from vector
    sx0 = st_state[0] # x-position in a global coordinate system
    sy0 = st_state[1] # y-position in a global coordinate system
    delta0 = st_state[2] # steering angle of front wheels
    vel0 = st_state[3] # speed of the car
    Psi0 = st_state[4] # yaw angle
    dotPsi0 = st_state[5] # yaw rate
    beta0 = st_state[6] # slip angle

    # if delta0 > s_max:
    #     delta0 = s_max
    
    # if delta0 < s_min:
    #     delta0 = s_min

    # if vel0 > v_max:
    #     vel0 = v_max

    # if vel0 < v_min:
    #     vel0 = v_min
    delta0 = jax.lax.select(delta0 > s_max, s_max, delta0)
    delta0 = jax.lax.select(delta0 < s_min, s_min, delta0)
    vel0 = jax.lax.select(vel0 > v_max, v_max, vel0)
    vel0 = jax.lax.select(vel0 < v_min, v_min, vel0)

    

    # auxiliary initial states
    F0_z_f = m_s * g * lr / ((lf + lr)) + m_uf * g
    F0_z_r = m_s * g * lf / ((lf + lr)) + m_ur * g

    # sprung mass states
    # x0 = np.zeros((29,))  # init initial state vector
    x0 = []
    x0.append(sx0)  # x-position in a global coordinate system
    x0.append(sy0)  # y-position in a global coordinate system
    x0.append(delta0)  # steering angle of front wheels
    x0.append(jnp.cos(beta0) * vel0)  # velocity in x-direction
    x0.append(Psi0)  # yaw angle
    x0.append(dotPsi0)  # yaw rate
    x0.append(0)  # roll angle
    x0.append(0)  # roll rate
    x0.append(0)  # pitch angle
    x0.append(0)  # pitch rate
    x0.append(jnp.sin(beta0) * vel0)  # velocity in y-direction
    x0.append(0)  # z-position (zero height corresponds to steady state solution)
    x0.append(0)  # velocity in z-direction

    # unsprung mass states (front)
    x0.append(0)  # roll angle front
    x0.append(0)  # roll rate front
    x0.append(jnp.sin(beta0) * vel0 + lf * dotPsi0)  # velocity in y-direction front
    x0.append((F0_z_f) / (2 * K_zt))  # z-position front
    x0.append(0)  # velocity in z-direction front

    # unsprung mass states (rear)
    x0.append(0)  # roll angle rear
    x0.append(0)  # roll rate rear
    x0.append(jnp.sin(beta0) * vel0 - lr * dotPsi0)  # velocity in y-direction rear
    x0.append((F0_z_r) / (2 * K_zt))  # z-position rear
    x0.append(0)  # velocity in z-direction rear

    # wheel states
    x0.append(x0[3] / (R_w))  # left front wheel angular speed
    x0.append(x0[3] / (R_w))  # right front wheel angular speed
    x0.append(x0[3] / (R_w))  # left rear wheel angular speed
    x0.append(x0[3] / (R_w))  # right rear wheel angular speed

    x0.append(0)  # delta_y_f
    x0.append(0)  # delta_y_r

    return jnp.array(x0)

@partial(jax.jit, static_argnums=(2,3))
def vehicle_dynamics_mb(x, u_init, params=params, use_kinematic=False):
    params = jnp.array(list(params.values()))
    # params_array = np.array(list(self.params.values()))
    """
    vehicleDynamics_mb - multi-body vehicle dynamics based on the DOT (department of transportation) vehicle dynamics
    reference point: center of mass

    Syntax:
        f = vehicleDynamics_mb(x,u,p)

    Inputs:
        :param x: vehicle state vector
        :param uInit: vehicle input vector
        :param params: vehicle parameter vector

    Outputs:
        :return f: right-hand side of differential equations

    Author: Matthias Althoff
    Written: 05-January-2017
    Last update: 17-December-2017
    Last revision: ---
    """

    #------------- BEGIN CODE --------------

    # set gravity constant
    g = 9.81  #[m/s^2]

    #states
    #x1 = x-position in a global coordinate system
    #x2 = y-position in a global coordinate system
    #x3 = steering angle of front wheels
    #x4 = velocity in x-direction
    #x5 = yaw angle
    #x6 = yaw rate

    #x7 = roll angle
    #x8 = roll rate
    #x9 = pitch angle
    #x10 = pitch rate
    #x11 = velocity in y-direction
    #x12 = z-position
    #x13 = velocity in z-direction

    #x14 = roll angle front
    #x15 = roll rate front
    #x16 = velocity in y-direction front
    #x17 = z-position front
    #x18 = velocity in z-direction front

    #x19 = roll angle rear
    #x20 = roll rate rear
    #x21 = velocity in y-direction rear
    #x22 = z-position rear
    #x23 = velocity in z-direction rear

    #x24 = left front wheel angular speed
    #x25 = right front wheel angular speed
    #x26 = left rear wheel angular speed
    #x27 = right rear wheel angular speed

    #x28 = delta_y_f
    #x29 = delta_y_r

    #u1 = steering angle velocity of front wheels
    #u2 = acceleration

    

    # vehicle body dimensions
    length =  params[0]  # vehicle length [m]
    width = params[1]  # vehicle width [m]

    # steering constraints
    s_min = params[2]  # minimum steering angle [rad]
    s_max = params[3]  # maximum steering angle [rad]
    sv_min = params[4] # minimum steering velocity [rad/s]
    sv_max = params[5] # maximum steering velocity [rad/s]

    # longitudinal constraints
    v_min = params[6]  # minimum velocity [m/s]
    v_max = params[7] # minimum velocity [m/s]
    v_switch = params[8]  # switching velocity [m/s]
    a_max = params[9] # maximum absolute acceleration [m/s^2]

    # masses
    m = params[10] # vehicle mass [kg]  MASS
    m_s = params[11] # sprung mass [kg]  SMASS
    m_uf = params[12] # unsprung mass front [kg]  UMASSF
    m_ur = params[13] # unsprung mass rear [kg]  UMASSR

    # axes distances
    lf = params[14] # distance from spring mass center of gravity to front axle [m]  LENA
    lr = params[15]  # distance from spring mass center of gravity to rear axle [m]  LENB

    # moments of inertia of sprung mass
    I_Phi_s = params[16]  # moment of inertia for sprung mass in roll [kg m^2]  IXS
    I_y_s = params[17]  # moment of inertia for sprung mass in pitch [kg m^2]  IYS
    I_z = params[18]  # moment of inertia for sprung mass in yaw [kg m^2]  IZZ
    I_xz_s = params[19]  # moment of inertia cross product [kg m^2]  IXZ

    # suspension parameters
    K_sf = params[20]  # suspension spring rate (front) [N/m]  KSF
    K_sdf = params[21]  # suspension damping rate (front) [N s/m]  KSDF
    K_sr = params[22]  # suspension spring rate (rear) [N/m]  KSR
    K_sdr = params[23]  # suspension damping rate (rear) [N s/m]  KSDR

    # geometric parameters
    T_f = params[24]   # track width front [m]  TRWF
    T_r = params[25]   # track width rear [m]  TRWB
    K_ras = params[26] # lateral spring rate at compliant compliant pin joint between M_s and M_u [N/m]  KRAS

    K_tsf = params[27]   # auxiliary torsion roll stiffness per axle (normally negative) (front) [N m/rad]  KTSF
    K_tsr = params[28] # auxiliary torsion roll stiffness per axle (normally negative) (rear) [N m/rad]  KTSR
    K_rad = params[29] # damping rate at compliant compliant pin joint between M_s and M_u [N s/m]  KRADP
    K_zt = params[30] # vertical spring rate of tire [N/m]  TSPRINGR

    h_cg = params[31]   # center of gravity height of total mass [m]  HCG (mainly required for conversion to other vehicle models)
    h_raf = params[32] # height of roll axis above ground (front) [m]  HRAF
    h_rar = params[33] # height of roll axis above ground (rear) [m]  HRAR

    h_s = params[34]   # M_s center of gravity above ground [m]  HS

    I_uf = params[35] # moment of inertia for unsprung mass about x-axis (front) [kg m^2]  IXUF
    I_ur = params[36] # moment of inertia for unsprung mass about x-axis (rear) [kg m^2]  IXUR
    I_y_w = params[37]  # wheel inertia, from internet forum for 235/65 R 17 [kg m^2]

    K_lt = params[38]  # lateral compliance rate of tire, wheel, and suspension, per tire [m/N]  KLT
    R_w = params[39]  # effective wheel/tire radius  chosen as tire rolling radius RR  taken from ADAMS documentation [m]

    # split of brake and engine torque
    T_sb = params[40]
    T_se = params[41]

    # suspension parameters
    D_f = params[42]  # [rad/m]  DF
    D_r = params[43]  # [rad/m]  DR
    E_f = params[44]  # [needs conversion if nonzero]  EF
    E_r = params[45]  # [needs conversion if nonzero]  ER
    
    # if u_init[1] > 0:
    #     T_B = 0.0
    #     T_E = m * R_w * u_init[1]
    # else:
    #     T_B = m * R_w * u_init[1]
    #     T_E = 0.0
        
    T_B = jax.lax.select(u_init[1] > 0, 0.0, m * R_w * u_init[1])
    T_E = jax.lax.select(u_init[1] > 0, m * R_w * u_init[1], 0.0)

    front_tire_force = 0.5 * T_sb * T_B + 0.5 * T_se * T_E
    rear_tire_force = 0.5 * (1 - T_sb) * T_B + 0.5 * (1 - T_se) * T_E

    u = jnp.array([u_init[0], front_tire_force, rear_tire_force])    
    # u = u_init


    # consider steering and acceleration constraints - outside of the integration
    # u = jnp.array([steering_constraint(x[2], u_init[0], s_min, s_max, sv_min, sv_max), accl_constraints(x[3], u_init[1], v_switch, a_max, v_min, v_max)])

    #compute slip angle at cg - outside of the integration
    # switch to kinematic model for small velocities handeled outside
    if use_kinematic:
        beta = 0.
    else:
        beta = jnp.arctan(x[10]/x[3])
        vel = jnp.sqrt(x[3]**2 + x[10]**2)

    #vertical tire forces
    F_z_LF = (x[16] + R_w*(jnp.cos(x[13]) - 1) - 0.5*T_f*jnp.sin(x[13]))*K_zt
    F_z_RF = (x[16] + R_w*(jnp.cos(x[13]) - 1) + 0.5*T_f*jnp.sin(x[13]))*K_zt
    F_z_LR = (x[21] + R_w*(jnp.cos(x[18]) - 1) - 0.5*T_r*jnp.sin(x[18]))*K_zt
    F_z_RR = (x[21] + R_w*(jnp.cos(x[18]) - 1) + 0.5*T_r*jnp.sin(x[18]))*K_zt

    #obtain individual tire speeds
    u_w_lf = (x[3] + 0.5*T_f*x[5])*jnp.cos(x[2]) + (x[10] + lf*x[5])*jnp.sin(x[2])
    u_w_rf = (x[3] - 0.5*T_f*x[5])*jnp.cos(x[2]) + (x[10] + lf*x[5])*jnp.sin(x[2])
    u_w_lr = x[3] + 0.5*T_r*x[5]
    u_w_rr = x[3] - 0.5*T_r*x[5]

    #negative wheel spin forbidden
    u_w_lf = jnp.max(jnp.array([u_w_lf, 0.0]))
    u_w_rf = jnp.max(jnp.array([u_w_rf, 0.0]))
    u_w_lr = jnp.max(jnp.array([u_w_lr, 0.0]))
    u_w_rr = jnp.max(jnp.array([u_w_rr, 0.0]))
    # if u_w_lf < 0.0:
    #     u_w_lf *= 0
    #
    # if u_w_rf < 0.0:
    #     u_w_rf *= 0
    #
    # if u_w_lr < 0.0:
    #     u_w_lr *= 0
    #
    # if u_w_rr < 0.0:
    #     u_w_rr *= 0

    #compute longitudinal slip
    #switch to kinematic model for small velocities
    if use_kinematic:
        s_lf = 0.
        s_rf = 0.
        s_lr = 0.
        s_rr = 0.
    else:
        s_lf = 1 - R_w*x[23]/u_w_lf
        s_rf = 1 - R_w*x[24]/u_w_rf
        s_lr = 1 - R_w*x[25]/u_w_lr
        s_rr = 1 - R_w*x[26]/u_w_rr

    #lateral slip angles
    #switch to kinematic model for small velocities
    if use_kinematic:
        alpha_LF = 0.
        alpha_RF = 0.
        alpha_LR = 0.
        alpha_RR = 0.
    else:
        alpha_LF = jnp.arctan((x[10] + lf*x[5] - x[14]*(R_w - x[16]))/(x[3] + 0.5*T_f*x[5])) - x[2]
        alpha_RF = jnp.arctan((x[10] + lf*x[5] - x[14]*(R_w - x[16]))/(x[3] - 0.5*T_f*x[5])) - x[2]
        alpha_LR = jnp.arctan((x[10] - lr*x[5] - x[19]*(R_w - x[21]))/(x[3] + 0.5*T_r*x[5]))
        alpha_RR = jnp.arctan((x[10] - lr*x[5] - x[19]*(R_w - x[21]))/(x[3] - 0.5*T_r*x[5]))

    #auxiliary suspension movement
    z_SLF = (h_s - R_w + x[16] - x[11])/jnp.cos(x[6]) - h_s + R_w + lf*x[8] + 0.5*(x[6] - x[13])*T_f
    z_SRF = (h_s - R_w + x[16] - x[11])/jnp.cos(x[6]) - h_s + R_w + lf*x[8] - 0.5*(x[6] - x[13])*T_f
    z_SLR = (h_s - R_w + x[21] - x[11])/jnp.cos(x[6]) - h_s + R_w - lr*x[8] + 0.5*(x[6] - x[18])*T_r
    z_SRR = (h_s - R_w + x[21] - x[11])/jnp.cos(x[6]) - h_s + R_w - lr*x[8] - 0.5*(x[6] - x[18])*T_r

    dz_SLF = x[17] - x[12] + lf*x[9] + 0.5*(x[7] - x[14])*T_f
    dz_SRF = x[17] - x[12] + lf*x[9] - 0.5*(x[7] - x[14])*T_f
    dz_SLR = x[22] - x[12] - lr*x[9] + 0.5*(x[7] - x[19])*T_r
    dz_SRR = x[22] - x[12] - lr*x[9] - 0.5*(x[7] - x[19])*T_r

    #camber angles
    gamma_LF = x[6] + D_f*z_SLF + E_f*(z_SLF)**2
    gamma_RF = x[6] - D_f*z_SRF - E_f*(z_SRF)**2
    gamma_LR = x[6] + D_r*z_SLR + E_r*(z_SLR)**2
    gamma_RR = x[6] - D_r*z_SRR - E_r*(z_SRR)**2

    #compute longitudinal tire forces using the magic formula for pure slip
    F0_x_LF = tireModel.formula_longitudinal(s_lf, gamma_LF, F_z_LF, params)
    F0_x_RF = tireModel.formula_longitudinal(s_rf, gamma_RF, F_z_RF, params)
    F0_x_LR = tireModel.formula_longitudinal(s_lr, gamma_LR, F_z_LR, params)
    F0_x_RR = tireModel.formula_longitudinal(s_rr, gamma_RR, F_z_RR, params)

    #compute lateral tire forces using the magic formula for pure slip
    res = tireModel.formula_lateral(alpha_LF, gamma_LF, F_z_LF, params)
    F0_y_LF = res[0]
    mu_y_LF = res[1]
    res = tireModel.formula_lateral(alpha_RF, gamma_RF, F_z_RF, params)
    F0_y_RF = res[0]
    mu_y_RF = res[1]
    res = tireModel.formula_lateral(alpha_LR, gamma_LR, F_z_LR, params)
    F0_y_LR = res[0]
    mu_y_LR = res[1]
    res = tireModel.formula_lateral(alpha_RR, gamma_RR, F_z_RR, params)
    F0_y_RR = res[0]
    mu_y_RR = res[1]

    #compute longitudinal tire forces using the magic formula for combined slip
    F_x_LF = tireModel.formula_longitudinal_comb(s_lf, alpha_LF, F0_x_LF, params)
    F_x_RF = tireModel.formula_longitudinal_comb(s_rf, alpha_RF, F0_x_RF, params)
    F_x_LR = tireModel.formula_longitudinal_comb(s_lr, alpha_LR, F0_x_LR, params)
    F_x_RR = tireModel.formula_longitudinal_comb(s_rr, alpha_RR, F0_x_RR, params)

    #compute lateral tire forces using the magic formula for combined slip
    F_y_LF = tireModel.formula_lateral_comb(s_lf, alpha_LF, gamma_LF, mu_y_LF, F_z_LF, F0_y_LF, params)
    F_y_RF = tireModel.formula_lateral_comb(s_rf, alpha_RF, gamma_RF, mu_y_RF, F_z_RF, F0_y_RF, params)
    F_y_LR = tireModel.formula_lateral_comb(s_lr, alpha_LR, gamma_LR, mu_y_LR, F_z_LR, F0_y_LR, params)
    F_y_RR = tireModel.formula_lateral_comb(s_rr, alpha_RR, gamma_RR, mu_y_RR, F_z_RR, F0_y_RR, params)

    #auxiliary movements for compliant joint equations
    delta_z_f = h_s - R_w + x[16] - x[11]
    delta_z_r = h_s - R_w + x[21] - x[11]

    delta_phi_f = x[6] - x[13]
    delta_phi_r = x[6] - x[18]

    dot_delta_phi_f = x[7] - x[14]
    dot_delta_phi_r = x[7] - x[19]

    dot_delta_z_f = x[17] - x[12]
    dot_delta_z_r = x[22] - x[12]

    dot_delta_y_f = x[10] + lf*x[5] - x[15]
    dot_delta_y_r = x[10] - lr*x[5] - x[20]

    delta_f = delta_z_f*jnp.sin(x[6]) - x[27]*jnp.cos(x[6]) - (h_raf - R_w)*jnp.sin(delta_phi_f)
    delta_r = delta_z_r*jnp.sin(x[6]) - x[28]*jnp.cos(x[6]) - (h_rar - R_w)*jnp.sin(delta_phi_r)

    dot_delta_f = (delta_z_f*jnp.cos(x[6]) + x[27]*jnp.sin(x[6]))*x[7] + dot_delta_z_f*jnp.sin(x[6]) - dot_delta_y_f*jnp.cos(x[6]) - (h_raf - R_w)*jnp.cos(delta_phi_f)*dot_delta_phi_f
    dot_delta_r = (delta_z_r*jnp.cos(x[6]) + x[28]*jnp.sin(x[6]))*x[7] + dot_delta_z_r*jnp.sin(x[6]) - dot_delta_y_r*jnp.cos(x[6]) - (h_rar - R_w)*jnp.cos(delta_phi_r)*dot_delta_phi_r

    #compliant joint forces
    F_RAF = delta_f*K_ras + dot_delta_f*K_rad
    F_RAR = delta_r*K_ras + dot_delta_r*K_rad

    #auxiliary suspension forces (bump stop neglected  squat/lift forces neglected)
    F_SLF = m_s*g*lr/(2*(lf+lr)) - z_SLF*K_sf - dz_SLF*K_sdf + (x[6] - x[13])*K_tsf/T_f

    F_SRF = m_s*g*lr/(2*(lf+lr)) - z_SRF*K_sf - dz_SRF*K_sdf - (x[6] - x[13])*K_tsf/T_f

    F_SLR = m_s*g*lf/(2*(lf+lr)) - z_SLR*K_sr - dz_SLR*K_sdr + (x[6] - x[18])*K_tsr/T_r

    F_SRR = m_s*g*lf/(2*(lf+lr)) - z_SRR*K_sr - dz_SRR*K_sdr - (x[6] - x[18])*K_tsr/T_r


    #auxiliary variables sprung mass
    sumX = F_x_LR + F_x_RR + (F_x_LF + F_x_RF)*jnp.cos(x[2]) - (F_y_LF + F_y_RF)*jnp.sin(x[2])

    sumN = (F_y_LF + F_y_RF)*lf*jnp.cos(x[2]) + (F_x_LF + F_x_RF)*lf*jnp.sin(x[2]) \
           + (F_y_RF - F_y_LF)*0.5*T_f*jnp.sin(x[2]) + (F_x_LF - F_x_RF)*0.5*T_f*jnp.cos(x[2]) \
           + (F_x_LR - F_x_RR)*0.5*T_r - (F_y_LR + F_y_RR)*lr

    sumY_s = (F_RAF + F_RAR)*jnp.cos(x[6]) + (F_SLF + F_SLR + F_SRF + F_SRR)*jnp.sin(x[6])

    sumL = 0.5*F_SLF*T_f + 0.5*F_SLR*T_r - 0.5*F_SRF*T_f - 0.5*F_SRR*T_r \
           - F_RAF/jnp.cos(x[6])*(h_s - x[11] - R_w + x[16] - (h_raf - R_w)*jnp.cos(x[13])) \
           - F_RAR/jnp.cos(x[6])*(h_s - x[11] - R_w + x[21] - (h_rar - R_w)*jnp.cos(x[18]))

    sumZ_s = (F_SLF + F_SLR + F_SRF + F_SRR)*jnp.cos(x[6]) - (F_RAF + F_RAR)*jnp.sin(x[6])

    sumM_s = lf*(F_SLF + F_SRF) - lr*(F_SLR + F_SRR) + ((F_x_LF + F_x_RF)*jnp.cos(x[2]) \
                                                          - (F_y_LF + F_y_RF)*jnp.sin(x[2]) + F_x_LR + F_x_RR)*(h_s - x[11])

    #auxiliary variables unsprung mass
    sumL_uf = 0.5*F_SRF*T_f - 0.5*F_SLF*T_f - F_RAF*(h_raf - R_w) \
              + F_z_LF*(R_w*jnp.sin(x[13]) + 0.5*T_f*jnp.cos(x[13]) - K_lt*F_y_LF) \
              - F_z_RF*(-R_w*jnp.sin(x[13]) + 0.5*T_f*jnp.cos(x[13]) + K_lt*F_y_RF) \
              - ((F_y_LF + F_y_RF)*jnp.cos(x[2]) + (F_x_LF + F_x_RF)*jnp.sin(x[2]))*(R_w - x[16])

    sumL_ur = 0.5*F_SRR*T_r - 0.5*F_SLR*T_r - F_RAR*(h_rar - R_w) \
              + F_z_LR*(R_w*jnp.sin(x[18]) + 0.5*T_r*jnp.cos(x[18]) - K_lt*F_y_LR) \
              - F_z_RR*(-R_w*jnp.sin(x[18]) + 0.5*T_r*jnp.cos(x[18]) + K_lt*F_y_RR) \
              - (F_y_LR + F_y_RR)*(R_w - x[21])

    sumZ_uf = F_z_LF + F_z_RF + F_RAF*jnp.sin(x[6]) - (F_SLF + F_SRF)*jnp.cos(x[6])

    sumZ_ur = F_z_LR + F_z_RR + F_RAR*jnp.sin(x[6]) - (F_SLR + F_SRR)*jnp.cos(x[6])

    sumY_uf = (F_y_LF + F_y_RF)*jnp.cos(x[2]) + (F_x_LF + F_x_RF)*jnp.sin(x[2]) \
              - F_RAF*jnp.cos(x[6]) - (F_SLF + F_SRF)*jnp.sin(x[6])

    sumY_ur = (F_y_LR + F_y_RR) \
              - F_RAR*jnp.cos(x[6]) - (F_SLR + F_SRR)*jnp.sin(x[6])


    # dynamics common with single-track model
    # f = np.zeros((29,))# init 'right hand side
    f = []
    # switch to kinematic model for small velocities
    if use_kinematic:
        # wheelbase
        # lwb = lf + lr

        # system dynamics
        # x_ks = [x[0],  x[1],  x[2],  x[3],  x[4]]
        # f_ks = vehicle_dynamics_ks(x_ks, u, p)
        # f.extend(f_ks)
        # f.append(u[1]*lwb*jnp.tan(x[2]) + x[3]/(lwb*jnp.cos(x[2])**2)*u[0])

        # Use kinematic model with reference point at center of mass
        # wheelbase
        lwb = lf + lr

        # system dynamics
        x_ks = x[0:5]
        # kinematic model
        # f_ks = vehicle_dynamics_ks_cog(x_ks, u, p)
        # f = [f_ks[0], f_ks[1], f_ks[2], f_ks[3], f_ks[4]] # 1 2 3 4 5
        f_ks = vehicle_dynamics_ks(x_ks, u, 0, 0, 0, lf, lr, 0, 0, 0, s_min, s_max, sv_min, sv_max, v_switch, a_max, v_min, v_max)
        f[0:5] = f_ks
        # derivative of slip angle and yaw rate
        d_beta = (lr * u[0]) / (lwb * jnp.cos(x[2]) ** 2 * (1 + (jnp.tan(x[2]) ** 2 * lr / lwb) ** 2))
        dd_psi = 1 / lwb * (u[1] * jnp.cos(x[6]) * jnp.tan(x[2]) -
                            x[3] * jnp.sin(x[6]) * d_beta * jnp.tan(x[2]) +
                            x[3] * jnp.cos(x[6]) * u[0] / jnp.cos(x[2]) ** 2)
        # f.append(dd_psi) # 6
        f[5] = dd_psi

    else:
        # f[0] = jnp.cos(beta + x[4])*vel # 1
        # f[1] = jnp.sin(beta + x[4])*vel # 2
        # f[2] = u[0] # 3
        # f[3] = 1/m*sumX + x[5]*x[10] # 4
        # f[4] = x[5] # 5
        # f[5] = 1/(I_z - (I_xz_s)**2/I_Phi_s)*(sumN + I_xz_s/I_Phi_s*sumL) # 6
        
        f.append(jnp.cos(beta + x[4])*vel) # 1
        f.append(jnp.sin(beta + x[4])*vel) # 2
        f.append(u[0]) # 3
        f.append(1/m*sumX + x[5]*x[10]) # 4
        f.append(x[5]) # 5
        f.append(1/(I_z - (I_xz_s)**2/I_Phi_s)*(sumN + I_xz_s/I_Phi_s*sumL)) # 6



    # remaining sprung mass dynamics
    f.append(x[7]) # 7
    f.append(1/(I_Phi_s - (I_xz_s)**2/I_z)*(I_xz_s/I_z*sumN + sumL)) # 8
    f.append(x[9]) # 9
    f.append(1/I_y_s*sumM_s) # 10
    f.append(1/m_s*sumY_s - x[5]*x[3]) # 11
    f.append(x[12]) # 12
    f.append(g - 1/m_s*sumZ_s) # 13

    #unsprung mass dynamics (front)
    f.append(x[14]) # 14
    f.append(1/I_uf*sumL_uf) # 15
    f.append(1/m_uf*sumY_uf - x[5]*x[3]) # 16
    f.append(x[17]) # 17
    f.append(g - 1/m_uf*sumZ_uf) # 18

    #unsprung mass dynamics (rear)
    f.append(x[19]) # 19
    f.append(1/I_ur*sumL_ur) # 20
    f.append(1/m_ur*sumY_ur - x[5]*x[3]) # 21
    f.append(x[22]) # 22
    f.append(g - 1/m_ur*sumZ_ur) # 23

    #convert acceleration input to brake and engine torque - splitting for brake/drive torque is outside of the integration
    # if u[1]>0:
    # T_B = 0.0
    # T_E = m*R_w*u[1]
    # else:
    #     T_B = m*R_w*u[1]
    #     T_E = 0.0



    #wheel dynamics (p.T  new parameter for torque splitting) - splitting for brake/drive torque is outside of the integration
    f.append(1/I_y_w*(-R_w*F_x_LF + u[1])) # 24
    f.append(1/I_y_w*(-R_w*F_x_RF + u[1])) # 25
    f.append(1/I_y_w*(-R_w*F_x_LR + u[2])) # 26
    f.append(1/I_y_w*(-R_w*F_x_RR + u[2])) # 27

    #negative wheel spin forbidden handeled outside of this function
    # for iState in range(23, 27):
    #     # if x[iState] < 0.0:
    #     x[iState] = 0.0
    #     f[iState] = 0.0

    #compliant joint equations
    f.append(dot_delta_y_f) # 28
    f.append(dot_delta_y_r) # 29

    # longitudinal slip
    # s_lf
    # s_rf
    # s_lr
    # s_rr

    # lateral slip
    # alpha_LF
    # alpha_RF
    # alpha_LR
    # alpha_RR
    f = jnp.array(f)
    # print(f.shape)
    return f

    # return f, F_x_LF, F_x_RF, F_x_LR, F_x_RR, F_y_LF, F_y_RF, F_y_LR, F_y_RR, \
    #     s_lf, s_rf, s_lr, s_rr, alpha_LF, alpha_RF, alpha_LR, alpha_RR, \
    #     F_z_LF, F_z_RF, F_z_LR, F_z_RR
