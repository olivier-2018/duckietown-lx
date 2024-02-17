from typing import Tuple

import numpy as np
import yaml
import os

def PIDController(v_0: float, y_ref: float, y_hat: float, prev_e_y: float, prev_int_y: float, delta_t: float
) -> Tuple[float, float, float, float]:
    """
    PID performing lateral control.

    Args:
        v_0:        linear Duckiebot speed (constant).
        y_ref:      target y coordinate.
        y_hat:      the current estimated y.
        prev_e_y:   tracking error at previous iteration.
        prev_int_y: previous integral error term.
        delta_t:    time interval since last call.

    Returns:
        v_0:        linear velocity of the Duckiebot
        omega:      angular velocity of the Duckiebot
        e:          current tracking error (automatically becomes prev_e_y at next iteration).
        e_int:      current integral error (automatically becomes prev_int_y at next iteration).
    """

    # Read PID gains from file
    script_dir = os.path.dirname(__file__)
    file_path = script_dir + "/GAINS.yaml"

    with open(file_path) as f:
        gains = yaml.full_load(f)
        f.close()
    
    kp = gains['kp']
    kd = gains['kd']
    ki = gains['ki']
    
    # Overwritten to keep track of submissions
    #kp, kd, ki = 5.0, 15.0, 0.1  # submission 26153 rank #43
    #kp, kd, ki = 5.0, 15.0, 0.4  # submission 26154 rank #60
    #kp, kd, ki = 5.0, 15.0, 0.4  # submission 26155 rank #89
    #kp, kd, ki = 6.0, 15.0, 0.1  # submission 26157 rank #36
    #kp, kd, ki = 5.0, 10.0, 0.0  # submission 26158 rank #43
    kp, kd, ki = 3.0, 10.0, 0.0  # submission 26159 rank #28 BEST
    #kp, kd, ki = 3.0, 15.0, 0.0  # submission 26160 rank #35
    #kp, kd, ki = 3.0,  6.0, 0.0  # submission 26162 rank #55
    #kp, kd, ki = 3.0,  8.0, 0.0  # submission 26163 rank #46
    #kp, kd, ki = 3.0, 10.0, 0.5  # submission 26164 rank #68
    #kp, kd, ki = 2.0, 10.0, 0.0  # submission 26165 rank #  crashed
    #kp, kd, ki = 2.3, 10.0, 0.05  # submission 26169 rank #84  
    #kp, kd, ki = 3.0, 10.0, 0.0  # submission 26170-1 rank #86,42  
    #kp, kd, ki = 3.0, 12.0, 0.0  # submission 26172 rank #  
    
    # ------------- DEFINE YOUR PID FUNCTION BELOW ---------

    # Slip & inacuracies compensation (unrecoverable drift from dynamic model)
    #y_hat -= 0.02  # only submission 26169 - 26171
    #y_hat -= 0.04  # only submission  26172

    # Tracking error
    e = y_ref - y_hat

    # integral of the error
    e_int = prev_int_y + e * delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int,2),-2)

    # derivative of the error
    e_der = (e - prev_e_y) / delta_t

    omega = kp * e + kd * e_der + ki * e_int 

    #omega = max(min(omega,0.5),-0.5)
    #if abs(omega) > 0.5:
    #    v_0 /= 10
    #    omega /= 10
    #else:
    #    v_0 = 0.22

    #print(f"y_ref={y_ref} y_hat={y_hat:.4f} e={e:.4f} e_der={e_der:.4f} e_int={e_int:.4f} ki={ki:.4f} Omega={omega:.5f} v0={v_0}")

    return v_0, omega, e, e_int
