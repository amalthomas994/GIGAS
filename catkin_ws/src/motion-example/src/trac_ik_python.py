#!/usr/bin/env python

import trac_ik_python

ik_solver = trac_ik_python.trac_ik.IK("torso_lift_link",
               "r_wrist_roll_link")

seed_state = [0.0] * ik_solver.number_of_joints

ik_solver.get_ik(seed_state,
                0.45, 0.1, 0.3,  # X, Y, Z
                0.0, 0.0, 0.0, 1.0)  # QX, QY, QZ, QW
# Returns:
# (0.537242808640495,
#  0.04673341230604478,
#  -0.053508394352190486,
#  -1.5099959208163785,
#  2.6007509004432596,
#  -1.506431092603137,
#  -3.040949079090651)
