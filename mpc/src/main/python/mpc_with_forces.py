#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 28 15:02:49 2023

@author: maxcai
"""

import sys
#sys.platform = 'darwin'
#print(sys.platform)
import aerosandbox.numpy as np
import casadi
import numba

# bypass the fingerprinter entirely
def get_settings(settings):
	settings['usysid'] = [19941.0, 51041.0, 35087.0, 10749.0, 9372.0]  # 4de5-c761-890f-29fd-249c
	settings['mac_addr'] = 'ac-de-48-00-11-22'
	settings['userid'] = 'eb60cdec-7ac5-48a5-87d1-7cfdd72c32b5'
	settings['hostname'] = 'max-macbook-pro.lan'
	settings['otherauth'] = 'maxcai'

numba.njit = lambda f: f  # disable njit for forces
numba.njit.disabled = True

from drive_simulation import RobotState, OptimisationParameters, RobotCommand

setattr(np, 'float', float)

# run this command:
# export PYTHONPATH="/Users/maxcai/Codes/forces_pro_client":$PYTHONPATH
from forcespro.nlp.symbolicModel import SymbolicModel

import forcespro
from forcespro import CodeOptions

forcespro.lowlevel.get_machine_settings = get_settings

# time_lookahead = 0.5
# frequency = 10
N = 5 # int(time_lookahead * frequency)
STEP_SIZE = 0.05

nin = RobotCommand.num_parameters()
nstate = 6  # x, y, theta and their velocities
npar = OptimisationParameters.num_parameters()

if __name__ == '__main__':
	model = SymbolicModel(N)
	model.nvar = nin + nstate
	model.neq = nstate
	model.npar = npar

	ls_obj_func = lambda z, p: OptimisationParameters.from_array(p).least_squares_objective(RobotState.from_array(z))

	model.LSobjective = [ls_obj_func] * N  # Different objective function for each stage
	continuous_dynamics = lambda x, u, p: OptimisationParameters.from_array(p).parameters.continuous_dynamics(RobotState.from_array(casadi.vertcat(u, x)))
	constraint_func = lambda z, p: forcespro.nlp.integrate(continuous_dynamics, z[nin:nin + nstate], z[0:nin], p, integrator=forcespro.nlp.integrators.RK4, stepsize=STEP_SIZE)
	model.eq = [constraint_func] * (N - 1)  # Different constraint function for each stage
	model.E = np.hstack((np.zeros((nstate, nin)), np.eye(nstate)))

	model.lb = np.concatenate((np.ones(nin) * -1, np.ones(nstate) * -np.inf))
	model.ub = np.concatenate((np.ones(nin) * 1, np.ones(nstate) * np.inf))
	model.xinitidx = range(nin, nin + nstate)

	codeoptions = CodeOptions("mecanum_mpc")

	codeoptions.platform = 'AARCH-Cortex-A53'  # For the Control Hub's ARM processor

	codeoptions.solvemethod = 'PDIP_NLP'  # Nonlinear Primal-Dual Interior-Point method
	# codeoptions.server = 'https://forces-6-0-1.embotech.com'
	codeoptions.server = 'https://forces.embotech.com/'
	codeoptions.cleanup = 0

	codeoptions.printlevel = 0  # todo: should be 0 when deploying to hardware
	codeoptions.optlevel = 3  # todo: should be 3 when deploying to hardware

	codeoptions.overwrite = 1

# 	codeoptions.sse = -1
# 	codeoptions.avx = -1
#	sys.platform = 'darwin'

	# We're not hackers!
	old_settings = forcespro.lowlevel.get_machine_settings
	
	def new_settings(settings):
		old_settings(settings)
		print("\nsettings dict:")
		useful_settings = ['usysid', 'hostname', 'mac_addr', 'otherauth']
		for setting in useful_settings:
			print(f"settings[{setting}] = {settings[setting]}")
	
	forcespro.lowlevel.get_machine_settings = new_settings  # wrap function with print statements

	m = model.generate_solver(codeoptions)
