import numpy
import ctypes

name = "mecanum_mpc"
requires_callback = True
lib = "lib/libmecanum_mpc.so"
lib_static = "lib/libmecanum_mpc.a"
c_header = "include/mecanum_mpc.h"
nstages = 5

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, ( 50,   1),   50),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,   1),    6),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (160,   1),  160)]

# Output                | Type    | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x1"                  , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x2"                  , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x3"                  , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x4"                  , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x5"                  , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
 ('it2opt', ctypes.c_int),
 ('res_eq', ctypes.c_double),
 ('res_ineq', ctypes.c_double),
 ('rsnorm', ctypes.c_double),
 ('rcompnorm', ctypes.c_double),
 ('pobj', ctypes.c_double),
 ('dobj', ctypes.c_double),
 ('dgap', ctypes.c_double),
 ('rdgap', ctypes.c_double),
 ('mu', ctypes.c_double),
 ('mu_aff', ctypes.c_double),
 ('sigma', ctypes.c_double),
 ('lsit_aff', ctypes.c_int),
 ('lsit_cc', ctypes.c_int),
 ('step_aff', ctypes.c_double),
 ('step_cc', ctypes.c_double),
 ('solvetime', ctypes.c_double),
 ('fevalstime', ctypes.c_double),
 ('solver_id', ctypes.c_int * 8)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(10, 6, 0, 32, 4, 4, 0, 0), 
	(10, 6, 0, 32, 4, 4, 0, 0), 
	(10, 6, 0, 32, 4, 4, 0, 0), 
	(10, 6, 0, 32, 4, 4, 0, 0), 
	(10, 0, 0, 32, 4, 4, 0, 0)
]