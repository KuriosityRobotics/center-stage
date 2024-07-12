/* This template is used when statically linking a solver and its external
 * evaluation functions (for nonlinearities). It simply exports a function that
 * is essentially a closure around the solver function, with the last argument
 * (the pointer to the external evaluation function) fixed.
 *
 * The template is used by setting the following preprocessor macros:
 *  - mecanum_mpc
 *  - "include/mecanum_mpc.h"
 *  - mecanum_mpc_adtool2forces
 *  - mecanum_mpc_interface
 *
 * Compare also the MEX interface, which exists for a similar purpose in the
 * MATLAB client.
 * 
 * This file is part of the FORCESPRO client, and carries the same license.
 * (C) embotech AG, Zurich, Switzerland, 2013-2023. All rights reserved.
 */

#define CONCAT(x, y) x ## y
#define CONCATENATE(x, y) CONCAT(x, y)
#define SOLVER_FLOAT CONCATENATE(mecanum_mpc, _float)
#define SOLVER_FUN_NAME CONCATENATE(mecanum_mpc, _solve)

#include "include/mecanum_mpc.h"

/* Header of external evaluation function */
solver_int32_default mecanum_mpc_adtool2forces(SOLVER_FLOAT *x, SOLVER_FLOAT *y, SOLVER_FLOAT *l,
                   SOLVER_FLOAT *p, SOLVER_FLOAT *f, SOLVER_FLOAT *nabla_f,
                   SOLVER_FLOAT *c, SOLVER_FLOAT *nabla_c, SOLVER_FLOAT *h,
                   SOLVER_FLOAT *nabla_h, SOLVER_FLOAT *hess,
                   solver_int32_default stage, solver_int32_default iteration, solver_int32_default threadID);

int mecanum_mpc_interface(void *params, void *outputs, void *info, void *mem, FILE *fp) {
    return SOLVER_FUN_NAME(params, outputs, info, mem, fp, &mecanum_mpc_adtool2forces);
}
