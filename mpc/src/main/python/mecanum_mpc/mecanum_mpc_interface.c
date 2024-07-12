/*
 * AD tool to FORCESPRO Template - missing information to be filled in by createADTool.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2023. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif

#include "include/mecanum_mpc.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "mecanum_mpc_model.h"



/* copies data from sparse matrix into a dense one */
static void mecanum_mpc_sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, mecanum_mpc_callback_float *data, mecanum_mpc_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((mecanum_mpc_float) data[j]);
        }
    }
}




/* AD tool to FORCESPRO interface */
extern solver_int32_default mecanum_mpc_adtool2forces(mecanum_mpc_float *x,        /* primal vars                                         */
                                 mecanum_mpc_float *y,        /* eq. constraint multiplers                           */
                                 mecanum_mpc_float *l,        /* ineq. constraint multipliers                        */
                                 mecanum_mpc_float *p,        /* parameters                                          */
                                 mecanum_mpc_float *f,        /* objective function (scalar)                         */
                                 mecanum_mpc_float *nabla_f,  /* gradient of objective function                      */
                                 mecanum_mpc_float *c,        /* dynamics                                            */
                                 mecanum_mpc_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 mecanum_mpc_float *h,        /* inequality constraints                              */
                                 mecanum_mpc_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 mecanum_mpc_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
                                 solver_int32_default iteration, /* iteration number of solver                         */
                                 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* AD tool input and output arrays */
    const mecanum_mpc_callback_float *in[4];
    mecanum_mpc_callback_float *out[7];
    

    /* Allocate working arrays for AD tool */
    
    mecanum_mpc_callback_float w[369];
	
    /* temporary storage for AD tool sparse output */
    mecanum_mpc_callback_float this_f = (mecanum_mpc_callback_float) 0.0;
    mecanum_mpc_float nabla_f_sparse[10];
    
    
    mecanum_mpc_float c_sparse[6];
    mecanum_mpc_float nabla_c_sparse[50];
    
    
    /* pointers to row and column info for 
     * column compressed format used by AD tool */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for AD tool */
    in[0] = x;
    in[1] = p;
    in[2] = l;
    in[3] = y;

	if ((0 <= stage && stage <= 3))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		mecanum_mpc_objective_0(in, out, NULL, w, 0);
		if( nabla_f != NULL )
		{
			nrow = mecanum_mpc_objective_0_sparsity_out(1)[0];
			ncol = mecanum_mpc_objective_0_sparsity_out(1)[1];
			colind = mecanum_mpc_objective_0_sparsity_out(1) + 2;
			row = mecanum_mpc_objective_0_sparsity_out(1) + 2 + (ncol + 1);
				
			mecanum_mpc_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		mecanum_mpc_dynamics_0(in, out, NULL, w, 0);
		if( c != NULL )
		{
			nrow = mecanum_mpc_dynamics_0_sparsity_out(0)[0];
			ncol = mecanum_mpc_dynamics_0_sparsity_out(0)[1];
			colind = mecanum_mpc_dynamics_0_sparsity_out(0) + 2;
			row = mecanum_mpc_dynamics_0_sparsity_out(0) + 2 + (ncol + 1);
				
			mecanum_mpc_sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}
		if( nabla_c != NULL )
		{
			nrow = mecanum_mpc_dynamics_0_sparsity_out(1)[0];
			ncol = mecanum_mpc_dynamics_0_sparsity_out(1)[1];
			colind = mecanum_mpc_dynamics_0_sparsity_out(1) + 2;
			row = mecanum_mpc_dynamics_0_sparsity_out(1) + 2 + (ncol + 1);
				
			mecanum_mpc_sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}
	}
	if ((4 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		mecanum_mpc_objective_1(in, out, NULL, w, 0);
		if( nabla_f != NULL )
		{
			nrow = mecanum_mpc_objective_1_sparsity_out(1)[0];
			ncol = mecanum_mpc_objective_1_sparsity_out(1)[1];
			colind = mecanum_mpc_objective_1_sparsity_out(1) + 2;
			row = mecanum_mpc_objective_1_sparsity_out(1) + 2 + (ncol + 1);
				
			mecanum_mpc_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((mecanum_mpc_float) this_f);
    }

    return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
