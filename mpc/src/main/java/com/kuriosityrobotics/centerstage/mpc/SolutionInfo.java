package com.kuriosityrobotics.centerstage.mpc;

/**
 * This class is based on the following struct:
 * {
 * //scalar: iteration number
 * solver_int32_default it;
 * <p>
 * // scalar: number of iterations needed to optimality (branch-and-bound)
 * solver_int32_default it2opt;
 * <p>
 * // scalar: inf-norm of equality constraint residuals
 * mecanum_mpc_float res_eq;
 * <p>
 * // scalar: inf-norm of inequality constraint residuals
 * mecanum_mpc_float res_ineq;
 * <p>
 * // scalar: norm of stationarity condition
 * mecanum_mpc_float rsnorm;
 * <p>
 * // scalar: max of all complementarity violations
 * mecanum_mpc_float rcompnorm;
 * <p>
 * // scalar: primal objective
 * mecanum_mpc_float pobj;
 * <p>
 * // scalar: dual objective
 * mecanum_mpc_float dobj;
 * <p>
 * // scalar: duality gap := pobj - dobj
 * mecanum_mpc_float dgap;
 * <p>
 * // scalar: relative duality gap := |dgap / pobj |
 * mecanum_mpc_float rdgap;
 * <p>
 * // scalar: duality measure
 * mecanum_mpc_float mu;
 * <p>
 * // scalar: duality measure (after affine step)
 * mecanum_mpc_float mu_aff;
 * <p>
 * // scalar: centering parameter
 * mecanum_mpc_float sigma;
 * <p>
 * // scalar: number of backtracking line search steps (affine direction)
 * solver_int32_default lsit_aff;
 * <p>
 * // scalar: number of backtracking line search steps (combined direction)
 * solver_int32_default lsit_cc;
 * <p>
 * // scalar: step size (affine direction)
 * mecanum_mpc_float step_aff;
 * <p>
 * // scalar: step size (combined direction)
 * mecanum_mpc_float step_cc;
 * <p>
 * // scalar: total solve time
 * mecanum_mpc_float solvetime;
 * <p>
 * // scalar: time spent in function evaluations
 * mecanum_mpc_float fevalstime;
 * <p>
 * // column vector of length 8: solver ID of FORCESPRO solver
 * solver_int32_default solver_id[8];
 * <p>
 * <p>
 * <p>
 * <p>
 * } mecanum_mpc_info;
 */
public final class SolutionInfo {
	public static final int SIZE = 19;
	private final int it;
	private final int it2opt;
	private final double resEq;
	private final double resIneq;
	private final double rsnorm;
	private final double rcompnorm;
	private final double pobj;
	private final double dobj;
	private final double dgap;
	private final double rdgap;
	private final double mu;
	private final double muAff;
	private final double sigma;
	private final int lsitAff;
	private final int lsitCc;
	private final double stepAff;
	private final double stepCc;
	private final double solvetime;
	private final double fevalstime;

	private SolutionInfo(
		int it, int it2opt, double resEq, double resIneq, double rsnorm, double rcompnorm, double pobj, double dobj, double dgap, double rdgap, double mu, double muAff, double sigma, int lsitAff, int lsitCc, double stepAff, double stepCc, double solvetime, double fevalstime
	) {
		this.it = it;
		this.it2opt = it2opt;
		this.resEq = resEq;
		this.resIneq = resIneq;
		this.rsnorm = rsnorm;
		this.rcompnorm = rcompnorm;
		this.pobj = pobj;
		this.dobj = dobj;
		this.dgap = dgap;
		this.rdgap = rdgap;
		this.mu = mu;
		this.muAff = muAff;
		this.sigma = sigma;
		this.lsitAff = lsitAff;
		this.lsitCc = lsitCc;
		this.stepAff = stepAff;
		this.stepCc = stepCc;
		this.solvetime = solvetime;
		this.fevalstime = fevalstime;
	}

	/**
	 * @return the number of iterations
	 */
	public int getIt() {
		return it;
	}

	/**
	 * @return the number of iterations needed to optimality
	 */
	public int getIt2opt() {
		return it2opt;
	}

	/**
	 * @return the inf-norm of equality constraint residuals
	 */
	public double getResEq() {
		return resEq;
	}

	/**
	 * @return the inf-norm of inequality constraint residuals
	 */
	public double getResIneq() {
		return resIneq;
	}

	/**
	 * @return the norm of stationarity condition
	 */
	public double getRsnorm() {
		return rsnorm;
	}

	/**
	 * @return the max of all complementarity violations
	 */
	public double getRcompnorm() {
		return rcompnorm;
	}

	/**
	 * @return the primal objective
	 */
	public double getPobj() {
		return pobj;
	}

	/**
	 * @return the dual objective
	 */
	public double getDobj() {
		return dobj;
	}

	/**
	 * @return the duality gap
	 */
	public double getDgap() {
		return dgap;
	}

	/**
	 * @return the relative duality gap
	 */
	public double getRdgap() {
		return rdgap;
	}

	/**
	 * @return the duality measure
	 */
	public double getMu() {
		return mu;
	}

	/**
	 * @return the duality measure (after affine step)
	 */
	public double getMuAff() {
		return muAff;
	}

	/**
	 * @return the centering parameter
	 */
	public double getSigma() {
		return sigma;
	}

	/**
	 * @return the number of backtracking line search steps (affine direction)
	 */
	public int getLsitAff() {
		return lsitAff;
	}

	/**
	 * @return the number of backtracking line search steps (combined direction)
	 */
	public int getLsitCc() {
		return lsitCc;
	}

	/**
	 * @return the step size (affine direction)
	 */
	public double getStepAff() {
		return stepAff;
	}

	/**
	 * @return the step size (combined direction)
	 */
	public double getStepCc() {
		return stepCc;
	}

	/**
	 * @return the total solve time
	 */
	public double getSolvetime() {
		return solvetime;
	}

	/**
	 * @return the time spent in function evaluations
	 */
	public double getFevalstime() {
		return fevalstime;
	}

	public static SolutionInfo ofDoubleArray(double[] array) {
		if (array.length != SIZE) {
			throw new IllegalArgumentException("Array must be of length " + SIZE);
		}
		return new SolutionInfo(
			(int) array[0], (int) array[1], array[2], array[3], array[4], array[5], array[6], array[7], array[8], array[9], array[10], array[11], array[12], (int) array[13], (int) array[14], array[15], array[16], array[17], array[18]
		);
	}

	@Override
	public String toString() {
		var sb = new StringBuilder();
		sb.append("SolutionInfo{");
		sb.append(" Solve time: ").append(solvetime * 1000).append("ms, ");
		sb.append("Iterations: ").append(it)
				.append(", primal objective: ").append(pobj)
				.append(", dual objective: ").append(dobj)
			.append(", equality constraint residuals: ").append(resEq);
		sb.append(" }");
		return sb.toString();
	}
}
