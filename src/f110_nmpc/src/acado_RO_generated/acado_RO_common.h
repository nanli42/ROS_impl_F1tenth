/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#ifndef ACADO_RO_COMMON_H
#define ACADO_RO_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup ACADO ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define ACADO_RO_QPOASES  0
#define ACADO_RO_QPOASES3 1
/** FORCES QP solver indicator.*/
#define ACADO_RO_FORCES   2
/** qpDUNES QP solver indicator.*/
#define ACADO_RO_QPDUNES  3
/** HPMPC QP solver indicator. */
#define ACADO_RO_HPMPC    4
#define ACADO_RO_GENERIC    5

/** Indicator for determining the QP solver used by the ACADO solver code. */
#define ACADO_RO_QP_SOLVER ACADO_RO_QPOASES

#include "acado_RO_qpoases_interface.hpp"
#include "acado_common.h"

/*
 * Common definitions
 */
/** User defined block based condensing. */
#define ACADO_RO_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define ACADO_RO_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define ACADO_RO_HARDCODED_CONSTRAINT_VALUES 0
/** Indicator for fixed initial state. */
#define ACADO_RO_INITIAL_STATE_FIXED 1
/** Number of control/estimation intervals. */
#define ACADO_RO_N 10
/** Number of online data values. */
#define ACADO_RO_NOD 1
/** Number of path constraints. */
#define ACADO_RO_NPAC 1
/** Number of control variables. */
#define ACADO_RO_NU 2
/** Number of differential variables. */
#define ACADO_RO_NX 8
/** Number of algebraic variables. */
#define ACADO_RO_NXA 0
/** Number of differential derivative variables. */
#define ACADO_RO_NXD 0
/** Number of references/measurements per node on the first N nodes. */
#define ACADO_RO_NY 0
/** Number of references/measurements on the last (N + 1)st node. */
#define ACADO_RO_NYN 0
/** Total number of QP optimization variables. */
#define ACADO_RO_QP_NV 20
/** Number of integration steps per shooting interval. */
#define ACADO_RO_RK_NIS 10
/** Number of Runge-Kutta stages per integration step. */
#define ACADO_RO_RK_NSTAGES 4
/** Single versus double precision data type representation. */
#define ACADO_RO_SINGLE_PRECISION 0
/** Providing interface for arrival cost. */
#define ACADO_RO_USE_ARRIVAL_COST 0
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define ACADO_RO_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define ACADO_RO_WEIGHTING_MATRICES_TYPE 2


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int acado_RO_integrate( real_t* const rk_eta, int resetIntegrator );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_RO_acado_RO_forward(const real_t* in, real_t* out);

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_RO_acado_RO_backward(const real_t* in, real_t* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int acado_RO_preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int acado_RO_feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int acado_RO_initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void acado_RO_initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 11 with xEnd. 2. Initialize node 11 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_RO_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_RO_shiftControls( real_t* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
real_t acado_RO_getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
real_t acado_RO_getObjective(  );

/** EVD-based regularization of a Hessian block.
 *
 */
void acado_RO_regularize( real_t* const hessian_block );


/* 
 * Extern declarations. 
 */

extern ACADOworkspace acadoWorkspace_RO;
extern ACADOvariables acadoVariables_RO;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* ACADO_RO_COMMON_H */
