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


extern "C"
{
#include "acado_RO_common.h"
}

#include "INCLUDE/QProblem.hpp"

#if ACADO_RO_COMPUTE_COVARIANCE_MATRIX == 1
#include "INCLUDE/EXTRAS/SolutionAnalysis.hpp"
#endif /* ACADO_RO_COMPUTE_COVARIANCE_MATRIX */

static int acado_RO_nWSR;



#if ACADO_RO_COMPUTE_COVARIANCE_MATRIX == 1
static SolutionAnalysis acado_RO_sa;
#endif /* ACADO_RO_COMPUTE_COVARIANCE_MATRIX */

int acado_RO_solve( void )
{
	acado_RO_nWSR = QPOASES_NWSRMAX;

	QProblem qp(20, 90);
	
	returnValue retVal = qp.init(acadoWorkspace_RO.H, acadoWorkspace_RO.g, acadoWorkspace_RO.A, acadoWorkspace_RO.lb, acadoWorkspace_RO.ub, acadoWorkspace_RO.lbA, acadoWorkspace_RO.ubA, acado_RO_nWSR, acadoWorkspace_RO.y);

    qp.getPrimalSolution( acadoWorkspace_RO.x );
    qp.getDualSolution( acadoWorkspace_RO.y );
	
#if ACADO_RO_COMPUTE_COVARIANCE_MATRIX == 1

	if (retVal != SUCCESSFUL_RETURN)
		return (int)retVal;
		
	retVal = acado_RO_sa.getHessianInverse( &qp, );

#endif /* ACADO_RO_COMPUTE_COVARIANCE_MATRIX */

	return (int)retVal;
}

int acado_RO_getNWSR( void )
{
	return acado_RO_nWSR;
}

const char* acado_RO_getErrorString( int error )
{
	return MessageHandling::getErrorString( error );
}
