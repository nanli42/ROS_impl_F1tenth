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
#include "acado_LO_common.h"
}

#include "INCLUDE/QProblem.hpp"

#if ACADO_LO_COMPUTE_COVARIANCE_MATRIX == 1
#include "INCLUDE/EXTRAS/SolutionAnalysis.hpp"
#endif /* ACADO_LO_COMPUTE_COVARIANCE_MATRIX */

static int acado_LO_nWSR;



#if ACADO_LO_COMPUTE_COVARIANCE_MATRIX == 1
static SolutionAnalysis acado_LO_sa;
#endif /* ACADO_LO_COMPUTE_COVARIANCE_MATRIX */

int acado_LO_solve( void )
{
	acado_LO_nWSR = QPOASES_NWSRMAX;

	QProblem qp(20, 90);
	
	returnValue retVal = qp.init(acadoWorkspace_LO.H, acadoWorkspace_LO.g, acadoWorkspace_LO.A, acadoWorkspace_LO.lb, acadoWorkspace_LO.ub, acadoWorkspace_LO.lbA, acadoWorkspace_LO.ubA, acado_LO_nWSR, acadoWorkspace_LO.y);

    qp.getPrimalSolution( acadoWorkspace_LO.x );
    qp.getDualSolution( acadoWorkspace_LO.y );
	
#if ACADO_LO_COMPUTE_COVARIANCE_MATRIX == 1

	if (retVal != SUCCESSFUL_RETURN)
		return (int)retVal;
		
	retVal = acado_LO_sa.getHessianInverse( &qp, );

#endif /* ACADO_LO_COMPUTE_COVARIANCE_MATRIX */

	return (int)retVal;
}

int acado_LO_getNWSR( void )
{
	return acado_LO_nWSR;
}

const char* acado_LO_getErrorString( int error )
{
	return MessageHandling::getErrorString( error );
}
