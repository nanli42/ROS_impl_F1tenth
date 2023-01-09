#include <acado_code_generation.hpp>

int main( )
{
  USING_NAMESPACE_ACADO

  DifferentialState   ey, ephi, vx, vy, omega, delta, t, s;
  IntermediateState   dsdt;
  Control             a, vd;

  OnlineData          ks;

  DifferentialEquation  f;

  const double lr = 0.17145;
  const double lf = 0.3302 - 0.17145;

  dsdt = ( vx*cos(ephi) - vy*sin(ephi) ) / ( 1 - ey*ks );
  f << dot(ey)      == 1/dsdt * ( vx*sin(ephi) + vy*cos(ephi) );
  f << dot(ephi)    == 1/dsdt * omega - ks;
  f << dot(vx)      == 1/dsdt * a;
  f << dot(vy)      == 1/dsdt * ( lr/(lr+lf) ) * ( vd*vx + delta*a );
  f << dot(omega)   == 1/dsdt * (  1/(lr+lf) ) * ( vd*vx + delta*a );
  f << dot(delta)   == 1/dsdt * vd;
  f << dot(t)       == 1/dsdt;
  f << dot(s)       == 1;

  const double s_start =  0.0;
  const double s_end   =  7.5;
  OCP ocp( s_start, s_end, 10 );
  ocp.minimizeMayerTerm( t );
  ocp.setModel( f );

  double margin = 0.4;
  double max_v = 2;
  ocp.subjectTo( -(1.7-margin) <= ey <= +(1.7-margin) );
  ocp.subjectTo( -0.75 <= ephi <= +0.75 );
  ocp.subjectTo( +0.05 <= vx <= +max_v );
  ocp.subjectTo( -0.5 <= vy <= +0.5 );
  ocp.subjectTo( -4.0 <= omega <= +4.0 );
  ocp.subjectTo( -0.41 <= delta <= +0.41 );
  ocp.subjectTo( +0.0 <= t <= 1000.0 );
  ocp.subjectTo( +0.0 <= s <= 1000.0 );

  ocp.subjectTo( -8.0 <= a <= +8.0 );
  ocp.subjectTo( -3.0 <= vd <= +3.0 );

  ocp.subjectTo( vx*vx+vy*vy <= max_v*max_v );

  ocp.setNOD(1);

  OCPexport mpc( ocp );

  mpc.set( HESSIAN_APPROXIMATION,       EXACT_HESSIAN    );
  mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
  mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
  mpc.set( NUM_INTEGRATOR_STEPS,        100              );
  mpc.set( QP_SOLVER,                   QP_QPOASES      );
  mpc.set( HOTSTART_QP,                 YES             		);
  mpc.set( GENERATE_TEST_FILE,          NO             );
  mpc.set( GENERATE_MAKE_FILE,          NO             );
  mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
  mpc.set( SPARSE_QP_SOLUTION, 		      FULL_CONDENSING_N2	);
  mpc.set( DYNAMIC_SENSITIVITY, 		  	SYMMETRIC				);

  mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );
  mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO 					);
  mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES 				);

  if (mpc.exportCode( "../../src/acado_generated_code_for_f110_nmpc" ) != SUCCESSFUL_RETURN)
    exit( EXIT_FAILURE );

  mpc.printDimensionsQP( );

  return EXIT_SUCCESS;
}
