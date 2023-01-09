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


#include "acado_LO_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_LO_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace_LO.state[0] = acadoVariables_LO.x[lRun1 * 8];
acadoWorkspace_LO.state[1] = acadoVariables_LO.x[lRun1 * 8 + 1];
acadoWorkspace_LO.state[2] = acadoVariables_LO.x[lRun1 * 8 + 2];
acadoWorkspace_LO.state[3] = acadoVariables_LO.x[lRun1 * 8 + 3];
acadoWorkspace_LO.state[4] = acadoVariables_LO.x[lRun1 * 8 + 4];
acadoWorkspace_LO.state[5] = acadoVariables_LO.x[lRun1 * 8 + 5];
acadoWorkspace_LO.state[6] = acadoVariables_LO.x[lRun1 * 8 + 6];
acadoWorkspace_LO.state[7] = acadoVariables_LO.x[lRun1 * 8 + 7];

acadoWorkspace_LO.state[8] = acadoVariables_LO.mu[lRun1 * 8];
acadoWorkspace_LO.state[9] = acadoVariables_LO.mu[lRun1 * 8 + 1];
acadoWorkspace_LO.state[10] = acadoVariables_LO.mu[lRun1 * 8 + 2];
acadoWorkspace_LO.state[11] = acadoVariables_LO.mu[lRun1 * 8 + 3];
acadoWorkspace_LO.state[12] = acadoVariables_LO.mu[lRun1 * 8 + 4];
acadoWorkspace_LO.state[13] = acadoVariables_LO.mu[lRun1 * 8 + 5];
acadoWorkspace_LO.state[14] = acadoVariables_LO.mu[lRun1 * 8 + 6];
acadoWorkspace_LO.state[15] = acadoVariables_LO.mu[lRun1 * 8 + 7];
acadoWorkspace_LO.state[151] = acadoVariables_LO.u[lRun1 * 2];
acadoWorkspace_LO.state[152] = acadoVariables_LO.u[lRun1 * 2 + 1];
acadoWorkspace_LO.state[153] = acadoVariables_LO.od[lRun1];

ret = acado_LO_integrate(acadoWorkspace_LO.state, 1);

acadoWorkspace_LO.d[lRun1 * 8] = acadoWorkspace_LO.state[0] - acadoVariables_LO.x[lRun1 * 8 + 8];
acadoWorkspace_LO.d[lRun1 * 8 + 1] = acadoWorkspace_LO.state[1] - acadoVariables_LO.x[lRun1 * 8 + 9];
acadoWorkspace_LO.d[lRun1 * 8 + 2] = acadoWorkspace_LO.state[2] - acadoVariables_LO.x[lRun1 * 8 + 10];
acadoWorkspace_LO.d[lRun1 * 8 + 3] = acadoWorkspace_LO.state[3] - acadoVariables_LO.x[lRun1 * 8 + 11];
acadoWorkspace_LO.d[lRun1 * 8 + 4] = acadoWorkspace_LO.state[4] - acadoVariables_LO.x[lRun1 * 8 + 12];
acadoWorkspace_LO.d[lRun1 * 8 + 5] = acadoWorkspace_LO.state[5] - acadoVariables_LO.x[lRun1 * 8 + 13];
acadoWorkspace_LO.d[lRun1 * 8 + 6] = acadoWorkspace_LO.state[6] - acadoVariables_LO.x[lRun1 * 8 + 14];
acadoWorkspace_LO.d[lRun1 * 8 + 7] = acadoWorkspace_LO.state[7] - acadoVariables_LO.x[lRun1 * 8 + 15];

acadoWorkspace_LO.evGx[lRun1 * 64] = acadoWorkspace_LO.state[16];
acadoWorkspace_LO.evGx[lRun1 * 64 + 1] = acadoWorkspace_LO.state[17];
acadoWorkspace_LO.evGx[lRun1 * 64 + 2] = acadoWorkspace_LO.state[18];
acadoWorkspace_LO.evGx[lRun1 * 64 + 3] = acadoWorkspace_LO.state[19];
acadoWorkspace_LO.evGx[lRun1 * 64 + 4] = acadoWorkspace_LO.state[20];
acadoWorkspace_LO.evGx[lRun1 * 64 + 5] = acadoWorkspace_LO.state[21];
acadoWorkspace_LO.evGx[lRun1 * 64 + 6] = acadoWorkspace_LO.state[22];
acadoWorkspace_LO.evGx[lRun1 * 64 + 7] = acadoWorkspace_LO.state[23];
acadoWorkspace_LO.evGx[lRun1 * 64 + 8] = acadoWorkspace_LO.state[24];
acadoWorkspace_LO.evGx[lRun1 * 64 + 9] = acadoWorkspace_LO.state[25];
acadoWorkspace_LO.evGx[lRun1 * 64 + 10] = acadoWorkspace_LO.state[26];
acadoWorkspace_LO.evGx[lRun1 * 64 + 11] = acadoWorkspace_LO.state[27];
acadoWorkspace_LO.evGx[lRun1 * 64 + 12] = acadoWorkspace_LO.state[28];
acadoWorkspace_LO.evGx[lRun1 * 64 + 13] = acadoWorkspace_LO.state[29];
acadoWorkspace_LO.evGx[lRun1 * 64 + 14] = acadoWorkspace_LO.state[30];
acadoWorkspace_LO.evGx[lRun1 * 64 + 15] = acadoWorkspace_LO.state[31];
acadoWorkspace_LO.evGx[lRun1 * 64 + 16] = acadoWorkspace_LO.state[32];
acadoWorkspace_LO.evGx[lRun1 * 64 + 17] = acadoWorkspace_LO.state[33];
acadoWorkspace_LO.evGx[lRun1 * 64 + 18] = acadoWorkspace_LO.state[34];
acadoWorkspace_LO.evGx[lRun1 * 64 + 19] = acadoWorkspace_LO.state[35];
acadoWorkspace_LO.evGx[lRun1 * 64 + 20] = acadoWorkspace_LO.state[36];
acadoWorkspace_LO.evGx[lRun1 * 64 + 21] = acadoWorkspace_LO.state[37];
acadoWorkspace_LO.evGx[lRun1 * 64 + 22] = acadoWorkspace_LO.state[38];
acadoWorkspace_LO.evGx[lRun1 * 64 + 23] = acadoWorkspace_LO.state[39];
acadoWorkspace_LO.evGx[lRun1 * 64 + 24] = acadoWorkspace_LO.state[40];
acadoWorkspace_LO.evGx[lRun1 * 64 + 25] = acadoWorkspace_LO.state[41];
acadoWorkspace_LO.evGx[lRun1 * 64 + 26] = acadoWorkspace_LO.state[42];
acadoWorkspace_LO.evGx[lRun1 * 64 + 27] = acadoWorkspace_LO.state[43];
acadoWorkspace_LO.evGx[lRun1 * 64 + 28] = acadoWorkspace_LO.state[44];
acadoWorkspace_LO.evGx[lRun1 * 64 + 29] = acadoWorkspace_LO.state[45];
acadoWorkspace_LO.evGx[lRun1 * 64 + 30] = acadoWorkspace_LO.state[46];
acadoWorkspace_LO.evGx[lRun1 * 64 + 31] = acadoWorkspace_LO.state[47];
acadoWorkspace_LO.evGx[lRun1 * 64 + 32] = acadoWorkspace_LO.state[48];
acadoWorkspace_LO.evGx[lRun1 * 64 + 33] = acadoWorkspace_LO.state[49];
acadoWorkspace_LO.evGx[lRun1 * 64 + 34] = acadoWorkspace_LO.state[50];
acadoWorkspace_LO.evGx[lRun1 * 64 + 35] = acadoWorkspace_LO.state[51];
acadoWorkspace_LO.evGx[lRun1 * 64 + 36] = acadoWorkspace_LO.state[52];
acadoWorkspace_LO.evGx[lRun1 * 64 + 37] = acadoWorkspace_LO.state[53];
acadoWorkspace_LO.evGx[lRun1 * 64 + 38] = acadoWorkspace_LO.state[54];
acadoWorkspace_LO.evGx[lRun1 * 64 + 39] = acadoWorkspace_LO.state[55];
acadoWorkspace_LO.evGx[lRun1 * 64 + 40] = acadoWorkspace_LO.state[56];
acadoWorkspace_LO.evGx[lRun1 * 64 + 41] = acadoWorkspace_LO.state[57];
acadoWorkspace_LO.evGx[lRun1 * 64 + 42] = acadoWorkspace_LO.state[58];
acadoWorkspace_LO.evGx[lRun1 * 64 + 43] = acadoWorkspace_LO.state[59];
acadoWorkspace_LO.evGx[lRun1 * 64 + 44] = acadoWorkspace_LO.state[60];
acadoWorkspace_LO.evGx[lRun1 * 64 + 45] = acadoWorkspace_LO.state[61];
acadoWorkspace_LO.evGx[lRun1 * 64 + 46] = acadoWorkspace_LO.state[62];
acadoWorkspace_LO.evGx[lRun1 * 64 + 47] = acadoWorkspace_LO.state[63];
acadoWorkspace_LO.evGx[lRun1 * 64 + 48] = acadoWorkspace_LO.state[64];
acadoWorkspace_LO.evGx[lRun1 * 64 + 49] = acadoWorkspace_LO.state[65];
acadoWorkspace_LO.evGx[lRun1 * 64 + 50] = acadoWorkspace_LO.state[66];
acadoWorkspace_LO.evGx[lRun1 * 64 + 51] = acadoWorkspace_LO.state[67];
acadoWorkspace_LO.evGx[lRun1 * 64 + 52] = acadoWorkspace_LO.state[68];
acadoWorkspace_LO.evGx[lRun1 * 64 + 53] = acadoWorkspace_LO.state[69];
acadoWorkspace_LO.evGx[lRun1 * 64 + 54] = acadoWorkspace_LO.state[70];
acadoWorkspace_LO.evGx[lRun1 * 64 + 55] = acadoWorkspace_LO.state[71];
acadoWorkspace_LO.evGx[lRun1 * 64 + 56] = acadoWorkspace_LO.state[72];
acadoWorkspace_LO.evGx[lRun1 * 64 + 57] = acadoWorkspace_LO.state[73];
acadoWorkspace_LO.evGx[lRun1 * 64 + 58] = acadoWorkspace_LO.state[74];
acadoWorkspace_LO.evGx[lRun1 * 64 + 59] = acadoWorkspace_LO.state[75];
acadoWorkspace_LO.evGx[lRun1 * 64 + 60] = acadoWorkspace_LO.state[76];
acadoWorkspace_LO.evGx[lRun1 * 64 + 61] = acadoWorkspace_LO.state[77];
acadoWorkspace_LO.evGx[lRun1 * 64 + 62] = acadoWorkspace_LO.state[78];
acadoWorkspace_LO.evGx[lRun1 * 64 + 63] = acadoWorkspace_LO.state[79];

acadoWorkspace_LO.evGu[lRun1 * 16] = acadoWorkspace_LO.state[80];
acadoWorkspace_LO.evGu[lRun1 * 16 + 1] = acadoWorkspace_LO.state[81];
acadoWorkspace_LO.evGu[lRun1 * 16 + 2] = acadoWorkspace_LO.state[82];
acadoWorkspace_LO.evGu[lRun1 * 16 + 3] = acadoWorkspace_LO.state[83];
acadoWorkspace_LO.evGu[lRun1 * 16 + 4] = acadoWorkspace_LO.state[84];
acadoWorkspace_LO.evGu[lRun1 * 16 + 5] = acadoWorkspace_LO.state[85];
acadoWorkspace_LO.evGu[lRun1 * 16 + 6] = acadoWorkspace_LO.state[86];
acadoWorkspace_LO.evGu[lRun1 * 16 + 7] = acadoWorkspace_LO.state[87];
acadoWorkspace_LO.evGu[lRun1 * 16 + 8] = acadoWorkspace_LO.state[88];
acadoWorkspace_LO.evGu[lRun1 * 16 + 9] = acadoWorkspace_LO.state[89];
acadoWorkspace_LO.evGu[lRun1 * 16 + 10] = acadoWorkspace_LO.state[90];
acadoWorkspace_LO.evGu[lRun1 * 16 + 11] = acadoWorkspace_LO.state[91];
acadoWorkspace_LO.evGu[lRun1 * 16 + 12] = acadoWorkspace_LO.state[92];
acadoWorkspace_LO.evGu[lRun1 * 16 + 13] = acadoWorkspace_LO.state[93];
acadoWorkspace_LO.evGu[lRun1 * 16 + 14] = acadoWorkspace_LO.state[94];
acadoWorkspace_LO.evGu[lRun1 * 16 + 15] = acadoWorkspace_LO.state[95];
acadoWorkspace_LO.EH[lRun1 * 100] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[96];
acadoWorkspace_LO.EH[lRun1 * 100 + 10] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[97];
acadoWorkspace_LO.EH[lRun1 * 100 + 1] = acadoWorkspace_LO.EH[lRun1 * 100 + 10];
acadoWorkspace_LO.EH[lRun1 * 100 + 11] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[98];
acadoWorkspace_LO.EH[lRun1 * 100 + 20] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[99];
acadoWorkspace_LO.EH[lRun1 * 100 + 2] = acadoWorkspace_LO.EH[lRun1 * 100 + 20];
acadoWorkspace_LO.EH[lRun1 * 100 + 21] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[100];
acadoWorkspace_LO.EH[lRun1 * 100 + 12] = acadoWorkspace_LO.EH[lRun1 * 100 + 21];
acadoWorkspace_LO.EH[lRun1 * 100 + 22] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[101];
acadoWorkspace_LO.EH[lRun1 * 100 + 30] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[102];
acadoWorkspace_LO.EH[lRun1 * 100 + 3] = acadoWorkspace_LO.EH[lRun1 * 100 + 30];
acadoWorkspace_LO.EH[lRun1 * 100 + 31] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[103];
acadoWorkspace_LO.EH[lRun1 * 100 + 13] = acadoWorkspace_LO.EH[lRun1 * 100 + 31];
acadoWorkspace_LO.EH[lRun1 * 100 + 32] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[104];
acadoWorkspace_LO.EH[lRun1 * 100 + 23] = acadoWorkspace_LO.EH[lRun1 * 100 + 32];
acadoWorkspace_LO.EH[lRun1 * 100 + 33] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[105];
acadoWorkspace_LO.EH[lRun1 * 100 + 40] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[106];
acadoWorkspace_LO.EH[lRun1 * 100 + 4] = acadoWorkspace_LO.EH[lRun1 * 100 + 40];
acadoWorkspace_LO.EH[lRun1 * 100 + 41] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[107];
acadoWorkspace_LO.EH[lRun1 * 100 + 14] = acadoWorkspace_LO.EH[lRun1 * 100 + 41];
acadoWorkspace_LO.EH[lRun1 * 100 + 42] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[108];
acadoWorkspace_LO.EH[lRun1 * 100 + 24] = acadoWorkspace_LO.EH[lRun1 * 100 + 42];
acadoWorkspace_LO.EH[lRun1 * 100 + 43] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[109];
acadoWorkspace_LO.EH[lRun1 * 100 + 34] = acadoWorkspace_LO.EH[lRun1 * 100 + 43];
acadoWorkspace_LO.EH[lRun1 * 100 + 44] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[110];
acadoWorkspace_LO.EH[lRun1 * 100 + 50] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[111];
acadoWorkspace_LO.EH[lRun1 * 100 + 5] = acadoWorkspace_LO.EH[lRun1 * 100 + 50];
acadoWorkspace_LO.EH[lRun1 * 100 + 51] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[112];
acadoWorkspace_LO.EH[lRun1 * 100 + 15] = acadoWorkspace_LO.EH[lRun1 * 100 + 51];
acadoWorkspace_LO.EH[lRun1 * 100 + 52] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[113];
acadoWorkspace_LO.EH[lRun1 * 100 + 25] = acadoWorkspace_LO.EH[lRun1 * 100 + 52];
acadoWorkspace_LO.EH[lRun1 * 100 + 53] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[114];
acadoWorkspace_LO.EH[lRun1 * 100 + 35] = acadoWorkspace_LO.EH[lRun1 * 100 + 53];
acadoWorkspace_LO.EH[lRun1 * 100 + 54] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[115];
acadoWorkspace_LO.EH[lRun1 * 100 + 45] = acadoWorkspace_LO.EH[lRun1 * 100 + 54];
acadoWorkspace_LO.EH[lRun1 * 100 + 55] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[116];
acadoWorkspace_LO.EH[lRun1 * 100 + 60] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[117];
acadoWorkspace_LO.EH[lRun1 * 100 + 6] = acadoWorkspace_LO.EH[lRun1 * 100 + 60];
acadoWorkspace_LO.EH[lRun1 * 100 + 61] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[118];
acadoWorkspace_LO.EH[lRun1 * 100 + 16] = acadoWorkspace_LO.EH[lRun1 * 100 + 61];
acadoWorkspace_LO.EH[lRun1 * 100 + 62] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[119];
acadoWorkspace_LO.EH[lRun1 * 100 + 26] = acadoWorkspace_LO.EH[lRun1 * 100 + 62];
acadoWorkspace_LO.EH[lRun1 * 100 + 63] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[120];
acadoWorkspace_LO.EH[lRun1 * 100 + 36] = acadoWorkspace_LO.EH[lRun1 * 100 + 63];
acadoWorkspace_LO.EH[lRun1 * 100 + 64] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[121];
acadoWorkspace_LO.EH[lRun1 * 100 + 46] = acadoWorkspace_LO.EH[lRun1 * 100 + 64];
acadoWorkspace_LO.EH[lRun1 * 100 + 65] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[122];
acadoWorkspace_LO.EH[lRun1 * 100 + 56] = acadoWorkspace_LO.EH[lRun1 * 100 + 65];
acadoWorkspace_LO.EH[lRun1 * 100 + 66] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[123];
acadoWorkspace_LO.EH[lRun1 * 100 + 70] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[124];
acadoWorkspace_LO.EH[lRun1 * 100 + 7] = acadoWorkspace_LO.EH[lRun1 * 100 + 70];
acadoWorkspace_LO.EH[lRun1 * 100 + 71] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[125];
acadoWorkspace_LO.EH[lRun1 * 100 + 17] = acadoWorkspace_LO.EH[lRun1 * 100 + 71];
acadoWorkspace_LO.EH[lRun1 * 100 + 72] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[126];
acadoWorkspace_LO.EH[lRun1 * 100 + 27] = acadoWorkspace_LO.EH[lRun1 * 100 + 72];
acadoWorkspace_LO.EH[lRun1 * 100 + 73] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[127];
acadoWorkspace_LO.EH[lRun1 * 100 + 37] = acadoWorkspace_LO.EH[lRun1 * 100 + 73];
acadoWorkspace_LO.EH[lRun1 * 100 + 74] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[128];
acadoWorkspace_LO.EH[lRun1 * 100 + 47] = acadoWorkspace_LO.EH[lRun1 * 100 + 74];
acadoWorkspace_LO.EH[lRun1 * 100 + 75] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[129];
acadoWorkspace_LO.EH[lRun1 * 100 + 57] = acadoWorkspace_LO.EH[lRun1 * 100 + 75];
acadoWorkspace_LO.EH[lRun1 * 100 + 76] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[130];
acadoWorkspace_LO.EH[lRun1 * 100 + 67] = acadoWorkspace_LO.EH[lRun1 * 100 + 76];
acadoWorkspace_LO.EH[lRun1 * 100 + 77] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[131];
acadoWorkspace_LO.EH[lRun1 * 100 + 80] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[132];
acadoWorkspace_LO.EH[lRun1 * 100 + 8] = acadoWorkspace_LO.EH[lRun1 * 100 + 80];
acadoWorkspace_LO.EH[lRun1 * 100 + 81] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[133];
acadoWorkspace_LO.EH[lRun1 * 100 + 18] = acadoWorkspace_LO.EH[lRun1 * 100 + 81];
acadoWorkspace_LO.EH[lRun1 * 100 + 82] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[134];
acadoWorkspace_LO.EH[lRun1 * 100 + 28] = acadoWorkspace_LO.EH[lRun1 * 100 + 82];
acadoWorkspace_LO.EH[lRun1 * 100 + 83] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[135];
acadoWorkspace_LO.EH[lRun1 * 100 + 38] = acadoWorkspace_LO.EH[lRun1 * 100 + 83];
acadoWorkspace_LO.EH[lRun1 * 100 + 84] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[136];
acadoWorkspace_LO.EH[lRun1 * 100 + 48] = acadoWorkspace_LO.EH[lRun1 * 100 + 84];
acadoWorkspace_LO.EH[lRun1 * 100 + 85] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[137];
acadoWorkspace_LO.EH[lRun1 * 100 + 58] = acadoWorkspace_LO.EH[lRun1 * 100 + 85];
acadoWorkspace_LO.EH[lRun1 * 100 + 86] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[138];
acadoWorkspace_LO.EH[lRun1 * 100 + 68] = acadoWorkspace_LO.EH[lRun1 * 100 + 86];
acadoWorkspace_LO.EH[lRun1 * 100 + 87] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[139];
acadoWorkspace_LO.EH[lRun1 * 100 + 78] = acadoWorkspace_LO.EH[lRun1 * 100 + 87];
acadoWorkspace_LO.EH[lRun1 * 100 + 88] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[140];
acadoWorkspace_LO.EH[lRun1 * 100 + 90] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[141];
acadoWorkspace_LO.EH[lRun1 * 100 + 9] = acadoWorkspace_LO.EH[lRun1 * 100 + 90];
acadoWorkspace_LO.EH[lRun1 * 100 + 91] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[142];
acadoWorkspace_LO.EH[lRun1 * 100 + 19] = acadoWorkspace_LO.EH[lRun1 * 100 + 91];
acadoWorkspace_LO.EH[lRun1 * 100 + 92] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[143];
acadoWorkspace_LO.EH[lRun1 * 100 + 29] = acadoWorkspace_LO.EH[lRun1 * 100 + 92];
acadoWorkspace_LO.EH[lRun1 * 100 + 93] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[144];
acadoWorkspace_LO.EH[lRun1 * 100 + 39] = acadoWorkspace_LO.EH[lRun1 * 100 + 93];
acadoWorkspace_LO.EH[lRun1 * 100 + 94] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[145];
acadoWorkspace_LO.EH[lRun1 * 100 + 49] = acadoWorkspace_LO.EH[lRun1 * 100 + 94];
acadoWorkspace_LO.EH[lRun1 * 100 + 95] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[146];
acadoWorkspace_LO.EH[lRun1 * 100 + 59] = acadoWorkspace_LO.EH[lRun1 * 100 + 95];
acadoWorkspace_LO.EH[lRun1 * 100 + 96] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[147];
acadoWorkspace_LO.EH[lRun1 * 100 + 69] = acadoWorkspace_LO.EH[lRun1 * 100 + 96];
acadoWorkspace_LO.EH[lRun1 * 100 + 97] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[148];
acadoWorkspace_LO.EH[lRun1 * 100 + 79] = acadoWorkspace_LO.EH[lRun1 * 100 + 97];
acadoWorkspace_LO.EH[lRun1 * 100 + 98] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[149];
acadoWorkspace_LO.EH[lRun1 * 100 + 89] = acadoWorkspace_LO.EH[lRun1 * 100 + 98];
acadoWorkspace_LO.EH[lRun1 * 100 + 99] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.state[150];
}
return ret;
}

void acado_LO_evaluateMayer(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 44. */
real_t* a = acadoWorkspace_LO.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(1.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (real_t)(0.0000000000000000e+00);
a[27] = (real_t)(0.0000000000000000e+00);
a[28] = (real_t)(0.0000000000000000e+00);
a[29] = (real_t)(0.0000000000000000e+00);
a[30] = (real_t)(0.0000000000000000e+00);
a[31] = (real_t)(0.0000000000000000e+00);
a[32] = (real_t)(0.0000000000000000e+00);
a[33] = (real_t)(0.0000000000000000e+00);
a[34] = (real_t)(0.0000000000000000e+00);
a[35] = (real_t)(0.0000000000000000e+00);
a[36] = (real_t)(0.0000000000000000e+00);
a[37] = (real_t)(0.0000000000000000e+00);
a[38] = (real_t)(0.0000000000000000e+00);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (real_t)(0.0000000000000000e+00);
a[42] = (real_t)(0.0000000000000000e+00);
a[43] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = xd[6];
out[1] = a[0];
out[2] = a[1];
out[3] = a[2];
out[4] = a[3];
out[5] = a[4];
out[6] = a[5];
out[7] = a[6];
out[8] = a[7];
out[9] = a[8];
out[10] = a[9];
out[11] = a[10];
out[12] = a[11];
out[13] = a[12];
out[14] = a[13];
out[15] = a[14];
out[16] = a[15];
out[17] = a[9];
out[18] = a[16];
out[19] = a[17];
out[20] = a[18];
out[21] = a[19];
out[22] = a[20];
out[23] = a[21];
out[24] = a[22];
out[25] = a[10];
out[26] = a[17];
out[27] = a[23];
out[28] = a[24];
out[29] = a[25];
out[30] = a[26];
out[31] = a[27];
out[32] = a[28];
out[33] = a[11];
out[34] = a[18];
out[35] = a[24];
out[36] = a[29];
out[37] = a[30];
out[38] = a[31];
out[39] = a[32];
out[40] = a[33];
out[41] = a[12];
out[42] = a[19];
out[43] = a[25];
out[44] = a[30];
out[45] = a[34];
out[46] = a[35];
out[47] = a[36];
out[48] = a[37];
out[49] = a[13];
out[50] = a[20];
out[51] = a[26];
out[52] = a[31];
out[53] = a[35];
out[54] = a[38];
out[55] = a[39];
out[56] = a[40];
out[57] = a[14];
out[58] = a[21];
out[59] = a[27];
out[60] = a[32];
out[61] = a[36];
out[62] = a[39];
out[63] = a[41];
out[64] = a[42];
out[65] = a[15];
out[66] = a[22];
out[67] = a[28];
out[68] = a[33];
out[69] = a[37];
out[70] = a[40];
out[71] = a[42];
out[72] = a[43];
}

void acado_LO_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 65. */
real_t* a = acadoWorkspace_LO.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (xd[2]+xd[2]);
a[3] = (xd[3]+xd[3]);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (real_t)(0.0000000000000000e+00);
a[27] = (real_t)(0.0000000000000000e+00);
a[28] = (real_t)(0.0000000000000000e+00);
a[29] = ((real_t)(2.0000000000000000e+00)*xd[8]);
a[30] = (real_t)(0.0000000000000000e+00);
a[31] = (real_t)(0.0000000000000000e+00);
a[32] = (real_t)(0.0000000000000000e+00);
a[33] = (real_t)(0.0000000000000000e+00);
a[34] = (real_t)(0.0000000000000000e+00);
a[35] = (real_t)(0.0000000000000000e+00);
a[36] = (real_t)(0.0000000000000000e+00);
a[37] = ((real_t)(2.0000000000000000e+00)*xd[8]);
a[38] = (real_t)(0.0000000000000000e+00);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (real_t)(0.0000000000000000e+00);
a[42] = (real_t)(0.0000000000000000e+00);
a[43] = (real_t)(0.0000000000000000e+00);
a[44] = (real_t)(0.0000000000000000e+00);
a[45] = (real_t)(0.0000000000000000e+00);
a[46] = (real_t)(0.0000000000000000e+00);
a[47] = (real_t)(0.0000000000000000e+00);
a[48] = (real_t)(0.0000000000000000e+00);
a[49] = (real_t)(0.0000000000000000e+00);
a[50] = (real_t)(0.0000000000000000e+00);
a[51] = (real_t)(0.0000000000000000e+00);
a[52] = (real_t)(0.0000000000000000e+00);
a[53] = (real_t)(0.0000000000000000e+00);
a[54] = (real_t)(0.0000000000000000e+00);
a[55] = (real_t)(0.0000000000000000e+00);
a[56] = (real_t)(0.0000000000000000e+00);
a[57] = (real_t)(0.0000000000000000e+00);
a[58] = (real_t)(0.0000000000000000e+00);
a[59] = (real_t)(0.0000000000000000e+00);
a[60] = (real_t)(0.0000000000000000e+00);
a[61] = (real_t)(0.0000000000000000e+00);
a[62] = (real_t)(0.0000000000000000e+00);
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = ((xd[2]*xd[2])+(xd[3]*xd[3]));
out[1] = a[0];
out[2] = a[1];
out[3] = a[2];
out[4] = a[3];
out[5] = a[4];
out[6] = a[5];
out[7] = a[6];
out[8] = a[7];
out[9] = a[8];
out[10] = a[9];
out[11] = a[10];
out[12] = a[11];
out[13] = a[12];
out[14] = a[13];
out[15] = a[14];
out[16] = a[15];
out[17] = a[16];
out[18] = a[17];
out[19] = a[18];
out[20] = a[19];
out[21] = a[11];
out[22] = a[20];
out[23] = a[21];
out[24] = a[22];
out[25] = a[23];
out[26] = a[24];
out[27] = a[25];
out[28] = a[26];
out[29] = a[27];
out[30] = a[28];
out[31] = a[12];
out[32] = a[21];
out[33] = a[29];
out[34] = a[30];
out[35] = a[31];
out[36] = a[32];
out[37] = a[33];
out[38] = a[34];
out[39] = a[35];
out[40] = a[36];
out[41] = a[13];
out[42] = a[22];
out[43] = a[30];
out[44] = a[37];
out[45] = a[38];
out[46] = a[39];
out[47] = a[40];
out[48] = a[41];
out[49] = a[42];
out[50] = a[43];
out[51] = a[14];
out[52] = a[23];
out[53] = a[31];
out[54] = a[38];
out[55] = a[44];
out[56] = a[45];
out[57] = a[46];
out[58] = a[47];
out[59] = a[48];
out[60] = a[49];
out[61] = a[15];
out[62] = a[24];
out[63] = a[32];
out[64] = a[39];
out[65] = a[45];
out[66] = a[50];
out[67] = a[51];
out[68] = a[52];
out[69] = a[53];
out[70] = a[54];
out[71] = a[16];
out[72] = a[25];
out[73] = a[33];
out[74] = a[40];
out[75] = a[46];
out[76] = a[51];
out[77] = a[55];
out[78] = a[56];
out[79] = a[57];
out[80] = a[58];
out[81] = a[17];
out[82] = a[26];
out[83] = a[34];
out[84] = a[41];
out[85] = a[47];
out[86] = a[52];
out[87] = a[56];
out[88] = a[59];
out[89] = a[60];
out[90] = a[61];
out[91] = a[18];
out[92] = a[27];
out[93] = a[35];
out[94] = a[42];
out[95] = a[48];
out[96] = a[53];
out[97] = a[57];
out[98] = a[60];
out[99] = a[62];
out[100] = a[63];
out[101] = a[19];
out[102] = a[28];
out[103] = a[36];
out[104] = a[43];
out[105] = a[49];
out[106] = a[54];
out[107] = a[58];
out[108] = a[61];
out[109] = a[63];
out[110] = a[64];
}

void acado_LO_addObjEndTerm( real_t* const tmpFxxEnd, real_t* const tmpEH_N )
{
tmpEH_N[0] = tmpFxxEnd[0];
tmpEH_N[1] = tmpFxxEnd[1];
tmpEH_N[2] = tmpFxxEnd[2];
tmpEH_N[3] = tmpFxxEnd[3];
tmpEH_N[4] = tmpFxxEnd[4];
tmpEH_N[5] = tmpFxxEnd[5];
tmpEH_N[6] = tmpFxxEnd[6];
tmpEH_N[7] = tmpFxxEnd[7];
tmpEH_N[8] = tmpFxxEnd[8];
tmpEH_N[9] = tmpFxxEnd[9];
tmpEH_N[10] = tmpFxxEnd[10];
tmpEH_N[11] = tmpFxxEnd[11];
tmpEH_N[12] = tmpFxxEnd[12];
tmpEH_N[13] = tmpFxxEnd[13];
tmpEH_N[14] = tmpFxxEnd[14];
tmpEH_N[15] = tmpFxxEnd[15];
tmpEH_N[16] = tmpFxxEnd[16];
tmpEH_N[17] = tmpFxxEnd[17];
tmpEH_N[18] = tmpFxxEnd[18];
tmpEH_N[19] = tmpFxxEnd[19];
tmpEH_N[20] = tmpFxxEnd[20];
tmpEH_N[21] = tmpFxxEnd[21];
tmpEH_N[22] = tmpFxxEnd[22];
tmpEH_N[23] = tmpFxxEnd[23];
tmpEH_N[24] = tmpFxxEnd[24];
tmpEH_N[25] = tmpFxxEnd[25];
tmpEH_N[26] = tmpFxxEnd[26];
tmpEH_N[27] = tmpFxxEnd[27];
tmpEH_N[28] = tmpFxxEnd[28];
tmpEH_N[29] = tmpFxxEnd[29];
tmpEH_N[30] = tmpFxxEnd[30];
tmpEH_N[31] = tmpFxxEnd[31];
tmpEH_N[32] = tmpFxxEnd[32];
tmpEH_N[33] = tmpFxxEnd[33];
tmpEH_N[34] = tmpFxxEnd[34];
tmpEH_N[35] = tmpFxxEnd[35];
tmpEH_N[36] = tmpFxxEnd[36];
tmpEH_N[37] = tmpFxxEnd[37];
tmpEH_N[38] = tmpFxxEnd[38];
tmpEH_N[39] = tmpFxxEnd[39];
tmpEH_N[40] = tmpFxxEnd[40];
tmpEH_N[41] = tmpFxxEnd[41];
tmpEH_N[42] = tmpFxxEnd[42];
tmpEH_N[43] = tmpFxxEnd[43];
tmpEH_N[44] = tmpFxxEnd[44];
tmpEH_N[45] = tmpFxxEnd[45];
tmpEH_N[46] = tmpFxxEnd[46];
tmpEH_N[47] = tmpFxxEnd[47];
tmpEH_N[48] = tmpFxxEnd[48];
tmpEH_N[49] = tmpFxxEnd[49];
tmpEH_N[50] = tmpFxxEnd[50];
tmpEH_N[51] = tmpFxxEnd[51];
tmpEH_N[52] = tmpFxxEnd[52];
tmpEH_N[53] = tmpFxxEnd[53];
tmpEH_N[54] = tmpFxxEnd[54];
tmpEH_N[55] = tmpFxxEnd[55];
tmpEH_N[56] = tmpFxxEnd[56];
tmpEH_N[57] = tmpFxxEnd[57];
tmpEH_N[58] = tmpFxxEnd[58];
tmpEH_N[59] = tmpFxxEnd[59];
tmpEH_N[60] = tmpFxxEnd[60];
tmpEH_N[61] = tmpFxxEnd[61];
tmpEH_N[62] = tmpFxxEnd[62];
tmpEH_N[63] = tmpFxxEnd[63];
}

void acado_LO_evaluateObjective(  )
{
int lRun2;
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace_LO.g[runObj * 2] = 0.0000000000000000e+00;
acadoWorkspace_LO.g[runObj * 2 + 1] = 0.0000000000000000e+00;
acadoWorkspace_LO.QDy[runObj * 8] = 0.0000000000000000e+00;
acadoWorkspace_LO.QDy[runObj * 8 + 1] = 0.0000000000000000e+00;
acadoWorkspace_LO.QDy[runObj * 8 + 2] = 0.0000000000000000e+00;
acadoWorkspace_LO.QDy[runObj * 8 + 3] = 0.0000000000000000e+00;
acadoWorkspace_LO.QDy[runObj * 8 + 4] = 0.0000000000000000e+00;
acadoWorkspace_LO.QDy[runObj * 8 + 5] = 0.0000000000000000e+00;
acadoWorkspace_LO.QDy[runObj * 8 + 6] = 0.0000000000000000e+00;
acadoWorkspace_LO.QDy[runObj * 8 + 7] = 0.0000000000000000e+00;
}
acadoWorkspace_LO.objValueIn[0] = acadoVariables_LO.x[80];
acadoWorkspace_LO.objValueIn[1] = acadoVariables_LO.x[81];
acadoWorkspace_LO.objValueIn[2] = acadoVariables_LO.x[82];
acadoWorkspace_LO.objValueIn[3] = acadoVariables_LO.x[83];
acadoWorkspace_LO.objValueIn[4] = acadoVariables_LO.x[84];
acadoWorkspace_LO.objValueIn[5] = acadoVariables_LO.x[85];
acadoWorkspace_LO.objValueIn[6] = acadoVariables_LO.x[86];
acadoWorkspace_LO.objValueIn[7] = acadoVariables_LO.x[87];
acadoWorkspace_LO.objValueIn[8] = acadoVariables_LO.od[10];
acado_LO_evaluateMayer( acadoWorkspace_LO.objValueIn, acadoWorkspace_LO.objValueOut );

acado_LO_addObjEndTerm( &(acadoWorkspace_LO.objValueOut[ 9 ]), acadoWorkspace_LO.EH_N );
acadoWorkspace_LO.QDy[80] = acadoWorkspace_LO.objValueOut[1];
acadoWorkspace_LO.QDy[81] = acadoWorkspace_LO.objValueOut[2];
acadoWorkspace_LO.QDy[82] = acadoWorkspace_LO.objValueOut[3];
acadoWorkspace_LO.QDy[83] = acadoWorkspace_LO.objValueOut[4];
acadoWorkspace_LO.QDy[84] = acadoWorkspace_LO.objValueOut[5];
acadoWorkspace_LO.QDy[85] = acadoWorkspace_LO.objValueOut[6];
acadoWorkspace_LO.QDy[86] = acadoWorkspace_LO.objValueOut[7];
acadoWorkspace_LO.QDy[87] = acadoWorkspace_LO.objValueOut[8];

for (lRun2 = 0; lRun2 < 10; ++lRun2)
{
acadoWorkspace_LO.conValueIn[0] = acadoVariables_LO.x[lRun2 * 8];
acadoWorkspace_LO.conValueIn[1] = acadoVariables_LO.x[lRun2 * 8 + 1];
acadoWorkspace_LO.conValueIn[2] = acadoVariables_LO.x[lRun2 * 8 + 2];
acadoWorkspace_LO.conValueIn[3] = acadoVariables_LO.x[lRun2 * 8 + 3];
acadoWorkspace_LO.conValueIn[4] = acadoVariables_LO.x[lRun2 * 8 + 4];
acadoWorkspace_LO.conValueIn[5] = acadoVariables_LO.x[lRun2 * 8 + 5];
acadoWorkspace_LO.conValueIn[6] = acadoVariables_LO.x[lRun2 * 8 + 6];
acadoWorkspace_LO.conValueIn[7] = acadoVariables_LO.x[lRun2 * 8 + 7];
acadoWorkspace_LO.conValueIn[8] = acadoWorkspace_LO.y[lRun2 + 100];
acadoWorkspace_LO.conValueIn[9] = acadoVariables_LO.u[lRun2 * 2];
acadoWorkspace_LO.conValueIn[10] = acadoVariables_LO.u[lRun2 * 2 + 1];
acadoWorkspace_LO.conValueIn[11] = acadoVariables_LO.od[lRun2];
acado_LO_evaluatePathConstraints( acadoWorkspace_LO.conValueIn, acadoWorkspace_LO.conValueOut );
acadoWorkspace_LO.evH[lRun2] = acadoWorkspace_LO.conValueOut[0];

acadoWorkspace_LO.evHx[lRun2 * 8] = acadoWorkspace_LO.conValueOut[1];
acadoWorkspace_LO.evHx[lRun2 * 8 + 1] = acadoWorkspace_LO.conValueOut[2];
acadoWorkspace_LO.evHx[lRun2 * 8 + 2] = acadoWorkspace_LO.conValueOut[3];
acadoWorkspace_LO.evHx[lRun2 * 8 + 3] = acadoWorkspace_LO.conValueOut[4];
acadoWorkspace_LO.evHx[lRun2 * 8 + 4] = acadoWorkspace_LO.conValueOut[5];
acadoWorkspace_LO.evHx[lRun2 * 8 + 5] = acadoWorkspace_LO.conValueOut[6];
acadoWorkspace_LO.evHx[lRun2 * 8 + 6] = acadoWorkspace_LO.conValueOut[7];
acadoWorkspace_LO.evHx[lRun2 * 8 + 7] = acadoWorkspace_LO.conValueOut[8];
acadoWorkspace_LO.evHu[lRun2 * 2] = acadoWorkspace_LO.conValueOut[9];
acadoWorkspace_LO.evHu[lRun2 * 2 + 1] = acadoWorkspace_LO.conValueOut[10];
acadoWorkspace_LO.evDDH[0] = acadoWorkspace_LO.conValueOut[11];
acadoWorkspace_LO.evDDH[1] = acadoWorkspace_LO.conValueOut[12];
acadoWorkspace_LO.evDDH[2] = acadoWorkspace_LO.conValueOut[13];
acadoWorkspace_LO.evDDH[3] = acadoWorkspace_LO.conValueOut[14];
acadoWorkspace_LO.evDDH[4] = acadoWorkspace_LO.conValueOut[15];
acadoWorkspace_LO.evDDH[5] = acadoWorkspace_LO.conValueOut[16];
acadoWorkspace_LO.evDDH[6] = acadoWorkspace_LO.conValueOut[17];
acadoWorkspace_LO.evDDH[7] = acadoWorkspace_LO.conValueOut[18];
acadoWorkspace_LO.evDDH[8] = acadoWorkspace_LO.conValueOut[19];
acadoWorkspace_LO.evDDH[9] = acadoWorkspace_LO.conValueOut[20];
acadoWorkspace_LO.evDDH[10] = acadoWorkspace_LO.conValueOut[21];
acadoWorkspace_LO.evDDH[11] = acadoWorkspace_LO.conValueOut[22];
acadoWorkspace_LO.evDDH[12] = acadoWorkspace_LO.conValueOut[23];
acadoWorkspace_LO.evDDH[13] = acadoWorkspace_LO.conValueOut[24];
acadoWorkspace_LO.evDDH[14] = acadoWorkspace_LO.conValueOut[25];
acadoWorkspace_LO.evDDH[15] = acadoWorkspace_LO.conValueOut[26];
acadoWorkspace_LO.evDDH[16] = acadoWorkspace_LO.conValueOut[27];
acadoWorkspace_LO.evDDH[17] = acadoWorkspace_LO.conValueOut[28];
acadoWorkspace_LO.evDDH[18] = acadoWorkspace_LO.conValueOut[29];
acadoWorkspace_LO.evDDH[19] = acadoWorkspace_LO.conValueOut[30];
acadoWorkspace_LO.evDDH[20] = acadoWorkspace_LO.conValueOut[31];
acadoWorkspace_LO.evDDH[21] = acadoWorkspace_LO.conValueOut[32];
acadoWorkspace_LO.evDDH[22] = acadoWorkspace_LO.conValueOut[33];
acadoWorkspace_LO.evDDH[23] = acadoWorkspace_LO.conValueOut[34];
acadoWorkspace_LO.evDDH[24] = acadoWorkspace_LO.conValueOut[35];
acadoWorkspace_LO.evDDH[25] = acadoWorkspace_LO.conValueOut[36];
acadoWorkspace_LO.evDDH[26] = acadoWorkspace_LO.conValueOut[37];
acadoWorkspace_LO.evDDH[27] = acadoWorkspace_LO.conValueOut[38];
acadoWorkspace_LO.evDDH[28] = acadoWorkspace_LO.conValueOut[39];
acadoWorkspace_LO.evDDH[29] = acadoWorkspace_LO.conValueOut[40];
acadoWorkspace_LO.evDDH[30] = acadoWorkspace_LO.conValueOut[41];
acadoWorkspace_LO.evDDH[31] = acadoWorkspace_LO.conValueOut[42];
acadoWorkspace_LO.evDDH[32] = acadoWorkspace_LO.conValueOut[43];
acadoWorkspace_LO.evDDH[33] = acadoWorkspace_LO.conValueOut[44];
acadoWorkspace_LO.evDDH[34] = acadoWorkspace_LO.conValueOut[45];
acadoWorkspace_LO.evDDH[35] = acadoWorkspace_LO.conValueOut[46];
acadoWorkspace_LO.evDDH[36] = acadoWorkspace_LO.conValueOut[47];
acadoWorkspace_LO.evDDH[37] = acadoWorkspace_LO.conValueOut[48];
acadoWorkspace_LO.evDDH[38] = acadoWorkspace_LO.conValueOut[49];
acadoWorkspace_LO.evDDH[39] = acadoWorkspace_LO.conValueOut[50];
acadoWorkspace_LO.evDDH[40] = acadoWorkspace_LO.conValueOut[51];
acadoWorkspace_LO.evDDH[41] = acadoWorkspace_LO.conValueOut[52];
acadoWorkspace_LO.evDDH[42] = acadoWorkspace_LO.conValueOut[53];
acadoWorkspace_LO.evDDH[43] = acadoWorkspace_LO.conValueOut[54];
acadoWorkspace_LO.evDDH[44] = acadoWorkspace_LO.conValueOut[55];
acadoWorkspace_LO.evDDH[45] = acadoWorkspace_LO.conValueOut[56];
acadoWorkspace_LO.evDDH[46] = acadoWorkspace_LO.conValueOut[57];
acadoWorkspace_LO.evDDH[47] = acadoWorkspace_LO.conValueOut[58];
acadoWorkspace_LO.evDDH[48] = acadoWorkspace_LO.conValueOut[59];
acadoWorkspace_LO.evDDH[49] = acadoWorkspace_LO.conValueOut[60];
acadoWorkspace_LO.evDDH[50] = acadoWorkspace_LO.conValueOut[61];
acadoWorkspace_LO.evDDH[51] = acadoWorkspace_LO.conValueOut[62];
acadoWorkspace_LO.evDDH[52] = acadoWorkspace_LO.conValueOut[63];
acadoWorkspace_LO.evDDH[53] = acadoWorkspace_LO.conValueOut[64];
acadoWorkspace_LO.evDDH[54] = acadoWorkspace_LO.conValueOut[65];
acadoWorkspace_LO.evDDH[55] = acadoWorkspace_LO.conValueOut[66];
acadoWorkspace_LO.evDDH[56] = acadoWorkspace_LO.conValueOut[67];
acadoWorkspace_LO.evDDH[57] = acadoWorkspace_LO.conValueOut[68];
acadoWorkspace_LO.evDDH[58] = acadoWorkspace_LO.conValueOut[69];
acadoWorkspace_LO.evDDH[59] = acadoWorkspace_LO.conValueOut[70];
acadoWorkspace_LO.evDDH[60] = acadoWorkspace_LO.conValueOut[71];
acadoWorkspace_LO.evDDH[61] = acadoWorkspace_LO.conValueOut[72];
acadoWorkspace_LO.evDDH[62] = acadoWorkspace_LO.conValueOut[73];
acadoWorkspace_LO.evDDH[63] = acadoWorkspace_LO.conValueOut[74];
acadoWorkspace_LO.evDDH[64] = acadoWorkspace_LO.conValueOut[75];
acadoWorkspace_LO.evDDH[65] = acadoWorkspace_LO.conValueOut[76];
acadoWorkspace_LO.evDDH[66] = acadoWorkspace_LO.conValueOut[77];
acadoWorkspace_LO.evDDH[67] = acadoWorkspace_LO.conValueOut[78];
acadoWorkspace_LO.evDDH[68] = acadoWorkspace_LO.conValueOut[79];
acadoWorkspace_LO.evDDH[69] = acadoWorkspace_LO.conValueOut[80];
acadoWorkspace_LO.evDDH[70] = acadoWorkspace_LO.conValueOut[81];
acadoWorkspace_LO.evDDH[71] = acadoWorkspace_LO.conValueOut[82];
acadoWorkspace_LO.evDDH[72] = acadoWorkspace_LO.conValueOut[83];
acadoWorkspace_LO.evDDH[73] = acadoWorkspace_LO.conValueOut[84];
acadoWorkspace_LO.evDDH[74] = acadoWorkspace_LO.conValueOut[85];
acadoWorkspace_LO.evDDH[75] = acadoWorkspace_LO.conValueOut[86];
acadoWorkspace_LO.evDDH[76] = acadoWorkspace_LO.conValueOut[87];
acadoWorkspace_LO.evDDH[77] = acadoWorkspace_LO.conValueOut[88];
acadoWorkspace_LO.evDDH[78] = acadoWorkspace_LO.conValueOut[89];
acadoWorkspace_LO.evDDH[79] = acadoWorkspace_LO.conValueOut[90];
acadoWorkspace_LO.evDDH[80] = acadoWorkspace_LO.conValueOut[91];
acadoWorkspace_LO.evDDH[81] = acadoWorkspace_LO.conValueOut[92];
acadoWorkspace_LO.evDDH[82] = acadoWorkspace_LO.conValueOut[93];
acadoWorkspace_LO.evDDH[83] = acadoWorkspace_LO.conValueOut[94];
acadoWorkspace_LO.evDDH[84] = acadoWorkspace_LO.conValueOut[95];
acadoWorkspace_LO.evDDH[85] = acadoWorkspace_LO.conValueOut[96];
acadoWorkspace_LO.evDDH[86] = acadoWorkspace_LO.conValueOut[97];
acadoWorkspace_LO.evDDH[87] = acadoWorkspace_LO.conValueOut[98];
acadoWorkspace_LO.evDDH[88] = acadoWorkspace_LO.conValueOut[99];
acadoWorkspace_LO.evDDH[89] = acadoWorkspace_LO.conValueOut[100];
acadoWorkspace_LO.evDDH[90] = acadoWorkspace_LO.conValueOut[101];
acadoWorkspace_LO.evDDH[91] = acadoWorkspace_LO.conValueOut[102];
acadoWorkspace_LO.evDDH[92] = acadoWorkspace_LO.conValueOut[103];
acadoWorkspace_LO.evDDH[93] = acadoWorkspace_LO.conValueOut[104];
acadoWorkspace_LO.evDDH[94] = acadoWorkspace_LO.conValueOut[105];
acadoWorkspace_LO.evDDH[95] = acadoWorkspace_LO.conValueOut[106];
acadoWorkspace_LO.evDDH[96] = acadoWorkspace_LO.conValueOut[107];
acadoWorkspace_LO.evDDH[97] = acadoWorkspace_LO.conValueOut[108];
acadoWorkspace_LO.evDDH[98] = acadoWorkspace_LO.conValueOut[109];
acadoWorkspace_LO.evDDH[99] = acadoWorkspace_LO.conValueOut[110];
acadoWorkspace_LO.EH[lRun2 * 100] += acadoWorkspace_LO.evDDH[0];
acadoWorkspace_LO.EH[lRun2 * 100 + 1] += acadoWorkspace_LO.evDDH[1];
acadoWorkspace_LO.EH[lRun2 * 100 + 2] += acadoWorkspace_LO.evDDH[2];
acadoWorkspace_LO.EH[lRun2 * 100 + 3] += acadoWorkspace_LO.evDDH[3];
acadoWorkspace_LO.EH[lRun2 * 100 + 4] += acadoWorkspace_LO.evDDH[4];
acadoWorkspace_LO.EH[lRun2 * 100 + 5] += acadoWorkspace_LO.evDDH[5];
acadoWorkspace_LO.EH[lRun2 * 100 + 6] += acadoWorkspace_LO.evDDH[6];
acadoWorkspace_LO.EH[lRun2 * 100 + 7] += acadoWorkspace_LO.evDDH[7];
acadoWorkspace_LO.EH[lRun2 * 100 + 8] += acadoWorkspace_LO.evDDH[8];
acadoWorkspace_LO.EH[lRun2 * 100 + 9] += acadoWorkspace_LO.evDDH[9];
acadoWorkspace_LO.EH[lRun2 * 100 + 10] += acadoWorkspace_LO.evDDH[10];
acadoWorkspace_LO.EH[lRun2 * 100 + 11] += acadoWorkspace_LO.evDDH[11];
acadoWorkspace_LO.EH[lRun2 * 100 + 12] += acadoWorkspace_LO.evDDH[12];
acadoWorkspace_LO.EH[lRun2 * 100 + 13] += acadoWorkspace_LO.evDDH[13];
acadoWorkspace_LO.EH[lRun2 * 100 + 14] += acadoWorkspace_LO.evDDH[14];
acadoWorkspace_LO.EH[lRun2 * 100 + 15] += acadoWorkspace_LO.evDDH[15];
acadoWorkspace_LO.EH[lRun2 * 100 + 16] += acadoWorkspace_LO.evDDH[16];
acadoWorkspace_LO.EH[lRun2 * 100 + 17] += acadoWorkspace_LO.evDDH[17];
acadoWorkspace_LO.EH[lRun2 * 100 + 18] += acadoWorkspace_LO.evDDH[18];
acadoWorkspace_LO.EH[lRun2 * 100 + 19] += acadoWorkspace_LO.evDDH[19];
acadoWorkspace_LO.EH[lRun2 * 100 + 20] += acadoWorkspace_LO.evDDH[20];
acadoWorkspace_LO.EH[lRun2 * 100 + 21] += acadoWorkspace_LO.evDDH[21];
acadoWorkspace_LO.EH[lRun2 * 100 + 22] += acadoWorkspace_LO.evDDH[22];
acadoWorkspace_LO.EH[lRun2 * 100 + 23] += acadoWorkspace_LO.evDDH[23];
acadoWorkspace_LO.EH[lRun2 * 100 + 24] += acadoWorkspace_LO.evDDH[24];
acadoWorkspace_LO.EH[lRun2 * 100 + 25] += acadoWorkspace_LO.evDDH[25];
acadoWorkspace_LO.EH[lRun2 * 100 + 26] += acadoWorkspace_LO.evDDH[26];
acadoWorkspace_LO.EH[lRun2 * 100 + 27] += acadoWorkspace_LO.evDDH[27];
acadoWorkspace_LO.EH[lRun2 * 100 + 28] += acadoWorkspace_LO.evDDH[28];
acadoWorkspace_LO.EH[lRun2 * 100 + 29] += acadoWorkspace_LO.evDDH[29];
acadoWorkspace_LO.EH[lRun2 * 100 + 30] += acadoWorkspace_LO.evDDH[30];
acadoWorkspace_LO.EH[lRun2 * 100 + 31] += acadoWorkspace_LO.evDDH[31];
acadoWorkspace_LO.EH[lRun2 * 100 + 32] += acadoWorkspace_LO.evDDH[32];
acadoWorkspace_LO.EH[lRun2 * 100 + 33] += acadoWorkspace_LO.evDDH[33];
acadoWorkspace_LO.EH[lRun2 * 100 + 34] += acadoWorkspace_LO.evDDH[34];
acadoWorkspace_LO.EH[lRun2 * 100 + 35] += acadoWorkspace_LO.evDDH[35];
acadoWorkspace_LO.EH[lRun2 * 100 + 36] += acadoWorkspace_LO.evDDH[36];
acadoWorkspace_LO.EH[lRun2 * 100 + 37] += acadoWorkspace_LO.evDDH[37];
acadoWorkspace_LO.EH[lRun2 * 100 + 38] += acadoWorkspace_LO.evDDH[38];
acadoWorkspace_LO.EH[lRun2 * 100 + 39] += acadoWorkspace_LO.evDDH[39];
acadoWorkspace_LO.EH[lRun2 * 100 + 40] += acadoWorkspace_LO.evDDH[40];
acadoWorkspace_LO.EH[lRun2 * 100 + 41] += acadoWorkspace_LO.evDDH[41];
acadoWorkspace_LO.EH[lRun2 * 100 + 42] += acadoWorkspace_LO.evDDH[42];
acadoWorkspace_LO.EH[lRun2 * 100 + 43] += acadoWorkspace_LO.evDDH[43];
acadoWorkspace_LO.EH[lRun2 * 100 + 44] += acadoWorkspace_LO.evDDH[44];
acadoWorkspace_LO.EH[lRun2 * 100 + 45] += acadoWorkspace_LO.evDDH[45];
acadoWorkspace_LO.EH[lRun2 * 100 + 46] += acadoWorkspace_LO.evDDH[46];
acadoWorkspace_LO.EH[lRun2 * 100 + 47] += acadoWorkspace_LO.evDDH[47];
acadoWorkspace_LO.EH[lRun2 * 100 + 48] += acadoWorkspace_LO.evDDH[48];
acadoWorkspace_LO.EH[lRun2 * 100 + 49] += acadoWorkspace_LO.evDDH[49];
acadoWorkspace_LO.EH[lRun2 * 100 + 50] += acadoWorkspace_LO.evDDH[50];
acadoWorkspace_LO.EH[lRun2 * 100 + 51] += acadoWorkspace_LO.evDDH[51];
acadoWorkspace_LO.EH[lRun2 * 100 + 52] += acadoWorkspace_LO.evDDH[52];
acadoWorkspace_LO.EH[lRun2 * 100 + 53] += acadoWorkspace_LO.evDDH[53];
acadoWorkspace_LO.EH[lRun2 * 100 + 54] += acadoWorkspace_LO.evDDH[54];
acadoWorkspace_LO.EH[lRun2 * 100 + 55] += acadoWorkspace_LO.evDDH[55];
acadoWorkspace_LO.EH[lRun2 * 100 + 56] += acadoWorkspace_LO.evDDH[56];
acadoWorkspace_LO.EH[lRun2 * 100 + 57] += acadoWorkspace_LO.evDDH[57];
acadoWorkspace_LO.EH[lRun2 * 100 + 58] += acadoWorkspace_LO.evDDH[58];
acadoWorkspace_LO.EH[lRun2 * 100 + 59] += acadoWorkspace_LO.evDDH[59];
acadoWorkspace_LO.EH[lRun2 * 100 + 60] += acadoWorkspace_LO.evDDH[60];
acadoWorkspace_LO.EH[lRun2 * 100 + 61] += acadoWorkspace_LO.evDDH[61];
acadoWorkspace_LO.EH[lRun2 * 100 + 62] += acadoWorkspace_LO.evDDH[62];
acadoWorkspace_LO.EH[lRun2 * 100 + 63] += acadoWorkspace_LO.evDDH[63];
acadoWorkspace_LO.EH[lRun2 * 100 + 64] += acadoWorkspace_LO.evDDH[64];
acadoWorkspace_LO.EH[lRun2 * 100 + 65] += acadoWorkspace_LO.evDDH[65];
acadoWorkspace_LO.EH[lRun2 * 100 + 66] += acadoWorkspace_LO.evDDH[66];
acadoWorkspace_LO.EH[lRun2 * 100 + 67] += acadoWorkspace_LO.evDDH[67];
acadoWorkspace_LO.EH[lRun2 * 100 + 68] += acadoWorkspace_LO.evDDH[68];
acadoWorkspace_LO.EH[lRun2 * 100 + 69] += acadoWorkspace_LO.evDDH[69];
acadoWorkspace_LO.EH[lRun2 * 100 + 70] += acadoWorkspace_LO.evDDH[70];
acadoWorkspace_LO.EH[lRun2 * 100 + 71] += acadoWorkspace_LO.evDDH[71];
acadoWorkspace_LO.EH[lRun2 * 100 + 72] += acadoWorkspace_LO.evDDH[72];
acadoWorkspace_LO.EH[lRun2 * 100 + 73] += acadoWorkspace_LO.evDDH[73];
acadoWorkspace_LO.EH[lRun2 * 100 + 74] += acadoWorkspace_LO.evDDH[74];
acadoWorkspace_LO.EH[lRun2 * 100 + 75] += acadoWorkspace_LO.evDDH[75];
acadoWorkspace_LO.EH[lRun2 * 100 + 76] += acadoWorkspace_LO.evDDH[76];
acadoWorkspace_LO.EH[lRun2 * 100 + 77] += acadoWorkspace_LO.evDDH[77];
acadoWorkspace_LO.EH[lRun2 * 100 + 78] += acadoWorkspace_LO.evDDH[78];
acadoWorkspace_LO.EH[lRun2 * 100 + 79] += acadoWorkspace_LO.evDDH[79];
acadoWorkspace_LO.EH[lRun2 * 100 + 80] += acadoWorkspace_LO.evDDH[80];
acadoWorkspace_LO.EH[lRun2 * 100 + 81] += acadoWorkspace_LO.evDDH[81];
acadoWorkspace_LO.EH[lRun2 * 100 + 82] += acadoWorkspace_LO.evDDH[82];
acadoWorkspace_LO.EH[lRun2 * 100 + 83] += acadoWorkspace_LO.evDDH[83];
acadoWorkspace_LO.EH[lRun2 * 100 + 84] += acadoWorkspace_LO.evDDH[84];
acadoWorkspace_LO.EH[lRun2 * 100 + 85] += acadoWorkspace_LO.evDDH[85];
acadoWorkspace_LO.EH[lRun2 * 100 + 86] += acadoWorkspace_LO.evDDH[86];
acadoWorkspace_LO.EH[lRun2 * 100 + 87] += acadoWorkspace_LO.evDDH[87];
acadoWorkspace_LO.EH[lRun2 * 100 + 88] += acadoWorkspace_LO.evDDH[88];
acadoWorkspace_LO.EH[lRun2 * 100 + 89] += acadoWorkspace_LO.evDDH[89];
acadoWorkspace_LO.EH[lRun2 * 100 + 90] += acadoWorkspace_LO.evDDH[90];
acadoWorkspace_LO.EH[lRun2 * 100 + 91] += acadoWorkspace_LO.evDDH[91];
acadoWorkspace_LO.EH[lRun2 * 100 + 92] += acadoWorkspace_LO.evDDH[92];
acadoWorkspace_LO.EH[lRun2 * 100 + 93] += acadoWorkspace_LO.evDDH[93];
acadoWorkspace_LO.EH[lRun2 * 100 + 94] += acadoWorkspace_LO.evDDH[94];
acadoWorkspace_LO.EH[lRun2 * 100 + 95] += acadoWorkspace_LO.evDDH[95];
acadoWorkspace_LO.EH[lRun2 * 100 + 96] += acadoWorkspace_LO.evDDH[96];
acadoWorkspace_LO.EH[lRun2 * 100 + 97] += acadoWorkspace_LO.evDDH[97];
acadoWorkspace_LO.EH[lRun2 * 100 + 98] += acadoWorkspace_LO.evDDH[98];
acadoWorkspace_LO.EH[lRun2 * 100 + 99] += acadoWorkspace_LO.evDDH[99];
}

}

void acado_LO_regularizeHessian(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acado_LO_regularize( &(acadoWorkspace_LO.EH[ lRun1 * 100 ]) );
acadoWorkspace_LO.Q1[lRun1 * 64] = acadoWorkspace_LO.EH[lRun1 * 100];
acadoWorkspace_LO.Q1[lRun1 * 64 + 1] = acadoWorkspace_LO.EH[lRun1 * 100 + 1];
acadoWorkspace_LO.Q1[lRun1 * 64 + 2] = acadoWorkspace_LO.EH[lRun1 * 100 + 2];
acadoWorkspace_LO.Q1[lRun1 * 64 + 3] = acadoWorkspace_LO.EH[lRun1 * 100 + 3];
acadoWorkspace_LO.Q1[lRun1 * 64 + 4] = acadoWorkspace_LO.EH[lRun1 * 100 + 4];
acadoWorkspace_LO.Q1[lRun1 * 64 + 5] = acadoWorkspace_LO.EH[lRun1 * 100 + 5];
acadoWorkspace_LO.Q1[lRun1 * 64 + 6] = acadoWorkspace_LO.EH[lRun1 * 100 + 6];
acadoWorkspace_LO.Q1[lRun1 * 64 + 7] = acadoWorkspace_LO.EH[lRun1 * 100 + 7];
acadoWorkspace_LO.Q1[lRun1 * 64 + 8] = acadoWorkspace_LO.EH[lRun1 * 100 + 10];
acadoWorkspace_LO.Q1[lRun1 * 64 + 9] = acadoWorkspace_LO.EH[lRun1 * 100 + 11];
acadoWorkspace_LO.Q1[lRun1 * 64 + 10] = acadoWorkspace_LO.EH[lRun1 * 100 + 12];
acadoWorkspace_LO.Q1[lRun1 * 64 + 11] = acadoWorkspace_LO.EH[lRun1 * 100 + 13];
acadoWorkspace_LO.Q1[lRun1 * 64 + 12] = acadoWorkspace_LO.EH[lRun1 * 100 + 14];
acadoWorkspace_LO.Q1[lRun1 * 64 + 13] = acadoWorkspace_LO.EH[lRun1 * 100 + 15];
acadoWorkspace_LO.Q1[lRun1 * 64 + 14] = acadoWorkspace_LO.EH[lRun1 * 100 + 16];
acadoWorkspace_LO.Q1[lRun1 * 64 + 15] = acadoWorkspace_LO.EH[lRun1 * 100 + 17];
acadoWorkspace_LO.Q1[lRun1 * 64 + 16] = acadoWorkspace_LO.EH[lRun1 * 100 + 20];
acadoWorkspace_LO.Q1[lRun1 * 64 + 17] = acadoWorkspace_LO.EH[lRun1 * 100 + 21];
acadoWorkspace_LO.Q1[lRun1 * 64 + 18] = acadoWorkspace_LO.EH[lRun1 * 100 + 22];
acadoWorkspace_LO.Q1[lRun1 * 64 + 19] = acadoWorkspace_LO.EH[lRun1 * 100 + 23];
acadoWorkspace_LO.Q1[lRun1 * 64 + 20] = acadoWorkspace_LO.EH[lRun1 * 100 + 24];
acadoWorkspace_LO.Q1[lRun1 * 64 + 21] = acadoWorkspace_LO.EH[lRun1 * 100 + 25];
acadoWorkspace_LO.Q1[lRun1 * 64 + 22] = acadoWorkspace_LO.EH[lRun1 * 100 + 26];
acadoWorkspace_LO.Q1[lRun1 * 64 + 23] = acadoWorkspace_LO.EH[lRun1 * 100 + 27];
acadoWorkspace_LO.Q1[lRun1 * 64 + 24] = acadoWorkspace_LO.EH[lRun1 * 100 + 30];
acadoWorkspace_LO.Q1[lRun1 * 64 + 25] = acadoWorkspace_LO.EH[lRun1 * 100 + 31];
acadoWorkspace_LO.Q1[lRun1 * 64 + 26] = acadoWorkspace_LO.EH[lRun1 * 100 + 32];
acadoWorkspace_LO.Q1[lRun1 * 64 + 27] = acadoWorkspace_LO.EH[lRun1 * 100 + 33];
acadoWorkspace_LO.Q1[lRun1 * 64 + 28] = acadoWorkspace_LO.EH[lRun1 * 100 + 34];
acadoWorkspace_LO.Q1[lRun1 * 64 + 29] = acadoWorkspace_LO.EH[lRun1 * 100 + 35];
acadoWorkspace_LO.Q1[lRun1 * 64 + 30] = acadoWorkspace_LO.EH[lRun1 * 100 + 36];
acadoWorkspace_LO.Q1[lRun1 * 64 + 31] = acadoWorkspace_LO.EH[lRun1 * 100 + 37];
acadoWorkspace_LO.Q1[lRun1 * 64 + 32] = acadoWorkspace_LO.EH[lRun1 * 100 + 40];
acadoWorkspace_LO.Q1[lRun1 * 64 + 33] = acadoWorkspace_LO.EH[lRun1 * 100 + 41];
acadoWorkspace_LO.Q1[lRun1 * 64 + 34] = acadoWorkspace_LO.EH[lRun1 * 100 + 42];
acadoWorkspace_LO.Q1[lRun1 * 64 + 35] = acadoWorkspace_LO.EH[lRun1 * 100 + 43];
acadoWorkspace_LO.Q1[lRun1 * 64 + 36] = acadoWorkspace_LO.EH[lRun1 * 100 + 44];
acadoWorkspace_LO.Q1[lRun1 * 64 + 37] = acadoWorkspace_LO.EH[lRun1 * 100 + 45];
acadoWorkspace_LO.Q1[lRun1 * 64 + 38] = acadoWorkspace_LO.EH[lRun1 * 100 + 46];
acadoWorkspace_LO.Q1[lRun1 * 64 + 39] = acadoWorkspace_LO.EH[lRun1 * 100 + 47];
acadoWorkspace_LO.Q1[lRun1 * 64 + 40] = acadoWorkspace_LO.EH[lRun1 * 100 + 50];
acadoWorkspace_LO.Q1[lRun1 * 64 + 41] = acadoWorkspace_LO.EH[lRun1 * 100 + 51];
acadoWorkspace_LO.Q1[lRun1 * 64 + 42] = acadoWorkspace_LO.EH[lRun1 * 100 + 52];
acadoWorkspace_LO.Q1[lRun1 * 64 + 43] = acadoWorkspace_LO.EH[lRun1 * 100 + 53];
acadoWorkspace_LO.Q1[lRun1 * 64 + 44] = acadoWorkspace_LO.EH[lRun1 * 100 + 54];
acadoWorkspace_LO.Q1[lRun1 * 64 + 45] = acadoWorkspace_LO.EH[lRun1 * 100 + 55];
acadoWorkspace_LO.Q1[lRun1 * 64 + 46] = acadoWorkspace_LO.EH[lRun1 * 100 + 56];
acadoWorkspace_LO.Q1[lRun1 * 64 + 47] = acadoWorkspace_LO.EH[lRun1 * 100 + 57];
acadoWorkspace_LO.Q1[lRun1 * 64 + 48] = acadoWorkspace_LO.EH[lRun1 * 100 + 60];
acadoWorkspace_LO.Q1[lRun1 * 64 + 49] = acadoWorkspace_LO.EH[lRun1 * 100 + 61];
acadoWorkspace_LO.Q1[lRun1 * 64 + 50] = acadoWorkspace_LO.EH[lRun1 * 100 + 62];
acadoWorkspace_LO.Q1[lRun1 * 64 + 51] = acadoWorkspace_LO.EH[lRun1 * 100 + 63];
acadoWorkspace_LO.Q1[lRun1 * 64 + 52] = acadoWorkspace_LO.EH[lRun1 * 100 + 64];
acadoWorkspace_LO.Q1[lRun1 * 64 + 53] = acadoWorkspace_LO.EH[lRun1 * 100 + 65];
acadoWorkspace_LO.Q1[lRun1 * 64 + 54] = acadoWorkspace_LO.EH[lRun1 * 100 + 66];
acadoWorkspace_LO.Q1[lRun1 * 64 + 55] = acadoWorkspace_LO.EH[lRun1 * 100 + 67];
acadoWorkspace_LO.Q1[lRun1 * 64 + 56] = acadoWorkspace_LO.EH[lRun1 * 100 + 70];
acadoWorkspace_LO.Q1[lRun1 * 64 + 57] = acadoWorkspace_LO.EH[lRun1 * 100 + 71];
acadoWorkspace_LO.Q1[lRun1 * 64 + 58] = acadoWorkspace_LO.EH[lRun1 * 100 + 72];
acadoWorkspace_LO.Q1[lRun1 * 64 + 59] = acadoWorkspace_LO.EH[lRun1 * 100 + 73];
acadoWorkspace_LO.Q1[lRun1 * 64 + 60] = acadoWorkspace_LO.EH[lRun1 * 100 + 74];
acadoWorkspace_LO.Q1[lRun1 * 64 + 61] = acadoWorkspace_LO.EH[lRun1 * 100 + 75];
acadoWorkspace_LO.Q1[lRun1 * 64 + 62] = acadoWorkspace_LO.EH[lRun1 * 100 + 76];
acadoWorkspace_LO.Q1[lRun1 * 64 + 63] = acadoWorkspace_LO.EH[lRun1 * 100 + 77];
acadoWorkspace_LO.S1[lRun1 * 16] = acadoWorkspace_LO.EH[lRun1 * 100 + 8];
acadoWorkspace_LO.S1[lRun1 * 16 + 1] = acadoWorkspace_LO.EH[lRun1 * 100 + 9];
acadoWorkspace_LO.S1[lRun1 * 16 + 2] = acadoWorkspace_LO.EH[lRun1 * 100 + 18];
acadoWorkspace_LO.S1[lRun1 * 16 + 3] = acadoWorkspace_LO.EH[lRun1 * 100 + 19];
acadoWorkspace_LO.S1[lRun1 * 16 + 4] = acadoWorkspace_LO.EH[lRun1 * 100 + 28];
acadoWorkspace_LO.S1[lRun1 * 16 + 5] = acadoWorkspace_LO.EH[lRun1 * 100 + 29];
acadoWorkspace_LO.S1[lRun1 * 16 + 6] = acadoWorkspace_LO.EH[lRun1 * 100 + 38];
acadoWorkspace_LO.S1[lRun1 * 16 + 7] = acadoWorkspace_LO.EH[lRun1 * 100 + 39];
acadoWorkspace_LO.S1[lRun1 * 16 + 8] = acadoWorkspace_LO.EH[lRun1 * 100 + 48];
acadoWorkspace_LO.S1[lRun1 * 16 + 9] = acadoWorkspace_LO.EH[lRun1 * 100 + 49];
acadoWorkspace_LO.S1[lRun1 * 16 + 10] = acadoWorkspace_LO.EH[lRun1 * 100 + 58];
acadoWorkspace_LO.S1[lRun1 * 16 + 11] = acadoWorkspace_LO.EH[lRun1 * 100 + 59];
acadoWorkspace_LO.S1[lRun1 * 16 + 12] = acadoWorkspace_LO.EH[lRun1 * 100 + 68];
acadoWorkspace_LO.S1[lRun1 * 16 + 13] = acadoWorkspace_LO.EH[lRun1 * 100 + 69];
acadoWorkspace_LO.S1[lRun1 * 16 + 14] = acadoWorkspace_LO.EH[lRun1 * 100 + 78];
acadoWorkspace_LO.S1[lRun1 * 16 + 15] = acadoWorkspace_LO.EH[lRun1 * 100 + 79];
acadoWorkspace_LO.R1[lRun1 * 4] = acadoWorkspace_LO.EH[lRun1 * 100 + 88];
acadoWorkspace_LO.R1[lRun1 * 4 + 1] = acadoWorkspace_LO.EH[lRun1 * 100 + 89];
acadoWorkspace_LO.R1[lRun1 * 4 + 2] = acadoWorkspace_LO.EH[lRun1 * 100 + 98];
acadoWorkspace_LO.R1[lRun1 * 4 + 3] = acadoWorkspace_LO.EH[lRun1 * 100 + 99];
}
acadoWorkspace_LO.QN1[0] = acadoWorkspace_LO.EH_N[0];
acadoWorkspace_LO.QN1[1] = acadoWorkspace_LO.EH_N[1];
acadoWorkspace_LO.QN1[2] = acadoWorkspace_LO.EH_N[2];
acadoWorkspace_LO.QN1[3] = acadoWorkspace_LO.EH_N[3];
acadoWorkspace_LO.QN1[4] = acadoWorkspace_LO.EH_N[4];
acadoWorkspace_LO.QN1[5] = acadoWorkspace_LO.EH_N[5];
acadoWorkspace_LO.QN1[6] = acadoWorkspace_LO.EH_N[6];
acadoWorkspace_LO.QN1[7] = acadoWorkspace_LO.EH_N[7];
acadoWorkspace_LO.QN1[8] = acadoWorkspace_LO.EH_N[8];
acadoWorkspace_LO.QN1[9] = acadoWorkspace_LO.EH_N[9];
acadoWorkspace_LO.QN1[10] = acadoWorkspace_LO.EH_N[10];
acadoWorkspace_LO.QN1[11] = acadoWorkspace_LO.EH_N[11];
acadoWorkspace_LO.QN1[12] = acadoWorkspace_LO.EH_N[12];
acadoWorkspace_LO.QN1[13] = acadoWorkspace_LO.EH_N[13];
acadoWorkspace_LO.QN1[14] = acadoWorkspace_LO.EH_N[14];
acadoWorkspace_LO.QN1[15] = acadoWorkspace_LO.EH_N[15];
acadoWorkspace_LO.QN1[16] = acadoWorkspace_LO.EH_N[16];
acadoWorkspace_LO.QN1[17] = acadoWorkspace_LO.EH_N[17];
acadoWorkspace_LO.QN1[18] = acadoWorkspace_LO.EH_N[18];
acadoWorkspace_LO.QN1[19] = acadoWorkspace_LO.EH_N[19];
acadoWorkspace_LO.QN1[20] = acadoWorkspace_LO.EH_N[20];
acadoWorkspace_LO.QN1[21] = acadoWorkspace_LO.EH_N[21];
acadoWorkspace_LO.QN1[22] = acadoWorkspace_LO.EH_N[22];
acadoWorkspace_LO.QN1[23] = acadoWorkspace_LO.EH_N[23];
acadoWorkspace_LO.QN1[24] = acadoWorkspace_LO.EH_N[24];
acadoWorkspace_LO.QN1[25] = acadoWorkspace_LO.EH_N[25];
acadoWorkspace_LO.QN1[26] = acadoWorkspace_LO.EH_N[26];
acadoWorkspace_LO.QN1[27] = acadoWorkspace_LO.EH_N[27];
acadoWorkspace_LO.QN1[28] = acadoWorkspace_LO.EH_N[28];
acadoWorkspace_LO.QN1[29] = acadoWorkspace_LO.EH_N[29];
acadoWorkspace_LO.QN1[30] = acadoWorkspace_LO.EH_N[30];
acadoWorkspace_LO.QN1[31] = acadoWorkspace_LO.EH_N[31];
acadoWorkspace_LO.QN1[32] = acadoWorkspace_LO.EH_N[32];
acadoWorkspace_LO.QN1[33] = acadoWorkspace_LO.EH_N[33];
acadoWorkspace_LO.QN1[34] = acadoWorkspace_LO.EH_N[34];
acadoWorkspace_LO.QN1[35] = acadoWorkspace_LO.EH_N[35];
acadoWorkspace_LO.QN1[36] = acadoWorkspace_LO.EH_N[36];
acadoWorkspace_LO.QN1[37] = acadoWorkspace_LO.EH_N[37];
acadoWorkspace_LO.QN1[38] = acadoWorkspace_LO.EH_N[38];
acadoWorkspace_LO.QN1[39] = acadoWorkspace_LO.EH_N[39];
acadoWorkspace_LO.QN1[40] = acadoWorkspace_LO.EH_N[40];
acadoWorkspace_LO.QN1[41] = acadoWorkspace_LO.EH_N[41];
acadoWorkspace_LO.QN1[42] = acadoWorkspace_LO.EH_N[42];
acadoWorkspace_LO.QN1[43] = acadoWorkspace_LO.EH_N[43];
acadoWorkspace_LO.QN1[44] = acadoWorkspace_LO.EH_N[44];
acadoWorkspace_LO.QN1[45] = acadoWorkspace_LO.EH_N[45];
acadoWorkspace_LO.QN1[46] = acadoWorkspace_LO.EH_N[46];
acadoWorkspace_LO.QN1[47] = acadoWorkspace_LO.EH_N[47];
acadoWorkspace_LO.QN1[48] = acadoWorkspace_LO.EH_N[48];
acadoWorkspace_LO.QN1[49] = acadoWorkspace_LO.EH_N[49];
acadoWorkspace_LO.QN1[50] = acadoWorkspace_LO.EH_N[50];
acadoWorkspace_LO.QN1[51] = acadoWorkspace_LO.EH_N[51];
acadoWorkspace_LO.QN1[52] = acadoWorkspace_LO.EH_N[52];
acadoWorkspace_LO.QN1[53] = acadoWorkspace_LO.EH_N[53];
acadoWorkspace_LO.QN1[54] = acadoWorkspace_LO.EH_N[54];
acadoWorkspace_LO.QN1[55] = acadoWorkspace_LO.EH_N[55];
acadoWorkspace_LO.QN1[56] = acadoWorkspace_LO.EH_N[56];
acadoWorkspace_LO.QN1[57] = acadoWorkspace_LO.EH_N[57];
acadoWorkspace_LO.QN1[58] = acadoWorkspace_LO.EH_N[58];
acadoWorkspace_LO.QN1[59] = acadoWorkspace_LO.EH_N[59];
acadoWorkspace_LO.QN1[60] = acadoWorkspace_LO.EH_N[60];
acadoWorkspace_LO.QN1[61] = acadoWorkspace_LO.EH_N[61];
acadoWorkspace_LO.QN1[62] = acadoWorkspace_LO.EH_N[62];
acadoWorkspace_LO.QN1[63] = acadoWorkspace_LO.EH_N[63];
}

void acado_LO_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
Gx2[36] = Gx1[36];
Gx2[37] = Gx1[37];
Gx2[38] = Gx1[38];
Gx2[39] = Gx1[39];
Gx2[40] = Gx1[40];
Gx2[41] = Gx1[41];
Gx2[42] = Gx1[42];
Gx2[43] = Gx1[43];
Gx2[44] = Gx1[44];
Gx2[45] = Gx1[45];
Gx2[46] = Gx1[46];
Gx2[47] = Gx1[47];
Gx2[48] = Gx1[48];
Gx2[49] = Gx1[49];
Gx2[50] = Gx1[50];
Gx2[51] = Gx1[51];
Gx2[52] = Gx1[52];
Gx2[53] = Gx1[53];
Gx2[54] = Gx1[54];
Gx2[55] = Gx1[55];
Gx2[56] = Gx1[56];
Gx2[57] = Gx1[57];
Gx2[58] = Gx1[58];
Gx2[59] = Gx1[59];
Gx2[60] = Gx1[60];
Gx2[61] = Gx1[61];
Gx2[62] = Gx1[62];
Gx2[63] = Gx1[63];
}

void acado_LO_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[24] + Gx1[4]*Gx2[32] + Gx1[5]*Gx2[40] + Gx1[6]*Gx2[48] + Gx1[7]*Gx2[56];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[25] + Gx1[4]*Gx2[33] + Gx1[5]*Gx2[41] + Gx1[6]*Gx2[49] + Gx1[7]*Gx2[57];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[18] + Gx1[3]*Gx2[26] + Gx1[4]*Gx2[34] + Gx1[5]*Gx2[42] + Gx1[6]*Gx2[50] + Gx1[7]*Gx2[58];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[19] + Gx1[3]*Gx2[27] + Gx1[4]*Gx2[35] + Gx1[5]*Gx2[43] + Gx1[6]*Gx2[51] + Gx1[7]*Gx2[59];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[20] + Gx1[3]*Gx2[28] + Gx1[4]*Gx2[36] + Gx1[5]*Gx2[44] + Gx1[6]*Gx2[52] + Gx1[7]*Gx2[60];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[21] + Gx1[3]*Gx2[29] + Gx1[4]*Gx2[37] + Gx1[5]*Gx2[45] + Gx1[6]*Gx2[53] + Gx1[7]*Gx2[61];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[22] + Gx1[3]*Gx2[30] + Gx1[4]*Gx2[38] + Gx1[5]*Gx2[46] + Gx1[6]*Gx2[54] + Gx1[7]*Gx2[62];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[23] + Gx1[3]*Gx2[31] + Gx1[4]*Gx2[39] + Gx1[5]*Gx2[47] + Gx1[6]*Gx2[55] + Gx1[7]*Gx2[63];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[8] + Gx1[10]*Gx2[16] + Gx1[11]*Gx2[24] + Gx1[12]*Gx2[32] + Gx1[13]*Gx2[40] + Gx1[14]*Gx2[48] + Gx1[15]*Gx2[56];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[9] + Gx1[10]*Gx2[17] + Gx1[11]*Gx2[25] + Gx1[12]*Gx2[33] + Gx1[13]*Gx2[41] + Gx1[14]*Gx2[49] + Gx1[15]*Gx2[57];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[10] + Gx1[10]*Gx2[18] + Gx1[11]*Gx2[26] + Gx1[12]*Gx2[34] + Gx1[13]*Gx2[42] + Gx1[14]*Gx2[50] + Gx1[15]*Gx2[58];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[11] + Gx1[10]*Gx2[19] + Gx1[11]*Gx2[27] + Gx1[12]*Gx2[35] + Gx1[13]*Gx2[43] + Gx1[14]*Gx2[51] + Gx1[15]*Gx2[59];
Gx3[12] = + Gx1[8]*Gx2[4] + Gx1[9]*Gx2[12] + Gx1[10]*Gx2[20] + Gx1[11]*Gx2[28] + Gx1[12]*Gx2[36] + Gx1[13]*Gx2[44] + Gx1[14]*Gx2[52] + Gx1[15]*Gx2[60];
Gx3[13] = + Gx1[8]*Gx2[5] + Gx1[9]*Gx2[13] + Gx1[10]*Gx2[21] + Gx1[11]*Gx2[29] + Gx1[12]*Gx2[37] + Gx1[13]*Gx2[45] + Gx1[14]*Gx2[53] + Gx1[15]*Gx2[61];
Gx3[14] = + Gx1[8]*Gx2[6] + Gx1[9]*Gx2[14] + Gx1[10]*Gx2[22] + Gx1[11]*Gx2[30] + Gx1[12]*Gx2[38] + Gx1[13]*Gx2[46] + Gx1[14]*Gx2[54] + Gx1[15]*Gx2[62];
Gx3[15] = + Gx1[8]*Gx2[7] + Gx1[9]*Gx2[15] + Gx1[10]*Gx2[23] + Gx1[11]*Gx2[31] + Gx1[12]*Gx2[39] + Gx1[13]*Gx2[47] + Gx1[14]*Gx2[55] + Gx1[15]*Gx2[63];
Gx3[16] = + Gx1[16]*Gx2[0] + Gx1[17]*Gx2[8] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[24] + Gx1[20]*Gx2[32] + Gx1[21]*Gx2[40] + Gx1[22]*Gx2[48] + Gx1[23]*Gx2[56];
Gx3[17] = + Gx1[16]*Gx2[1] + Gx1[17]*Gx2[9] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[25] + Gx1[20]*Gx2[33] + Gx1[21]*Gx2[41] + Gx1[22]*Gx2[49] + Gx1[23]*Gx2[57];
Gx3[18] = + Gx1[16]*Gx2[2] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[26] + Gx1[20]*Gx2[34] + Gx1[21]*Gx2[42] + Gx1[22]*Gx2[50] + Gx1[23]*Gx2[58];
Gx3[19] = + Gx1[16]*Gx2[3] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[27] + Gx1[20]*Gx2[35] + Gx1[21]*Gx2[43] + Gx1[22]*Gx2[51] + Gx1[23]*Gx2[59];
Gx3[20] = + Gx1[16]*Gx2[4] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[20] + Gx1[19]*Gx2[28] + Gx1[20]*Gx2[36] + Gx1[21]*Gx2[44] + Gx1[22]*Gx2[52] + Gx1[23]*Gx2[60];
Gx3[21] = + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[21] + Gx1[19]*Gx2[29] + Gx1[20]*Gx2[37] + Gx1[21]*Gx2[45] + Gx1[22]*Gx2[53] + Gx1[23]*Gx2[61];
Gx3[22] = + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[22] + Gx1[19]*Gx2[30] + Gx1[20]*Gx2[38] + Gx1[21]*Gx2[46] + Gx1[22]*Gx2[54] + Gx1[23]*Gx2[62];
Gx3[23] = + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[15] + Gx1[18]*Gx2[23] + Gx1[19]*Gx2[31] + Gx1[20]*Gx2[39] + Gx1[21]*Gx2[47] + Gx1[22]*Gx2[55] + Gx1[23]*Gx2[63];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[8] + Gx1[26]*Gx2[16] + Gx1[27]*Gx2[24] + Gx1[28]*Gx2[32] + Gx1[29]*Gx2[40] + Gx1[30]*Gx2[48] + Gx1[31]*Gx2[56];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[9] + Gx1[26]*Gx2[17] + Gx1[27]*Gx2[25] + Gx1[28]*Gx2[33] + Gx1[29]*Gx2[41] + Gx1[30]*Gx2[49] + Gx1[31]*Gx2[57];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[10] + Gx1[26]*Gx2[18] + Gx1[27]*Gx2[26] + Gx1[28]*Gx2[34] + Gx1[29]*Gx2[42] + Gx1[30]*Gx2[50] + Gx1[31]*Gx2[58];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[11] + Gx1[26]*Gx2[19] + Gx1[27]*Gx2[27] + Gx1[28]*Gx2[35] + Gx1[29]*Gx2[43] + Gx1[30]*Gx2[51] + Gx1[31]*Gx2[59];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[12] + Gx1[26]*Gx2[20] + Gx1[27]*Gx2[28] + Gx1[28]*Gx2[36] + Gx1[29]*Gx2[44] + Gx1[30]*Gx2[52] + Gx1[31]*Gx2[60];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[13] + Gx1[26]*Gx2[21] + Gx1[27]*Gx2[29] + Gx1[28]*Gx2[37] + Gx1[29]*Gx2[45] + Gx1[30]*Gx2[53] + Gx1[31]*Gx2[61];
Gx3[30] = + Gx1[24]*Gx2[6] + Gx1[25]*Gx2[14] + Gx1[26]*Gx2[22] + Gx1[27]*Gx2[30] + Gx1[28]*Gx2[38] + Gx1[29]*Gx2[46] + Gx1[30]*Gx2[54] + Gx1[31]*Gx2[62];
Gx3[31] = + Gx1[24]*Gx2[7] + Gx1[25]*Gx2[15] + Gx1[26]*Gx2[23] + Gx1[27]*Gx2[31] + Gx1[28]*Gx2[39] + Gx1[29]*Gx2[47] + Gx1[30]*Gx2[55] + Gx1[31]*Gx2[63];
Gx3[32] = + Gx1[32]*Gx2[0] + Gx1[33]*Gx2[8] + Gx1[34]*Gx2[16] + Gx1[35]*Gx2[24] + Gx1[36]*Gx2[32] + Gx1[37]*Gx2[40] + Gx1[38]*Gx2[48] + Gx1[39]*Gx2[56];
Gx3[33] = + Gx1[32]*Gx2[1] + Gx1[33]*Gx2[9] + Gx1[34]*Gx2[17] + Gx1[35]*Gx2[25] + Gx1[36]*Gx2[33] + Gx1[37]*Gx2[41] + Gx1[38]*Gx2[49] + Gx1[39]*Gx2[57];
Gx3[34] = + Gx1[32]*Gx2[2] + Gx1[33]*Gx2[10] + Gx1[34]*Gx2[18] + Gx1[35]*Gx2[26] + Gx1[36]*Gx2[34] + Gx1[37]*Gx2[42] + Gx1[38]*Gx2[50] + Gx1[39]*Gx2[58];
Gx3[35] = + Gx1[32]*Gx2[3] + Gx1[33]*Gx2[11] + Gx1[34]*Gx2[19] + Gx1[35]*Gx2[27] + Gx1[36]*Gx2[35] + Gx1[37]*Gx2[43] + Gx1[38]*Gx2[51] + Gx1[39]*Gx2[59];
Gx3[36] = + Gx1[32]*Gx2[4] + Gx1[33]*Gx2[12] + Gx1[34]*Gx2[20] + Gx1[35]*Gx2[28] + Gx1[36]*Gx2[36] + Gx1[37]*Gx2[44] + Gx1[38]*Gx2[52] + Gx1[39]*Gx2[60];
Gx3[37] = + Gx1[32]*Gx2[5] + Gx1[33]*Gx2[13] + Gx1[34]*Gx2[21] + Gx1[35]*Gx2[29] + Gx1[36]*Gx2[37] + Gx1[37]*Gx2[45] + Gx1[38]*Gx2[53] + Gx1[39]*Gx2[61];
Gx3[38] = + Gx1[32]*Gx2[6] + Gx1[33]*Gx2[14] + Gx1[34]*Gx2[22] + Gx1[35]*Gx2[30] + Gx1[36]*Gx2[38] + Gx1[37]*Gx2[46] + Gx1[38]*Gx2[54] + Gx1[39]*Gx2[62];
Gx3[39] = + Gx1[32]*Gx2[7] + Gx1[33]*Gx2[15] + Gx1[34]*Gx2[23] + Gx1[35]*Gx2[31] + Gx1[36]*Gx2[39] + Gx1[37]*Gx2[47] + Gx1[38]*Gx2[55] + Gx1[39]*Gx2[63];
Gx3[40] = + Gx1[40]*Gx2[0] + Gx1[41]*Gx2[8] + Gx1[42]*Gx2[16] + Gx1[43]*Gx2[24] + Gx1[44]*Gx2[32] + Gx1[45]*Gx2[40] + Gx1[46]*Gx2[48] + Gx1[47]*Gx2[56];
Gx3[41] = + Gx1[40]*Gx2[1] + Gx1[41]*Gx2[9] + Gx1[42]*Gx2[17] + Gx1[43]*Gx2[25] + Gx1[44]*Gx2[33] + Gx1[45]*Gx2[41] + Gx1[46]*Gx2[49] + Gx1[47]*Gx2[57];
Gx3[42] = + Gx1[40]*Gx2[2] + Gx1[41]*Gx2[10] + Gx1[42]*Gx2[18] + Gx1[43]*Gx2[26] + Gx1[44]*Gx2[34] + Gx1[45]*Gx2[42] + Gx1[46]*Gx2[50] + Gx1[47]*Gx2[58];
Gx3[43] = + Gx1[40]*Gx2[3] + Gx1[41]*Gx2[11] + Gx1[42]*Gx2[19] + Gx1[43]*Gx2[27] + Gx1[44]*Gx2[35] + Gx1[45]*Gx2[43] + Gx1[46]*Gx2[51] + Gx1[47]*Gx2[59];
Gx3[44] = + Gx1[40]*Gx2[4] + Gx1[41]*Gx2[12] + Gx1[42]*Gx2[20] + Gx1[43]*Gx2[28] + Gx1[44]*Gx2[36] + Gx1[45]*Gx2[44] + Gx1[46]*Gx2[52] + Gx1[47]*Gx2[60];
Gx3[45] = + Gx1[40]*Gx2[5] + Gx1[41]*Gx2[13] + Gx1[42]*Gx2[21] + Gx1[43]*Gx2[29] + Gx1[44]*Gx2[37] + Gx1[45]*Gx2[45] + Gx1[46]*Gx2[53] + Gx1[47]*Gx2[61];
Gx3[46] = + Gx1[40]*Gx2[6] + Gx1[41]*Gx2[14] + Gx1[42]*Gx2[22] + Gx1[43]*Gx2[30] + Gx1[44]*Gx2[38] + Gx1[45]*Gx2[46] + Gx1[46]*Gx2[54] + Gx1[47]*Gx2[62];
Gx3[47] = + Gx1[40]*Gx2[7] + Gx1[41]*Gx2[15] + Gx1[42]*Gx2[23] + Gx1[43]*Gx2[31] + Gx1[44]*Gx2[39] + Gx1[45]*Gx2[47] + Gx1[46]*Gx2[55] + Gx1[47]*Gx2[63];
Gx3[48] = + Gx1[48]*Gx2[0] + Gx1[49]*Gx2[8] + Gx1[50]*Gx2[16] + Gx1[51]*Gx2[24] + Gx1[52]*Gx2[32] + Gx1[53]*Gx2[40] + Gx1[54]*Gx2[48] + Gx1[55]*Gx2[56];
Gx3[49] = + Gx1[48]*Gx2[1] + Gx1[49]*Gx2[9] + Gx1[50]*Gx2[17] + Gx1[51]*Gx2[25] + Gx1[52]*Gx2[33] + Gx1[53]*Gx2[41] + Gx1[54]*Gx2[49] + Gx1[55]*Gx2[57];
Gx3[50] = + Gx1[48]*Gx2[2] + Gx1[49]*Gx2[10] + Gx1[50]*Gx2[18] + Gx1[51]*Gx2[26] + Gx1[52]*Gx2[34] + Gx1[53]*Gx2[42] + Gx1[54]*Gx2[50] + Gx1[55]*Gx2[58];
Gx3[51] = + Gx1[48]*Gx2[3] + Gx1[49]*Gx2[11] + Gx1[50]*Gx2[19] + Gx1[51]*Gx2[27] + Gx1[52]*Gx2[35] + Gx1[53]*Gx2[43] + Gx1[54]*Gx2[51] + Gx1[55]*Gx2[59];
Gx3[52] = + Gx1[48]*Gx2[4] + Gx1[49]*Gx2[12] + Gx1[50]*Gx2[20] + Gx1[51]*Gx2[28] + Gx1[52]*Gx2[36] + Gx1[53]*Gx2[44] + Gx1[54]*Gx2[52] + Gx1[55]*Gx2[60];
Gx3[53] = + Gx1[48]*Gx2[5] + Gx1[49]*Gx2[13] + Gx1[50]*Gx2[21] + Gx1[51]*Gx2[29] + Gx1[52]*Gx2[37] + Gx1[53]*Gx2[45] + Gx1[54]*Gx2[53] + Gx1[55]*Gx2[61];
Gx3[54] = + Gx1[48]*Gx2[6] + Gx1[49]*Gx2[14] + Gx1[50]*Gx2[22] + Gx1[51]*Gx2[30] + Gx1[52]*Gx2[38] + Gx1[53]*Gx2[46] + Gx1[54]*Gx2[54] + Gx1[55]*Gx2[62];
Gx3[55] = + Gx1[48]*Gx2[7] + Gx1[49]*Gx2[15] + Gx1[50]*Gx2[23] + Gx1[51]*Gx2[31] + Gx1[52]*Gx2[39] + Gx1[53]*Gx2[47] + Gx1[54]*Gx2[55] + Gx1[55]*Gx2[63];
Gx3[56] = + Gx1[56]*Gx2[0] + Gx1[57]*Gx2[8] + Gx1[58]*Gx2[16] + Gx1[59]*Gx2[24] + Gx1[60]*Gx2[32] + Gx1[61]*Gx2[40] + Gx1[62]*Gx2[48] + Gx1[63]*Gx2[56];
Gx3[57] = + Gx1[56]*Gx2[1] + Gx1[57]*Gx2[9] + Gx1[58]*Gx2[17] + Gx1[59]*Gx2[25] + Gx1[60]*Gx2[33] + Gx1[61]*Gx2[41] + Gx1[62]*Gx2[49] + Gx1[63]*Gx2[57];
Gx3[58] = + Gx1[56]*Gx2[2] + Gx1[57]*Gx2[10] + Gx1[58]*Gx2[18] + Gx1[59]*Gx2[26] + Gx1[60]*Gx2[34] + Gx1[61]*Gx2[42] + Gx1[62]*Gx2[50] + Gx1[63]*Gx2[58];
Gx3[59] = + Gx1[56]*Gx2[3] + Gx1[57]*Gx2[11] + Gx1[58]*Gx2[19] + Gx1[59]*Gx2[27] + Gx1[60]*Gx2[35] + Gx1[61]*Gx2[43] + Gx1[62]*Gx2[51] + Gx1[63]*Gx2[59];
Gx3[60] = + Gx1[56]*Gx2[4] + Gx1[57]*Gx2[12] + Gx1[58]*Gx2[20] + Gx1[59]*Gx2[28] + Gx1[60]*Gx2[36] + Gx1[61]*Gx2[44] + Gx1[62]*Gx2[52] + Gx1[63]*Gx2[60];
Gx3[61] = + Gx1[56]*Gx2[5] + Gx1[57]*Gx2[13] + Gx1[58]*Gx2[21] + Gx1[59]*Gx2[29] + Gx1[60]*Gx2[37] + Gx1[61]*Gx2[45] + Gx1[62]*Gx2[53] + Gx1[63]*Gx2[61];
Gx3[62] = + Gx1[56]*Gx2[6] + Gx1[57]*Gx2[14] + Gx1[58]*Gx2[22] + Gx1[59]*Gx2[30] + Gx1[60]*Gx2[38] + Gx1[61]*Gx2[46] + Gx1[62]*Gx2[54] + Gx1[63]*Gx2[62];
Gx3[63] = + Gx1[56]*Gx2[7] + Gx1[57]*Gx2[15] + Gx1[58]*Gx2[23] + Gx1[59]*Gx2[31] + Gx1[60]*Gx2[39] + Gx1[61]*Gx2[47] + Gx1[62]*Gx2[55] + Gx1[63]*Gx2[63];
}

void acado_LO_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8] + Gx1[5]*Gu1[10] + Gx1[6]*Gu1[12] + Gx1[7]*Gu1[14];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9] + Gx1[5]*Gu1[11] + Gx1[6]*Gu1[13] + Gx1[7]*Gu1[15];
Gu2[2] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[8] + Gx1[13]*Gu1[10] + Gx1[14]*Gu1[12] + Gx1[15]*Gu1[14];
Gu2[3] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[11] + Gx1[14]*Gu1[13] + Gx1[15]*Gu1[15];
Gu2[4] = + Gx1[16]*Gu1[0] + Gx1[17]*Gu1[2] + Gx1[18]*Gu1[4] + Gx1[19]*Gu1[6] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[10] + Gx1[22]*Gu1[12] + Gx1[23]*Gu1[14];
Gu2[5] = + Gx1[16]*Gu1[1] + Gx1[17]*Gu1[3] + Gx1[18]*Gu1[5] + Gx1[19]*Gu1[7] + Gx1[20]*Gu1[9] + Gx1[21]*Gu1[11] + Gx1[22]*Gu1[13] + Gx1[23]*Gu1[15];
Gu2[6] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[2] + Gx1[26]*Gu1[4] + Gx1[27]*Gu1[6] + Gx1[28]*Gu1[8] + Gx1[29]*Gu1[10] + Gx1[30]*Gu1[12] + Gx1[31]*Gu1[14];
Gu2[7] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[3] + Gx1[26]*Gu1[5] + Gx1[27]*Gu1[7] + Gx1[28]*Gu1[9] + Gx1[29]*Gu1[11] + Gx1[30]*Gu1[13] + Gx1[31]*Gu1[15];
Gu2[8] = + Gx1[32]*Gu1[0] + Gx1[33]*Gu1[2] + Gx1[34]*Gu1[4] + Gx1[35]*Gu1[6] + Gx1[36]*Gu1[8] + Gx1[37]*Gu1[10] + Gx1[38]*Gu1[12] + Gx1[39]*Gu1[14];
Gu2[9] = + Gx1[32]*Gu1[1] + Gx1[33]*Gu1[3] + Gx1[34]*Gu1[5] + Gx1[35]*Gu1[7] + Gx1[36]*Gu1[9] + Gx1[37]*Gu1[11] + Gx1[38]*Gu1[13] + Gx1[39]*Gu1[15];
Gu2[10] = + Gx1[40]*Gu1[0] + Gx1[41]*Gu1[2] + Gx1[42]*Gu1[4] + Gx1[43]*Gu1[6] + Gx1[44]*Gu1[8] + Gx1[45]*Gu1[10] + Gx1[46]*Gu1[12] + Gx1[47]*Gu1[14];
Gu2[11] = + Gx1[40]*Gu1[1] + Gx1[41]*Gu1[3] + Gx1[42]*Gu1[5] + Gx1[43]*Gu1[7] + Gx1[44]*Gu1[9] + Gx1[45]*Gu1[11] + Gx1[46]*Gu1[13] + Gx1[47]*Gu1[15];
Gu2[12] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[2] + Gx1[50]*Gu1[4] + Gx1[51]*Gu1[6] + Gx1[52]*Gu1[8] + Gx1[53]*Gu1[10] + Gx1[54]*Gu1[12] + Gx1[55]*Gu1[14];
Gu2[13] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[3] + Gx1[50]*Gu1[5] + Gx1[51]*Gu1[7] + Gx1[52]*Gu1[9] + Gx1[53]*Gu1[11] + Gx1[54]*Gu1[13] + Gx1[55]*Gu1[15];
Gu2[14] = + Gx1[56]*Gu1[0] + Gx1[57]*Gu1[2] + Gx1[58]*Gu1[4] + Gx1[59]*Gu1[6] + Gx1[60]*Gu1[8] + Gx1[61]*Gu1[10] + Gx1[62]*Gu1[12] + Gx1[63]*Gu1[14];
Gu2[15] = + Gx1[56]*Gu1[1] + Gx1[57]*Gu1[3] + Gx1[58]*Gu1[5] + Gx1[59]*Gu1[7] + Gx1[60]*Gu1[9] + Gx1[61]*Gu1[11] + Gx1[62]*Gu1[13] + Gx1[63]*Gu1[15];
}

void acado_LO_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
}

void acado_LO_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace_LO.H[(iRow * 40) + (iCol * 2)] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10] + Gu1[12]*Gu2[12] + Gu1[14]*Gu2[14];
acadoWorkspace_LO.H[(iRow * 40) + (iCol * 2 + 1)] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11] + Gu1[12]*Gu2[13] + Gu1[14]*Gu2[15];
acadoWorkspace_LO.H[(iRow * 40 + 20) + (iCol * 2)] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10] + Gu1[13]*Gu2[12] + Gu1[15]*Gu2[14];
acadoWorkspace_LO.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11] + Gu1[13]*Gu2[13] + Gu1[15]*Gu2[15];
}

void acado_LO_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace_LO.H[(iRow * 40) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10] + Gu1[12]*Gu2[12] + Gu1[14]*Gu2[14];
acadoWorkspace_LO.H[(iRow * 40) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11] + Gu1[12]*Gu2[13] + Gu1[14]*Gu2[15];
acadoWorkspace_LO.H[(iRow * 40 + 20) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10] + Gu1[13]*Gu2[12] + Gu1[15]*Gu2[14];
acadoWorkspace_LO.H[(iRow * 40 + 20) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11] + Gu1[13]*Gu2[13] + Gu1[15]*Gu2[15];
}

void acado_LO_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace_LO.H[iRow * 42] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10] + Gu1[12]*Gu2[12] + Gu1[14]*Gu2[14] + R11[0];
acadoWorkspace_LO.H[iRow * 42 + 1] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11] + Gu1[12]*Gu2[13] + Gu1[14]*Gu2[15] + R11[1];
acadoWorkspace_LO.H[iRow * 42 + 20] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10] + Gu1[13]*Gu2[12] + Gu1[15]*Gu2[14] + R11[2];
acadoWorkspace_LO.H[iRow * 42 + 21] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11] + Gu1[13]*Gu2[13] + Gu1[15]*Gu2[15] + R11[3];
}

void acado_LO_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[8]*Gu1[2] + Gx1[16]*Gu1[4] + Gx1[24]*Gu1[6] + Gx1[32]*Gu1[8] + Gx1[40]*Gu1[10] + Gx1[48]*Gu1[12] + Gx1[56]*Gu1[14];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[8]*Gu1[3] + Gx1[16]*Gu1[5] + Gx1[24]*Gu1[7] + Gx1[32]*Gu1[9] + Gx1[40]*Gu1[11] + Gx1[48]*Gu1[13] + Gx1[56]*Gu1[15];
Gu2[2] = + Gx1[1]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[17]*Gu1[4] + Gx1[25]*Gu1[6] + Gx1[33]*Gu1[8] + Gx1[41]*Gu1[10] + Gx1[49]*Gu1[12] + Gx1[57]*Gu1[14];
Gu2[3] = + Gx1[1]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[17]*Gu1[5] + Gx1[25]*Gu1[7] + Gx1[33]*Gu1[9] + Gx1[41]*Gu1[11] + Gx1[49]*Gu1[13] + Gx1[57]*Gu1[15];
Gu2[4] = + Gx1[2]*Gu1[0] + Gx1[10]*Gu1[2] + Gx1[18]*Gu1[4] + Gx1[26]*Gu1[6] + Gx1[34]*Gu1[8] + Gx1[42]*Gu1[10] + Gx1[50]*Gu1[12] + Gx1[58]*Gu1[14];
Gu2[5] = + Gx1[2]*Gu1[1] + Gx1[10]*Gu1[3] + Gx1[18]*Gu1[5] + Gx1[26]*Gu1[7] + Gx1[34]*Gu1[9] + Gx1[42]*Gu1[11] + Gx1[50]*Gu1[13] + Gx1[58]*Gu1[15];
Gu2[6] = + Gx1[3]*Gu1[0] + Gx1[11]*Gu1[2] + Gx1[19]*Gu1[4] + Gx1[27]*Gu1[6] + Gx1[35]*Gu1[8] + Gx1[43]*Gu1[10] + Gx1[51]*Gu1[12] + Gx1[59]*Gu1[14];
Gu2[7] = + Gx1[3]*Gu1[1] + Gx1[11]*Gu1[3] + Gx1[19]*Gu1[5] + Gx1[27]*Gu1[7] + Gx1[35]*Gu1[9] + Gx1[43]*Gu1[11] + Gx1[51]*Gu1[13] + Gx1[59]*Gu1[15];
Gu2[8] = + Gx1[4]*Gu1[0] + Gx1[12]*Gu1[2] + Gx1[20]*Gu1[4] + Gx1[28]*Gu1[6] + Gx1[36]*Gu1[8] + Gx1[44]*Gu1[10] + Gx1[52]*Gu1[12] + Gx1[60]*Gu1[14];
Gu2[9] = + Gx1[4]*Gu1[1] + Gx1[12]*Gu1[3] + Gx1[20]*Gu1[5] + Gx1[28]*Gu1[7] + Gx1[36]*Gu1[9] + Gx1[44]*Gu1[11] + Gx1[52]*Gu1[13] + Gx1[60]*Gu1[15];
Gu2[10] = + Gx1[5]*Gu1[0] + Gx1[13]*Gu1[2] + Gx1[21]*Gu1[4] + Gx1[29]*Gu1[6] + Gx1[37]*Gu1[8] + Gx1[45]*Gu1[10] + Gx1[53]*Gu1[12] + Gx1[61]*Gu1[14];
Gu2[11] = + Gx1[5]*Gu1[1] + Gx1[13]*Gu1[3] + Gx1[21]*Gu1[5] + Gx1[29]*Gu1[7] + Gx1[37]*Gu1[9] + Gx1[45]*Gu1[11] + Gx1[53]*Gu1[13] + Gx1[61]*Gu1[15];
Gu2[12] = + Gx1[6]*Gu1[0] + Gx1[14]*Gu1[2] + Gx1[22]*Gu1[4] + Gx1[30]*Gu1[6] + Gx1[38]*Gu1[8] + Gx1[46]*Gu1[10] + Gx1[54]*Gu1[12] + Gx1[62]*Gu1[14];
Gu2[13] = + Gx1[6]*Gu1[1] + Gx1[14]*Gu1[3] + Gx1[22]*Gu1[5] + Gx1[30]*Gu1[7] + Gx1[38]*Gu1[9] + Gx1[46]*Gu1[11] + Gx1[54]*Gu1[13] + Gx1[62]*Gu1[15];
Gu2[14] = + Gx1[7]*Gu1[0] + Gx1[15]*Gu1[2] + Gx1[23]*Gu1[4] + Gx1[31]*Gu1[6] + Gx1[39]*Gu1[8] + Gx1[47]*Gu1[10] + Gx1[55]*Gu1[12] + Gx1[63]*Gu1[14];
Gu2[15] = + Gx1[7]*Gu1[1] + Gx1[15]*Gu1[3] + Gx1[23]*Gu1[5] + Gx1[31]*Gu1[7] + Gx1[39]*Gu1[9] + Gx1[47]*Gu1[11] + Gx1[55]*Gu1[13] + Gx1[63]*Gu1[15];
}

void acado_LO_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[2] + Q11[2]*Gu1[4] + Q11[3]*Gu1[6] + Q11[4]*Gu1[8] + Q11[5]*Gu1[10] + Q11[6]*Gu1[12] + Q11[7]*Gu1[14] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[3] + Q11[2]*Gu1[5] + Q11[3]*Gu1[7] + Q11[4]*Gu1[9] + Q11[5]*Gu1[11] + Q11[6]*Gu1[13] + Q11[7]*Gu1[15] + Gu2[1];
Gu3[2] = + Q11[8]*Gu1[0] + Q11[9]*Gu1[2] + Q11[10]*Gu1[4] + Q11[11]*Gu1[6] + Q11[12]*Gu1[8] + Q11[13]*Gu1[10] + Q11[14]*Gu1[12] + Q11[15]*Gu1[14] + Gu2[2];
Gu3[3] = + Q11[8]*Gu1[1] + Q11[9]*Gu1[3] + Q11[10]*Gu1[5] + Q11[11]*Gu1[7] + Q11[12]*Gu1[9] + Q11[13]*Gu1[11] + Q11[14]*Gu1[13] + Q11[15]*Gu1[15] + Gu2[3];
Gu3[4] = + Q11[16]*Gu1[0] + Q11[17]*Gu1[2] + Q11[18]*Gu1[4] + Q11[19]*Gu1[6] + Q11[20]*Gu1[8] + Q11[21]*Gu1[10] + Q11[22]*Gu1[12] + Q11[23]*Gu1[14] + Gu2[4];
Gu3[5] = + Q11[16]*Gu1[1] + Q11[17]*Gu1[3] + Q11[18]*Gu1[5] + Q11[19]*Gu1[7] + Q11[20]*Gu1[9] + Q11[21]*Gu1[11] + Q11[22]*Gu1[13] + Q11[23]*Gu1[15] + Gu2[5];
Gu3[6] = + Q11[24]*Gu1[0] + Q11[25]*Gu1[2] + Q11[26]*Gu1[4] + Q11[27]*Gu1[6] + Q11[28]*Gu1[8] + Q11[29]*Gu1[10] + Q11[30]*Gu1[12] + Q11[31]*Gu1[14] + Gu2[6];
Gu3[7] = + Q11[24]*Gu1[1] + Q11[25]*Gu1[3] + Q11[26]*Gu1[5] + Q11[27]*Gu1[7] + Q11[28]*Gu1[9] + Q11[29]*Gu1[11] + Q11[30]*Gu1[13] + Q11[31]*Gu1[15] + Gu2[7];
Gu3[8] = + Q11[32]*Gu1[0] + Q11[33]*Gu1[2] + Q11[34]*Gu1[4] + Q11[35]*Gu1[6] + Q11[36]*Gu1[8] + Q11[37]*Gu1[10] + Q11[38]*Gu1[12] + Q11[39]*Gu1[14] + Gu2[8];
Gu3[9] = + Q11[32]*Gu1[1] + Q11[33]*Gu1[3] + Q11[34]*Gu1[5] + Q11[35]*Gu1[7] + Q11[36]*Gu1[9] + Q11[37]*Gu1[11] + Q11[38]*Gu1[13] + Q11[39]*Gu1[15] + Gu2[9];
Gu3[10] = + Q11[40]*Gu1[0] + Q11[41]*Gu1[2] + Q11[42]*Gu1[4] + Q11[43]*Gu1[6] + Q11[44]*Gu1[8] + Q11[45]*Gu1[10] + Q11[46]*Gu1[12] + Q11[47]*Gu1[14] + Gu2[10];
Gu3[11] = + Q11[40]*Gu1[1] + Q11[41]*Gu1[3] + Q11[42]*Gu1[5] + Q11[43]*Gu1[7] + Q11[44]*Gu1[9] + Q11[45]*Gu1[11] + Q11[46]*Gu1[13] + Q11[47]*Gu1[15] + Gu2[11];
Gu3[12] = + Q11[48]*Gu1[0] + Q11[49]*Gu1[2] + Q11[50]*Gu1[4] + Q11[51]*Gu1[6] + Q11[52]*Gu1[8] + Q11[53]*Gu1[10] + Q11[54]*Gu1[12] + Q11[55]*Gu1[14] + Gu2[12];
Gu3[13] = + Q11[48]*Gu1[1] + Q11[49]*Gu1[3] + Q11[50]*Gu1[5] + Q11[51]*Gu1[7] + Q11[52]*Gu1[9] + Q11[53]*Gu1[11] + Q11[54]*Gu1[13] + Q11[55]*Gu1[15] + Gu2[13];
Gu3[14] = + Q11[56]*Gu1[0] + Q11[57]*Gu1[2] + Q11[58]*Gu1[4] + Q11[59]*Gu1[6] + Q11[60]*Gu1[8] + Q11[61]*Gu1[10] + Q11[62]*Gu1[12] + Q11[63]*Gu1[14] + Gu2[14];
Gu3[15] = + Q11[56]*Gu1[1] + Q11[57]*Gu1[3] + Q11[58]*Gu1[5] + Q11[59]*Gu1[7] + Q11[60]*Gu1[9] + Q11[61]*Gu1[11] + Q11[62]*Gu1[13] + Q11[63]*Gu1[15] + Gu2[15];
}

void acado_LO_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[8]*w11[1] + Gx1[16]*w11[2] + Gx1[24]*w11[3] + Gx1[32]*w11[4] + Gx1[40]*w11[5] + Gx1[48]*w11[6] + Gx1[56]*w11[7] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[9]*w11[1] + Gx1[17]*w11[2] + Gx1[25]*w11[3] + Gx1[33]*w11[4] + Gx1[41]*w11[5] + Gx1[49]*w11[6] + Gx1[57]*w11[7] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[10]*w11[1] + Gx1[18]*w11[2] + Gx1[26]*w11[3] + Gx1[34]*w11[4] + Gx1[42]*w11[5] + Gx1[50]*w11[6] + Gx1[58]*w11[7] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[11]*w11[1] + Gx1[19]*w11[2] + Gx1[27]*w11[3] + Gx1[35]*w11[4] + Gx1[43]*w11[5] + Gx1[51]*w11[6] + Gx1[59]*w11[7] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[12]*w11[1] + Gx1[20]*w11[2] + Gx1[28]*w11[3] + Gx1[36]*w11[4] + Gx1[44]*w11[5] + Gx1[52]*w11[6] + Gx1[60]*w11[7] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[13]*w11[1] + Gx1[21]*w11[2] + Gx1[29]*w11[3] + Gx1[37]*w11[4] + Gx1[45]*w11[5] + Gx1[53]*w11[6] + Gx1[61]*w11[7] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[14]*w11[1] + Gx1[22]*w11[2] + Gx1[30]*w11[3] + Gx1[38]*w11[4] + Gx1[46]*w11[5] + Gx1[54]*w11[6] + Gx1[62]*w11[7] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[15]*w11[1] + Gx1[23]*w11[2] + Gx1[31]*w11[3] + Gx1[39]*w11[4] + Gx1[47]*w11[5] + Gx1[55]*w11[6] + Gx1[63]*w11[7] + w12[7];
}

void acado_LO_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2] + Gu1[6]*w11[3] + Gu1[8]*w11[4] + Gu1[10]*w11[5] + Gu1[12]*w11[6] + Gu1[14]*w11[7];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2] + Gu1[7]*w11[3] + Gu1[9]*w11[4] + Gu1[11]*w11[5] + Gu1[13]*w11[6] + Gu1[15]*w11[7];
}

void acado_LO_macS1TSbar( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2] + Gu1[6]*w11[3] + Gu1[8]*w11[4] + Gu1[10]*w11[5] + Gu1[12]*w11[6] + Gu1[14]*w11[7];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2] + Gu1[7]*w11[3] + Gu1[9]*w11[4] + Gu1[11]*w11[5] + Gu1[13]*w11[6] + Gu1[15]*w11[7];
}

void acado_LO_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + w12[0];
w13[1] = + Q11[8]*w11[0] + Q11[9]*w11[1] + Q11[10]*w11[2] + Q11[11]*w11[3] + Q11[12]*w11[4] + Q11[13]*w11[5] + Q11[14]*w11[6] + Q11[15]*w11[7] + w12[1];
w13[2] = + Q11[16]*w11[0] + Q11[17]*w11[1] + Q11[18]*w11[2] + Q11[19]*w11[3] + Q11[20]*w11[4] + Q11[21]*w11[5] + Q11[22]*w11[6] + Q11[23]*w11[7] + w12[2];
w13[3] = + Q11[24]*w11[0] + Q11[25]*w11[1] + Q11[26]*w11[2] + Q11[27]*w11[3] + Q11[28]*w11[4] + Q11[29]*w11[5] + Q11[30]*w11[6] + Q11[31]*w11[7] + w12[3];
w13[4] = + Q11[32]*w11[0] + Q11[33]*w11[1] + Q11[34]*w11[2] + Q11[35]*w11[3] + Q11[36]*w11[4] + Q11[37]*w11[5] + Q11[38]*w11[6] + Q11[39]*w11[7] + w12[4];
w13[5] = + Q11[40]*w11[0] + Q11[41]*w11[1] + Q11[42]*w11[2] + Q11[43]*w11[3] + Q11[44]*w11[4] + Q11[45]*w11[5] + Q11[46]*w11[6] + Q11[47]*w11[7] + w12[5];
w13[6] = + Q11[48]*w11[0] + Q11[49]*w11[1] + Q11[50]*w11[2] + Q11[51]*w11[3] + Q11[52]*w11[4] + Q11[53]*w11[5] + Q11[54]*w11[6] + Q11[55]*w11[7] + w12[6];
w13[7] = + Q11[56]*w11[0] + Q11[57]*w11[1] + Q11[58]*w11[2] + Q11[59]*w11[3] + Q11[60]*w11[4] + Q11[61]*w11[5] + Q11[62]*w11[6] + Q11[63]*w11[7] + w12[7];
}

void acado_LO_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7];
w12[1] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3] + Gx1[12]*w11[4] + Gx1[13]*w11[5] + Gx1[14]*w11[6] + Gx1[15]*w11[7];
w12[2] += + Gx1[16]*w11[0] + Gx1[17]*w11[1] + Gx1[18]*w11[2] + Gx1[19]*w11[3] + Gx1[20]*w11[4] + Gx1[21]*w11[5] + Gx1[22]*w11[6] + Gx1[23]*w11[7];
w12[3] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5] + Gx1[30]*w11[6] + Gx1[31]*w11[7];
w12[4] += + Gx1[32]*w11[0] + Gx1[33]*w11[1] + Gx1[34]*w11[2] + Gx1[35]*w11[3] + Gx1[36]*w11[4] + Gx1[37]*w11[5] + Gx1[38]*w11[6] + Gx1[39]*w11[7];
w12[5] += + Gx1[40]*w11[0] + Gx1[41]*w11[1] + Gx1[42]*w11[2] + Gx1[43]*w11[3] + Gx1[44]*w11[4] + Gx1[45]*w11[5] + Gx1[46]*w11[6] + Gx1[47]*w11[7];
w12[6] += + Gx1[48]*w11[0] + Gx1[49]*w11[1] + Gx1[50]*w11[2] + Gx1[51]*w11[3] + Gx1[52]*w11[4] + Gx1[53]*w11[5] + Gx1[54]*w11[6] + Gx1[55]*w11[7];
w12[7] += + Gx1[56]*w11[0] + Gx1[57]*w11[1] + Gx1[58]*w11[2] + Gx1[59]*w11[3] + Gx1[60]*w11[4] + Gx1[61]*w11[5] + Gx1[62]*w11[6] + Gx1[63]*w11[7];
}

void acado_LO_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7];
w12[1] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3] + Gx1[12]*w11[4] + Gx1[13]*w11[5] + Gx1[14]*w11[6] + Gx1[15]*w11[7];
w12[2] += + Gx1[16]*w11[0] + Gx1[17]*w11[1] + Gx1[18]*w11[2] + Gx1[19]*w11[3] + Gx1[20]*w11[4] + Gx1[21]*w11[5] + Gx1[22]*w11[6] + Gx1[23]*w11[7];
w12[3] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5] + Gx1[30]*w11[6] + Gx1[31]*w11[7];
w12[4] += + Gx1[32]*w11[0] + Gx1[33]*w11[1] + Gx1[34]*w11[2] + Gx1[35]*w11[3] + Gx1[36]*w11[4] + Gx1[37]*w11[5] + Gx1[38]*w11[6] + Gx1[39]*w11[7];
w12[5] += + Gx1[40]*w11[0] + Gx1[41]*w11[1] + Gx1[42]*w11[2] + Gx1[43]*w11[3] + Gx1[44]*w11[4] + Gx1[45]*w11[5] + Gx1[46]*w11[6] + Gx1[47]*w11[7];
w12[6] += + Gx1[48]*w11[0] + Gx1[49]*w11[1] + Gx1[50]*w11[2] + Gx1[51]*w11[3] + Gx1[52]*w11[4] + Gx1[53]*w11[5] + Gx1[54]*w11[6] + Gx1[55]*w11[7];
w12[7] += + Gx1[56]*w11[0] + Gx1[57]*w11[1] + Gx1[58]*w11[2] + Gx1[59]*w11[3] + Gx1[60]*w11[4] + Gx1[61]*w11[5] + Gx1[62]*w11[6] + Gx1[63]*w11[7];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1];
w12[1] += + Gu1[2]*U1[0] + Gu1[3]*U1[1];
w12[2] += + Gu1[4]*U1[0] + Gu1[5]*U1[1];
w12[3] += + Gu1[6]*U1[0] + Gu1[7]*U1[1];
w12[4] += + Gu1[8]*U1[0] + Gu1[9]*U1[1];
w12[5] += + Gu1[10]*U1[0] + Gu1[11]*U1[1];
w12[6] += + Gu1[12]*U1[0] + Gu1[13]*U1[1];
w12[7] += + Gu1[14]*U1[0] + Gu1[15]*U1[1];
}

void acado_LO_expansionStep2( real_t* const QDy1, real_t* const Q11, real_t* const w11, real_t* const Gu1, real_t* const U1, real_t* const Gx1, real_t* const mu1, real_t* const mu2 )
{
mu1[0] += QDy1[0];
mu1[1] += QDy1[1];
mu1[2] += QDy1[2];
mu1[3] += QDy1[3];
mu1[4] += QDy1[4];
mu1[5] += QDy1[5];
mu1[6] += QDy1[6];
mu1[7] += QDy1[7];
mu1[0] += + w11[0]*Q11[0] + w11[1]*Q11[1] + w11[2]*Q11[2] + w11[3]*Q11[3] + w11[4]*Q11[4] + w11[5]*Q11[5] + w11[6]*Q11[6] + w11[7]*Q11[7];
mu1[1] += + w11[0]*Q11[8] + w11[1]*Q11[9] + w11[2]*Q11[10] + w11[3]*Q11[11] + w11[4]*Q11[12] + w11[5]*Q11[13] + w11[6]*Q11[14] + w11[7]*Q11[15];
mu1[2] += + w11[0]*Q11[16] + w11[1]*Q11[17] + w11[2]*Q11[18] + w11[3]*Q11[19] + w11[4]*Q11[20] + w11[5]*Q11[21] + w11[6]*Q11[22] + w11[7]*Q11[23];
mu1[3] += + w11[0]*Q11[24] + w11[1]*Q11[25] + w11[2]*Q11[26] + w11[3]*Q11[27] + w11[4]*Q11[28] + w11[5]*Q11[29] + w11[6]*Q11[30] + w11[7]*Q11[31];
mu1[4] += + w11[0]*Q11[32] + w11[1]*Q11[33] + w11[2]*Q11[34] + w11[3]*Q11[35] + w11[4]*Q11[36] + w11[5]*Q11[37] + w11[6]*Q11[38] + w11[7]*Q11[39];
mu1[5] += + w11[0]*Q11[40] + w11[1]*Q11[41] + w11[2]*Q11[42] + w11[3]*Q11[43] + w11[4]*Q11[44] + w11[5]*Q11[45] + w11[6]*Q11[46] + w11[7]*Q11[47];
mu1[6] += + w11[0]*Q11[48] + w11[1]*Q11[49] + w11[2]*Q11[50] + w11[3]*Q11[51] + w11[4]*Q11[52] + w11[5]*Q11[53] + w11[6]*Q11[54] + w11[7]*Q11[55];
mu1[7] += + w11[0]*Q11[56] + w11[1]*Q11[57] + w11[2]*Q11[58] + w11[3]*Q11[59] + w11[4]*Q11[60] + w11[5]*Q11[61] + w11[6]*Q11[62] + w11[7]*Q11[63];
mu1[0] += + U1[0]*Gu1[0] + U1[1]*Gu1[1];
mu1[1] += + U1[0]*Gu1[2] + U1[1]*Gu1[3];
mu1[2] += + U1[0]*Gu1[4] + U1[1]*Gu1[5];
mu1[3] += + U1[0]*Gu1[6] + U1[1]*Gu1[7];
mu1[4] += + U1[0]*Gu1[8] + U1[1]*Gu1[9];
mu1[5] += + U1[0]*Gu1[10] + U1[1]*Gu1[11];
mu1[6] += + U1[0]*Gu1[12] + U1[1]*Gu1[13];
mu1[7] += + U1[0]*Gu1[14] + U1[1]*Gu1[15];
mu1[0] += + mu2[0]*Gx1[0] + mu2[1]*Gx1[8] + mu2[2]*Gx1[16] + mu2[3]*Gx1[24] + mu2[4]*Gx1[32] + mu2[5]*Gx1[40] + mu2[6]*Gx1[48] + mu2[7]*Gx1[56];
mu1[1] += + mu2[0]*Gx1[1] + mu2[1]*Gx1[9] + mu2[2]*Gx1[17] + mu2[3]*Gx1[25] + mu2[4]*Gx1[33] + mu2[5]*Gx1[41] + mu2[6]*Gx1[49] + mu2[7]*Gx1[57];
mu1[2] += + mu2[0]*Gx1[2] + mu2[1]*Gx1[10] + mu2[2]*Gx1[18] + mu2[3]*Gx1[26] + mu2[4]*Gx1[34] + mu2[5]*Gx1[42] + mu2[6]*Gx1[50] + mu2[7]*Gx1[58];
mu1[3] += + mu2[0]*Gx1[3] + mu2[1]*Gx1[11] + mu2[2]*Gx1[19] + mu2[3]*Gx1[27] + mu2[4]*Gx1[35] + mu2[5]*Gx1[43] + mu2[6]*Gx1[51] + mu2[7]*Gx1[59];
mu1[4] += + mu2[0]*Gx1[4] + mu2[1]*Gx1[12] + mu2[2]*Gx1[20] + mu2[3]*Gx1[28] + mu2[4]*Gx1[36] + mu2[5]*Gx1[44] + mu2[6]*Gx1[52] + mu2[7]*Gx1[60];
mu1[5] += + mu2[0]*Gx1[5] + mu2[1]*Gx1[13] + mu2[2]*Gx1[21] + mu2[3]*Gx1[29] + mu2[4]*Gx1[37] + mu2[5]*Gx1[45] + mu2[6]*Gx1[53] + mu2[7]*Gx1[61];
mu1[6] += + mu2[0]*Gx1[6] + mu2[1]*Gx1[14] + mu2[2]*Gx1[22] + mu2[3]*Gx1[30] + mu2[4]*Gx1[38] + mu2[5]*Gx1[46] + mu2[6]*Gx1[54] + mu2[7]*Gx1[62];
mu1[7] += + mu2[0]*Gx1[7] + mu2[1]*Gx1[15] + mu2[2]*Gx1[23] + mu2[3]*Gx1[31] + mu2[4]*Gx1[39] + mu2[5]*Gx1[47] + mu2[6]*Gx1[55] + mu2[7]*Gx1[63];
}

void acado_LO_copyHTH( int iRow, int iCol )
{
acadoWorkspace_LO.H[(iRow * 40) + (iCol * 2)] = acadoWorkspace_LO.H[(iCol * 40) + (iRow * 2)];
acadoWorkspace_LO.H[(iRow * 40) + (iCol * 2 + 1)] = acadoWorkspace_LO.H[(iCol * 40 + 20) + (iRow * 2)];
acadoWorkspace_LO.H[(iRow * 40 + 20) + (iCol * 2)] = acadoWorkspace_LO.H[(iCol * 40) + (iRow * 2 + 1)];
acadoWorkspace_LO.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = acadoWorkspace_LO.H[(iCol * 40 + 20) + (iRow * 2 + 1)];
}

void acado_LO_multRDy( real_t* const RDy1 )
{
}

void acado_LO_multQDy( real_t* const QDy1 )
{
}

void acado_LO_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace_LO.A[(row * 20 + 1600) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4] + Hx[3]*E[6] + Hx[4]*E[8] + Hx[5]*E[10] + Hx[6]*E[12] + Hx[7]*E[14];
acadoWorkspace_LO.A[(row * 20 + 1600) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5] + Hx[3]*E[7] + Hx[4]*E[9] + Hx[5]*E[11] + Hx[6]*E[13] + Hx[7]*E[15];
}

void acado_LO_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace_LO.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4] + Hx[5]*tmpd[5] + Hx[6]*tmpd[6] + Hx[7]*tmpd[7];
lbA[0] -= acadoWorkspace_LO.evHxd[0];
ubA[0] -= acadoWorkspace_LO.evHxd[0];
}

void acado_LO_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 80 */
static const int xBoundIndices[ 80 ] = 
{ 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87 };
acado_LO_moveGxT( acadoWorkspace_LO.evGx, acadoWorkspace_LO.C );
acado_LO_multGxGx( &(acadoWorkspace_LO.evGx[ 64 ]), acadoWorkspace_LO.C, &(acadoWorkspace_LO.C[ 64 ]) );
acado_LO_multGxGx( &(acadoWorkspace_LO.evGx[ 128 ]), &(acadoWorkspace_LO.C[ 64 ]), &(acadoWorkspace_LO.C[ 128 ]) );
acado_LO_multGxGx( &(acadoWorkspace_LO.evGx[ 192 ]), &(acadoWorkspace_LO.C[ 128 ]), &(acadoWorkspace_LO.C[ 192 ]) );
acado_LO_multGxGx( &(acadoWorkspace_LO.evGx[ 256 ]), &(acadoWorkspace_LO.C[ 192 ]), &(acadoWorkspace_LO.C[ 256 ]) );
acado_LO_multGxGx( &(acadoWorkspace_LO.evGx[ 320 ]), &(acadoWorkspace_LO.C[ 256 ]), &(acadoWorkspace_LO.C[ 320 ]) );
acado_LO_multGxGx( &(acadoWorkspace_LO.evGx[ 384 ]), &(acadoWorkspace_LO.C[ 320 ]), &(acadoWorkspace_LO.C[ 384 ]) );
acado_LO_multGxGx( &(acadoWorkspace_LO.evGx[ 448 ]), &(acadoWorkspace_LO.C[ 384 ]), &(acadoWorkspace_LO.C[ 448 ]) );
acado_LO_multGxGx( &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoWorkspace_LO.C[ 448 ]), &(acadoWorkspace_LO.C[ 512 ]) );
acado_LO_multGxGx( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.C[ 512 ]), &(acadoWorkspace_LO.C[ 576 ]) );
/* Column: 0 */
acado_LO_moveGuE( acadoWorkspace_LO.evGu, acadoWorkspace_LO.E );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 64 ]), acadoWorkspace_LO.E, &(acadoWorkspace_LO.E[ 16 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 128 ]), &(acadoWorkspace_LO.E[ 16 ]), &(acadoWorkspace_LO.E[ 32 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 192 ]), &(acadoWorkspace_LO.E[ 32 ]), &(acadoWorkspace_LO.E[ 48 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 256 ]), &(acadoWorkspace_LO.E[ 48 ]), &(acadoWorkspace_LO.E[ 64 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 320 ]), &(acadoWorkspace_LO.E[ 64 ]), &(acadoWorkspace_LO.E[ 80 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 384 ]), &(acadoWorkspace_LO.E[ 80 ]), &(acadoWorkspace_LO.E[ 96 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 448 ]), &(acadoWorkspace_LO.E[ 96 ]), &(acadoWorkspace_LO.E[ 112 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoWorkspace_LO.E[ 112 ]), &(acadoWorkspace_LO.E[ 128 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.E[ 128 ]), &(acadoWorkspace_LO.E[ 144 ]) );

acado_LO_multGxGu( acadoWorkspace_LO.QN1, &(acadoWorkspace_LO.E[ 144 ]), acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 144 ]), acadoWorkspace_LO.W1, 9, 0 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 144 ]), &(acadoWorkspace_LO.E[ 128 ]), 9, 0 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 576 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 576 ]), &(acadoWorkspace_LO.E[ 128 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 128 ]), acadoWorkspace_LO.W1, 8, 0 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 128 ]), &(acadoWorkspace_LO.E[ 112 ]), 8, 0 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 512 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 512 ]), &(acadoWorkspace_LO.E[ 112 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 112 ]), acadoWorkspace_LO.W1, 7, 0 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 112 ]), &(acadoWorkspace_LO.E[ 96 ]), 7, 0 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 448 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 448 ]), &(acadoWorkspace_LO.E[ 96 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 96 ]), acadoWorkspace_LO.W1, 6, 0 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 96 ]), &(acadoWorkspace_LO.E[ 80 ]), 6, 0 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 384 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 384 ]), &(acadoWorkspace_LO.E[ 80 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 80 ]), acadoWorkspace_LO.W1, 5, 0 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 80 ]), &(acadoWorkspace_LO.E[ 64 ]), 5, 0 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 320 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 320 ]), &(acadoWorkspace_LO.E[ 64 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 64 ]), acadoWorkspace_LO.W1, 4, 0 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 64 ]), &(acadoWorkspace_LO.E[ 48 ]), 4, 0 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 256 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 256 ]), &(acadoWorkspace_LO.E[ 48 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 48 ]), acadoWorkspace_LO.W1, 3, 0 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 48 ]), &(acadoWorkspace_LO.E[ 32 ]), 3, 0 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 192 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 192 ]), &(acadoWorkspace_LO.E[ 32 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 32 ]), acadoWorkspace_LO.W1, 2, 0 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 32 ]), &(acadoWorkspace_LO.E[ 16 ]), 2, 0 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 128 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 128 ]), &(acadoWorkspace_LO.E[ 16 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 16 ]), acadoWorkspace_LO.W1, 1, 0 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 16 ]), acadoWorkspace_LO.E, 1, 0 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 64 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 64 ]), acadoWorkspace_LO.E, acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1_R1( acadoWorkspace_LO.R1, acadoWorkspace_LO.evGu, acadoWorkspace_LO.W1, 0 );

/* Column: 1 */
acado_LO_moveGuE( &(acadoWorkspace_LO.evGu[ 16 ]), &(acadoWorkspace_LO.E[ 160 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 128 ]), &(acadoWorkspace_LO.E[ 160 ]), &(acadoWorkspace_LO.E[ 176 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 192 ]), &(acadoWorkspace_LO.E[ 176 ]), &(acadoWorkspace_LO.E[ 192 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 256 ]), &(acadoWorkspace_LO.E[ 192 ]), &(acadoWorkspace_LO.E[ 208 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 320 ]), &(acadoWorkspace_LO.E[ 208 ]), &(acadoWorkspace_LO.E[ 224 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 384 ]), &(acadoWorkspace_LO.E[ 224 ]), &(acadoWorkspace_LO.E[ 240 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 448 ]), &(acadoWorkspace_LO.E[ 240 ]), &(acadoWorkspace_LO.E[ 256 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoWorkspace_LO.E[ 256 ]), &(acadoWorkspace_LO.E[ 272 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.E[ 272 ]), &(acadoWorkspace_LO.E[ 288 ]) );

acado_LO_multGxGu( acadoWorkspace_LO.QN1, &(acadoWorkspace_LO.E[ 288 ]), acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 144 ]), acadoWorkspace_LO.W1, 9, 1 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 144 ]), &(acadoWorkspace_LO.E[ 272 ]), 9, 1 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 576 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 576 ]), &(acadoWorkspace_LO.E[ 272 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 128 ]), acadoWorkspace_LO.W1, 8, 1 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 128 ]), &(acadoWorkspace_LO.E[ 256 ]), 8, 1 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 512 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 512 ]), &(acadoWorkspace_LO.E[ 256 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 112 ]), acadoWorkspace_LO.W1, 7, 1 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 112 ]), &(acadoWorkspace_LO.E[ 240 ]), 7, 1 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 448 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 448 ]), &(acadoWorkspace_LO.E[ 240 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 96 ]), acadoWorkspace_LO.W1, 6, 1 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 96 ]), &(acadoWorkspace_LO.E[ 224 ]), 6, 1 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 384 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 384 ]), &(acadoWorkspace_LO.E[ 224 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 80 ]), acadoWorkspace_LO.W1, 5, 1 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 80 ]), &(acadoWorkspace_LO.E[ 208 ]), 5, 1 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 320 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 320 ]), &(acadoWorkspace_LO.E[ 208 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 64 ]), acadoWorkspace_LO.W1, 4, 1 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 64 ]), &(acadoWorkspace_LO.E[ 192 ]), 4, 1 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 256 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 256 ]), &(acadoWorkspace_LO.E[ 192 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 48 ]), acadoWorkspace_LO.W1, 3, 1 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 48 ]), &(acadoWorkspace_LO.E[ 176 ]), 3, 1 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 192 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 192 ]), &(acadoWorkspace_LO.E[ 176 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 32 ]), acadoWorkspace_LO.W1, 2, 1 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 32 ]), &(acadoWorkspace_LO.E[ 160 ]), 2, 1 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 128 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 128 ]), &(acadoWorkspace_LO.E[ 160 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1_R1( &(acadoWorkspace_LO.R1[ 4 ]), &(acadoWorkspace_LO.evGu[ 16 ]), acadoWorkspace_LO.W1, 1 );

/* Column: 2 */
acado_LO_moveGuE( &(acadoWorkspace_LO.evGu[ 32 ]), &(acadoWorkspace_LO.E[ 304 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 192 ]), &(acadoWorkspace_LO.E[ 304 ]), &(acadoWorkspace_LO.E[ 320 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 256 ]), &(acadoWorkspace_LO.E[ 320 ]), &(acadoWorkspace_LO.E[ 336 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 320 ]), &(acadoWorkspace_LO.E[ 336 ]), &(acadoWorkspace_LO.E[ 352 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 384 ]), &(acadoWorkspace_LO.E[ 352 ]), &(acadoWorkspace_LO.E[ 368 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 448 ]), &(acadoWorkspace_LO.E[ 368 ]), &(acadoWorkspace_LO.E[ 384 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoWorkspace_LO.E[ 384 ]), &(acadoWorkspace_LO.E[ 400 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.E[ 400 ]), &(acadoWorkspace_LO.E[ 416 ]) );

acado_LO_multGxGu( acadoWorkspace_LO.QN1, &(acadoWorkspace_LO.E[ 416 ]), acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 144 ]), acadoWorkspace_LO.W1, 9, 2 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 144 ]), &(acadoWorkspace_LO.E[ 400 ]), 9, 2 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 576 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 576 ]), &(acadoWorkspace_LO.E[ 400 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 128 ]), acadoWorkspace_LO.W1, 8, 2 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 128 ]), &(acadoWorkspace_LO.E[ 384 ]), 8, 2 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 512 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 512 ]), &(acadoWorkspace_LO.E[ 384 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 112 ]), acadoWorkspace_LO.W1, 7, 2 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 112 ]), &(acadoWorkspace_LO.E[ 368 ]), 7, 2 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 448 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 448 ]), &(acadoWorkspace_LO.E[ 368 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 96 ]), acadoWorkspace_LO.W1, 6, 2 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 96 ]), &(acadoWorkspace_LO.E[ 352 ]), 6, 2 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 384 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 384 ]), &(acadoWorkspace_LO.E[ 352 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 80 ]), acadoWorkspace_LO.W1, 5, 2 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 80 ]), &(acadoWorkspace_LO.E[ 336 ]), 5, 2 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 320 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 320 ]), &(acadoWorkspace_LO.E[ 336 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 64 ]), acadoWorkspace_LO.W1, 4, 2 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 64 ]), &(acadoWorkspace_LO.E[ 320 ]), 4, 2 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 256 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 256 ]), &(acadoWorkspace_LO.E[ 320 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 48 ]), acadoWorkspace_LO.W1, 3, 2 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 48 ]), &(acadoWorkspace_LO.E[ 304 ]), 3, 2 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 192 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 192 ]), &(acadoWorkspace_LO.E[ 304 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1_R1( &(acadoWorkspace_LO.R1[ 8 ]), &(acadoWorkspace_LO.evGu[ 32 ]), acadoWorkspace_LO.W1, 2 );

/* Column: 3 */
acado_LO_moveGuE( &(acadoWorkspace_LO.evGu[ 48 ]), &(acadoWorkspace_LO.E[ 432 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 256 ]), &(acadoWorkspace_LO.E[ 432 ]), &(acadoWorkspace_LO.E[ 448 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 320 ]), &(acadoWorkspace_LO.E[ 448 ]), &(acadoWorkspace_LO.E[ 464 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 384 ]), &(acadoWorkspace_LO.E[ 464 ]), &(acadoWorkspace_LO.E[ 480 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 448 ]), &(acadoWorkspace_LO.E[ 480 ]), &(acadoWorkspace_LO.E[ 496 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoWorkspace_LO.E[ 496 ]), &(acadoWorkspace_LO.E[ 512 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.E[ 512 ]), &(acadoWorkspace_LO.E[ 528 ]) );

acado_LO_multGxGu( acadoWorkspace_LO.QN1, &(acadoWorkspace_LO.E[ 528 ]), acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 144 ]), acadoWorkspace_LO.W1, 9, 3 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 144 ]), &(acadoWorkspace_LO.E[ 512 ]), 9, 3 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 576 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 576 ]), &(acadoWorkspace_LO.E[ 512 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 128 ]), acadoWorkspace_LO.W1, 8, 3 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 128 ]), &(acadoWorkspace_LO.E[ 496 ]), 8, 3 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 512 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 512 ]), &(acadoWorkspace_LO.E[ 496 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 112 ]), acadoWorkspace_LO.W1, 7, 3 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 112 ]), &(acadoWorkspace_LO.E[ 480 ]), 7, 3 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 448 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 448 ]), &(acadoWorkspace_LO.E[ 480 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 96 ]), acadoWorkspace_LO.W1, 6, 3 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 96 ]), &(acadoWorkspace_LO.E[ 464 ]), 6, 3 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 384 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 384 ]), &(acadoWorkspace_LO.E[ 464 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 80 ]), acadoWorkspace_LO.W1, 5, 3 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 80 ]), &(acadoWorkspace_LO.E[ 448 ]), 5, 3 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 320 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 320 ]), &(acadoWorkspace_LO.E[ 448 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 64 ]), acadoWorkspace_LO.W1, 4, 3 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 64 ]), &(acadoWorkspace_LO.E[ 432 ]), 4, 3 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 256 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 256 ]), &(acadoWorkspace_LO.E[ 432 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1_R1( &(acadoWorkspace_LO.R1[ 12 ]), &(acadoWorkspace_LO.evGu[ 48 ]), acadoWorkspace_LO.W1, 3 );

/* Column: 4 */
acado_LO_moveGuE( &(acadoWorkspace_LO.evGu[ 64 ]), &(acadoWorkspace_LO.E[ 544 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 320 ]), &(acadoWorkspace_LO.E[ 544 ]), &(acadoWorkspace_LO.E[ 560 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 384 ]), &(acadoWorkspace_LO.E[ 560 ]), &(acadoWorkspace_LO.E[ 576 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 448 ]), &(acadoWorkspace_LO.E[ 576 ]), &(acadoWorkspace_LO.E[ 592 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoWorkspace_LO.E[ 592 ]), &(acadoWorkspace_LO.E[ 608 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.E[ 608 ]), &(acadoWorkspace_LO.E[ 624 ]) );

acado_LO_multGxGu( acadoWorkspace_LO.QN1, &(acadoWorkspace_LO.E[ 624 ]), acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 144 ]), acadoWorkspace_LO.W1, 9, 4 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 144 ]), &(acadoWorkspace_LO.E[ 608 ]), 9, 4 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 576 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 576 ]), &(acadoWorkspace_LO.E[ 608 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 128 ]), acadoWorkspace_LO.W1, 8, 4 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 128 ]), &(acadoWorkspace_LO.E[ 592 ]), 8, 4 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 512 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 512 ]), &(acadoWorkspace_LO.E[ 592 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 112 ]), acadoWorkspace_LO.W1, 7, 4 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 112 ]), &(acadoWorkspace_LO.E[ 576 ]), 7, 4 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 448 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 448 ]), &(acadoWorkspace_LO.E[ 576 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 96 ]), acadoWorkspace_LO.W1, 6, 4 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 96 ]), &(acadoWorkspace_LO.E[ 560 ]), 6, 4 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 384 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 384 ]), &(acadoWorkspace_LO.E[ 560 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 80 ]), acadoWorkspace_LO.W1, 5, 4 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 80 ]), &(acadoWorkspace_LO.E[ 544 ]), 5, 4 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 320 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 320 ]), &(acadoWorkspace_LO.E[ 544 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1_R1( &(acadoWorkspace_LO.R1[ 16 ]), &(acadoWorkspace_LO.evGu[ 64 ]), acadoWorkspace_LO.W1, 4 );

/* Column: 5 */
acado_LO_moveGuE( &(acadoWorkspace_LO.evGu[ 80 ]), &(acadoWorkspace_LO.E[ 640 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 384 ]), &(acadoWorkspace_LO.E[ 640 ]), &(acadoWorkspace_LO.E[ 656 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 448 ]), &(acadoWorkspace_LO.E[ 656 ]), &(acadoWorkspace_LO.E[ 672 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoWorkspace_LO.E[ 672 ]), &(acadoWorkspace_LO.E[ 688 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.E[ 688 ]), &(acadoWorkspace_LO.E[ 704 ]) );

acado_LO_multGxGu( acadoWorkspace_LO.QN1, &(acadoWorkspace_LO.E[ 704 ]), acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 144 ]), acadoWorkspace_LO.W1, 9, 5 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 144 ]), &(acadoWorkspace_LO.E[ 688 ]), 9, 5 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 576 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 576 ]), &(acadoWorkspace_LO.E[ 688 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 128 ]), acadoWorkspace_LO.W1, 8, 5 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 128 ]), &(acadoWorkspace_LO.E[ 672 ]), 8, 5 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 512 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 512 ]), &(acadoWorkspace_LO.E[ 672 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 112 ]), acadoWorkspace_LO.W1, 7, 5 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 112 ]), &(acadoWorkspace_LO.E[ 656 ]), 7, 5 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 448 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 448 ]), &(acadoWorkspace_LO.E[ 656 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 96 ]), acadoWorkspace_LO.W1, 6, 5 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 96 ]), &(acadoWorkspace_LO.E[ 640 ]), 6, 5 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 384 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 384 ]), &(acadoWorkspace_LO.E[ 640 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1_R1( &(acadoWorkspace_LO.R1[ 20 ]), &(acadoWorkspace_LO.evGu[ 80 ]), acadoWorkspace_LO.W1, 5 );

/* Column: 6 */
acado_LO_moveGuE( &(acadoWorkspace_LO.evGu[ 96 ]), &(acadoWorkspace_LO.E[ 720 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 448 ]), &(acadoWorkspace_LO.E[ 720 ]), &(acadoWorkspace_LO.E[ 736 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoWorkspace_LO.E[ 736 ]), &(acadoWorkspace_LO.E[ 752 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.E[ 752 ]), &(acadoWorkspace_LO.E[ 768 ]) );

acado_LO_multGxGu( acadoWorkspace_LO.QN1, &(acadoWorkspace_LO.E[ 768 ]), acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 144 ]), acadoWorkspace_LO.W1, 9, 6 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 144 ]), &(acadoWorkspace_LO.E[ 752 ]), 9, 6 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 576 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 576 ]), &(acadoWorkspace_LO.E[ 752 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 128 ]), acadoWorkspace_LO.W1, 8, 6 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 128 ]), &(acadoWorkspace_LO.E[ 736 ]), 8, 6 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 512 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 512 ]), &(acadoWorkspace_LO.E[ 736 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 112 ]), acadoWorkspace_LO.W1, 7, 6 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 112 ]), &(acadoWorkspace_LO.E[ 720 ]), 7, 6 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 448 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 448 ]), &(acadoWorkspace_LO.E[ 720 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1_R1( &(acadoWorkspace_LO.R1[ 24 ]), &(acadoWorkspace_LO.evGu[ 96 ]), acadoWorkspace_LO.W1, 6 );

/* Column: 7 */
acado_LO_moveGuE( &(acadoWorkspace_LO.evGu[ 112 ]), &(acadoWorkspace_LO.E[ 784 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoWorkspace_LO.E[ 784 ]), &(acadoWorkspace_LO.E[ 800 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.E[ 800 ]), &(acadoWorkspace_LO.E[ 816 ]) );

acado_LO_multGxGu( acadoWorkspace_LO.QN1, &(acadoWorkspace_LO.E[ 816 ]), acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 144 ]), acadoWorkspace_LO.W1, 9, 7 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 144 ]), &(acadoWorkspace_LO.E[ 800 ]), 9, 7 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 576 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 576 ]), &(acadoWorkspace_LO.E[ 800 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 128 ]), acadoWorkspace_LO.W1, 8, 7 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 128 ]), &(acadoWorkspace_LO.E[ 784 ]), 8, 7 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 512 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 512 ]), &(acadoWorkspace_LO.E[ 784 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1_R1( &(acadoWorkspace_LO.R1[ 28 ]), &(acadoWorkspace_LO.evGu[ 112 ]), acadoWorkspace_LO.W1, 7 );

/* Column: 8 */
acado_LO_moveGuE( &(acadoWorkspace_LO.evGu[ 128 ]), &(acadoWorkspace_LO.E[ 832 ]) );
acado_LO_multGxGu( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.E[ 832 ]), &(acadoWorkspace_LO.E[ 848 ]) );

acado_LO_multGxGu( acadoWorkspace_LO.QN1, &(acadoWorkspace_LO.E[ 848 ]), acadoWorkspace_LO.W1 );
acado_LO_multBTW1( &(acadoWorkspace_LO.evGu[ 144 ]), acadoWorkspace_LO.W1, 9, 8 );
acado_LO_mac_S1T_E( &(acadoWorkspace_LO.S1[ 144 ]), &(acadoWorkspace_LO.E[ 832 ]), 9, 8 );
acado_LO_multGxTGu( &(acadoWorkspace_LO.evGx[ 576 ]), acadoWorkspace_LO.W1, acadoWorkspace_LO.W2 );
acado_LO_multQEW2( &(acadoWorkspace_LO.Q1[ 576 ]), &(acadoWorkspace_LO.E[ 832 ]), acadoWorkspace_LO.W2, acadoWorkspace_LO.W1 );
acado_LO_multBTW1_R1( &(acadoWorkspace_LO.R1[ 32 ]), &(acadoWorkspace_LO.evGu[ 128 ]), acadoWorkspace_LO.W1, 8 );

/* Column: 9 */
acado_LO_moveGuE( &(acadoWorkspace_LO.evGu[ 144 ]), &(acadoWorkspace_LO.E[ 864 ]) );

acado_LO_multGxGu( acadoWorkspace_LO.QN1, &(acadoWorkspace_LO.E[ 864 ]), acadoWorkspace_LO.W1 );
acado_LO_multBTW1_R1( &(acadoWorkspace_LO.R1[ 36 ]), &(acadoWorkspace_LO.evGu[ 144 ]), acadoWorkspace_LO.W1, 9 );

acado_LO_copyHTH( 0, 1 );
acado_LO_copyHTH( 0, 2 );
acado_LO_copyHTH( 1, 2 );
acado_LO_copyHTH( 0, 3 );
acado_LO_copyHTH( 1, 3 );
acado_LO_copyHTH( 2, 3 );
acado_LO_copyHTH( 0, 4 );
acado_LO_copyHTH( 1, 4 );
acado_LO_copyHTH( 2, 4 );
acado_LO_copyHTH( 3, 4 );
acado_LO_copyHTH( 0, 5 );
acado_LO_copyHTH( 1, 5 );
acado_LO_copyHTH( 2, 5 );
acado_LO_copyHTH( 3, 5 );
acado_LO_copyHTH( 4, 5 );
acado_LO_copyHTH( 0, 6 );
acado_LO_copyHTH( 1, 6 );
acado_LO_copyHTH( 2, 6 );
acado_LO_copyHTH( 3, 6 );
acado_LO_copyHTH( 4, 6 );
acado_LO_copyHTH( 5, 6 );
acado_LO_copyHTH( 0, 7 );
acado_LO_copyHTH( 1, 7 );
acado_LO_copyHTH( 2, 7 );
acado_LO_copyHTH( 3, 7 );
acado_LO_copyHTH( 4, 7 );
acado_LO_copyHTH( 5, 7 );
acado_LO_copyHTH( 6, 7 );
acado_LO_copyHTH( 0, 8 );
acado_LO_copyHTH( 1, 8 );
acado_LO_copyHTH( 2, 8 );
acado_LO_copyHTH( 3, 8 );
acado_LO_copyHTH( 4, 8 );
acado_LO_copyHTH( 5, 8 );
acado_LO_copyHTH( 6, 8 );
acado_LO_copyHTH( 7, 8 );
acado_LO_copyHTH( 0, 9 );
acado_LO_copyHTH( 1, 9 );
acado_LO_copyHTH( 2, 9 );
acado_LO_copyHTH( 3, 9 );
acado_LO_copyHTH( 4, 9 );
acado_LO_copyHTH( 5, 9 );
acado_LO_copyHTH( 6, 9 );
acado_LO_copyHTH( 7, 9 );
acado_LO_copyHTH( 8, 9 );

acadoWorkspace_LO.sbar[8] = acadoWorkspace_LO.d[0];
acadoWorkspace_LO.sbar[9] = acadoWorkspace_LO.d[1];
acadoWorkspace_LO.sbar[10] = acadoWorkspace_LO.d[2];
acadoWorkspace_LO.sbar[11] = acadoWorkspace_LO.d[3];
acadoWorkspace_LO.sbar[12] = acadoWorkspace_LO.d[4];
acadoWorkspace_LO.sbar[13] = acadoWorkspace_LO.d[5];
acadoWorkspace_LO.sbar[14] = acadoWorkspace_LO.d[6];
acadoWorkspace_LO.sbar[15] = acadoWorkspace_LO.d[7];
acadoWorkspace_LO.sbar[16] = acadoWorkspace_LO.d[8];
acadoWorkspace_LO.sbar[17] = acadoWorkspace_LO.d[9];
acadoWorkspace_LO.sbar[18] = acadoWorkspace_LO.d[10];
acadoWorkspace_LO.sbar[19] = acadoWorkspace_LO.d[11];
acadoWorkspace_LO.sbar[20] = acadoWorkspace_LO.d[12];
acadoWorkspace_LO.sbar[21] = acadoWorkspace_LO.d[13];
acadoWorkspace_LO.sbar[22] = acadoWorkspace_LO.d[14];
acadoWorkspace_LO.sbar[23] = acadoWorkspace_LO.d[15];
acadoWorkspace_LO.sbar[24] = acadoWorkspace_LO.d[16];
acadoWorkspace_LO.sbar[25] = acadoWorkspace_LO.d[17];
acadoWorkspace_LO.sbar[26] = acadoWorkspace_LO.d[18];
acadoWorkspace_LO.sbar[27] = acadoWorkspace_LO.d[19];
acadoWorkspace_LO.sbar[28] = acadoWorkspace_LO.d[20];
acadoWorkspace_LO.sbar[29] = acadoWorkspace_LO.d[21];
acadoWorkspace_LO.sbar[30] = acadoWorkspace_LO.d[22];
acadoWorkspace_LO.sbar[31] = acadoWorkspace_LO.d[23];
acadoWorkspace_LO.sbar[32] = acadoWorkspace_LO.d[24];
acadoWorkspace_LO.sbar[33] = acadoWorkspace_LO.d[25];
acadoWorkspace_LO.sbar[34] = acadoWorkspace_LO.d[26];
acadoWorkspace_LO.sbar[35] = acadoWorkspace_LO.d[27];
acadoWorkspace_LO.sbar[36] = acadoWorkspace_LO.d[28];
acadoWorkspace_LO.sbar[37] = acadoWorkspace_LO.d[29];
acadoWorkspace_LO.sbar[38] = acadoWorkspace_LO.d[30];
acadoWorkspace_LO.sbar[39] = acadoWorkspace_LO.d[31];
acadoWorkspace_LO.sbar[40] = acadoWorkspace_LO.d[32];
acadoWorkspace_LO.sbar[41] = acadoWorkspace_LO.d[33];
acadoWorkspace_LO.sbar[42] = acadoWorkspace_LO.d[34];
acadoWorkspace_LO.sbar[43] = acadoWorkspace_LO.d[35];
acadoWorkspace_LO.sbar[44] = acadoWorkspace_LO.d[36];
acadoWorkspace_LO.sbar[45] = acadoWorkspace_LO.d[37];
acadoWorkspace_LO.sbar[46] = acadoWorkspace_LO.d[38];
acadoWorkspace_LO.sbar[47] = acadoWorkspace_LO.d[39];
acadoWorkspace_LO.sbar[48] = acadoWorkspace_LO.d[40];
acadoWorkspace_LO.sbar[49] = acadoWorkspace_LO.d[41];
acadoWorkspace_LO.sbar[50] = acadoWorkspace_LO.d[42];
acadoWorkspace_LO.sbar[51] = acadoWorkspace_LO.d[43];
acadoWorkspace_LO.sbar[52] = acadoWorkspace_LO.d[44];
acadoWorkspace_LO.sbar[53] = acadoWorkspace_LO.d[45];
acadoWorkspace_LO.sbar[54] = acadoWorkspace_LO.d[46];
acadoWorkspace_LO.sbar[55] = acadoWorkspace_LO.d[47];
acadoWorkspace_LO.sbar[56] = acadoWorkspace_LO.d[48];
acadoWorkspace_LO.sbar[57] = acadoWorkspace_LO.d[49];
acadoWorkspace_LO.sbar[58] = acadoWorkspace_LO.d[50];
acadoWorkspace_LO.sbar[59] = acadoWorkspace_LO.d[51];
acadoWorkspace_LO.sbar[60] = acadoWorkspace_LO.d[52];
acadoWorkspace_LO.sbar[61] = acadoWorkspace_LO.d[53];
acadoWorkspace_LO.sbar[62] = acadoWorkspace_LO.d[54];
acadoWorkspace_LO.sbar[63] = acadoWorkspace_LO.d[55];
acadoWorkspace_LO.sbar[64] = acadoWorkspace_LO.d[56];
acadoWorkspace_LO.sbar[65] = acadoWorkspace_LO.d[57];
acadoWorkspace_LO.sbar[66] = acadoWorkspace_LO.d[58];
acadoWorkspace_LO.sbar[67] = acadoWorkspace_LO.d[59];
acadoWorkspace_LO.sbar[68] = acadoWorkspace_LO.d[60];
acadoWorkspace_LO.sbar[69] = acadoWorkspace_LO.d[61];
acadoWorkspace_LO.sbar[70] = acadoWorkspace_LO.d[62];
acadoWorkspace_LO.sbar[71] = acadoWorkspace_LO.d[63];
acadoWorkspace_LO.sbar[72] = acadoWorkspace_LO.d[64];
acadoWorkspace_LO.sbar[73] = acadoWorkspace_LO.d[65];
acadoWorkspace_LO.sbar[74] = acadoWorkspace_LO.d[66];
acadoWorkspace_LO.sbar[75] = acadoWorkspace_LO.d[67];
acadoWorkspace_LO.sbar[76] = acadoWorkspace_LO.d[68];
acadoWorkspace_LO.sbar[77] = acadoWorkspace_LO.d[69];
acadoWorkspace_LO.sbar[78] = acadoWorkspace_LO.d[70];
acadoWorkspace_LO.sbar[79] = acadoWorkspace_LO.d[71];
acadoWorkspace_LO.sbar[80] = acadoWorkspace_LO.d[72];
acadoWorkspace_LO.sbar[81] = acadoWorkspace_LO.d[73];
acadoWorkspace_LO.sbar[82] = acadoWorkspace_LO.d[74];
acadoWorkspace_LO.sbar[83] = acadoWorkspace_LO.d[75];
acadoWorkspace_LO.sbar[84] = acadoWorkspace_LO.d[76];
acadoWorkspace_LO.sbar[85] = acadoWorkspace_LO.d[77];
acadoWorkspace_LO.sbar[86] = acadoWorkspace_LO.d[78];
acadoWorkspace_LO.sbar[87] = acadoWorkspace_LO.d[79];

for (lRun1 = 0; lRun1 < 80; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 8;
lRun4 = ((lRun3) / (8)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 19)) / (2)) + (lRun4)) - (1)) * (8)) + ((lRun3) % (8));
acadoWorkspace_LO.A[(lRun1 * 20) + (lRun2 * 2)] = acadoWorkspace_LO.E[lRun5 * 2];
acadoWorkspace_LO.A[(lRun1 * 20) + (lRun2 * 2 + 1)] = acadoWorkspace_LO.E[lRun5 * 2 + 1];
}
}



acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 8 ]), acadoWorkspace_LO.E, 1, 0 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 16 ]), &(acadoWorkspace_LO.E[ 16 ]), 2, 0 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 16 ]), &(acadoWorkspace_LO.E[ 160 ]), 2, 1 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 24 ]), &(acadoWorkspace_LO.E[ 32 ]), 3, 0 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 24 ]), &(acadoWorkspace_LO.E[ 176 ]), 3, 1 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 24 ]), &(acadoWorkspace_LO.E[ 304 ]), 3, 2 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 32 ]), &(acadoWorkspace_LO.E[ 48 ]), 4, 0 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 32 ]), &(acadoWorkspace_LO.E[ 192 ]), 4, 1 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 32 ]), &(acadoWorkspace_LO.E[ 320 ]), 4, 2 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 32 ]), &(acadoWorkspace_LO.E[ 432 ]), 4, 3 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 40 ]), &(acadoWorkspace_LO.E[ 64 ]), 5, 0 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 40 ]), &(acadoWorkspace_LO.E[ 208 ]), 5, 1 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 40 ]), &(acadoWorkspace_LO.E[ 336 ]), 5, 2 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 40 ]), &(acadoWorkspace_LO.E[ 448 ]), 5, 3 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 40 ]), &(acadoWorkspace_LO.E[ 544 ]), 5, 4 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 48 ]), &(acadoWorkspace_LO.E[ 80 ]), 6, 0 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 48 ]), &(acadoWorkspace_LO.E[ 224 ]), 6, 1 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 48 ]), &(acadoWorkspace_LO.E[ 352 ]), 6, 2 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 48 ]), &(acadoWorkspace_LO.E[ 464 ]), 6, 3 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 48 ]), &(acadoWorkspace_LO.E[ 560 ]), 6, 4 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 48 ]), &(acadoWorkspace_LO.E[ 640 ]), 6, 5 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 56 ]), &(acadoWorkspace_LO.E[ 96 ]), 7, 0 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 56 ]), &(acadoWorkspace_LO.E[ 240 ]), 7, 1 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 56 ]), &(acadoWorkspace_LO.E[ 368 ]), 7, 2 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 56 ]), &(acadoWorkspace_LO.E[ 480 ]), 7, 3 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 56 ]), &(acadoWorkspace_LO.E[ 576 ]), 7, 4 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 56 ]), &(acadoWorkspace_LO.E[ 656 ]), 7, 5 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 56 ]), &(acadoWorkspace_LO.E[ 720 ]), 7, 6 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 64 ]), &(acadoWorkspace_LO.E[ 112 ]), 8, 0 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 64 ]), &(acadoWorkspace_LO.E[ 256 ]), 8, 1 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 64 ]), &(acadoWorkspace_LO.E[ 384 ]), 8, 2 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 64 ]), &(acadoWorkspace_LO.E[ 496 ]), 8, 3 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 64 ]), &(acadoWorkspace_LO.E[ 592 ]), 8, 4 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 64 ]), &(acadoWorkspace_LO.E[ 672 ]), 8, 5 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 64 ]), &(acadoWorkspace_LO.E[ 736 ]), 8, 6 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 64 ]), &(acadoWorkspace_LO.E[ 784 ]), 8, 7 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 72 ]), &(acadoWorkspace_LO.E[ 128 ]), 9, 0 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 72 ]), &(acadoWorkspace_LO.E[ 272 ]), 9, 1 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 72 ]), &(acadoWorkspace_LO.E[ 400 ]), 9, 2 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 72 ]), &(acadoWorkspace_LO.E[ 512 ]), 9, 3 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 72 ]), &(acadoWorkspace_LO.E[ 608 ]), 9, 4 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 72 ]), &(acadoWorkspace_LO.E[ 688 ]), 9, 5 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 72 ]), &(acadoWorkspace_LO.E[ 752 ]), 9, 6 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 72 ]), &(acadoWorkspace_LO.E[ 800 ]), 9, 7 );
acado_LO_multHxE( &(acadoWorkspace_LO.evHx[ 72 ]), &(acadoWorkspace_LO.E[ 832 ]), 9, 8 );

acadoWorkspace_LO.A[1600] = acadoWorkspace_LO.evHu[0];
acadoWorkspace_LO.A[1601] = acadoWorkspace_LO.evHu[1];
acadoWorkspace_LO.A[1622] = acadoWorkspace_LO.evHu[2];
acadoWorkspace_LO.A[1623] = acadoWorkspace_LO.evHu[3];
acadoWorkspace_LO.A[1644] = acadoWorkspace_LO.evHu[4];
acadoWorkspace_LO.A[1645] = acadoWorkspace_LO.evHu[5];
acadoWorkspace_LO.A[1666] = acadoWorkspace_LO.evHu[6];
acadoWorkspace_LO.A[1667] = acadoWorkspace_LO.evHu[7];
acadoWorkspace_LO.A[1688] = acadoWorkspace_LO.evHu[8];
acadoWorkspace_LO.A[1689] = acadoWorkspace_LO.evHu[9];
acadoWorkspace_LO.A[1710] = acadoWorkspace_LO.evHu[10];
acadoWorkspace_LO.A[1711] = acadoWorkspace_LO.evHu[11];
acadoWorkspace_LO.A[1732] = acadoWorkspace_LO.evHu[12];
acadoWorkspace_LO.A[1733] = acadoWorkspace_LO.evHu[13];
acadoWorkspace_LO.A[1754] = acadoWorkspace_LO.evHu[14];
acadoWorkspace_LO.A[1755] = acadoWorkspace_LO.evHu[15];
acadoWorkspace_LO.A[1776] = acadoWorkspace_LO.evHu[16];
acadoWorkspace_LO.A[1777] = acadoWorkspace_LO.evHu[17];
acadoWorkspace_LO.A[1798] = acadoWorkspace_LO.evHu[18];
acadoWorkspace_LO.A[1799] = acadoWorkspace_LO.evHu[19];
acadoWorkspace_LO.lbA[80] = acadoVariables_LO.lbAValues[80] - acadoWorkspace_LO.evH[0];
acadoWorkspace_LO.lbA[81] = acadoVariables_LO.lbAValues[81] - acadoWorkspace_LO.evH[1];
acadoWorkspace_LO.lbA[82] = acadoVariables_LO.lbAValues[82] - acadoWorkspace_LO.evH[2];
acadoWorkspace_LO.lbA[83] = acadoVariables_LO.lbAValues[83] - acadoWorkspace_LO.evH[3];
acadoWorkspace_LO.lbA[84] = acadoVariables_LO.lbAValues[84] - acadoWorkspace_LO.evH[4];
acadoWorkspace_LO.lbA[85] = acadoVariables_LO.lbAValues[85] - acadoWorkspace_LO.evH[5];
acadoWorkspace_LO.lbA[86] = acadoVariables_LO.lbAValues[86] - acadoWorkspace_LO.evH[6];
acadoWorkspace_LO.lbA[87] = acadoVariables_LO.lbAValues[87] - acadoWorkspace_LO.evH[7];
acadoWorkspace_LO.lbA[88] = acadoVariables_LO.lbAValues[88] - acadoWorkspace_LO.evH[8];
acadoWorkspace_LO.lbA[89] = acadoVariables_LO.lbAValues[89] - acadoWorkspace_LO.evH[9];

acadoWorkspace_LO.ubA[80] = acadoVariables_LO.ubAValues[80] - acadoWorkspace_LO.evH[0];
acadoWorkspace_LO.ubA[81] = acadoVariables_LO.ubAValues[81] - acadoWorkspace_LO.evH[1];
acadoWorkspace_LO.ubA[82] = acadoVariables_LO.ubAValues[82] - acadoWorkspace_LO.evH[2];
acadoWorkspace_LO.ubA[83] = acadoVariables_LO.ubAValues[83] - acadoWorkspace_LO.evH[3];
acadoWorkspace_LO.ubA[84] = acadoVariables_LO.ubAValues[84] - acadoWorkspace_LO.evH[4];
acadoWorkspace_LO.ubA[85] = acadoVariables_LO.ubAValues[85] - acadoWorkspace_LO.evH[5];
acadoWorkspace_LO.ubA[86] = acadoVariables_LO.ubAValues[86] - acadoWorkspace_LO.evH[6];
acadoWorkspace_LO.ubA[87] = acadoVariables_LO.ubAValues[87] - acadoWorkspace_LO.evH[7];
acadoWorkspace_LO.ubA[88] = acadoVariables_LO.ubAValues[88] - acadoWorkspace_LO.evH[8];
acadoWorkspace_LO.ubA[89] = acadoVariables_LO.ubAValues[89] - acadoWorkspace_LO.evH[9];

}

void acado_LO_condenseFdb(  )
{
real_t tmp;

acadoWorkspace_LO.Dx0[0] = acadoVariables_LO.x0[0] - acadoVariables_LO.x[0];
acadoWorkspace_LO.Dx0[1] = acadoVariables_LO.x0[1] - acadoVariables_LO.x[1];
acadoWorkspace_LO.Dx0[2] = acadoVariables_LO.x0[2] - acadoVariables_LO.x[2];
acadoWorkspace_LO.Dx0[3] = acadoVariables_LO.x0[3] - acadoVariables_LO.x[3];
acadoWorkspace_LO.Dx0[4] = acadoVariables_LO.x0[4] - acadoVariables_LO.x[4];
acadoWorkspace_LO.Dx0[5] = acadoVariables_LO.x0[5] - acadoVariables_LO.x[5];
acadoWorkspace_LO.Dx0[6] = acadoVariables_LO.x0[6] - acadoVariables_LO.x[6];
acadoWorkspace_LO.Dx0[7] = acadoVariables_LO.x0[7] - acadoVariables_LO.x[7];

acadoWorkspace_LO.sbar[0] = acadoWorkspace_LO.Dx0[0];
acadoWorkspace_LO.sbar[1] = acadoWorkspace_LO.Dx0[1];
acadoWorkspace_LO.sbar[2] = acadoWorkspace_LO.Dx0[2];
acadoWorkspace_LO.sbar[3] = acadoWorkspace_LO.Dx0[3];
acadoWorkspace_LO.sbar[4] = acadoWorkspace_LO.Dx0[4];
acadoWorkspace_LO.sbar[5] = acadoWorkspace_LO.Dx0[5];
acadoWorkspace_LO.sbar[6] = acadoWorkspace_LO.Dx0[6];
acadoWorkspace_LO.sbar[7] = acadoWorkspace_LO.Dx0[7];
acado_LO_macASbar( acadoWorkspace_LO.evGx, acadoWorkspace_LO.sbar, &(acadoWorkspace_LO.sbar[ 8 ]) );
acado_LO_macASbar( &(acadoWorkspace_LO.evGx[ 64 ]), &(acadoWorkspace_LO.sbar[ 8 ]), &(acadoWorkspace_LO.sbar[ 16 ]) );
acado_LO_macASbar( &(acadoWorkspace_LO.evGx[ 128 ]), &(acadoWorkspace_LO.sbar[ 16 ]), &(acadoWorkspace_LO.sbar[ 24 ]) );
acado_LO_macASbar( &(acadoWorkspace_LO.evGx[ 192 ]), &(acadoWorkspace_LO.sbar[ 24 ]), &(acadoWorkspace_LO.sbar[ 32 ]) );
acado_LO_macASbar( &(acadoWorkspace_LO.evGx[ 256 ]), &(acadoWorkspace_LO.sbar[ 32 ]), &(acadoWorkspace_LO.sbar[ 40 ]) );
acado_LO_macASbar( &(acadoWorkspace_LO.evGx[ 320 ]), &(acadoWorkspace_LO.sbar[ 40 ]), &(acadoWorkspace_LO.sbar[ 48 ]) );
acado_LO_macASbar( &(acadoWorkspace_LO.evGx[ 384 ]), &(acadoWorkspace_LO.sbar[ 48 ]), &(acadoWorkspace_LO.sbar[ 56 ]) );
acado_LO_macASbar( &(acadoWorkspace_LO.evGx[ 448 ]), &(acadoWorkspace_LO.sbar[ 56 ]), &(acadoWorkspace_LO.sbar[ 64 ]) );
acado_LO_macASbar( &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoWorkspace_LO.sbar[ 64 ]), &(acadoWorkspace_LO.sbar[ 72 ]) );
acado_LO_macASbar( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.sbar[ 72 ]), &(acadoWorkspace_LO.sbar[ 80 ]) );

acadoWorkspace_LO.w1[0] = + acadoWorkspace_LO.QN1[0]*acadoWorkspace_LO.sbar[80] + acadoWorkspace_LO.QN1[1]*acadoWorkspace_LO.sbar[81] + acadoWorkspace_LO.QN1[2]*acadoWorkspace_LO.sbar[82] + acadoWorkspace_LO.QN1[3]*acadoWorkspace_LO.sbar[83] + acadoWorkspace_LO.QN1[4]*acadoWorkspace_LO.sbar[84] + acadoWorkspace_LO.QN1[5]*acadoWorkspace_LO.sbar[85] + acadoWorkspace_LO.QN1[6]*acadoWorkspace_LO.sbar[86] + acadoWorkspace_LO.QN1[7]*acadoWorkspace_LO.sbar[87] + acadoWorkspace_LO.QDy[80];
acadoWorkspace_LO.w1[1] = + acadoWorkspace_LO.QN1[8]*acadoWorkspace_LO.sbar[80] + acadoWorkspace_LO.QN1[9]*acadoWorkspace_LO.sbar[81] + acadoWorkspace_LO.QN1[10]*acadoWorkspace_LO.sbar[82] + acadoWorkspace_LO.QN1[11]*acadoWorkspace_LO.sbar[83] + acadoWorkspace_LO.QN1[12]*acadoWorkspace_LO.sbar[84] + acadoWorkspace_LO.QN1[13]*acadoWorkspace_LO.sbar[85] + acadoWorkspace_LO.QN1[14]*acadoWorkspace_LO.sbar[86] + acadoWorkspace_LO.QN1[15]*acadoWorkspace_LO.sbar[87] + acadoWorkspace_LO.QDy[81];
acadoWorkspace_LO.w1[2] = + acadoWorkspace_LO.QN1[16]*acadoWorkspace_LO.sbar[80] + acadoWorkspace_LO.QN1[17]*acadoWorkspace_LO.sbar[81] + acadoWorkspace_LO.QN1[18]*acadoWorkspace_LO.sbar[82] + acadoWorkspace_LO.QN1[19]*acadoWorkspace_LO.sbar[83] + acadoWorkspace_LO.QN1[20]*acadoWorkspace_LO.sbar[84] + acadoWorkspace_LO.QN1[21]*acadoWorkspace_LO.sbar[85] + acadoWorkspace_LO.QN1[22]*acadoWorkspace_LO.sbar[86] + acadoWorkspace_LO.QN1[23]*acadoWorkspace_LO.sbar[87] + acadoWorkspace_LO.QDy[82];
acadoWorkspace_LO.w1[3] = + acadoWorkspace_LO.QN1[24]*acadoWorkspace_LO.sbar[80] + acadoWorkspace_LO.QN1[25]*acadoWorkspace_LO.sbar[81] + acadoWorkspace_LO.QN1[26]*acadoWorkspace_LO.sbar[82] + acadoWorkspace_LO.QN1[27]*acadoWorkspace_LO.sbar[83] + acadoWorkspace_LO.QN1[28]*acadoWorkspace_LO.sbar[84] + acadoWorkspace_LO.QN1[29]*acadoWorkspace_LO.sbar[85] + acadoWorkspace_LO.QN1[30]*acadoWorkspace_LO.sbar[86] + acadoWorkspace_LO.QN1[31]*acadoWorkspace_LO.sbar[87] + acadoWorkspace_LO.QDy[83];
acadoWorkspace_LO.w1[4] = + acadoWorkspace_LO.QN1[32]*acadoWorkspace_LO.sbar[80] + acadoWorkspace_LO.QN1[33]*acadoWorkspace_LO.sbar[81] + acadoWorkspace_LO.QN1[34]*acadoWorkspace_LO.sbar[82] + acadoWorkspace_LO.QN1[35]*acadoWorkspace_LO.sbar[83] + acadoWorkspace_LO.QN1[36]*acadoWorkspace_LO.sbar[84] + acadoWorkspace_LO.QN1[37]*acadoWorkspace_LO.sbar[85] + acadoWorkspace_LO.QN1[38]*acadoWorkspace_LO.sbar[86] + acadoWorkspace_LO.QN1[39]*acadoWorkspace_LO.sbar[87] + acadoWorkspace_LO.QDy[84];
acadoWorkspace_LO.w1[5] = + acadoWorkspace_LO.QN1[40]*acadoWorkspace_LO.sbar[80] + acadoWorkspace_LO.QN1[41]*acadoWorkspace_LO.sbar[81] + acadoWorkspace_LO.QN1[42]*acadoWorkspace_LO.sbar[82] + acadoWorkspace_LO.QN1[43]*acadoWorkspace_LO.sbar[83] + acadoWorkspace_LO.QN1[44]*acadoWorkspace_LO.sbar[84] + acadoWorkspace_LO.QN1[45]*acadoWorkspace_LO.sbar[85] + acadoWorkspace_LO.QN1[46]*acadoWorkspace_LO.sbar[86] + acadoWorkspace_LO.QN1[47]*acadoWorkspace_LO.sbar[87] + acadoWorkspace_LO.QDy[85];
acadoWorkspace_LO.w1[6] = + acadoWorkspace_LO.QN1[48]*acadoWorkspace_LO.sbar[80] + acadoWorkspace_LO.QN1[49]*acadoWorkspace_LO.sbar[81] + acadoWorkspace_LO.QN1[50]*acadoWorkspace_LO.sbar[82] + acadoWorkspace_LO.QN1[51]*acadoWorkspace_LO.sbar[83] + acadoWorkspace_LO.QN1[52]*acadoWorkspace_LO.sbar[84] + acadoWorkspace_LO.QN1[53]*acadoWorkspace_LO.sbar[85] + acadoWorkspace_LO.QN1[54]*acadoWorkspace_LO.sbar[86] + acadoWorkspace_LO.QN1[55]*acadoWorkspace_LO.sbar[87] + acadoWorkspace_LO.QDy[86];
acadoWorkspace_LO.w1[7] = + acadoWorkspace_LO.QN1[56]*acadoWorkspace_LO.sbar[80] + acadoWorkspace_LO.QN1[57]*acadoWorkspace_LO.sbar[81] + acadoWorkspace_LO.QN1[58]*acadoWorkspace_LO.sbar[82] + acadoWorkspace_LO.QN1[59]*acadoWorkspace_LO.sbar[83] + acadoWorkspace_LO.QN1[60]*acadoWorkspace_LO.sbar[84] + acadoWorkspace_LO.QN1[61]*acadoWorkspace_LO.sbar[85] + acadoWorkspace_LO.QN1[62]*acadoWorkspace_LO.sbar[86] + acadoWorkspace_LO.QN1[63]*acadoWorkspace_LO.sbar[87] + acadoWorkspace_LO.QDy[87];
acado_LO_macBTw1( &(acadoWorkspace_LO.evGu[ 144 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.g[ 18 ]) );
acado_LO_macS1TSbar( &(acadoWorkspace_LO.S1[ 144 ]), &(acadoWorkspace_LO.sbar[ 72 ]), &(acadoWorkspace_LO.g[ 18 ]) );
acado_LO_macATw1QDy( &(acadoWorkspace_LO.evGx[ 576 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.QDy[ 72 ]), acadoWorkspace_LO.w2 );
acado_LO_macQSbarW2( &(acadoWorkspace_LO.Q1[ 576 ]), &(acadoWorkspace_LO.sbar[ 72 ]), acadoWorkspace_LO.w2, acadoWorkspace_LO.w1 );
acado_LO_macBTw1( &(acadoWorkspace_LO.evGu[ 128 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.g[ 16 ]) );
acado_LO_macS1TSbar( &(acadoWorkspace_LO.S1[ 128 ]), &(acadoWorkspace_LO.sbar[ 64 ]), &(acadoWorkspace_LO.g[ 16 ]) );
acado_LO_macATw1QDy( &(acadoWorkspace_LO.evGx[ 512 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.QDy[ 64 ]), acadoWorkspace_LO.w2 );
acado_LO_macQSbarW2( &(acadoWorkspace_LO.Q1[ 512 ]), &(acadoWorkspace_LO.sbar[ 64 ]), acadoWorkspace_LO.w2, acadoWorkspace_LO.w1 );
acado_LO_macBTw1( &(acadoWorkspace_LO.evGu[ 112 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.g[ 14 ]) );
acado_LO_macS1TSbar( &(acadoWorkspace_LO.S1[ 112 ]), &(acadoWorkspace_LO.sbar[ 56 ]), &(acadoWorkspace_LO.g[ 14 ]) );
acado_LO_macATw1QDy( &(acadoWorkspace_LO.evGx[ 448 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.QDy[ 56 ]), acadoWorkspace_LO.w2 );
acado_LO_macQSbarW2( &(acadoWorkspace_LO.Q1[ 448 ]), &(acadoWorkspace_LO.sbar[ 56 ]), acadoWorkspace_LO.w2, acadoWorkspace_LO.w1 );
acado_LO_macBTw1( &(acadoWorkspace_LO.evGu[ 96 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.g[ 12 ]) );
acado_LO_macS1TSbar( &(acadoWorkspace_LO.S1[ 96 ]), &(acadoWorkspace_LO.sbar[ 48 ]), &(acadoWorkspace_LO.g[ 12 ]) );
acado_LO_macATw1QDy( &(acadoWorkspace_LO.evGx[ 384 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.QDy[ 48 ]), acadoWorkspace_LO.w2 );
acado_LO_macQSbarW2( &(acadoWorkspace_LO.Q1[ 384 ]), &(acadoWorkspace_LO.sbar[ 48 ]), acadoWorkspace_LO.w2, acadoWorkspace_LO.w1 );
acado_LO_macBTw1( &(acadoWorkspace_LO.evGu[ 80 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.g[ 10 ]) );
acado_LO_macS1TSbar( &(acadoWorkspace_LO.S1[ 80 ]), &(acadoWorkspace_LO.sbar[ 40 ]), &(acadoWorkspace_LO.g[ 10 ]) );
acado_LO_macATw1QDy( &(acadoWorkspace_LO.evGx[ 320 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.QDy[ 40 ]), acadoWorkspace_LO.w2 );
acado_LO_macQSbarW2( &(acadoWorkspace_LO.Q1[ 320 ]), &(acadoWorkspace_LO.sbar[ 40 ]), acadoWorkspace_LO.w2, acadoWorkspace_LO.w1 );
acado_LO_macBTw1( &(acadoWorkspace_LO.evGu[ 64 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.g[ 8 ]) );
acado_LO_macS1TSbar( &(acadoWorkspace_LO.S1[ 64 ]), &(acadoWorkspace_LO.sbar[ 32 ]), &(acadoWorkspace_LO.g[ 8 ]) );
acado_LO_macATw1QDy( &(acadoWorkspace_LO.evGx[ 256 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.QDy[ 32 ]), acadoWorkspace_LO.w2 );
acado_LO_macQSbarW2( &(acadoWorkspace_LO.Q1[ 256 ]), &(acadoWorkspace_LO.sbar[ 32 ]), acadoWorkspace_LO.w2, acadoWorkspace_LO.w1 );
acado_LO_macBTw1( &(acadoWorkspace_LO.evGu[ 48 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.g[ 6 ]) );
acado_LO_macS1TSbar( &(acadoWorkspace_LO.S1[ 48 ]), &(acadoWorkspace_LO.sbar[ 24 ]), &(acadoWorkspace_LO.g[ 6 ]) );
acado_LO_macATw1QDy( &(acadoWorkspace_LO.evGx[ 192 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.QDy[ 24 ]), acadoWorkspace_LO.w2 );
acado_LO_macQSbarW2( &(acadoWorkspace_LO.Q1[ 192 ]), &(acadoWorkspace_LO.sbar[ 24 ]), acadoWorkspace_LO.w2, acadoWorkspace_LO.w1 );
acado_LO_macBTw1( &(acadoWorkspace_LO.evGu[ 32 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.g[ 4 ]) );
acado_LO_macS1TSbar( &(acadoWorkspace_LO.S1[ 32 ]), &(acadoWorkspace_LO.sbar[ 16 ]), &(acadoWorkspace_LO.g[ 4 ]) );
acado_LO_macATw1QDy( &(acadoWorkspace_LO.evGx[ 128 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.QDy[ 16 ]), acadoWorkspace_LO.w2 );
acado_LO_macQSbarW2( &(acadoWorkspace_LO.Q1[ 128 ]), &(acadoWorkspace_LO.sbar[ 16 ]), acadoWorkspace_LO.w2, acadoWorkspace_LO.w1 );
acado_LO_macBTw1( &(acadoWorkspace_LO.evGu[ 16 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.g[ 2 ]) );
acado_LO_macS1TSbar( &(acadoWorkspace_LO.S1[ 16 ]), &(acadoWorkspace_LO.sbar[ 8 ]), &(acadoWorkspace_LO.g[ 2 ]) );
acado_LO_macATw1QDy( &(acadoWorkspace_LO.evGx[ 64 ]), acadoWorkspace_LO.w1, &(acadoWorkspace_LO.QDy[ 8 ]), acadoWorkspace_LO.w2 );
acado_LO_macQSbarW2( &(acadoWorkspace_LO.Q1[ 64 ]), &(acadoWorkspace_LO.sbar[ 8 ]), acadoWorkspace_LO.w2, acadoWorkspace_LO.w1 );
acado_LO_macBTw1( acadoWorkspace_LO.evGu, acadoWorkspace_LO.w1, acadoWorkspace_LO.g );
acado_LO_macS1TSbar( acadoWorkspace_LO.S1, acadoWorkspace_LO.sbar, acadoWorkspace_LO.g );

acadoWorkspace_LO.lb[0] = acadoVariables_LO.lbValues[0] - acadoVariables_LO.u[0];
acadoWorkspace_LO.lb[1] = acadoVariables_LO.lbValues[1] - acadoVariables_LO.u[1];
acadoWorkspace_LO.lb[2] = acadoVariables_LO.lbValues[2] - acadoVariables_LO.u[2];
acadoWorkspace_LO.lb[3] = acadoVariables_LO.lbValues[3] - acadoVariables_LO.u[3];
acadoWorkspace_LO.lb[4] = acadoVariables_LO.lbValues[4] - acadoVariables_LO.u[4];
acadoWorkspace_LO.lb[5] = acadoVariables_LO.lbValues[5] - acadoVariables_LO.u[5];
acadoWorkspace_LO.lb[6] = acadoVariables_LO.lbValues[6] - acadoVariables_LO.u[6];
acadoWorkspace_LO.lb[7] = acadoVariables_LO.lbValues[7] - acadoVariables_LO.u[7];
acadoWorkspace_LO.lb[8] = acadoVariables_LO.lbValues[8] - acadoVariables_LO.u[8];
acadoWorkspace_LO.lb[9] = acadoVariables_LO.lbValues[9] - acadoVariables_LO.u[9];
acadoWorkspace_LO.lb[10] = acadoVariables_LO.lbValues[10] - acadoVariables_LO.u[10];
acadoWorkspace_LO.lb[11] = acadoVariables_LO.lbValues[11] - acadoVariables_LO.u[11];
acadoWorkspace_LO.lb[12] = acadoVariables_LO.lbValues[12] - acadoVariables_LO.u[12];
acadoWorkspace_LO.lb[13] = acadoVariables_LO.lbValues[13] - acadoVariables_LO.u[13];
acadoWorkspace_LO.lb[14] = acadoVariables_LO.lbValues[14] - acadoVariables_LO.u[14];
acadoWorkspace_LO.lb[15] = acadoVariables_LO.lbValues[15] - acadoVariables_LO.u[15];
acadoWorkspace_LO.lb[16] = acadoVariables_LO.lbValues[16] - acadoVariables_LO.u[16];
acadoWorkspace_LO.lb[17] = acadoVariables_LO.lbValues[17] - acadoVariables_LO.u[17];
acadoWorkspace_LO.lb[18] = acadoVariables_LO.lbValues[18] - acadoVariables_LO.u[18];
acadoWorkspace_LO.lb[19] = acadoVariables_LO.lbValues[19] - acadoVariables_LO.u[19];
acadoWorkspace_LO.ub[0] = acadoVariables_LO.ubValues[0] - acadoVariables_LO.u[0];
acadoWorkspace_LO.ub[1] = acadoVariables_LO.ubValues[1] - acadoVariables_LO.u[1];
acadoWorkspace_LO.ub[2] = acadoVariables_LO.ubValues[2] - acadoVariables_LO.u[2];
acadoWorkspace_LO.ub[3] = acadoVariables_LO.ubValues[3] - acadoVariables_LO.u[3];
acadoWorkspace_LO.ub[4] = acadoVariables_LO.ubValues[4] - acadoVariables_LO.u[4];
acadoWorkspace_LO.ub[5] = acadoVariables_LO.ubValues[5] - acadoVariables_LO.u[5];
acadoWorkspace_LO.ub[6] = acadoVariables_LO.ubValues[6] - acadoVariables_LO.u[6];
acadoWorkspace_LO.ub[7] = acadoVariables_LO.ubValues[7] - acadoVariables_LO.u[7];
acadoWorkspace_LO.ub[8] = acadoVariables_LO.ubValues[8] - acadoVariables_LO.u[8];
acadoWorkspace_LO.ub[9] = acadoVariables_LO.ubValues[9] - acadoVariables_LO.u[9];
acadoWorkspace_LO.ub[10] = acadoVariables_LO.ubValues[10] - acadoVariables_LO.u[10];
acadoWorkspace_LO.ub[11] = acadoVariables_LO.ubValues[11] - acadoVariables_LO.u[11];
acadoWorkspace_LO.ub[12] = acadoVariables_LO.ubValues[12] - acadoVariables_LO.u[12];
acadoWorkspace_LO.ub[13] = acadoVariables_LO.ubValues[13] - acadoVariables_LO.u[13];
acadoWorkspace_LO.ub[14] = acadoVariables_LO.ubValues[14] - acadoVariables_LO.u[14];
acadoWorkspace_LO.ub[15] = acadoVariables_LO.ubValues[15] - acadoVariables_LO.u[15];
acadoWorkspace_LO.ub[16] = acadoVariables_LO.ubValues[16] - acadoVariables_LO.u[16];
acadoWorkspace_LO.ub[17] = acadoVariables_LO.ubValues[17] - acadoVariables_LO.u[17];
acadoWorkspace_LO.ub[18] = acadoVariables_LO.ubValues[18] - acadoVariables_LO.u[18];
acadoWorkspace_LO.ub[19] = acadoVariables_LO.ubValues[19] - acadoVariables_LO.u[19];

tmp = acadoWorkspace_LO.sbar[8] + acadoVariables_LO.x[8];
acadoWorkspace_LO.lbA[0] = acadoVariables_LO.lbAValues[0] - tmp;
acadoWorkspace_LO.ubA[0] = acadoVariables_LO.ubAValues[0] - tmp;
tmp = acadoWorkspace_LO.sbar[9] + acadoVariables_LO.x[9];
acadoWorkspace_LO.lbA[1] = acadoVariables_LO.lbAValues[1] - tmp;
acadoWorkspace_LO.ubA[1] = acadoVariables_LO.ubAValues[1] - tmp;
tmp = acadoWorkspace_LO.sbar[10] + acadoVariables_LO.x[10];
acadoWorkspace_LO.lbA[2] = acadoVariables_LO.lbAValues[2] - tmp;
acadoWorkspace_LO.ubA[2] = acadoVariables_LO.ubAValues[2] - tmp;
tmp = acadoWorkspace_LO.sbar[11] + acadoVariables_LO.x[11];
acadoWorkspace_LO.lbA[3] = acadoVariables_LO.lbAValues[3] - tmp;
acadoWorkspace_LO.ubA[3] = acadoVariables_LO.ubAValues[3] - tmp;
tmp = acadoWorkspace_LO.sbar[12] + acadoVariables_LO.x[12];
acadoWorkspace_LO.lbA[4] = acadoVariables_LO.lbAValues[4] - tmp;
acadoWorkspace_LO.ubA[4] = acadoVariables_LO.ubAValues[4] - tmp;
tmp = acadoWorkspace_LO.sbar[13] + acadoVariables_LO.x[13];
acadoWorkspace_LO.lbA[5] = acadoVariables_LO.lbAValues[5] - tmp;
acadoWorkspace_LO.ubA[5] = acadoVariables_LO.ubAValues[5] - tmp;
tmp = acadoWorkspace_LO.sbar[14] + acadoVariables_LO.x[14];
acadoWorkspace_LO.lbA[6] = acadoVariables_LO.lbAValues[6] - tmp;
acadoWorkspace_LO.ubA[6] = acadoVariables_LO.ubAValues[6] - tmp;
tmp = acadoWorkspace_LO.sbar[15] + acadoVariables_LO.x[15];
acadoWorkspace_LO.lbA[7] = acadoVariables_LO.lbAValues[7] - tmp;
acadoWorkspace_LO.ubA[7] = acadoVariables_LO.ubAValues[7] - tmp;
tmp = acadoWorkspace_LO.sbar[16] + acadoVariables_LO.x[16];
acadoWorkspace_LO.lbA[8] = acadoVariables_LO.lbAValues[8] - tmp;
acadoWorkspace_LO.ubA[8] = acadoVariables_LO.ubAValues[8] - tmp;
tmp = acadoWorkspace_LO.sbar[17] + acadoVariables_LO.x[17];
acadoWorkspace_LO.lbA[9] = acadoVariables_LO.lbAValues[9] - tmp;
acadoWorkspace_LO.ubA[9] = acadoVariables_LO.ubAValues[9] - tmp;
tmp = acadoWorkspace_LO.sbar[18] + acadoVariables_LO.x[18];
acadoWorkspace_LO.lbA[10] = acadoVariables_LO.lbAValues[10] - tmp;
acadoWorkspace_LO.ubA[10] = acadoVariables_LO.ubAValues[10] - tmp;
tmp = acadoWorkspace_LO.sbar[19] + acadoVariables_LO.x[19];
acadoWorkspace_LO.lbA[11] = acadoVariables_LO.lbAValues[11] - tmp;
acadoWorkspace_LO.ubA[11] = acadoVariables_LO.ubAValues[11] - tmp;
tmp = acadoWorkspace_LO.sbar[20] + acadoVariables_LO.x[20];
acadoWorkspace_LO.lbA[12] = acadoVariables_LO.lbAValues[12] - tmp;
acadoWorkspace_LO.ubA[12] = acadoVariables_LO.ubAValues[12] - tmp;
tmp = acadoWorkspace_LO.sbar[21] + acadoVariables_LO.x[21];
acadoWorkspace_LO.lbA[13] = acadoVariables_LO.lbAValues[13] - tmp;
acadoWorkspace_LO.ubA[13] = acadoVariables_LO.ubAValues[13] - tmp;
tmp = acadoWorkspace_LO.sbar[22] + acadoVariables_LO.x[22];
acadoWorkspace_LO.lbA[14] = acadoVariables_LO.lbAValues[14] - tmp;
acadoWorkspace_LO.ubA[14] = acadoVariables_LO.ubAValues[14] - tmp;
tmp = acadoWorkspace_LO.sbar[23] + acadoVariables_LO.x[23];
acadoWorkspace_LO.lbA[15] = acadoVariables_LO.lbAValues[15] - tmp;
acadoWorkspace_LO.ubA[15] = acadoVariables_LO.ubAValues[15] - tmp;
tmp = acadoWorkspace_LO.sbar[24] + acadoVariables_LO.x[24];
acadoWorkspace_LO.lbA[16] = acadoVariables_LO.lbAValues[16] - tmp;
acadoWorkspace_LO.ubA[16] = acadoVariables_LO.ubAValues[16] - tmp;
tmp = acadoWorkspace_LO.sbar[25] + acadoVariables_LO.x[25];
acadoWorkspace_LO.lbA[17] = acadoVariables_LO.lbAValues[17] - tmp;
acadoWorkspace_LO.ubA[17] = acadoVariables_LO.ubAValues[17] - tmp;
tmp = acadoWorkspace_LO.sbar[26] + acadoVariables_LO.x[26];
acadoWorkspace_LO.lbA[18] = acadoVariables_LO.lbAValues[18] - tmp;
acadoWorkspace_LO.ubA[18] = acadoVariables_LO.ubAValues[18] - tmp;
tmp = acadoWorkspace_LO.sbar[27] + acadoVariables_LO.x[27];
acadoWorkspace_LO.lbA[19] = acadoVariables_LO.lbAValues[19] - tmp;
acadoWorkspace_LO.ubA[19] = acadoVariables_LO.ubAValues[19] - tmp;
tmp = acadoWorkspace_LO.sbar[28] + acadoVariables_LO.x[28];
acadoWorkspace_LO.lbA[20] = acadoVariables_LO.lbAValues[20] - tmp;
acadoWorkspace_LO.ubA[20] = acadoVariables_LO.ubAValues[20] - tmp;
tmp = acadoWorkspace_LO.sbar[29] + acadoVariables_LO.x[29];
acadoWorkspace_LO.lbA[21] = acadoVariables_LO.lbAValues[21] - tmp;
acadoWorkspace_LO.ubA[21] = acadoVariables_LO.ubAValues[21] - tmp;
tmp = acadoWorkspace_LO.sbar[30] + acadoVariables_LO.x[30];
acadoWorkspace_LO.lbA[22] = acadoVariables_LO.lbAValues[22] - tmp;
acadoWorkspace_LO.ubA[22] = acadoVariables_LO.ubAValues[22] - tmp;
tmp = acadoWorkspace_LO.sbar[31] + acadoVariables_LO.x[31];
acadoWorkspace_LO.lbA[23] = acadoVariables_LO.lbAValues[23] - tmp;
acadoWorkspace_LO.ubA[23] = acadoVariables_LO.ubAValues[23] - tmp;
tmp = acadoWorkspace_LO.sbar[32] + acadoVariables_LO.x[32];
acadoWorkspace_LO.lbA[24] = acadoVariables_LO.lbAValues[24] - tmp;
acadoWorkspace_LO.ubA[24] = acadoVariables_LO.ubAValues[24] - tmp;
tmp = acadoWorkspace_LO.sbar[33] + acadoVariables_LO.x[33];
acadoWorkspace_LO.lbA[25] = acadoVariables_LO.lbAValues[25] - tmp;
acadoWorkspace_LO.ubA[25] = acadoVariables_LO.ubAValues[25] - tmp;
tmp = acadoWorkspace_LO.sbar[34] + acadoVariables_LO.x[34];
acadoWorkspace_LO.lbA[26] = acadoVariables_LO.lbAValues[26] - tmp;
acadoWorkspace_LO.ubA[26] = acadoVariables_LO.ubAValues[26] - tmp;
tmp = acadoWorkspace_LO.sbar[35] + acadoVariables_LO.x[35];
acadoWorkspace_LO.lbA[27] = acadoVariables_LO.lbAValues[27] - tmp;
acadoWorkspace_LO.ubA[27] = acadoVariables_LO.ubAValues[27] - tmp;
tmp = acadoWorkspace_LO.sbar[36] + acadoVariables_LO.x[36];
acadoWorkspace_LO.lbA[28] = acadoVariables_LO.lbAValues[28] - tmp;
acadoWorkspace_LO.ubA[28] = acadoVariables_LO.ubAValues[28] - tmp;
tmp = acadoWorkspace_LO.sbar[37] + acadoVariables_LO.x[37];
acadoWorkspace_LO.lbA[29] = acadoVariables_LO.lbAValues[29] - tmp;
acadoWorkspace_LO.ubA[29] = acadoVariables_LO.ubAValues[29] - tmp;
tmp = acadoWorkspace_LO.sbar[38] + acadoVariables_LO.x[38];
acadoWorkspace_LO.lbA[30] = acadoVariables_LO.lbAValues[30] - tmp;
acadoWorkspace_LO.ubA[30] = acadoVariables_LO.ubAValues[30] - tmp;
tmp = acadoWorkspace_LO.sbar[39] + acadoVariables_LO.x[39];
acadoWorkspace_LO.lbA[31] = acadoVariables_LO.lbAValues[31] - tmp;
acadoWorkspace_LO.ubA[31] = acadoVariables_LO.ubAValues[31] - tmp;
tmp = acadoWorkspace_LO.sbar[40] + acadoVariables_LO.x[40];
acadoWorkspace_LO.lbA[32] = acadoVariables_LO.lbAValues[32] - tmp;
acadoWorkspace_LO.ubA[32] = acadoVariables_LO.ubAValues[32] - tmp;
tmp = acadoWorkspace_LO.sbar[41] + acadoVariables_LO.x[41];
acadoWorkspace_LO.lbA[33] = acadoVariables_LO.lbAValues[33] - tmp;
acadoWorkspace_LO.ubA[33] = acadoVariables_LO.ubAValues[33] - tmp;
tmp = acadoWorkspace_LO.sbar[42] + acadoVariables_LO.x[42];
acadoWorkspace_LO.lbA[34] = acadoVariables_LO.lbAValues[34] - tmp;
acadoWorkspace_LO.ubA[34] = acadoVariables_LO.ubAValues[34] - tmp;
tmp = acadoWorkspace_LO.sbar[43] + acadoVariables_LO.x[43];
acadoWorkspace_LO.lbA[35] = acadoVariables_LO.lbAValues[35] - tmp;
acadoWorkspace_LO.ubA[35] = acadoVariables_LO.ubAValues[35] - tmp;
tmp = acadoWorkspace_LO.sbar[44] + acadoVariables_LO.x[44];
acadoWorkspace_LO.lbA[36] = acadoVariables_LO.lbAValues[36] - tmp;
acadoWorkspace_LO.ubA[36] = acadoVariables_LO.ubAValues[36] - tmp;
tmp = acadoWorkspace_LO.sbar[45] + acadoVariables_LO.x[45];
acadoWorkspace_LO.lbA[37] = acadoVariables_LO.lbAValues[37] - tmp;
acadoWorkspace_LO.ubA[37] = acadoVariables_LO.ubAValues[37] - tmp;
tmp = acadoWorkspace_LO.sbar[46] + acadoVariables_LO.x[46];
acadoWorkspace_LO.lbA[38] = acadoVariables_LO.lbAValues[38] - tmp;
acadoWorkspace_LO.ubA[38] = acadoVariables_LO.ubAValues[38] - tmp;
tmp = acadoWorkspace_LO.sbar[47] + acadoVariables_LO.x[47];
acadoWorkspace_LO.lbA[39] = acadoVariables_LO.lbAValues[39] - tmp;
acadoWorkspace_LO.ubA[39] = acadoVariables_LO.ubAValues[39] - tmp;
tmp = acadoWorkspace_LO.sbar[48] + acadoVariables_LO.x[48];
acadoWorkspace_LO.lbA[40] = acadoVariables_LO.lbAValues[40] - tmp;
acadoWorkspace_LO.ubA[40] = acadoVariables_LO.ubAValues[40] - tmp;
tmp = acadoWorkspace_LO.sbar[49] + acadoVariables_LO.x[49];
acadoWorkspace_LO.lbA[41] = acadoVariables_LO.lbAValues[41] - tmp;
acadoWorkspace_LO.ubA[41] = acadoVariables_LO.ubAValues[41] - tmp;
tmp = acadoWorkspace_LO.sbar[50] + acadoVariables_LO.x[50];
acadoWorkspace_LO.lbA[42] = acadoVariables_LO.lbAValues[42] - tmp;
acadoWorkspace_LO.ubA[42] = acadoVariables_LO.ubAValues[42] - tmp;
tmp = acadoWorkspace_LO.sbar[51] + acadoVariables_LO.x[51];
acadoWorkspace_LO.lbA[43] = acadoVariables_LO.lbAValues[43] - tmp;
acadoWorkspace_LO.ubA[43] = acadoVariables_LO.ubAValues[43] - tmp;
tmp = acadoWorkspace_LO.sbar[52] + acadoVariables_LO.x[52];
acadoWorkspace_LO.lbA[44] = acadoVariables_LO.lbAValues[44] - tmp;
acadoWorkspace_LO.ubA[44] = acadoVariables_LO.ubAValues[44] - tmp;
tmp = acadoWorkspace_LO.sbar[53] + acadoVariables_LO.x[53];
acadoWorkspace_LO.lbA[45] = acadoVariables_LO.lbAValues[45] - tmp;
acadoWorkspace_LO.ubA[45] = acadoVariables_LO.ubAValues[45] - tmp;
tmp = acadoWorkspace_LO.sbar[54] + acadoVariables_LO.x[54];
acadoWorkspace_LO.lbA[46] = acadoVariables_LO.lbAValues[46] - tmp;
acadoWorkspace_LO.ubA[46] = acadoVariables_LO.ubAValues[46] - tmp;
tmp = acadoWorkspace_LO.sbar[55] + acadoVariables_LO.x[55];
acadoWorkspace_LO.lbA[47] = acadoVariables_LO.lbAValues[47] - tmp;
acadoWorkspace_LO.ubA[47] = acadoVariables_LO.ubAValues[47] - tmp;
tmp = acadoWorkspace_LO.sbar[56] + acadoVariables_LO.x[56];
acadoWorkspace_LO.lbA[48] = acadoVariables_LO.lbAValues[48] - tmp;
acadoWorkspace_LO.ubA[48] = acadoVariables_LO.ubAValues[48] - tmp;
tmp = acadoWorkspace_LO.sbar[57] + acadoVariables_LO.x[57];
acadoWorkspace_LO.lbA[49] = acadoVariables_LO.lbAValues[49] - tmp;
acadoWorkspace_LO.ubA[49] = acadoVariables_LO.ubAValues[49] - tmp;
tmp = acadoWorkspace_LO.sbar[58] + acadoVariables_LO.x[58];
acadoWorkspace_LO.lbA[50] = acadoVariables_LO.lbAValues[50] - tmp;
acadoWorkspace_LO.ubA[50] = acadoVariables_LO.ubAValues[50] - tmp;
tmp = acadoWorkspace_LO.sbar[59] + acadoVariables_LO.x[59];
acadoWorkspace_LO.lbA[51] = acadoVariables_LO.lbAValues[51] - tmp;
acadoWorkspace_LO.ubA[51] = acadoVariables_LO.ubAValues[51] - tmp;
tmp = acadoWorkspace_LO.sbar[60] + acadoVariables_LO.x[60];
acadoWorkspace_LO.lbA[52] = acadoVariables_LO.lbAValues[52] - tmp;
acadoWorkspace_LO.ubA[52] = acadoVariables_LO.ubAValues[52] - tmp;
tmp = acadoWorkspace_LO.sbar[61] + acadoVariables_LO.x[61];
acadoWorkspace_LO.lbA[53] = acadoVariables_LO.lbAValues[53] - tmp;
acadoWorkspace_LO.ubA[53] = acadoVariables_LO.ubAValues[53] - tmp;
tmp = acadoWorkspace_LO.sbar[62] + acadoVariables_LO.x[62];
acadoWorkspace_LO.lbA[54] = acadoVariables_LO.lbAValues[54] - tmp;
acadoWorkspace_LO.ubA[54] = acadoVariables_LO.ubAValues[54] - tmp;
tmp = acadoWorkspace_LO.sbar[63] + acadoVariables_LO.x[63];
acadoWorkspace_LO.lbA[55] = acadoVariables_LO.lbAValues[55] - tmp;
acadoWorkspace_LO.ubA[55] = acadoVariables_LO.ubAValues[55] - tmp;
tmp = acadoWorkspace_LO.sbar[64] + acadoVariables_LO.x[64];
acadoWorkspace_LO.lbA[56] = acadoVariables_LO.lbAValues[56] - tmp;
acadoWorkspace_LO.ubA[56] = acadoVariables_LO.ubAValues[56] - tmp;
tmp = acadoWorkspace_LO.sbar[65] + acadoVariables_LO.x[65];
acadoWorkspace_LO.lbA[57] = acadoVariables_LO.lbAValues[57] - tmp;
acadoWorkspace_LO.ubA[57] = acadoVariables_LO.ubAValues[57] - tmp;
tmp = acadoWorkspace_LO.sbar[66] + acadoVariables_LO.x[66];
acadoWorkspace_LO.lbA[58] = acadoVariables_LO.lbAValues[58] - tmp;
acadoWorkspace_LO.ubA[58] = acadoVariables_LO.ubAValues[58] - tmp;
tmp = acadoWorkspace_LO.sbar[67] + acadoVariables_LO.x[67];
acadoWorkspace_LO.lbA[59] = acadoVariables_LO.lbAValues[59] - tmp;
acadoWorkspace_LO.ubA[59] = acadoVariables_LO.ubAValues[59] - tmp;
tmp = acadoWorkspace_LO.sbar[68] + acadoVariables_LO.x[68];
acadoWorkspace_LO.lbA[60] = acadoVariables_LO.lbAValues[60] - tmp;
acadoWorkspace_LO.ubA[60] = acadoVariables_LO.ubAValues[60] - tmp;
tmp = acadoWorkspace_LO.sbar[69] + acadoVariables_LO.x[69];
acadoWorkspace_LO.lbA[61] = acadoVariables_LO.lbAValues[61] - tmp;
acadoWorkspace_LO.ubA[61] = acadoVariables_LO.ubAValues[61] - tmp;
tmp = acadoWorkspace_LO.sbar[70] + acadoVariables_LO.x[70];
acadoWorkspace_LO.lbA[62] = acadoVariables_LO.lbAValues[62] - tmp;
acadoWorkspace_LO.ubA[62] = acadoVariables_LO.ubAValues[62] - tmp;
tmp = acadoWorkspace_LO.sbar[71] + acadoVariables_LO.x[71];
acadoWorkspace_LO.lbA[63] = acadoVariables_LO.lbAValues[63] - tmp;
acadoWorkspace_LO.ubA[63] = acadoVariables_LO.ubAValues[63] - tmp;
tmp = acadoWorkspace_LO.sbar[72] + acadoVariables_LO.x[72];
acadoWorkspace_LO.lbA[64] = acadoVariables_LO.lbAValues[64] - tmp;
acadoWorkspace_LO.ubA[64] = acadoVariables_LO.ubAValues[64] - tmp;
tmp = acadoWorkspace_LO.sbar[73] + acadoVariables_LO.x[73];
acadoWorkspace_LO.lbA[65] = acadoVariables_LO.lbAValues[65] - tmp;
acadoWorkspace_LO.ubA[65] = acadoVariables_LO.ubAValues[65] - tmp;
tmp = acadoWorkspace_LO.sbar[74] + acadoVariables_LO.x[74];
acadoWorkspace_LO.lbA[66] = acadoVariables_LO.lbAValues[66] - tmp;
acadoWorkspace_LO.ubA[66] = acadoVariables_LO.ubAValues[66] - tmp;
tmp = acadoWorkspace_LO.sbar[75] + acadoVariables_LO.x[75];
acadoWorkspace_LO.lbA[67] = acadoVariables_LO.lbAValues[67] - tmp;
acadoWorkspace_LO.ubA[67] = acadoVariables_LO.ubAValues[67] - tmp;
tmp = acadoWorkspace_LO.sbar[76] + acadoVariables_LO.x[76];
acadoWorkspace_LO.lbA[68] = acadoVariables_LO.lbAValues[68] - tmp;
acadoWorkspace_LO.ubA[68] = acadoVariables_LO.ubAValues[68] - tmp;
tmp = acadoWorkspace_LO.sbar[77] + acadoVariables_LO.x[77];
acadoWorkspace_LO.lbA[69] = acadoVariables_LO.lbAValues[69] - tmp;
acadoWorkspace_LO.ubA[69] = acadoVariables_LO.ubAValues[69] - tmp;
tmp = acadoWorkspace_LO.sbar[78] + acadoVariables_LO.x[78];
acadoWorkspace_LO.lbA[70] = acadoVariables_LO.lbAValues[70] - tmp;
acadoWorkspace_LO.ubA[70] = acadoVariables_LO.ubAValues[70] - tmp;
tmp = acadoWorkspace_LO.sbar[79] + acadoVariables_LO.x[79];
acadoWorkspace_LO.lbA[71] = acadoVariables_LO.lbAValues[71] - tmp;
acadoWorkspace_LO.ubA[71] = acadoVariables_LO.ubAValues[71] - tmp;
tmp = acadoWorkspace_LO.sbar[80] + acadoVariables_LO.x[80];
acadoWorkspace_LO.lbA[72] = acadoVariables_LO.lbAValues[72] - tmp;
acadoWorkspace_LO.ubA[72] = acadoVariables_LO.ubAValues[72] - tmp;
tmp = acadoWorkspace_LO.sbar[81] + acadoVariables_LO.x[81];
acadoWorkspace_LO.lbA[73] = acadoVariables_LO.lbAValues[73] - tmp;
acadoWorkspace_LO.ubA[73] = acadoVariables_LO.ubAValues[73] - tmp;
tmp = acadoWorkspace_LO.sbar[82] + acadoVariables_LO.x[82];
acadoWorkspace_LO.lbA[74] = acadoVariables_LO.lbAValues[74] - tmp;
acadoWorkspace_LO.ubA[74] = acadoVariables_LO.ubAValues[74] - tmp;
tmp = acadoWorkspace_LO.sbar[83] + acadoVariables_LO.x[83];
acadoWorkspace_LO.lbA[75] = acadoVariables_LO.lbAValues[75] - tmp;
acadoWorkspace_LO.ubA[75] = acadoVariables_LO.ubAValues[75] - tmp;
tmp = acadoWorkspace_LO.sbar[84] + acadoVariables_LO.x[84];
acadoWorkspace_LO.lbA[76] = acadoVariables_LO.lbAValues[76] - tmp;
acadoWorkspace_LO.ubA[76] = acadoVariables_LO.ubAValues[76] - tmp;
tmp = acadoWorkspace_LO.sbar[85] + acadoVariables_LO.x[85];
acadoWorkspace_LO.lbA[77] = acadoVariables_LO.lbAValues[77] - tmp;
acadoWorkspace_LO.ubA[77] = acadoVariables_LO.ubAValues[77] - tmp;
tmp = acadoWorkspace_LO.sbar[86] + acadoVariables_LO.x[86];
acadoWorkspace_LO.lbA[78] = acadoVariables_LO.lbAValues[78] - tmp;
acadoWorkspace_LO.ubA[78] = acadoVariables_LO.ubAValues[78] - tmp;
tmp = acadoWorkspace_LO.sbar[87] + acadoVariables_LO.x[87];
acadoWorkspace_LO.lbA[79] = acadoVariables_LO.lbAValues[79] - tmp;
acadoWorkspace_LO.ubA[79] = acadoVariables_LO.ubAValues[79] - tmp;

acado_LO_macHxd( acadoWorkspace_LO.evHx, acadoWorkspace_LO.sbar, &(acadoWorkspace_LO.lbA[ 80 ]), &(acadoWorkspace_LO.ubA[ 80 ]) );
acado_LO_macHxd( &(acadoWorkspace_LO.evHx[ 8 ]), &(acadoWorkspace_LO.sbar[ 8 ]), &(acadoWorkspace_LO.lbA[ 81 ]), &(acadoWorkspace_LO.ubA[ 81 ]) );
acado_LO_macHxd( &(acadoWorkspace_LO.evHx[ 16 ]), &(acadoWorkspace_LO.sbar[ 16 ]), &(acadoWorkspace_LO.lbA[ 82 ]), &(acadoWorkspace_LO.ubA[ 82 ]) );
acado_LO_macHxd( &(acadoWorkspace_LO.evHx[ 24 ]), &(acadoWorkspace_LO.sbar[ 24 ]), &(acadoWorkspace_LO.lbA[ 83 ]), &(acadoWorkspace_LO.ubA[ 83 ]) );
acado_LO_macHxd( &(acadoWorkspace_LO.evHx[ 32 ]), &(acadoWorkspace_LO.sbar[ 32 ]), &(acadoWorkspace_LO.lbA[ 84 ]), &(acadoWorkspace_LO.ubA[ 84 ]) );
acado_LO_macHxd( &(acadoWorkspace_LO.evHx[ 40 ]), &(acadoWorkspace_LO.sbar[ 40 ]), &(acadoWorkspace_LO.lbA[ 85 ]), &(acadoWorkspace_LO.ubA[ 85 ]) );
acado_LO_macHxd( &(acadoWorkspace_LO.evHx[ 48 ]), &(acadoWorkspace_LO.sbar[ 48 ]), &(acadoWorkspace_LO.lbA[ 86 ]), &(acadoWorkspace_LO.ubA[ 86 ]) );
acado_LO_macHxd( &(acadoWorkspace_LO.evHx[ 56 ]), &(acadoWorkspace_LO.sbar[ 56 ]), &(acadoWorkspace_LO.lbA[ 87 ]), &(acadoWorkspace_LO.ubA[ 87 ]) );
acado_LO_macHxd( &(acadoWorkspace_LO.evHx[ 64 ]), &(acadoWorkspace_LO.sbar[ 64 ]), &(acadoWorkspace_LO.lbA[ 88 ]), &(acadoWorkspace_LO.ubA[ 88 ]) );
acado_LO_macHxd( &(acadoWorkspace_LO.evHx[ 72 ]), &(acadoWorkspace_LO.sbar[ 72 ]), &(acadoWorkspace_LO.lbA[ 89 ]), &(acadoWorkspace_LO.ubA[ 89 ]) );

}

void acado_LO_expand(  )
{
acadoVariables_LO.u[0] += acadoWorkspace_LO.x[0];
acadoVariables_LO.u[1] += acadoWorkspace_LO.x[1];
acadoVariables_LO.u[2] += acadoWorkspace_LO.x[2];
acadoVariables_LO.u[3] += acadoWorkspace_LO.x[3];
acadoVariables_LO.u[4] += acadoWorkspace_LO.x[4];
acadoVariables_LO.u[5] += acadoWorkspace_LO.x[5];
acadoVariables_LO.u[6] += acadoWorkspace_LO.x[6];
acadoVariables_LO.u[7] += acadoWorkspace_LO.x[7];
acadoVariables_LO.u[8] += acadoWorkspace_LO.x[8];
acadoVariables_LO.u[9] += acadoWorkspace_LO.x[9];
acadoVariables_LO.u[10] += acadoWorkspace_LO.x[10];
acadoVariables_LO.u[11] += acadoWorkspace_LO.x[11];
acadoVariables_LO.u[12] += acadoWorkspace_LO.x[12];
acadoVariables_LO.u[13] += acadoWorkspace_LO.x[13];
acadoVariables_LO.u[14] += acadoWorkspace_LO.x[14];
acadoVariables_LO.u[15] += acadoWorkspace_LO.x[15];
acadoVariables_LO.u[16] += acadoWorkspace_LO.x[16];
acadoVariables_LO.u[17] += acadoWorkspace_LO.x[17];
acadoVariables_LO.u[18] += acadoWorkspace_LO.x[18];
acadoVariables_LO.u[19] += acadoWorkspace_LO.x[19];
acadoWorkspace_LO.sbar[0] = acadoWorkspace_LO.Dx0[0];
acadoWorkspace_LO.sbar[1] = acadoWorkspace_LO.Dx0[1];
acadoWorkspace_LO.sbar[2] = acadoWorkspace_LO.Dx0[2];
acadoWorkspace_LO.sbar[3] = acadoWorkspace_LO.Dx0[3];
acadoWorkspace_LO.sbar[4] = acadoWorkspace_LO.Dx0[4];
acadoWorkspace_LO.sbar[5] = acadoWorkspace_LO.Dx0[5];
acadoWorkspace_LO.sbar[6] = acadoWorkspace_LO.Dx0[6];
acadoWorkspace_LO.sbar[7] = acadoWorkspace_LO.Dx0[7];
acadoWorkspace_LO.sbar[8] = acadoWorkspace_LO.d[0];
acadoWorkspace_LO.sbar[9] = acadoWorkspace_LO.d[1];
acadoWorkspace_LO.sbar[10] = acadoWorkspace_LO.d[2];
acadoWorkspace_LO.sbar[11] = acadoWorkspace_LO.d[3];
acadoWorkspace_LO.sbar[12] = acadoWorkspace_LO.d[4];
acadoWorkspace_LO.sbar[13] = acadoWorkspace_LO.d[5];
acadoWorkspace_LO.sbar[14] = acadoWorkspace_LO.d[6];
acadoWorkspace_LO.sbar[15] = acadoWorkspace_LO.d[7];
acadoWorkspace_LO.sbar[16] = acadoWorkspace_LO.d[8];
acadoWorkspace_LO.sbar[17] = acadoWorkspace_LO.d[9];
acadoWorkspace_LO.sbar[18] = acadoWorkspace_LO.d[10];
acadoWorkspace_LO.sbar[19] = acadoWorkspace_LO.d[11];
acadoWorkspace_LO.sbar[20] = acadoWorkspace_LO.d[12];
acadoWorkspace_LO.sbar[21] = acadoWorkspace_LO.d[13];
acadoWorkspace_LO.sbar[22] = acadoWorkspace_LO.d[14];
acadoWorkspace_LO.sbar[23] = acadoWorkspace_LO.d[15];
acadoWorkspace_LO.sbar[24] = acadoWorkspace_LO.d[16];
acadoWorkspace_LO.sbar[25] = acadoWorkspace_LO.d[17];
acadoWorkspace_LO.sbar[26] = acadoWorkspace_LO.d[18];
acadoWorkspace_LO.sbar[27] = acadoWorkspace_LO.d[19];
acadoWorkspace_LO.sbar[28] = acadoWorkspace_LO.d[20];
acadoWorkspace_LO.sbar[29] = acadoWorkspace_LO.d[21];
acadoWorkspace_LO.sbar[30] = acadoWorkspace_LO.d[22];
acadoWorkspace_LO.sbar[31] = acadoWorkspace_LO.d[23];
acadoWorkspace_LO.sbar[32] = acadoWorkspace_LO.d[24];
acadoWorkspace_LO.sbar[33] = acadoWorkspace_LO.d[25];
acadoWorkspace_LO.sbar[34] = acadoWorkspace_LO.d[26];
acadoWorkspace_LO.sbar[35] = acadoWorkspace_LO.d[27];
acadoWorkspace_LO.sbar[36] = acadoWorkspace_LO.d[28];
acadoWorkspace_LO.sbar[37] = acadoWorkspace_LO.d[29];
acadoWorkspace_LO.sbar[38] = acadoWorkspace_LO.d[30];
acadoWorkspace_LO.sbar[39] = acadoWorkspace_LO.d[31];
acadoWorkspace_LO.sbar[40] = acadoWorkspace_LO.d[32];
acadoWorkspace_LO.sbar[41] = acadoWorkspace_LO.d[33];
acadoWorkspace_LO.sbar[42] = acadoWorkspace_LO.d[34];
acadoWorkspace_LO.sbar[43] = acadoWorkspace_LO.d[35];
acadoWorkspace_LO.sbar[44] = acadoWorkspace_LO.d[36];
acadoWorkspace_LO.sbar[45] = acadoWorkspace_LO.d[37];
acadoWorkspace_LO.sbar[46] = acadoWorkspace_LO.d[38];
acadoWorkspace_LO.sbar[47] = acadoWorkspace_LO.d[39];
acadoWorkspace_LO.sbar[48] = acadoWorkspace_LO.d[40];
acadoWorkspace_LO.sbar[49] = acadoWorkspace_LO.d[41];
acadoWorkspace_LO.sbar[50] = acadoWorkspace_LO.d[42];
acadoWorkspace_LO.sbar[51] = acadoWorkspace_LO.d[43];
acadoWorkspace_LO.sbar[52] = acadoWorkspace_LO.d[44];
acadoWorkspace_LO.sbar[53] = acadoWorkspace_LO.d[45];
acadoWorkspace_LO.sbar[54] = acadoWorkspace_LO.d[46];
acadoWorkspace_LO.sbar[55] = acadoWorkspace_LO.d[47];
acadoWorkspace_LO.sbar[56] = acadoWorkspace_LO.d[48];
acadoWorkspace_LO.sbar[57] = acadoWorkspace_LO.d[49];
acadoWorkspace_LO.sbar[58] = acadoWorkspace_LO.d[50];
acadoWorkspace_LO.sbar[59] = acadoWorkspace_LO.d[51];
acadoWorkspace_LO.sbar[60] = acadoWorkspace_LO.d[52];
acadoWorkspace_LO.sbar[61] = acadoWorkspace_LO.d[53];
acadoWorkspace_LO.sbar[62] = acadoWorkspace_LO.d[54];
acadoWorkspace_LO.sbar[63] = acadoWorkspace_LO.d[55];
acadoWorkspace_LO.sbar[64] = acadoWorkspace_LO.d[56];
acadoWorkspace_LO.sbar[65] = acadoWorkspace_LO.d[57];
acadoWorkspace_LO.sbar[66] = acadoWorkspace_LO.d[58];
acadoWorkspace_LO.sbar[67] = acadoWorkspace_LO.d[59];
acadoWorkspace_LO.sbar[68] = acadoWorkspace_LO.d[60];
acadoWorkspace_LO.sbar[69] = acadoWorkspace_LO.d[61];
acadoWorkspace_LO.sbar[70] = acadoWorkspace_LO.d[62];
acadoWorkspace_LO.sbar[71] = acadoWorkspace_LO.d[63];
acadoWorkspace_LO.sbar[72] = acadoWorkspace_LO.d[64];
acadoWorkspace_LO.sbar[73] = acadoWorkspace_LO.d[65];
acadoWorkspace_LO.sbar[74] = acadoWorkspace_LO.d[66];
acadoWorkspace_LO.sbar[75] = acadoWorkspace_LO.d[67];
acadoWorkspace_LO.sbar[76] = acadoWorkspace_LO.d[68];
acadoWorkspace_LO.sbar[77] = acadoWorkspace_LO.d[69];
acadoWorkspace_LO.sbar[78] = acadoWorkspace_LO.d[70];
acadoWorkspace_LO.sbar[79] = acadoWorkspace_LO.d[71];
acadoWorkspace_LO.sbar[80] = acadoWorkspace_LO.d[72];
acadoWorkspace_LO.sbar[81] = acadoWorkspace_LO.d[73];
acadoWorkspace_LO.sbar[82] = acadoWorkspace_LO.d[74];
acadoWorkspace_LO.sbar[83] = acadoWorkspace_LO.d[75];
acadoWorkspace_LO.sbar[84] = acadoWorkspace_LO.d[76];
acadoWorkspace_LO.sbar[85] = acadoWorkspace_LO.d[77];
acadoWorkspace_LO.sbar[86] = acadoWorkspace_LO.d[78];
acadoWorkspace_LO.sbar[87] = acadoWorkspace_LO.d[79];
acado_LO_expansionStep( acadoWorkspace_LO.evGx, acadoWorkspace_LO.evGu, acadoWorkspace_LO.x, acadoWorkspace_LO.sbar, &(acadoWorkspace_LO.sbar[ 8 ]) );
acado_LO_expansionStep( &(acadoWorkspace_LO.evGx[ 64 ]), &(acadoWorkspace_LO.evGu[ 16 ]), &(acadoWorkspace_LO.x[ 2 ]), &(acadoWorkspace_LO.sbar[ 8 ]), &(acadoWorkspace_LO.sbar[ 16 ]) );
acado_LO_expansionStep( &(acadoWorkspace_LO.evGx[ 128 ]), &(acadoWorkspace_LO.evGu[ 32 ]), &(acadoWorkspace_LO.x[ 4 ]), &(acadoWorkspace_LO.sbar[ 16 ]), &(acadoWorkspace_LO.sbar[ 24 ]) );
acado_LO_expansionStep( &(acadoWorkspace_LO.evGx[ 192 ]), &(acadoWorkspace_LO.evGu[ 48 ]), &(acadoWorkspace_LO.x[ 6 ]), &(acadoWorkspace_LO.sbar[ 24 ]), &(acadoWorkspace_LO.sbar[ 32 ]) );
acado_LO_expansionStep( &(acadoWorkspace_LO.evGx[ 256 ]), &(acadoWorkspace_LO.evGu[ 64 ]), &(acadoWorkspace_LO.x[ 8 ]), &(acadoWorkspace_LO.sbar[ 32 ]), &(acadoWorkspace_LO.sbar[ 40 ]) );
acado_LO_expansionStep( &(acadoWorkspace_LO.evGx[ 320 ]), &(acadoWorkspace_LO.evGu[ 80 ]), &(acadoWorkspace_LO.x[ 10 ]), &(acadoWorkspace_LO.sbar[ 40 ]), &(acadoWorkspace_LO.sbar[ 48 ]) );
acado_LO_expansionStep( &(acadoWorkspace_LO.evGx[ 384 ]), &(acadoWorkspace_LO.evGu[ 96 ]), &(acadoWorkspace_LO.x[ 12 ]), &(acadoWorkspace_LO.sbar[ 48 ]), &(acadoWorkspace_LO.sbar[ 56 ]) );
acado_LO_expansionStep( &(acadoWorkspace_LO.evGx[ 448 ]), &(acadoWorkspace_LO.evGu[ 112 ]), &(acadoWorkspace_LO.x[ 14 ]), &(acadoWorkspace_LO.sbar[ 56 ]), &(acadoWorkspace_LO.sbar[ 64 ]) );
acado_LO_expansionStep( &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoWorkspace_LO.evGu[ 128 ]), &(acadoWorkspace_LO.x[ 16 ]), &(acadoWorkspace_LO.sbar[ 64 ]), &(acadoWorkspace_LO.sbar[ 72 ]) );
acado_LO_expansionStep( &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoWorkspace_LO.evGu[ 144 ]), &(acadoWorkspace_LO.x[ 18 ]), &(acadoWorkspace_LO.sbar[ 72 ]), &(acadoWorkspace_LO.sbar[ 80 ]) );
acadoVariables_LO.x[0] += acadoWorkspace_LO.sbar[0];
acadoVariables_LO.x[1] += acadoWorkspace_LO.sbar[1];
acadoVariables_LO.x[2] += acadoWorkspace_LO.sbar[2];
acadoVariables_LO.x[3] += acadoWorkspace_LO.sbar[3];
acadoVariables_LO.x[4] += acadoWorkspace_LO.sbar[4];
acadoVariables_LO.x[5] += acadoWorkspace_LO.sbar[5];
acadoVariables_LO.x[6] += acadoWorkspace_LO.sbar[6];
acadoVariables_LO.x[7] += acadoWorkspace_LO.sbar[7];
acadoVariables_LO.x[8] += acadoWorkspace_LO.sbar[8];
acadoVariables_LO.x[9] += acadoWorkspace_LO.sbar[9];
acadoVariables_LO.x[10] += acadoWorkspace_LO.sbar[10];
acadoVariables_LO.x[11] += acadoWorkspace_LO.sbar[11];
acadoVariables_LO.x[12] += acadoWorkspace_LO.sbar[12];
acadoVariables_LO.x[13] += acadoWorkspace_LO.sbar[13];
acadoVariables_LO.x[14] += acadoWorkspace_LO.sbar[14];
acadoVariables_LO.x[15] += acadoWorkspace_LO.sbar[15];
acadoVariables_LO.x[16] += acadoWorkspace_LO.sbar[16];
acadoVariables_LO.x[17] += acadoWorkspace_LO.sbar[17];
acadoVariables_LO.x[18] += acadoWorkspace_LO.sbar[18];
acadoVariables_LO.x[19] += acadoWorkspace_LO.sbar[19];
acadoVariables_LO.x[20] += acadoWorkspace_LO.sbar[20];
acadoVariables_LO.x[21] += acadoWorkspace_LO.sbar[21];
acadoVariables_LO.x[22] += acadoWorkspace_LO.sbar[22];
acadoVariables_LO.x[23] += acadoWorkspace_LO.sbar[23];
acadoVariables_LO.x[24] += acadoWorkspace_LO.sbar[24];
acadoVariables_LO.x[25] += acadoWorkspace_LO.sbar[25];
acadoVariables_LO.x[26] += acadoWorkspace_LO.sbar[26];
acadoVariables_LO.x[27] += acadoWorkspace_LO.sbar[27];
acadoVariables_LO.x[28] += acadoWorkspace_LO.sbar[28];
acadoVariables_LO.x[29] += acadoWorkspace_LO.sbar[29];
acadoVariables_LO.x[30] += acadoWorkspace_LO.sbar[30];
acadoVariables_LO.x[31] += acadoWorkspace_LO.sbar[31];
acadoVariables_LO.x[32] += acadoWorkspace_LO.sbar[32];
acadoVariables_LO.x[33] += acadoWorkspace_LO.sbar[33];
acadoVariables_LO.x[34] += acadoWorkspace_LO.sbar[34];
acadoVariables_LO.x[35] += acadoWorkspace_LO.sbar[35];
acadoVariables_LO.x[36] += acadoWorkspace_LO.sbar[36];
acadoVariables_LO.x[37] += acadoWorkspace_LO.sbar[37];
acadoVariables_LO.x[38] += acadoWorkspace_LO.sbar[38];
acadoVariables_LO.x[39] += acadoWorkspace_LO.sbar[39];
acadoVariables_LO.x[40] += acadoWorkspace_LO.sbar[40];
acadoVariables_LO.x[41] += acadoWorkspace_LO.sbar[41];
acadoVariables_LO.x[42] += acadoWorkspace_LO.sbar[42];
acadoVariables_LO.x[43] += acadoWorkspace_LO.sbar[43];
acadoVariables_LO.x[44] += acadoWorkspace_LO.sbar[44];
acadoVariables_LO.x[45] += acadoWorkspace_LO.sbar[45];
acadoVariables_LO.x[46] += acadoWorkspace_LO.sbar[46];
acadoVariables_LO.x[47] += acadoWorkspace_LO.sbar[47];
acadoVariables_LO.x[48] += acadoWorkspace_LO.sbar[48];
acadoVariables_LO.x[49] += acadoWorkspace_LO.sbar[49];
acadoVariables_LO.x[50] += acadoWorkspace_LO.sbar[50];
acadoVariables_LO.x[51] += acadoWorkspace_LO.sbar[51];
acadoVariables_LO.x[52] += acadoWorkspace_LO.sbar[52];
acadoVariables_LO.x[53] += acadoWorkspace_LO.sbar[53];
acadoVariables_LO.x[54] += acadoWorkspace_LO.sbar[54];
acadoVariables_LO.x[55] += acadoWorkspace_LO.sbar[55];
acadoVariables_LO.x[56] += acadoWorkspace_LO.sbar[56];
acadoVariables_LO.x[57] += acadoWorkspace_LO.sbar[57];
acadoVariables_LO.x[58] += acadoWorkspace_LO.sbar[58];
acadoVariables_LO.x[59] += acadoWorkspace_LO.sbar[59];
acadoVariables_LO.x[60] += acadoWorkspace_LO.sbar[60];
acadoVariables_LO.x[61] += acadoWorkspace_LO.sbar[61];
acadoVariables_LO.x[62] += acadoWorkspace_LO.sbar[62];
acadoVariables_LO.x[63] += acadoWorkspace_LO.sbar[63];
acadoVariables_LO.x[64] += acadoWorkspace_LO.sbar[64];
acadoVariables_LO.x[65] += acadoWorkspace_LO.sbar[65];
acadoVariables_LO.x[66] += acadoWorkspace_LO.sbar[66];
acadoVariables_LO.x[67] += acadoWorkspace_LO.sbar[67];
acadoVariables_LO.x[68] += acadoWorkspace_LO.sbar[68];
acadoVariables_LO.x[69] += acadoWorkspace_LO.sbar[69];
acadoVariables_LO.x[70] += acadoWorkspace_LO.sbar[70];
acadoVariables_LO.x[71] += acadoWorkspace_LO.sbar[71];
acadoVariables_LO.x[72] += acadoWorkspace_LO.sbar[72];
acadoVariables_LO.x[73] += acadoWorkspace_LO.sbar[73];
acadoVariables_LO.x[74] += acadoWorkspace_LO.sbar[74];
acadoVariables_LO.x[75] += acadoWorkspace_LO.sbar[75];
acadoVariables_LO.x[76] += acadoWorkspace_LO.sbar[76];
acadoVariables_LO.x[77] += acadoWorkspace_LO.sbar[77];
acadoVariables_LO.x[78] += acadoWorkspace_LO.sbar[78];
acadoVariables_LO.x[79] += acadoWorkspace_LO.sbar[79];
acadoVariables_LO.x[80] += acadoWorkspace_LO.sbar[80];
acadoVariables_LO.x[81] += acadoWorkspace_LO.sbar[81];
acadoVariables_LO.x[82] += acadoWorkspace_LO.sbar[82];
acadoVariables_LO.x[83] += acadoWorkspace_LO.sbar[83];
acadoVariables_LO.x[84] += acadoWorkspace_LO.sbar[84];
acadoVariables_LO.x[85] += acadoWorkspace_LO.sbar[85];
acadoVariables_LO.x[86] += acadoWorkspace_LO.sbar[86];
acadoVariables_LO.x[87] += acadoWorkspace_LO.sbar[87];
acadoVariables_LO.mu[72] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[92];
acadoVariables_LO.mu[73] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[93];
acadoVariables_LO.mu[74] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[94];
acadoVariables_LO.mu[75] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[95];
acadoVariables_LO.mu[76] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[96];
acadoVariables_LO.mu[77] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[97];
acadoVariables_LO.mu[78] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[98];
acadoVariables_LO.mu[79] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[99];
acadoVariables_LO.mu[72] += + acadoWorkspace_LO.sbar[80]*acadoWorkspace_LO.QN1[0] + acadoWorkspace_LO.sbar[81]*acadoWorkspace_LO.QN1[8] + acadoWorkspace_LO.sbar[82]*acadoWorkspace_LO.QN1[16] + acadoWorkspace_LO.sbar[83]*acadoWorkspace_LO.QN1[24] + acadoWorkspace_LO.sbar[84]*acadoWorkspace_LO.QN1[32] + acadoWorkspace_LO.sbar[85]*acadoWorkspace_LO.QN1[40] + acadoWorkspace_LO.sbar[86]*acadoWorkspace_LO.QN1[48] + acadoWorkspace_LO.sbar[87]*acadoWorkspace_LO.QN1[56];
acadoVariables_LO.mu[73] += + acadoWorkspace_LO.sbar[80]*acadoWorkspace_LO.QN1[1] + acadoWorkspace_LO.sbar[81]*acadoWorkspace_LO.QN1[9] + acadoWorkspace_LO.sbar[82]*acadoWorkspace_LO.QN1[17] + acadoWorkspace_LO.sbar[83]*acadoWorkspace_LO.QN1[25] + acadoWorkspace_LO.sbar[84]*acadoWorkspace_LO.QN1[33] + acadoWorkspace_LO.sbar[85]*acadoWorkspace_LO.QN1[41] + acadoWorkspace_LO.sbar[86]*acadoWorkspace_LO.QN1[49] + acadoWorkspace_LO.sbar[87]*acadoWorkspace_LO.QN1[57];
acadoVariables_LO.mu[74] += + acadoWorkspace_LO.sbar[80]*acadoWorkspace_LO.QN1[2] + acadoWorkspace_LO.sbar[81]*acadoWorkspace_LO.QN1[10] + acadoWorkspace_LO.sbar[82]*acadoWorkspace_LO.QN1[18] + acadoWorkspace_LO.sbar[83]*acadoWorkspace_LO.QN1[26] + acadoWorkspace_LO.sbar[84]*acadoWorkspace_LO.QN1[34] + acadoWorkspace_LO.sbar[85]*acadoWorkspace_LO.QN1[42] + acadoWorkspace_LO.sbar[86]*acadoWorkspace_LO.QN1[50] + acadoWorkspace_LO.sbar[87]*acadoWorkspace_LO.QN1[58];
acadoVariables_LO.mu[75] += + acadoWorkspace_LO.sbar[80]*acadoWorkspace_LO.QN1[3] + acadoWorkspace_LO.sbar[81]*acadoWorkspace_LO.QN1[11] + acadoWorkspace_LO.sbar[82]*acadoWorkspace_LO.QN1[19] + acadoWorkspace_LO.sbar[83]*acadoWorkspace_LO.QN1[27] + acadoWorkspace_LO.sbar[84]*acadoWorkspace_LO.QN1[35] + acadoWorkspace_LO.sbar[85]*acadoWorkspace_LO.QN1[43] + acadoWorkspace_LO.sbar[86]*acadoWorkspace_LO.QN1[51] + acadoWorkspace_LO.sbar[87]*acadoWorkspace_LO.QN1[59];
acadoVariables_LO.mu[76] += + acadoWorkspace_LO.sbar[80]*acadoWorkspace_LO.QN1[4] + acadoWorkspace_LO.sbar[81]*acadoWorkspace_LO.QN1[12] + acadoWorkspace_LO.sbar[82]*acadoWorkspace_LO.QN1[20] + acadoWorkspace_LO.sbar[83]*acadoWorkspace_LO.QN1[28] + acadoWorkspace_LO.sbar[84]*acadoWorkspace_LO.QN1[36] + acadoWorkspace_LO.sbar[85]*acadoWorkspace_LO.QN1[44] + acadoWorkspace_LO.sbar[86]*acadoWorkspace_LO.QN1[52] + acadoWorkspace_LO.sbar[87]*acadoWorkspace_LO.QN1[60];
acadoVariables_LO.mu[77] += + acadoWorkspace_LO.sbar[80]*acadoWorkspace_LO.QN1[5] + acadoWorkspace_LO.sbar[81]*acadoWorkspace_LO.QN1[13] + acadoWorkspace_LO.sbar[82]*acadoWorkspace_LO.QN1[21] + acadoWorkspace_LO.sbar[83]*acadoWorkspace_LO.QN1[29] + acadoWorkspace_LO.sbar[84]*acadoWorkspace_LO.QN1[37] + acadoWorkspace_LO.sbar[85]*acadoWorkspace_LO.QN1[45] + acadoWorkspace_LO.sbar[86]*acadoWorkspace_LO.QN1[53] + acadoWorkspace_LO.sbar[87]*acadoWorkspace_LO.QN1[61];
acadoVariables_LO.mu[78] += + acadoWorkspace_LO.sbar[80]*acadoWorkspace_LO.QN1[6] + acadoWorkspace_LO.sbar[81]*acadoWorkspace_LO.QN1[14] + acadoWorkspace_LO.sbar[82]*acadoWorkspace_LO.QN1[22] + acadoWorkspace_LO.sbar[83]*acadoWorkspace_LO.QN1[30] + acadoWorkspace_LO.sbar[84]*acadoWorkspace_LO.QN1[38] + acadoWorkspace_LO.sbar[85]*acadoWorkspace_LO.QN1[46] + acadoWorkspace_LO.sbar[86]*acadoWorkspace_LO.QN1[54] + acadoWorkspace_LO.sbar[87]*acadoWorkspace_LO.QN1[62];
acadoVariables_LO.mu[79] += + acadoWorkspace_LO.sbar[80]*acadoWorkspace_LO.QN1[7] + acadoWorkspace_LO.sbar[81]*acadoWorkspace_LO.QN1[15] + acadoWorkspace_LO.sbar[82]*acadoWorkspace_LO.QN1[23] + acadoWorkspace_LO.sbar[83]*acadoWorkspace_LO.QN1[31] + acadoWorkspace_LO.sbar[84]*acadoWorkspace_LO.QN1[39] + acadoWorkspace_LO.sbar[85]*acadoWorkspace_LO.QN1[47] + acadoWorkspace_LO.sbar[86]*acadoWorkspace_LO.QN1[55] + acadoWorkspace_LO.sbar[87]*acadoWorkspace_LO.QN1[63];
acadoVariables_LO.mu[72] += acadoWorkspace_LO.QDy[80];
acadoVariables_LO.mu[73] += acadoWorkspace_LO.QDy[81];
acadoVariables_LO.mu[74] += acadoWorkspace_LO.QDy[82];
acadoVariables_LO.mu[75] += acadoWorkspace_LO.QDy[83];
acadoVariables_LO.mu[76] += acadoWorkspace_LO.QDy[84];
acadoVariables_LO.mu[77] += acadoWorkspace_LO.QDy[85];
acadoVariables_LO.mu[78] += acadoWorkspace_LO.QDy[86];
acadoVariables_LO.mu[79] += acadoWorkspace_LO.QDy[87];
acadoVariables_LO.mu[64] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[84];
acadoVariables_LO.mu[64] -= + acadoWorkspace_LO.y[109]*acadoWorkspace_LO.evHx[72];
acadoVariables_LO.mu[65] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[85];
acadoVariables_LO.mu[65] -= + acadoWorkspace_LO.y[109]*acadoWorkspace_LO.evHx[73];
acadoVariables_LO.mu[66] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[86];
acadoVariables_LO.mu[66] -= + acadoWorkspace_LO.y[109]*acadoWorkspace_LO.evHx[74];
acadoVariables_LO.mu[67] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[87];
acadoVariables_LO.mu[67] -= + acadoWorkspace_LO.y[109]*acadoWorkspace_LO.evHx[75];
acadoVariables_LO.mu[68] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[88];
acadoVariables_LO.mu[68] -= + acadoWorkspace_LO.y[109]*acadoWorkspace_LO.evHx[76];
acadoVariables_LO.mu[69] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[89];
acadoVariables_LO.mu[69] -= + acadoWorkspace_LO.y[109]*acadoWorkspace_LO.evHx[77];
acadoVariables_LO.mu[70] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[90];
acadoVariables_LO.mu[70] -= + acadoWorkspace_LO.y[109]*acadoWorkspace_LO.evHx[78];
acadoVariables_LO.mu[71] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[91];
acadoVariables_LO.mu[71] -= + acadoWorkspace_LO.y[109]*acadoWorkspace_LO.evHx[79];
acado_LO_expansionStep2( &(acadoWorkspace_LO.QDy[ 72 ]), &(acadoWorkspace_LO.Q1[ 576 ]), &(acadoWorkspace_LO.sbar[ 72 ]), &(acadoWorkspace_LO.S1[ 144 ]), &(acadoWorkspace_LO.x[ 18 ]), &(acadoWorkspace_LO.evGx[ 576 ]), &(acadoVariables_LO.mu[ 64 ]), &(acadoVariables_LO.mu[ 72 ]) );
acadoVariables_LO.mu[56] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[76];
acadoVariables_LO.mu[56] -= + acadoWorkspace_LO.y[108]*acadoWorkspace_LO.evHx[64];
acadoVariables_LO.mu[57] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[77];
acadoVariables_LO.mu[57] -= + acadoWorkspace_LO.y[108]*acadoWorkspace_LO.evHx[65];
acadoVariables_LO.mu[58] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[78];
acadoVariables_LO.mu[58] -= + acadoWorkspace_LO.y[108]*acadoWorkspace_LO.evHx[66];
acadoVariables_LO.mu[59] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[79];
acadoVariables_LO.mu[59] -= + acadoWorkspace_LO.y[108]*acadoWorkspace_LO.evHx[67];
acadoVariables_LO.mu[60] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[80];
acadoVariables_LO.mu[60] -= + acadoWorkspace_LO.y[108]*acadoWorkspace_LO.evHx[68];
acadoVariables_LO.mu[61] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[81];
acadoVariables_LO.mu[61] -= + acadoWorkspace_LO.y[108]*acadoWorkspace_LO.evHx[69];
acadoVariables_LO.mu[62] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[82];
acadoVariables_LO.mu[62] -= + acadoWorkspace_LO.y[108]*acadoWorkspace_LO.evHx[70];
acadoVariables_LO.mu[63] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[83];
acadoVariables_LO.mu[63] -= + acadoWorkspace_LO.y[108]*acadoWorkspace_LO.evHx[71];
acado_LO_expansionStep2( &(acadoWorkspace_LO.QDy[ 64 ]), &(acadoWorkspace_LO.Q1[ 512 ]), &(acadoWorkspace_LO.sbar[ 64 ]), &(acadoWorkspace_LO.S1[ 128 ]), &(acadoWorkspace_LO.x[ 16 ]), &(acadoWorkspace_LO.evGx[ 512 ]), &(acadoVariables_LO.mu[ 56 ]), &(acadoVariables_LO.mu[ 64 ]) );
acadoVariables_LO.mu[48] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[68];
acadoVariables_LO.mu[48] -= + acadoWorkspace_LO.y[107]*acadoWorkspace_LO.evHx[56];
acadoVariables_LO.mu[49] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[69];
acadoVariables_LO.mu[49] -= + acadoWorkspace_LO.y[107]*acadoWorkspace_LO.evHx[57];
acadoVariables_LO.mu[50] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[70];
acadoVariables_LO.mu[50] -= + acadoWorkspace_LO.y[107]*acadoWorkspace_LO.evHx[58];
acadoVariables_LO.mu[51] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[71];
acadoVariables_LO.mu[51] -= + acadoWorkspace_LO.y[107]*acadoWorkspace_LO.evHx[59];
acadoVariables_LO.mu[52] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[72];
acadoVariables_LO.mu[52] -= + acadoWorkspace_LO.y[107]*acadoWorkspace_LO.evHx[60];
acadoVariables_LO.mu[53] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[73];
acadoVariables_LO.mu[53] -= + acadoWorkspace_LO.y[107]*acadoWorkspace_LO.evHx[61];
acadoVariables_LO.mu[54] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[74];
acadoVariables_LO.mu[54] -= + acadoWorkspace_LO.y[107]*acadoWorkspace_LO.evHx[62];
acadoVariables_LO.mu[55] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[75];
acadoVariables_LO.mu[55] -= + acadoWorkspace_LO.y[107]*acadoWorkspace_LO.evHx[63];
acado_LO_expansionStep2( &(acadoWorkspace_LO.QDy[ 56 ]), &(acadoWorkspace_LO.Q1[ 448 ]), &(acadoWorkspace_LO.sbar[ 56 ]), &(acadoWorkspace_LO.S1[ 112 ]), &(acadoWorkspace_LO.x[ 14 ]), &(acadoWorkspace_LO.evGx[ 448 ]), &(acadoVariables_LO.mu[ 48 ]), &(acadoVariables_LO.mu[ 56 ]) );
acadoVariables_LO.mu[40] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[60];
acadoVariables_LO.mu[40] -= + acadoWorkspace_LO.y[106]*acadoWorkspace_LO.evHx[48];
acadoVariables_LO.mu[41] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[61];
acadoVariables_LO.mu[41] -= + acadoWorkspace_LO.y[106]*acadoWorkspace_LO.evHx[49];
acadoVariables_LO.mu[42] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[62];
acadoVariables_LO.mu[42] -= + acadoWorkspace_LO.y[106]*acadoWorkspace_LO.evHx[50];
acadoVariables_LO.mu[43] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[63];
acadoVariables_LO.mu[43] -= + acadoWorkspace_LO.y[106]*acadoWorkspace_LO.evHx[51];
acadoVariables_LO.mu[44] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[64];
acadoVariables_LO.mu[44] -= + acadoWorkspace_LO.y[106]*acadoWorkspace_LO.evHx[52];
acadoVariables_LO.mu[45] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[65];
acadoVariables_LO.mu[45] -= + acadoWorkspace_LO.y[106]*acadoWorkspace_LO.evHx[53];
acadoVariables_LO.mu[46] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[66];
acadoVariables_LO.mu[46] -= + acadoWorkspace_LO.y[106]*acadoWorkspace_LO.evHx[54];
acadoVariables_LO.mu[47] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[67];
acadoVariables_LO.mu[47] -= + acadoWorkspace_LO.y[106]*acadoWorkspace_LO.evHx[55];
acado_LO_expansionStep2( &(acadoWorkspace_LO.QDy[ 48 ]), &(acadoWorkspace_LO.Q1[ 384 ]), &(acadoWorkspace_LO.sbar[ 48 ]), &(acadoWorkspace_LO.S1[ 96 ]), &(acadoWorkspace_LO.x[ 12 ]), &(acadoWorkspace_LO.evGx[ 384 ]), &(acadoVariables_LO.mu[ 40 ]), &(acadoVariables_LO.mu[ 48 ]) );
acadoVariables_LO.mu[32] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[52];
acadoVariables_LO.mu[32] -= + acadoWorkspace_LO.y[105]*acadoWorkspace_LO.evHx[40];
acadoVariables_LO.mu[33] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[53];
acadoVariables_LO.mu[33] -= + acadoWorkspace_LO.y[105]*acadoWorkspace_LO.evHx[41];
acadoVariables_LO.mu[34] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[54];
acadoVariables_LO.mu[34] -= + acadoWorkspace_LO.y[105]*acadoWorkspace_LO.evHx[42];
acadoVariables_LO.mu[35] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[55];
acadoVariables_LO.mu[35] -= + acadoWorkspace_LO.y[105]*acadoWorkspace_LO.evHx[43];
acadoVariables_LO.mu[36] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[56];
acadoVariables_LO.mu[36] -= + acadoWorkspace_LO.y[105]*acadoWorkspace_LO.evHx[44];
acadoVariables_LO.mu[37] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[57];
acadoVariables_LO.mu[37] -= + acadoWorkspace_LO.y[105]*acadoWorkspace_LO.evHx[45];
acadoVariables_LO.mu[38] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[58];
acadoVariables_LO.mu[38] -= + acadoWorkspace_LO.y[105]*acadoWorkspace_LO.evHx[46];
acadoVariables_LO.mu[39] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[59];
acadoVariables_LO.mu[39] -= + acadoWorkspace_LO.y[105]*acadoWorkspace_LO.evHx[47];
acado_LO_expansionStep2( &(acadoWorkspace_LO.QDy[ 40 ]), &(acadoWorkspace_LO.Q1[ 320 ]), &(acadoWorkspace_LO.sbar[ 40 ]), &(acadoWorkspace_LO.S1[ 80 ]), &(acadoWorkspace_LO.x[ 10 ]), &(acadoWorkspace_LO.evGx[ 320 ]), &(acadoVariables_LO.mu[ 32 ]), &(acadoVariables_LO.mu[ 40 ]) );
acadoVariables_LO.mu[24] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[44];
acadoVariables_LO.mu[24] -= + acadoWorkspace_LO.y[104]*acadoWorkspace_LO.evHx[32];
acadoVariables_LO.mu[25] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[45];
acadoVariables_LO.mu[25] -= + acadoWorkspace_LO.y[104]*acadoWorkspace_LO.evHx[33];
acadoVariables_LO.mu[26] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[46];
acadoVariables_LO.mu[26] -= + acadoWorkspace_LO.y[104]*acadoWorkspace_LO.evHx[34];
acadoVariables_LO.mu[27] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[47];
acadoVariables_LO.mu[27] -= + acadoWorkspace_LO.y[104]*acadoWorkspace_LO.evHx[35];
acadoVariables_LO.mu[28] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[48];
acadoVariables_LO.mu[28] -= + acadoWorkspace_LO.y[104]*acadoWorkspace_LO.evHx[36];
acadoVariables_LO.mu[29] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[49];
acadoVariables_LO.mu[29] -= + acadoWorkspace_LO.y[104]*acadoWorkspace_LO.evHx[37];
acadoVariables_LO.mu[30] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[50];
acadoVariables_LO.mu[30] -= + acadoWorkspace_LO.y[104]*acadoWorkspace_LO.evHx[38];
acadoVariables_LO.mu[31] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[51];
acadoVariables_LO.mu[31] -= + acadoWorkspace_LO.y[104]*acadoWorkspace_LO.evHx[39];
acado_LO_expansionStep2( &(acadoWorkspace_LO.QDy[ 32 ]), &(acadoWorkspace_LO.Q1[ 256 ]), &(acadoWorkspace_LO.sbar[ 32 ]), &(acadoWorkspace_LO.S1[ 64 ]), &(acadoWorkspace_LO.x[ 8 ]), &(acadoWorkspace_LO.evGx[ 256 ]), &(acadoVariables_LO.mu[ 24 ]), &(acadoVariables_LO.mu[ 32 ]) );
acadoVariables_LO.mu[16] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[36];
acadoVariables_LO.mu[16] -= + acadoWorkspace_LO.y[103]*acadoWorkspace_LO.evHx[24];
acadoVariables_LO.mu[17] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[37];
acadoVariables_LO.mu[17] -= + acadoWorkspace_LO.y[103]*acadoWorkspace_LO.evHx[25];
acadoVariables_LO.mu[18] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[38];
acadoVariables_LO.mu[18] -= + acadoWorkspace_LO.y[103]*acadoWorkspace_LO.evHx[26];
acadoVariables_LO.mu[19] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[39];
acadoVariables_LO.mu[19] -= + acadoWorkspace_LO.y[103]*acadoWorkspace_LO.evHx[27];
acadoVariables_LO.mu[20] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[40];
acadoVariables_LO.mu[20] -= + acadoWorkspace_LO.y[103]*acadoWorkspace_LO.evHx[28];
acadoVariables_LO.mu[21] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[41];
acadoVariables_LO.mu[21] -= + acadoWorkspace_LO.y[103]*acadoWorkspace_LO.evHx[29];
acadoVariables_LO.mu[22] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[42];
acadoVariables_LO.mu[22] -= + acadoWorkspace_LO.y[103]*acadoWorkspace_LO.evHx[30];
acadoVariables_LO.mu[23] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[43];
acadoVariables_LO.mu[23] -= + acadoWorkspace_LO.y[103]*acadoWorkspace_LO.evHx[31];
acado_LO_expansionStep2( &(acadoWorkspace_LO.QDy[ 24 ]), &(acadoWorkspace_LO.Q1[ 192 ]), &(acadoWorkspace_LO.sbar[ 24 ]), &(acadoWorkspace_LO.S1[ 48 ]), &(acadoWorkspace_LO.x[ 6 ]), &(acadoWorkspace_LO.evGx[ 192 ]), &(acadoVariables_LO.mu[ 16 ]), &(acadoVariables_LO.mu[ 24 ]) );
acadoVariables_LO.mu[8] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[28];
acadoVariables_LO.mu[8] -= + acadoWorkspace_LO.y[102]*acadoWorkspace_LO.evHx[16];
acadoVariables_LO.mu[9] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[29];
acadoVariables_LO.mu[9] -= + acadoWorkspace_LO.y[102]*acadoWorkspace_LO.evHx[17];
acadoVariables_LO.mu[10] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[30];
acadoVariables_LO.mu[10] -= + acadoWorkspace_LO.y[102]*acadoWorkspace_LO.evHx[18];
acadoVariables_LO.mu[11] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[31];
acadoVariables_LO.mu[11] -= + acadoWorkspace_LO.y[102]*acadoWorkspace_LO.evHx[19];
acadoVariables_LO.mu[12] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[32];
acadoVariables_LO.mu[12] -= + acadoWorkspace_LO.y[102]*acadoWorkspace_LO.evHx[20];
acadoVariables_LO.mu[13] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[33];
acadoVariables_LO.mu[13] -= + acadoWorkspace_LO.y[102]*acadoWorkspace_LO.evHx[21];
acadoVariables_LO.mu[14] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[34];
acadoVariables_LO.mu[14] -= + acadoWorkspace_LO.y[102]*acadoWorkspace_LO.evHx[22];
acadoVariables_LO.mu[15] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[35];
acadoVariables_LO.mu[15] -= + acadoWorkspace_LO.y[102]*acadoWorkspace_LO.evHx[23];
acado_LO_expansionStep2( &(acadoWorkspace_LO.QDy[ 16 ]), &(acadoWorkspace_LO.Q1[ 128 ]), &(acadoWorkspace_LO.sbar[ 16 ]), &(acadoWorkspace_LO.S1[ 32 ]), &(acadoWorkspace_LO.x[ 4 ]), &(acadoWorkspace_LO.evGx[ 128 ]), &(acadoVariables_LO.mu[ 8 ]), &(acadoVariables_LO.mu[ 16 ]) );
acadoVariables_LO.mu[0] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[20];
acadoVariables_LO.mu[0] -= + acadoWorkspace_LO.y[101]*acadoWorkspace_LO.evHx[8];
acadoVariables_LO.mu[1] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[21];
acadoVariables_LO.mu[1] -= + acadoWorkspace_LO.y[101]*acadoWorkspace_LO.evHx[9];
acadoVariables_LO.mu[2] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[22];
acadoVariables_LO.mu[2] -= + acadoWorkspace_LO.y[101]*acadoWorkspace_LO.evHx[10];
acadoVariables_LO.mu[3] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[23];
acadoVariables_LO.mu[3] -= + acadoWorkspace_LO.y[101]*acadoWorkspace_LO.evHx[11];
acadoVariables_LO.mu[4] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[24];
acadoVariables_LO.mu[4] -= + acadoWorkspace_LO.y[101]*acadoWorkspace_LO.evHx[12];
acadoVariables_LO.mu[5] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[25];
acadoVariables_LO.mu[5] -= + acadoWorkspace_LO.y[101]*acadoWorkspace_LO.evHx[13];
acadoVariables_LO.mu[6] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[26];
acadoVariables_LO.mu[6] -= + acadoWorkspace_LO.y[101]*acadoWorkspace_LO.evHx[14];
acadoVariables_LO.mu[7] = + (real_t)-1.0000000000000000e+00*acadoWorkspace_LO.y[27];
acadoVariables_LO.mu[7] -= + acadoWorkspace_LO.y[101]*acadoWorkspace_LO.evHx[15];
acado_LO_expansionStep2( &(acadoWorkspace_LO.QDy[ 8 ]), &(acadoWorkspace_LO.Q1[ 64 ]), &(acadoWorkspace_LO.sbar[ 8 ]), &(acadoWorkspace_LO.S1[ 16 ]), &(acadoWorkspace_LO.x[ 2 ]), &(acadoWorkspace_LO.evGx[ 64 ]), acadoVariables_LO.mu, &(acadoVariables_LO.mu[ 8 ]) );
}

int acado_LO_preparationStep(  )
{
int ret;

ret = acado_LO_modelSimulation();
acado_LO_evaluateObjective(  );
acado_LO_regularizeHessian(  );
acado_LO_condensePrep(  );
return ret;
}

int acado_LO_feedbackStep(  )
{
int tmp;

acado_LO_condenseFdb(  );

tmp = acado_LO_solve( );

acado_LO_expand(  );
return tmp;
}

int acado_LO_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace_LO, 0, sizeof( acadoWorkspace_LO ));
acadoVariables_LO.lbValues[0] = -8.0000000000000000e+00;
acadoVariables_LO.lbValues[1] = -3.0000000000000000e+00;
acadoVariables_LO.lbValues[2] = -8.0000000000000000e+00;
acadoVariables_LO.lbValues[3] = -3.0000000000000000e+00;
acadoVariables_LO.lbValues[4] = -8.0000000000000000e+00;
acadoVariables_LO.lbValues[5] = -3.0000000000000000e+00;
acadoVariables_LO.lbValues[6] = -8.0000000000000000e+00;
acadoVariables_LO.lbValues[7] = -3.0000000000000000e+00;
acadoVariables_LO.lbValues[8] = -8.0000000000000000e+00;
acadoVariables_LO.lbValues[9] = -3.0000000000000000e+00;
acadoVariables_LO.lbValues[10] = -8.0000000000000000e+00;
acadoVariables_LO.lbValues[11] = -3.0000000000000000e+00;
acadoVariables_LO.lbValues[12] = -8.0000000000000000e+00;
acadoVariables_LO.lbValues[13] = -3.0000000000000000e+00;
acadoVariables_LO.lbValues[14] = -8.0000000000000000e+00;
acadoVariables_LO.lbValues[15] = -3.0000000000000000e+00;
acadoVariables_LO.lbValues[16] = -8.0000000000000000e+00;
acadoVariables_LO.lbValues[17] = -3.0000000000000000e+00;
acadoVariables_LO.lbValues[18] = -8.0000000000000000e+00;
acadoVariables_LO.lbValues[19] = -3.0000000000000000e+00;
acadoVariables_LO.ubValues[0] = 8.0000000000000000e+00;
acadoVariables_LO.ubValues[1] = 3.0000000000000000e+00;
acadoVariables_LO.ubValues[2] = 8.0000000000000000e+00;
acadoVariables_LO.ubValues[3] = 3.0000000000000000e+00;
acadoVariables_LO.ubValues[4] = 8.0000000000000000e+00;
acadoVariables_LO.ubValues[5] = 3.0000000000000000e+00;
acadoVariables_LO.ubValues[6] = 8.0000000000000000e+00;
acadoVariables_LO.ubValues[7] = 3.0000000000000000e+00;
acadoVariables_LO.ubValues[8] = 8.0000000000000000e+00;
acadoVariables_LO.ubValues[9] = 3.0000000000000000e+00;
acadoVariables_LO.ubValues[10] = 8.0000000000000000e+00;
acadoVariables_LO.ubValues[11] = 3.0000000000000000e+00;
acadoVariables_LO.ubValues[12] = 8.0000000000000000e+00;
acadoVariables_LO.ubValues[13] = 3.0000000000000000e+00;
acadoVariables_LO.ubValues[14] = 8.0000000000000000e+00;
acadoVariables_LO.ubValues[15] = 3.0000000000000000e+00;
acadoVariables_LO.ubValues[16] = 8.0000000000000000e+00;
acadoVariables_LO.ubValues[17] = 3.0000000000000000e+00;
acadoVariables_LO.ubValues[18] = 8.0000000000000000e+00;
acadoVariables_LO.ubValues[19] = 3.0000000000000000e+00;
acadoVariables_LO.lbAValues[0] = -1.2999999999999998e+00;
acadoVariables_LO.lbAValues[1] = -7.5000000000000000e-01;
acadoVariables_LO.lbAValues[2] = 5.0000000000000003e-02;
acadoVariables_LO.lbAValues[3] = -5.0000000000000000e-01;
acadoVariables_LO.lbAValues[4] = -4.0000000000000000e+00;
acadoVariables_LO.lbAValues[5] = -4.0999999999999998e-01;
acadoVariables_LO.lbAValues[6] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[7] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[8] = -1.2999999999999998e+00;
acadoVariables_LO.lbAValues[9] = -7.5000000000000000e-01;
acadoVariables_LO.lbAValues[10] = 5.0000000000000003e-02;
acadoVariables_LO.lbAValues[11] = -5.0000000000000000e-01;
acadoVariables_LO.lbAValues[12] = -4.0000000000000000e+00;
acadoVariables_LO.lbAValues[13] = -4.0999999999999998e-01;
acadoVariables_LO.lbAValues[14] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[15] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[16] = -1.2999999999999998e+00;
acadoVariables_LO.lbAValues[17] = -7.5000000000000000e-01;
acadoVariables_LO.lbAValues[18] = 5.0000000000000003e-02;
acadoVariables_LO.lbAValues[19] = -5.0000000000000000e-01;
acadoVariables_LO.lbAValues[20] = -4.0000000000000000e+00;
acadoVariables_LO.lbAValues[21] = -4.0999999999999998e-01;
acadoVariables_LO.lbAValues[22] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[23] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[24] = -1.2999999999999998e+00;
acadoVariables_LO.lbAValues[25] = -7.5000000000000000e-01;
acadoVariables_LO.lbAValues[26] = 5.0000000000000003e-02;
acadoVariables_LO.lbAValues[27] = -5.0000000000000000e-01;
acadoVariables_LO.lbAValues[28] = -4.0000000000000000e+00;
acadoVariables_LO.lbAValues[29] = -4.0999999999999998e-01;
acadoVariables_LO.lbAValues[30] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[31] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[32] = -1.2999999999999998e+00;
acadoVariables_LO.lbAValues[33] = -7.5000000000000000e-01;
acadoVariables_LO.lbAValues[34] = 5.0000000000000003e-02;
acadoVariables_LO.lbAValues[35] = -5.0000000000000000e-01;
acadoVariables_LO.lbAValues[36] = -4.0000000000000000e+00;
acadoVariables_LO.lbAValues[37] = -4.0999999999999998e-01;
acadoVariables_LO.lbAValues[38] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[39] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[40] = -1.2999999999999998e+00;
acadoVariables_LO.lbAValues[41] = -7.5000000000000000e-01;
acadoVariables_LO.lbAValues[42] = 5.0000000000000003e-02;
acadoVariables_LO.lbAValues[43] = -5.0000000000000000e-01;
acadoVariables_LO.lbAValues[44] = -4.0000000000000000e+00;
acadoVariables_LO.lbAValues[45] = -4.0999999999999998e-01;
acadoVariables_LO.lbAValues[46] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[47] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[48] = -1.2999999999999998e+00;
acadoVariables_LO.lbAValues[49] = -7.5000000000000000e-01;
acadoVariables_LO.lbAValues[50] = 5.0000000000000003e-02;
acadoVariables_LO.lbAValues[51] = -5.0000000000000000e-01;
acadoVariables_LO.lbAValues[52] = -4.0000000000000000e+00;
acadoVariables_LO.lbAValues[53] = -4.0999999999999998e-01;
acadoVariables_LO.lbAValues[54] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[55] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[56] = -1.2999999999999998e+00;
acadoVariables_LO.lbAValues[57] = -7.5000000000000000e-01;
acadoVariables_LO.lbAValues[58] = 5.0000000000000003e-02;
acadoVariables_LO.lbAValues[59] = -5.0000000000000000e-01;
acadoVariables_LO.lbAValues[60] = -4.0000000000000000e+00;
acadoVariables_LO.lbAValues[61] = -4.0999999999999998e-01;
acadoVariables_LO.lbAValues[62] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[63] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[64] = -1.2999999999999998e+00;
acadoVariables_LO.lbAValues[65] = -7.5000000000000000e-01;
acadoVariables_LO.lbAValues[66] = 5.0000000000000003e-02;
acadoVariables_LO.lbAValues[67] = -5.0000000000000000e-01;
acadoVariables_LO.lbAValues[68] = -4.0000000000000000e+00;
acadoVariables_LO.lbAValues[69] = -4.0999999999999998e-01;
acadoVariables_LO.lbAValues[70] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[71] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[72] = -1.2999999999999998e+00;
acadoVariables_LO.lbAValues[73] = -7.5000000000000000e-01;
acadoVariables_LO.lbAValues[74] = 5.0000000000000003e-02;
acadoVariables_LO.lbAValues[75] = -5.0000000000000000e-01;
acadoVariables_LO.lbAValues[76] = -4.0000000000000000e+00;
acadoVariables_LO.lbAValues[77] = -4.0999999999999998e-01;
acadoVariables_LO.lbAValues[78] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[79] = 0.0000000000000000e+00;
acadoVariables_LO.lbAValues[80] = -1.0000000000000000e+12;
acadoVariables_LO.lbAValues[81] = -1.0000000000000000e+12;
acadoVariables_LO.lbAValues[82] = -1.0000000000000000e+12;
acadoVariables_LO.lbAValues[83] = -1.0000000000000000e+12;
acadoVariables_LO.lbAValues[84] = -1.0000000000000000e+12;
acadoVariables_LO.lbAValues[85] = -1.0000000000000000e+12;
acadoVariables_LO.lbAValues[86] = -1.0000000000000000e+12;
acadoVariables_LO.lbAValues[87] = -1.0000000000000000e+12;
acadoVariables_LO.lbAValues[88] = -1.0000000000000000e+12;
acadoVariables_LO.lbAValues[89] = -1.0000000000000000e+12;
acadoVariables_LO.ubAValues[0] = 1.2999999999999998e+00;
acadoVariables_LO.ubAValues[1] = 7.5000000000000000e-01;
acadoVariables_LO.ubAValues[2] = 2.0000000000000000e+00;
acadoVariables_LO.ubAValues[3] = 5.0000000000000000e-01;
acadoVariables_LO.ubAValues[4] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[5] = 4.0999999999999998e-01;
acadoVariables_LO.ubAValues[6] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[7] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[8] = 1.2999999999999998e+00;
acadoVariables_LO.ubAValues[9] = 7.5000000000000000e-01;
acadoVariables_LO.ubAValues[10] = 2.0000000000000000e+00;
acadoVariables_LO.ubAValues[11] = 5.0000000000000000e-01;
acadoVariables_LO.ubAValues[12] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[13] = 4.0999999999999998e-01;
acadoVariables_LO.ubAValues[14] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[15] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[16] = 1.2999999999999998e+00;
acadoVariables_LO.ubAValues[17] = 7.5000000000000000e-01;
acadoVariables_LO.ubAValues[18] = 2.0000000000000000e+00;
acadoVariables_LO.ubAValues[19] = 5.0000000000000000e-01;
acadoVariables_LO.ubAValues[20] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[21] = 4.0999999999999998e-01;
acadoVariables_LO.ubAValues[22] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[23] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[24] = 1.2999999999999998e+00;
acadoVariables_LO.ubAValues[25] = 7.5000000000000000e-01;
acadoVariables_LO.ubAValues[26] = 2.0000000000000000e+00;
acadoVariables_LO.ubAValues[27] = 5.0000000000000000e-01;
acadoVariables_LO.ubAValues[28] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[29] = 4.0999999999999998e-01;
acadoVariables_LO.ubAValues[30] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[31] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[32] = 1.2999999999999998e+00;
acadoVariables_LO.ubAValues[33] = 7.5000000000000000e-01;
acadoVariables_LO.ubAValues[34] = 2.0000000000000000e+00;
acadoVariables_LO.ubAValues[35] = 5.0000000000000000e-01;
acadoVariables_LO.ubAValues[36] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[37] = 4.0999999999999998e-01;
acadoVariables_LO.ubAValues[38] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[39] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[40] = 1.2999999999999998e+00;
acadoVariables_LO.ubAValues[41] = 7.5000000000000000e-01;
acadoVariables_LO.ubAValues[42] = 2.0000000000000000e+00;
acadoVariables_LO.ubAValues[43] = 5.0000000000000000e-01;
acadoVariables_LO.ubAValues[44] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[45] = 4.0999999999999998e-01;
acadoVariables_LO.ubAValues[46] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[47] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[48] = 1.2999999999999998e+00;
acadoVariables_LO.ubAValues[49] = 7.5000000000000000e-01;
acadoVariables_LO.ubAValues[50] = 2.0000000000000000e+00;
acadoVariables_LO.ubAValues[51] = 5.0000000000000000e-01;
acadoVariables_LO.ubAValues[52] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[53] = 4.0999999999999998e-01;
acadoVariables_LO.ubAValues[54] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[55] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[56] = 1.2999999999999998e+00;
acadoVariables_LO.ubAValues[57] = 7.5000000000000000e-01;
acadoVariables_LO.ubAValues[58] = 2.0000000000000000e+00;
acadoVariables_LO.ubAValues[59] = 5.0000000000000000e-01;
acadoVariables_LO.ubAValues[60] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[61] = 4.0999999999999998e-01;
acadoVariables_LO.ubAValues[62] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[63] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[64] = 1.2999999999999998e+00;
acadoVariables_LO.ubAValues[65] = 7.5000000000000000e-01;
acadoVariables_LO.ubAValues[66] = 2.0000000000000000e+00;
acadoVariables_LO.ubAValues[67] = 5.0000000000000000e-01;
acadoVariables_LO.ubAValues[68] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[69] = 4.0999999999999998e-01;
acadoVariables_LO.ubAValues[70] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[71] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[72] = 1.2999999999999998e+00;
acadoVariables_LO.ubAValues[73] = 7.5000000000000000e-01;
acadoVariables_LO.ubAValues[74] = 2.0000000000000000e+00;
acadoVariables_LO.ubAValues[75] = 5.0000000000000000e-01;
acadoVariables_LO.ubAValues[76] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[77] = 4.0999999999999998e-01;
acadoVariables_LO.ubAValues[78] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[79] = 1.0000000000000000e+03;
acadoVariables_LO.ubAValues[80] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[81] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[82] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[83] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[84] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[85] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[86] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[87] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[88] = 4.0000000000000000e+00;
acadoVariables_LO.ubAValues[89] = 4.0000000000000000e+00;
return ret;
}

void acado_LO_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace_LO.state[0] = acadoVariables_LO.x[index * 8];
acadoWorkspace_LO.state[1] = acadoVariables_LO.x[index * 8 + 1];
acadoWorkspace_LO.state[2] = acadoVariables_LO.x[index * 8 + 2];
acadoWorkspace_LO.state[3] = acadoVariables_LO.x[index * 8 + 3];
acadoWorkspace_LO.state[4] = acadoVariables_LO.x[index * 8 + 4];
acadoWorkspace_LO.state[5] = acadoVariables_LO.x[index * 8 + 5];
acadoWorkspace_LO.state[6] = acadoVariables_LO.x[index * 8 + 6];
acadoWorkspace_LO.state[7] = acadoVariables_LO.x[index * 8 + 7];
acadoWorkspace_LO.state[151] = acadoVariables_LO.u[index * 2];
acadoWorkspace_LO.state[152] = acadoVariables_LO.u[index * 2 + 1];
acadoWorkspace_LO.state[153] = acadoVariables_LO.od[index];

acado_LO_integrate(acadoWorkspace_LO.state, index == 0);

acadoVariables_LO.x[index * 8 + 8] = acadoWorkspace_LO.state[0];
acadoVariables_LO.x[index * 8 + 9] = acadoWorkspace_LO.state[1];
acadoVariables_LO.x[index * 8 + 10] = acadoWorkspace_LO.state[2];
acadoVariables_LO.x[index * 8 + 11] = acadoWorkspace_LO.state[3];
acadoVariables_LO.x[index * 8 + 12] = acadoWorkspace_LO.state[4];
acadoVariables_LO.x[index * 8 + 13] = acadoWorkspace_LO.state[5];
acadoVariables_LO.x[index * 8 + 14] = acadoWorkspace_LO.state[6];
acadoVariables_LO.x[index * 8 + 15] = acadoWorkspace_LO.state[7];
}
}

void acado_LO_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables_LO.x[index * 8] = acadoVariables_LO.x[index * 8 + 8];
acadoVariables_LO.x[index * 8 + 1] = acadoVariables_LO.x[index * 8 + 9];
acadoVariables_LO.x[index * 8 + 2] = acadoVariables_LO.x[index * 8 + 10];
acadoVariables_LO.x[index * 8 + 3] = acadoVariables_LO.x[index * 8 + 11];
acadoVariables_LO.x[index * 8 + 4] = acadoVariables_LO.x[index * 8 + 12];
acadoVariables_LO.x[index * 8 + 5] = acadoVariables_LO.x[index * 8 + 13];
acadoVariables_LO.x[index * 8 + 6] = acadoVariables_LO.x[index * 8 + 14];
acadoVariables_LO.x[index * 8 + 7] = acadoVariables_LO.x[index * 8 + 15];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables_LO.x[80] = xEnd[0];
acadoVariables_LO.x[81] = xEnd[1];
acadoVariables_LO.x[82] = xEnd[2];
acadoVariables_LO.x[83] = xEnd[3];
acadoVariables_LO.x[84] = xEnd[4];
acadoVariables_LO.x[85] = xEnd[5];
acadoVariables_LO.x[86] = xEnd[6];
acadoVariables_LO.x[87] = xEnd[7];
}
else if (strategy == 2) 
{
acadoWorkspace_LO.state[0] = acadoVariables_LO.x[80];
acadoWorkspace_LO.state[1] = acadoVariables_LO.x[81];
acadoWorkspace_LO.state[2] = acadoVariables_LO.x[82];
acadoWorkspace_LO.state[3] = acadoVariables_LO.x[83];
acadoWorkspace_LO.state[4] = acadoVariables_LO.x[84];
acadoWorkspace_LO.state[5] = acadoVariables_LO.x[85];
acadoWorkspace_LO.state[6] = acadoVariables_LO.x[86];
acadoWorkspace_LO.state[7] = acadoVariables_LO.x[87];
if (uEnd != 0)
{
acadoWorkspace_LO.state[151] = uEnd[0];
acadoWorkspace_LO.state[152] = uEnd[1];
}
else
{
acadoWorkspace_LO.state[151] = acadoVariables_LO.u[18];
acadoWorkspace_LO.state[152] = acadoVariables_LO.u[19];
}
acadoWorkspace_LO.state[153] = acadoVariables_LO.od[10];

acado_LO_integrate(acadoWorkspace_LO.state, 1);

acadoVariables_LO.x[80] = acadoWorkspace_LO.state[0];
acadoVariables_LO.x[81] = acadoWorkspace_LO.state[1];
acadoVariables_LO.x[82] = acadoWorkspace_LO.state[2];
acadoVariables_LO.x[83] = acadoWorkspace_LO.state[3];
acadoVariables_LO.x[84] = acadoWorkspace_LO.state[4];
acadoVariables_LO.x[85] = acadoWorkspace_LO.state[5];
acadoVariables_LO.x[86] = acadoWorkspace_LO.state[6];
acadoVariables_LO.x[87] = acadoWorkspace_LO.state[7];
}
}

void acado_LO_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables_LO.u[index * 2] = acadoVariables_LO.u[index * 2 + 2];
acadoVariables_LO.u[index * 2 + 1] = acadoVariables_LO.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables_LO.u[18] = uEnd[0];
acadoVariables_LO.u[19] = uEnd[1];
}
}

real_t acado_LO_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace_LO.g[0]*acadoWorkspace_LO.x[0] + acadoWorkspace_LO.g[1]*acadoWorkspace_LO.x[1] + acadoWorkspace_LO.g[2]*acadoWorkspace_LO.x[2] + acadoWorkspace_LO.g[3]*acadoWorkspace_LO.x[3] + acadoWorkspace_LO.g[4]*acadoWorkspace_LO.x[4] + acadoWorkspace_LO.g[5]*acadoWorkspace_LO.x[5] + acadoWorkspace_LO.g[6]*acadoWorkspace_LO.x[6] + acadoWorkspace_LO.g[7]*acadoWorkspace_LO.x[7] + acadoWorkspace_LO.g[8]*acadoWorkspace_LO.x[8] + acadoWorkspace_LO.g[9]*acadoWorkspace_LO.x[9] + acadoWorkspace_LO.g[10]*acadoWorkspace_LO.x[10] + acadoWorkspace_LO.g[11]*acadoWorkspace_LO.x[11] + acadoWorkspace_LO.g[12]*acadoWorkspace_LO.x[12] + acadoWorkspace_LO.g[13]*acadoWorkspace_LO.x[13] + acadoWorkspace_LO.g[14]*acadoWorkspace_LO.x[14] + acadoWorkspace_LO.g[15]*acadoWorkspace_LO.x[15] + acadoWorkspace_LO.g[16]*acadoWorkspace_LO.x[16] + acadoWorkspace_LO.g[17]*acadoWorkspace_LO.x[17] + acadoWorkspace_LO.g[18]*acadoWorkspace_LO.x[18] + acadoWorkspace_LO.g[19]*acadoWorkspace_LO.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace_LO.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace_LO.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace_LO.ub[index] * prd);
}
for (index = 0; index < 90; ++index)
{
prd = acadoWorkspace_LO.y[index + 20];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace_LO.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace_LO.ubA[index] * prd);
}
return kkt;
}

real_t acado_LO_getObjective(  )
{
real_t objVal;

int lRun1;
objVal = 0.0000000000000000e+00;
acadoWorkspace_LO.objValueIn[0] = acadoVariables_LO.x[80];
acadoWorkspace_LO.objValueIn[1] = acadoVariables_LO.x[81];
acadoWorkspace_LO.objValueIn[2] = acadoVariables_LO.x[82];
acadoWorkspace_LO.objValueIn[3] = acadoVariables_LO.x[83];
acadoWorkspace_LO.objValueIn[4] = acadoVariables_LO.x[84];
acadoWorkspace_LO.objValueIn[5] = acadoVariables_LO.x[85];
acadoWorkspace_LO.objValueIn[6] = acadoVariables_LO.x[86];
acadoWorkspace_LO.objValueIn[7] = acadoVariables_LO.x[87];
acadoWorkspace_LO.objValueIn[8] = acadoVariables_LO.od[10];
acado_LO_evaluateMayer( acadoWorkspace_LO.objValueIn, acadoWorkspace_LO.objValueOut );
objVal += acadoWorkspace_LO.objValueOut[0];
return objVal;
}

