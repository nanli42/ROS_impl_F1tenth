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


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 8];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 8 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 8 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 8 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 8 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 8 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 8 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 8 + 7];

acadoWorkspace.state[8] = acadoVariables.mu[lRun1 * 8];
acadoWorkspace.state[9] = acadoVariables.mu[lRun1 * 8 + 1];
acadoWorkspace.state[10] = acadoVariables.mu[lRun1 * 8 + 2];
acadoWorkspace.state[11] = acadoVariables.mu[lRun1 * 8 + 3];
acadoWorkspace.state[12] = acadoVariables.mu[lRun1 * 8 + 4];
acadoWorkspace.state[13] = acadoVariables.mu[lRun1 * 8 + 5];
acadoWorkspace.state[14] = acadoVariables.mu[lRun1 * 8 + 6];
acadoWorkspace.state[15] = acadoVariables.mu[lRun1 * 8 + 7];
acadoWorkspace.state[151] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[152] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[153] = acadoVariables.od[lRun1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 8] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 8 + 8];
acadoWorkspace.d[lRun1 * 8 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 8 + 9];
acadoWorkspace.d[lRun1 * 8 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 8 + 10];
acadoWorkspace.d[lRun1 * 8 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 8 + 11];
acadoWorkspace.d[lRun1 * 8 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 8 + 12];
acadoWorkspace.d[lRun1 * 8 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 8 + 13];
acadoWorkspace.d[lRun1 * 8 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 8 + 14];
acadoWorkspace.d[lRun1 * 8 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 8 + 15];

acadoWorkspace.evGx[lRun1 * 64] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 64 + 1] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 64 + 2] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 64 + 3] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 64 + 4] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 64 + 5] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 64 + 6] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 64 + 7] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 64 + 8] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 64 + 9] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 64 + 10] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 64 + 11] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 64 + 12] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 64 + 13] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 64 + 14] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 64 + 15] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 64 + 16] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 64 + 17] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 64 + 18] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 64 + 19] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 64 + 20] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 64 + 21] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 64 + 22] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 64 + 23] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 64 + 24] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 64 + 25] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 64 + 26] = acadoWorkspace.state[42];
acadoWorkspace.evGx[lRun1 * 64 + 27] = acadoWorkspace.state[43];
acadoWorkspace.evGx[lRun1 * 64 + 28] = acadoWorkspace.state[44];
acadoWorkspace.evGx[lRun1 * 64 + 29] = acadoWorkspace.state[45];
acadoWorkspace.evGx[lRun1 * 64 + 30] = acadoWorkspace.state[46];
acadoWorkspace.evGx[lRun1 * 64 + 31] = acadoWorkspace.state[47];
acadoWorkspace.evGx[lRun1 * 64 + 32] = acadoWorkspace.state[48];
acadoWorkspace.evGx[lRun1 * 64 + 33] = acadoWorkspace.state[49];
acadoWorkspace.evGx[lRun1 * 64 + 34] = acadoWorkspace.state[50];
acadoWorkspace.evGx[lRun1 * 64 + 35] = acadoWorkspace.state[51];
acadoWorkspace.evGx[lRun1 * 64 + 36] = acadoWorkspace.state[52];
acadoWorkspace.evGx[lRun1 * 64 + 37] = acadoWorkspace.state[53];
acadoWorkspace.evGx[lRun1 * 64 + 38] = acadoWorkspace.state[54];
acadoWorkspace.evGx[lRun1 * 64 + 39] = acadoWorkspace.state[55];
acadoWorkspace.evGx[lRun1 * 64 + 40] = acadoWorkspace.state[56];
acadoWorkspace.evGx[lRun1 * 64 + 41] = acadoWorkspace.state[57];
acadoWorkspace.evGx[lRun1 * 64 + 42] = acadoWorkspace.state[58];
acadoWorkspace.evGx[lRun1 * 64 + 43] = acadoWorkspace.state[59];
acadoWorkspace.evGx[lRun1 * 64 + 44] = acadoWorkspace.state[60];
acadoWorkspace.evGx[lRun1 * 64 + 45] = acadoWorkspace.state[61];
acadoWorkspace.evGx[lRun1 * 64 + 46] = acadoWorkspace.state[62];
acadoWorkspace.evGx[lRun1 * 64 + 47] = acadoWorkspace.state[63];
acadoWorkspace.evGx[lRun1 * 64 + 48] = acadoWorkspace.state[64];
acadoWorkspace.evGx[lRun1 * 64 + 49] = acadoWorkspace.state[65];
acadoWorkspace.evGx[lRun1 * 64 + 50] = acadoWorkspace.state[66];
acadoWorkspace.evGx[lRun1 * 64 + 51] = acadoWorkspace.state[67];
acadoWorkspace.evGx[lRun1 * 64 + 52] = acadoWorkspace.state[68];
acadoWorkspace.evGx[lRun1 * 64 + 53] = acadoWorkspace.state[69];
acadoWorkspace.evGx[lRun1 * 64 + 54] = acadoWorkspace.state[70];
acadoWorkspace.evGx[lRun1 * 64 + 55] = acadoWorkspace.state[71];
acadoWorkspace.evGx[lRun1 * 64 + 56] = acadoWorkspace.state[72];
acadoWorkspace.evGx[lRun1 * 64 + 57] = acadoWorkspace.state[73];
acadoWorkspace.evGx[lRun1 * 64 + 58] = acadoWorkspace.state[74];
acadoWorkspace.evGx[lRun1 * 64 + 59] = acadoWorkspace.state[75];
acadoWorkspace.evGx[lRun1 * 64 + 60] = acadoWorkspace.state[76];
acadoWorkspace.evGx[lRun1 * 64 + 61] = acadoWorkspace.state[77];
acadoWorkspace.evGx[lRun1 * 64 + 62] = acadoWorkspace.state[78];
acadoWorkspace.evGx[lRun1 * 64 + 63] = acadoWorkspace.state[79];

acadoWorkspace.evGu[lRun1 * 16] = acadoWorkspace.state[80];
acadoWorkspace.evGu[lRun1 * 16 + 1] = acadoWorkspace.state[81];
acadoWorkspace.evGu[lRun1 * 16 + 2] = acadoWorkspace.state[82];
acadoWorkspace.evGu[lRun1 * 16 + 3] = acadoWorkspace.state[83];
acadoWorkspace.evGu[lRun1 * 16 + 4] = acadoWorkspace.state[84];
acadoWorkspace.evGu[lRun1 * 16 + 5] = acadoWorkspace.state[85];
acadoWorkspace.evGu[lRun1 * 16 + 6] = acadoWorkspace.state[86];
acadoWorkspace.evGu[lRun1 * 16 + 7] = acadoWorkspace.state[87];
acadoWorkspace.evGu[lRun1 * 16 + 8] = acadoWorkspace.state[88];
acadoWorkspace.evGu[lRun1 * 16 + 9] = acadoWorkspace.state[89];
acadoWorkspace.evGu[lRun1 * 16 + 10] = acadoWorkspace.state[90];
acadoWorkspace.evGu[lRun1 * 16 + 11] = acadoWorkspace.state[91];
acadoWorkspace.evGu[lRun1 * 16 + 12] = acadoWorkspace.state[92];
acadoWorkspace.evGu[lRun1 * 16 + 13] = acadoWorkspace.state[93];
acadoWorkspace.evGu[lRun1 * 16 + 14] = acadoWorkspace.state[94];
acadoWorkspace.evGu[lRun1 * 16 + 15] = acadoWorkspace.state[95];
acadoWorkspace.EH[lRun1 * 100] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[96];
acadoWorkspace.EH[lRun1 * 100 + 10] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[97];
acadoWorkspace.EH[lRun1 * 100 + 1] = acadoWorkspace.EH[lRun1 * 100 + 10];
acadoWorkspace.EH[lRun1 * 100 + 11] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[98];
acadoWorkspace.EH[lRun1 * 100 + 20] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[99];
acadoWorkspace.EH[lRun1 * 100 + 2] = acadoWorkspace.EH[lRun1 * 100 + 20];
acadoWorkspace.EH[lRun1 * 100 + 21] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[100];
acadoWorkspace.EH[lRun1 * 100 + 12] = acadoWorkspace.EH[lRun1 * 100 + 21];
acadoWorkspace.EH[lRun1 * 100 + 22] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[101];
acadoWorkspace.EH[lRun1 * 100 + 30] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[102];
acadoWorkspace.EH[lRun1 * 100 + 3] = acadoWorkspace.EH[lRun1 * 100 + 30];
acadoWorkspace.EH[lRun1 * 100 + 31] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[103];
acadoWorkspace.EH[lRun1 * 100 + 13] = acadoWorkspace.EH[lRun1 * 100 + 31];
acadoWorkspace.EH[lRun1 * 100 + 32] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[104];
acadoWorkspace.EH[lRun1 * 100 + 23] = acadoWorkspace.EH[lRun1 * 100 + 32];
acadoWorkspace.EH[lRun1 * 100 + 33] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[105];
acadoWorkspace.EH[lRun1 * 100 + 40] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[106];
acadoWorkspace.EH[lRun1 * 100 + 4] = acadoWorkspace.EH[lRun1 * 100 + 40];
acadoWorkspace.EH[lRun1 * 100 + 41] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[107];
acadoWorkspace.EH[lRun1 * 100 + 14] = acadoWorkspace.EH[lRun1 * 100 + 41];
acadoWorkspace.EH[lRun1 * 100 + 42] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[108];
acadoWorkspace.EH[lRun1 * 100 + 24] = acadoWorkspace.EH[lRun1 * 100 + 42];
acadoWorkspace.EH[lRun1 * 100 + 43] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[109];
acadoWorkspace.EH[lRun1 * 100 + 34] = acadoWorkspace.EH[lRun1 * 100 + 43];
acadoWorkspace.EH[lRun1 * 100 + 44] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[110];
acadoWorkspace.EH[lRun1 * 100 + 50] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[111];
acadoWorkspace.EH[lRun1 * 100 + 5] = acadoWorkspace.EH[lRun1 * 100 + 50];
acadoWorkspace.EH[lRun1 * 100 + 51] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[112];
acadoWorkspace.EH[lRun1 * 100 + 15] = acadoWorkspace.EH[lRun1 * 100 + 51];
acadoWorkspace.EH[lRun1 * 100 + 52] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[113];
acadoWorkspace.EH[lRun1 * 100 + 25] = acadoWorkspace.EH[lRun1 * 100 + 52];
acadoWorkspace.EH[lRun1 * 100 + 53] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[114];
acadoWorkspace.EH[lRun1 * 100 + 35] = acadoWorkspace.EH[lRun1 * 100 + 53];
acadoWorkspace.EH[lRun1 * 100 + 54] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[115];
acadoWorkspace.EH[lRun1 * 100 + 45] = acadoWorkspace.EH[lRun1 * 100 + 54];
acadoWorkspace.EH[lRun1 * 100 + 55] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[116];
acadoWorkspace.EH[lRun1 * 100 + 60] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[117];
acadoWorkspace.EH[lRun1 * 100 + 6] = acadoWorkspace.EH[lRun1 * 100 + 60];
acadoWorkspace.EH[lRun1 * 100 + 61] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[118];
acadoWorkspace.EH[lRun1 * 100 + 16] = acadoWorkspace.EH[lRun1 * 100 + 61];
acadoWorkspace.EH[lRun1 * 100 + 62] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[119];
acadoWorkspace.EH[lRun1 * 100 + 26] = acadoWorkspace.EH[lRun1 * 100 + 62];
acadoWorkspace.EH[lRun1 * 100 + 63] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[120];
acadoWorkspace.EH[lRun1 * 100 + 36] = acadoWorkspace.EH[lRun1 * 100 + 63];
acadoWorkspace.EH[lRun1 * 100 + 64] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[121];
acadoWorkspace.EH[lRun1 * 100 + 46] = acadoWorkspace.EH[lRun1 * 100 + 64];
acadoWorkspace.EH[lRun1 * 100 + 65] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[122];
acadoWorkspace.EH[lRun1 * 100 + 56] = acadoWorkspace.EH[lRun1 * 100 + 65];
acadoWorkspace.EH[lRun1 * 100 + 66] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[123];
acadoWorkspace.EH[lRun1 * 100 + 70] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[124];
acadoWorkspace.EH[lRun1 * 100 + 7] = acadoWorkspace.EH[lRun1 * 100 + 70];
acadoWorkspace.EH[lRun1 * 100 + 71] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[125];
acadoWorkspace.EH[lRun1 * 100 + 17] = acadoWorkspace.EH[lRun1 * 100 + 71];
acadoWorkspace.EH[lRun1 * 100 + 72] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[126];
acadoWorkspace.EH[lRun1 * 100 + 27] = acadoWorkspace.EH[lRun1 * 100 + 72];
acadoWorkspace.EH[lRun1 * 100 + 73] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[127];
acadoWorkspace.EH[lRun1 * 100 + 37] = acadoWorkspace.EH[lRun1 * 100 + 73];
acadoWorkspace.EH[lRun1 * 100 + 74] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[128];
acadoWorkspace.EH[lRun1 * 100 + 47] = acadoWorkspace.EH[lRun1 * 100 + 74];
acadoWorkspace.EH[lRun1 * 100 + 75] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[129];
acadoWorkspace.EH[lRun1 * 100 + 57] = acadoWorkspace.EH[lRun1 * 100 + 75];
acadoWorkspace.EH[lRun1 * 100 + 76] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[130];
acadoWorkspace.EH[lRun1 * 100 + 67] = acadoWorkspace.EH[lRun1 * 100 + 76];
acadoWorkspace.EH[lRun1 * 100 + 77] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[131];
acadoWorkspace.EH[lRun1 * 100 + 80] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[132];
acadoWorkspace.EH[lRun1 * 100 + 8] = acadoWorkspace.EH[lRun1 * 100 + 80];
acadoWorkspace.EH[lRun1 * 100 + 81] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[133];
acadoWorkspace.EH[lRun1 * 100 + 18] = acadoWorkspace.EH[lRun1 * 100 + 81];
acadoWorkspace.EH[lRun1 * 100 + 82] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[134];
acadoWorkspace.EH[lRun1 * 100 + 28] = acadoWorkspace.EH[lRun1 * 100 + 82];
acadoWorkspace.EH[lRun1 * 100 + 83] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[135];
acadoWorkspace.EH[lRun1 * 100 + 38] = acadoWorkspace.EH[lRun1 * 100 + 83];
acadoWorkspace.EH[lRun1 * 100 + 84] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[136];
acadoWorkspace.EH[lRun1 * 100 + 48] = acadoWorkspace.EH[lRun1 * 100 + 84];
acadoWorkspace.EH[lRun1 * 100 + 85] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[137];
acadoWorkspace.EH[lRun1 * 100 + 58] = acadoWorkspace.EH[lRun1 * 100 + 85];
acadoWorkspace.EH[lRun1 * 100 + 86] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[138];
acadoWorkspace.EH[lRun1 * 100 + 68] = acadoWorkspace.EH[lRun1 * 100 + 86];
acadoWorkspace.EH[lRun1 * 100 + 87] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[139];
acadoWorkspace.EH[lRun1 * 100 + 78] = acadoWorkspace.EH[lRun1 * 100 + 87];
acadoWorkspace.EH[lRun1 * 100 + 88] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[140];
acadoWorkspace.EH[lRun1 * 100 + 90] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[141];
acadoWorkspace.EH[lRun1 * 100 + 9] = acadoWorkspace.EH[lRun1 * 100 + 90];
acadoWorkspace.EH[lRun1 * 100 + 91] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[142];
acadoWorkspace.EH[lRun1 * 100 + 19] = acadoWorkspace.EH[lRun1 * 100 + 91];
acadoWorkspace.EH[lRun1 * 100 + 92] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[143];
acadoWorkspace.EH[lRun1 * 100 + 29] = acadoWorkspace.EH[lRun1 * 100 + 92];
acadoWorkspace.EH[lRun1 * 100 + 93] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[144];
acadoWorkspace.EH[lRun1 * 100 + 39] = acadoWorkspace.EH[lRun1 * 100 + 93];
acadoWorkspace.EH[lRun1 * 100 + 94] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[145];
acadoWorkspace.EH[lRun1 * 100 + 49] = acadoWorkspace.EH[lRun1 * 100 + 94];
acadoWorkspace.EH[lRun1 * 100 + 95] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[146];
acadoWorkspace.EH[lRun1 * 100 + 59] = acadoWorkspace.EH[lRun1 * 100 + 95];
acadoWorkspace.EH[lRun1 * 100 + 96] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[147];
acadoWorkspace.EH[lRun1 * 100 + 69] = acadoWorkspace.EH[lRun1 * 100 + 96];
acadoWorkspace.EH[lRun1 * 100 + 97] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[148];
acadoWorkspace.EH[lRun1 * 100 + 79] = acadoWorkspace.EH[lRun1 * 100 + 97];
acadoWorkspace.EH[lRun1 * 100 + 98] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[149];
acadoWorkspace.EH[lRun1 * 100 + 89] = acadoWorkspace.EH[lRun1 * 100 + 98];
acadoWorkspace.EH[lRun1 * 100 + 99] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[150];
}
return ret;
}

void acado_evaluateMayer(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 44. */
real_t* a = acadoWorkspace.objAuxVar;

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

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 65. */
real_t* a = acadoWorkspace.conAuxVar;

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

void acado_addObjEndTerm( real_t* const tmpFxxEnd, real_t* const tmpEH_N )
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

void acado_evaluateObjective(  )
{
int lRun2;
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.g[runObj * 2] = 0.0000000000000000e+00;
acadoWorkspace.g[runObj * 2 + 1] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 8] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 8 + 1] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 8 + 2] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 8 + 3] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 8 + 4] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 8 + 5] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 8 + 6] = 0.0000000000000000e+00;
acadoWorkspace.QDy[runObj * 8 + 7] = 0.0000000000000000e+00;
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[80];
acadoWorkspace.objValueIn[1] = acadoVariables.x[81];
acadoWorkspace.objValueIn[2] = acadoVariables.x[82];
acadoWorkspace.objValueIn[3] = acadoVariables.x[83];
acadoWorkspace.objValueIn[4] = acadoVariables.x[84];
acadoWorkspace.objValueIn[5] = acadoVariables.x[85];
acadoWorkspace.objValueIn[6] = acadoVariables.x[86];
acadoWorkspace.objValueIn[7] = acadoVariables.x[87];
acadoWorkspace.objValueIn[8] = acadoVariables.od[10];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjEndTerm( &(acadoWorkspace.objValueOut[ 9 ]), acadoWorkspace.EH_N );
acadoWorkspace.QDy[80] = acadoWorkspace.objValueOut[1];
acadoWorkspace.QDy[81] = acadoWorkspace.objValueOut[2];
acadoWorkspace.QDy[82] = acadoWorkspace.objValueOut[3];
acadoWorkspace.QDy[83] = acadoWorkspace.objValueOut[4];
acadoWorkspace.QDy[84] = acadoWorkspace.objValueOut[5];
acadoWorkspace.QDy[85] = acadoWorkspace.objValueOut[6];
acadoWorkspace.QDy[86] = acadoWorkspace.objValueOut[7];
acadoWorkspace.QDy[87] = acadoWorkspace.objValueOut[8];

for (lRun2 = 0; lRun2 < 10; ++lRun2)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun2 * 8];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun2 * 8 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun2 * 8 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun2 * 8 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun2 * 8 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.x[lRun2 * 8 + 5];
acadoWorkspace.conValueIn[6] = acadoVariables.x[lRun2 * 8 + 6];
acadoWorkspace.conValueIn[7] = acadoVariables.x[lRun2 * 8 + 7];
acadoWorkspace.conValueIn[8] = acadoWorkspace.y[lRun2 + 100];
acadoWorkspace.conValueIn[9] = acadoVariables.u[lRun2 * 2];
acadoWorkspace.conValueIn[10] = acadoVariables.u[lRun2 * 2 + 1];
acadoWorkspace.conValueIn[11] = acadoVariables.od[lRun2];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun2] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun2 * 8] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun2 * 8 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun2 * 8 + 2] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun2 * 8 + 3] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun2 * 8 + 4] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun2 * 8 + 5] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun2 * 8 + 6] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun2 * 8 + 7] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHu[lRun2 * 2] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHu[lRun2 * 2 + 1] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evDDH[0] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evDDH[1] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evDDH[2] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evDDH[3] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evDDH[4] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evDDH[5] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evDDH[6] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evDDH[7] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evDDH[8] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evDDH[9] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evDDH[10] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evDDH[11] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evDDH[12] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evDDH[13] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evDDH[14] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evDDH[15] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evDDH[16] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evDDH[17] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evDDH[18] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evDDH[19] = acadoWorkspace.conValueOut[30];
acadoWorkspace.evDDH[20] = acadoWorkspace.conValueOut[31];
acadoWorkspace.evDDH[21] = acadoWorkspace.conValueOut[32];
acadoWorkspace.evDDH[22] = acadoWorkspace.conValueOut[33];
acadoWorkspace.evDDH[23] = acadoWorkspace.conValueOut[34];
acadoWorkspace.evDDH[24] = acadoWorkspace.conValueOut[35];
acadoWorkspace.evDDH[25] = acadoWorkspace.conValueOut[36];
acadoWorkspace.evDDH[26] = acadoWorkspace.conValueOut[37];
acadoWorkspace.evDDH[27] = acadoWorkspace.conValueOut[38];
acadoWorkspace.evDDH[28] = acadoWorkspace.conValueOut[39];
acadoWorkspace.evDDH[29] = acadoWorkspace.conValueOut[40];
acadoWorkspace.evDDH[30] = acadoWorkspace.conValueOut[41];
acadoWorkspace.evDDH[31] = acadoWorkspace.conValueOut[42];
acadoWorkspace.evDDH[32] = acadoWorkspace.conValueOut[43];
acadoWorkspace.evDDH[33] = acadoWorkspace.conValueOut[44];
acadoWorkspace.evDDH[34] = acadoWorkspace.conValueOut[45];
acadoWorkspace.evDDH[35] = acadoWorkspace.conValueOut[46];
acadoWorkspace.evDDH[36] = acadoWorkspace.conValueOut[47];
acadoWorkspace.evDDH[37] = acadoWorkspace.conValueOut[48];
acadoWorkspace.evDDH[38] = acadoWorkspace.conValueOut[49];
acadoWorkspace.evDDH[39] = acadoWorkspace.conValueOut[50];
acadoWorkspace.evDDH[40] = acadoWorkspace.conValueOut[51];
acadoWorkspace.evDDH[41] = acadoWorkspace.conValueOut[52];
acadoWorkspace.evDDH[42] = acadoWorkspace.conValueOut[53];
acadoWorkspace.evDDH[43] = acadoWorkspace.conValueOut[54];
acadoWorkspace.evDDH[44] = acadoWorkspace.conValueOut[55];
acadoWorkspace.evDDH[45] = acadoWorkspace.conValueOut[56];
acadoWorkspace.evDDH[46] = acadoWorkspace.conValueOut[57];
acadoWorkspace.evDDH[47] = acadoWorkspace.conValueOut[58];
acadoWorkspace.evDDH[48] = acadoWorkspace.conValueOut[59];
acadoWorkspace.evDDH[49] = acadoWorkspace.conValueOut[60];
acadoWorkspace.evDDH[50] = acadoWorkspace.conValueOut[61];
acadoWorkspace.evDDH[51] = acadoWorkspace.conValueOut[62];
acadoWorkspace.evDDH[52] = acadoWorkspace.conValueOut[63];
acadoWorkspace.evDDH[53] = acadoWorkspace.conValueOut[64];
acadoWorkspace.evDDH[54] = acadoWorkspace.conValueOut[65];
acadoWorkspace.evDDH[55] = acadoWorkspace.conValueOut[66];
acadoWorkspace.evDDH[56] = acadoWorkspace.conValueOut[67];
acadoWorkspace.evDDH[57] = acadoWorkspace.conValueOut[68];
acadoWorkspace.evDDH[58] = acadoWorkspace.conValueOut[69];
acadoWorkspace.evDDH[59] = acadoWorkspace.conValueOut[70];
acadoWorkspace.evDDH[60] = acadoWorkspace.conValueOut[71];
acadoWorkspace.evDDH[61] = acadoWorkspace.conValueOut[72];
acadoWorkspace.evDDH[62] = acadoWorkspace.conValueOut[73];
acadoWorkspace.evDDH[63] = acadoWorkspace.conValueOut[74];
acadoWorkspace.evDDH[64] = acadoWorkspace.conValueOut[75];
acadoWorkspace.evDDH[65] = acadoWorkspace.conValueOut[76];
acadoWorkspace.evDDH[66] = acadoWorkspace.conValueOut[77];
acadoWorkspace.evDDH[67] = acadoWorkspace.conValueOut[78];
acadoWorkspace.evDDH[68] = acadoWorkspace.conValueOut[79];
acadoWorkspace.evDDH[69] = acadoWorkspace.conValueOut[80];
acadoWorkspace.evDDH[70] = acadoWorkspace.conValueOut[81];
acadoWorkspace.evDDH[71] = acadoWorkspace.conValueOut[82];
acadoWorkspace.evDDH[72] = acadoWorkspace.conValueOut[83];
acadoWorkspace.evDDH[73] = acadoWorkspace.conValueOut[84];
acadoWorkspace.evDDH[74] = acadoWorkspace.conValueOut[85];
acadoWorkspace.evDDH[75] = acadoWorkspace.conValueOut[86];
acadoWorkspace.evDDH[76] = acadoWorkspace.conValueOut[87];
acadoWorkspace.evDDH[77] = acadoWorkspace.conValueOut[88];
acadoWorkspace.evDDH[78] = acadoWorkspace.conValueOut[89];
acadoWorkspace.evDDH[79] = acadoWorkspace.conValueOut[90];
acadoWorkspace.evDDH[80] = acadoWorkspace.conValueOut[91];
acadoWorkspace.evDDH[81] = acadoWorkspace.conValueOut[92];
acadoWorkspace.evDDH[82] = acadoWorkspace.conValueOut[93];
acadoWorkspace.evDDH[83] = acadoWorkspace.conValueOut[94];
acadoWorkspace.evDDH[84] = acadoWorkspace.conValueOut[95];
acadoWorkspace.evDDH[85] = acadoWorkspace.conValueOut[96];
acadoWorkspace.evDDH[86] = acadoWorkspace.conValueOut[97];
acadoWorkspace.evDDH[87] = acadoWorkspace.conValueOut[98];
acadoWorkspace.evDDH[88] = acadoWorkspace.conValueOut[99];
acadoWorkspace.evDDH[89] = acadoWorkspace.conValueOut[100];
acadoWorkspace.evDDH[90] = acadoWorkspace.conValueOut[101];
acadoWorkspace.evDDH[91] = acadoWorkspace.conValueOut[102];
acadoWorkspace.evDDH[92] = acadoWorkspace.conValueOut[103];
acadoWorkspace.evDDH[93] = acadoWorkspace.conValueOut[104];
acadoWorkspace.evDDH[94] = acadoWorkspace.conValueOut[105];
acadoWorkspace.evDDH[95] = acadoWorkspace.conValueOut[106];
acadoWorkspace.evDDH[96] = acadoWorkspace.conValueOut[107];
acadoWorkspace.evDDH[97] = acadoWorkspace.conValueOut[108];
acadoWorkspace.evDDH[98] = acadoWorkspace.conValueOut[109];
acadoWorkspace.evDDH[99] = acadoWorkspace.conValueOut[110];
acadoWorkspace.EH[lRun2 * 100] += acadoWorkspace.evDDH[0];
acadoWorkspace.EH[lRun2 * 100 + 1] += acadoWorkspace.evDDH[1];
acadoWorkspace.EH[lRun2 * 100 + 2] += acadoWorkspace.evDDH[2];
acadoWorkspace.EH[lRun2 * 100 + 3] += acadoWorkspace.evDDH[3];
acadoWorkspace.EH[lRun2 * 100 + 4] += acadoWorkspace.evDDH[4];
acadoWorkspace.EH[lRun2 * 100 + 5] += acadoWorkspace.evDDH[5];
acadoWorkspace.EH[lRun2 * 100 + 6] += acadoWorkspace.evDDH[6];
acadoWorkspace.EH[lRun2 * 100 + 7] += acadoWorkspace.evDDH[7];
acadoWorkspace.EH[lRun2 * 100 + 8] += acadoWorkspace.evDDH[8];
acadoWorkspace.EH[lRun2 * 100 + 9] += acadoWorkspace.evDDH[9];
acadoWorkspace.EH[lRun2 * 100 + 10] += acadoWorkspace.evDDH[10];
acadoWorkspace.EH[lRun2 * 100 + 11] += acadoWorkspace.evDDH[11];
acadoWorkspace.EH[lRun2 * 100 + 12] += acadoWorkspace.evDDH[12];
acadoWorkspace.EH[lRun2 * 100 + 13] += acadoWorkspace.evDDH[13];
acadoWorkspace.EH[lRun2 * 100 + 14] += acadoWorkspace.evDDH[14];
acadoWorkspace.EH[lRun2 * 100 + 15] += acadoWorkspace.evDDH[15];
acadoWorkspace.EH[lRun2 * 100 + 16] += acadoWorkspace.evDDH[16];
acadoWorkspace.EH[lRun2 * 100 + 17] += acadoWorkspace.evDDH[17];
acadoWorkspace.EH[lRun2 * 100 + 18] += acadoWorkspace.evDDH[18];
acadoWorkspace.EH[lRun2 * 100 + 19] += acadoWorkspace.evDDH[19];
acadoWorkspace.EH[lRun2 * 100 + 20] += acadoWorkspace.evDDH[20];
acadoWorkspace.EH[lRun2 * 100 + 21] += acadoWorkspace.evDDH[21];
acadoWorkspace.EH[lRun2 * 100 + 22] += acadoWorkspace.evDDH[22];
acadoWorkspace.EH[lRun2 * 100 + 23] += acadoWorkspace.evDDH[23];
acadoWorkspace.EH[lRun2 * 100 + 24] += acadoWorkspace.evDDH[24];
acadoWorkspace.EH[lRun2 * 100 + 25] += acadoWorkspace.evDDH[25];
acadoWorkspace.EH[lRun2 * 100 + 26] += acadoWorkspace.evDDH[26];
acadoWorkspace.EH[lRun2 * 100 + 27] += acadoWorkspace.evDDH[27];
acadoWorkspace.EH[lRun2 * 100 + 28] += acadoWorkspace.evDDH[28];
acadoWorkspace.EH[lRun2 * 100 + 29] += acadoWorkspace.evDDH[29];
acadoWorkspace.EH[lRun2 * 100 + 30] += acadoWorkspace.evDDH[30];
acadoWorkspace.EH[lRun2 * 100 + 31] += acadoWorkspace.evDDH[31];
acadoWorkspace.EH[lRun2 * 100 + 32] += acadoWorkspace.evDDH[32];
acadoWorkspace.EH[lRun2 * 100 + 33] += acadoWorkspace.evDDH[33];
acadoWorkspace.EH[lRun2 * 100 + 34] += acadoWorkspace.evDDH[34];
acadoWorkspace.EH[lRun2 * 100 + 35] += acadoWorkspace.evDDH[35];
acadoWorkspace.EH[lRun2 * 100 + 36] += acadoWorkspace.evDDH[36];
acadoWorkspace.EH[lRun2 * 100 + 37] += acadoWorkspace.evDDH[37];
acadoWorkspace.EH[lRun2 * 100 + 38] += acadoWorkspace.evDDH[38];
acadoWorkspace.EH[lRun2 * 100 + 39] += acadoWorkspace.evDDH[39];
acadoWorkspace.EH[lRun2 * 100 + 40] += acadoWorkspace.evDDH[40];
acadoWorkspace.EH[lRun2 * 100 + 41] += acadoWorkspace.evDDH[41];
acadoWorkspace.EH[lRun2 * 100 + 42] += acadoWorkspace.evDDH[42];
acadoWorkspace.EH[lRun2 * 100 + 43] += acadoWorkspace.evDDH[43];
acadoWorkspace.EH[lRun2 * 100 + 44] += acadoWorkspace.evDDH[44];
acadoWorkspace.EH[lRun2 * 100 + 45] += acadoWorkspace.evDDH[45];
acadoWorkspace.EH[lRun2 * 100 + 46] += acadoWorkspace.evDDH[46];
acadoWorkspace.EH[lRun2 * 100 + 47] += acadoWorkspace.evDDH[47];
acadoWorkspace.EH[lRun2 * 100 + 48] += acadoWorkspace.evDDH[48];
acadoWorkspace.EH[lRun2 * 100 + 49] += acadoWorkspace.evDDH[49];
acadoWorkspace.EH[lRun2 * 100 + 50] += acadoWorkspace.evDDH[50];
acadoWorkspace.EH[lRun2 * 100 + 51] += acadoWorkspace.evDDH[51];
acadoWorkspace.EH[lRun2 * 100 + 52] += acadoWorkspace.evDDH[52];
acadoWorkspace.EH[lRun2 * 100 + 53] += acadoWorkspace.evDDH[53];
acadoWorkspace.EH[lRun2 * 100 + 54] += acadoWorkspace.evDDH[54];
acadoWorkspace.EH[lRun2 * 100 + 55] += acadoWorkspace.evDDH[55];
acadoWorkspace.EH[lRun2 * 100 + 56] += acadoWorkspace.evDDH[56];
acadoWorkspace.EH[lRun2 * 100 + 57] += acadoWorkspace.evDDH[57];
acadoWorkspace.EH[lRun2 * 100 + 58] += acadoWorkspace.evDDH[58];
acadoWorkspace.EH[lRun2 * 100 + 59] += acadoWorkspace.evDDH[59];
acadoWorkspace.EH[lRun2 * 100 + 60] += acadoWorkspace.evDDH[60];
acadoWorkspace.EH[lRun2 * 100 + 61] += acadoWorkspace.evDDH[61];
acadoWorkspace.EH[lRun2 * 100 + 62] += acadoWorkspace.evDDH[62];
acadoWorkspace.EH[lRun2 * 100 + 63] += acadoWorkspace.evDDH[63];
acadoWorkspace.EH[lRun2 * 100 + 64] += acadoWorkspace.evDDH[64];
acadoWorkspace.EH[lRun2 * 100 + 65] += acadoWorkspace.evDDH[65];
acadoWorkspace.EH[lRun2 * 100 + 66] += acadoWorkspace.evDDH[66];
acadoWorkspace.EH[lRun2 * 100 + 67] += acadoWorkspace.evDDH[67];
acadoWorkspace.EH[lRun2 * 100 + 68] += acadoWorkspace.evDDH[68];
acadoWorkspace.EH[lRun2 * 100 + 69] += acadoWorkspace.evDDH[69];
acadoWorkspace.EH[lRun2 * 100 + 70] += acadoWorkspace.evDDH[70];
acadoWorkspace.EH[lRun2 * 100 + 71] += acadoWorkspace.evDDH[71];
acadoWorkspace.EH[lRun2 * 100 + 72] += acadoWorkspace.evDDH[72];
acadoWorkspace.EH[lRun2 * 100 + 73] += acadoWorkspace.evDDH[73];
acadoWorkspace.EH[lRun2 * 100 + 74] += acadoWorkspace.evDDH[74];
acadoWorkspace.EH[lRun2 * 100 + 75] += acadoWorkspace.evDDH[75];
acadoWorkspace.EH[lRun2 * 100 + 76] += acadoWorkspace.evDDH[76];
acadoWorkspace.EH[lRun2 * 100 + 77] += acadoWorkspace.evDDH[77];
acadoWorkspace.EH[lRun2 * 100 + 78] += acadoWorkspace.evDDH[78];
acadoWorkspace.EH[lRun2 * 100 + 79] += acadoWorkspace.evDDH[79];
acadoWorkspace.EH[lRun2 * 100 + 80] += acadoWorkspace.evDDH[80];
acadoWorkspace.EH[lRun2 * 100 + 81] += acadoWorkspace.evDDH[81];
acadoWorkspace.EH[lRun2 * 100 + 82] += acadoWorkspace.evDDH[82];
acadoWorkspace.EH[lRun2 * 100 + 83] += acadoWorkspace.evDDH[83];
acadoWorkspace.EH[lRun2 * 100 + 84] += acadoWorkspace.evDDH[84];
acadoWorkspace.EH[lRun2 * 100 + 85] += acadoWorkspace.evDDH[85];
acadoWorkspace.EH[lRun2 * 100 + 86] += acadoWorkspace.evDDH[86];
acadoWorkspace.EH[lRun2 * 100 + 87] += acadoWorkspace.evDDH[87];
acadoWorkspace.EH[lRun2 * 100 + 88] += acadoWorkspace.evDDH[88];
acadoWorkspace.EH[lRun2 * 100 + 89] += acadoWorkspace.evDDH[89];
acadoWorkspace.EH[lRun2 * 100 + 90] += acadoWorkspace.evDDH[90];
acadoWorkspace.EH[lRun2 * 100 + 91] += acadoWorkspace.evDDH[91];
acadoWorkspace.EH[lRun2 * 100 + 92] += acadoWorkspace.evDDH[92];
acadoWorkspace.EH[lRun2 * 100 + 93] += acadoWorkspace.evDDH[93];
acadoWorkspace.EH[lRun2 * 100 + 94] += acadoWorkspace.evDDH[94];
acadoWorkspace.EH[lRun2 * 100 + 95] += acadoWorkspace.evDDH[95];
acadoWorkspace.EH[lRun2 * 100 + 96] += acadoWorkspace.evDDH[96];
acadoWorkspace.EH[lRun2 * 100 + 97] += acadoWorkspace.evDDH[97];
acadoWorkspace.EH[lRun2 * 100 + 98] += acadoWorkspace.evDDH[98];
acadoWorkspace.EH[lRun2 * 100 + 99] += acadoWorkspace.evDDH[99];
}

}

void acado_regularizeHessian(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acado_regularize( &(acadoWorkspace.EH[ lRun1 * 100 ]) );
acadoWorkspace.Q1[lRun1 * 64] = acadoWorkspace.EH[lRun1 * 100];
acadoWorkspace.Q1[lRun1 * 64 + 1] = acadoWorkspace.EH[lRun1 * 100 + 1];
acadoWorkspace.Q1[lRun1 * 64 + 2] = acadoWorkspace.EH[lRun1 * 100 + 2];
acadoWorkspace.Q1[lRun1 * 64 + 3] = acadoWorkspace.EH[lRun1 * 100 + 3];
acadoWorkspace.Q1[lRun1 * 64 + 4] = acadoWorkspace.EH[lRun1 * 100 + 4];
acadoWorkspace.Q1[lRun1 * 64 + 5] = acadoWorkspace.EH[lRun1 * 100 + 5];
acadoWorkspace.Q1[lRun1 * 64 + 6] = acadoWorkspace.EH[lRun1 * 100 + 6];
acadoWorkspace.Q1[lRun1 * 64 + 7] = acadoWorkspace.EH[lRun1 * 100 + 7];
acadoWorkspace.Q1[lRun1 * 64 + 8] = acadoWorkspace.EH[lRun1 * 100 + 10];
acadoWorkspace.Q1[lRun1 * 64 + 9] = acadoWorkspace.EH[lRun1 * 100 + 11];
acadoWorkspace.Q1[lRun1 * 64 + 10] = acadoWorkspace.EH[lRun1 * 100 + 12];
acadoWorkspace.Q1[lRun1 * 64 + 11] = acadoWorkspace.EH[lRun1 * 100 + 13];
acadoWorkspace.Q1[lRun1 * 64 + 12] = acadoWorkspace.EH[lRun1 * 100 + 14];
acadoWorkspace.Q1[lRun1 * 64 + 13] = acadoWorkspace.EH[lRun1 * 100 + 15];
acadoWorkspace.Q1[lRun1 * 64 + 14] = acadoWorkspace.EH[lRun1 * 100 + 16];
acadoWorkspace.Q1[lRun1 * 64 + 15] = acadoWorkspace.EH[lRun1 * 100 + 17];
acadoWorkspace.Q1[lRun1 * 64 + 16] = acadoWorkspace.EH[lRun1 * 100 + 20];
acadoWorkspace.Q1[lRun1 * 64 + 17] = acadoWorkspace.EH[lRun1 * 100 + 21];
acadoWorkspace.Q1[lRun1 * 64 + 18] = acadoWorkspace.EH[lRun1 * 100 + 22];
acadoWorkspace.Q1[lRun1 * 64 + 19] = acadoWorkspace.EH[lRun1 * 100 + 23];
acadoWorkspace.Q1[lRun1 * 64 + 20] = acadoWorkspace.EH[lRun1 * 100 + 24];
acadoWorkspace.Q1[lRun1 * 64 + 21] = acadoWorkspace.EH[lRun1 * 100 + 25];
acadoWorkspace.Q1[lRun1 * 64 + 22] = acadoWorkspace.EH[lRun1 * 100 + 26];
acadoWorkspace.Q1[lRun1 * 64 + 23] = acadoWorkspace.EH[lRun1 * 100 + 27];
acadoWorkspace.Q1[lRun1 * 64 + 24] = acadoWorkspace.EH[lRun1 * 100 + 30];
acadoWorkspace.Q1[lRun1 * 64 + 25] = acadoWorkspace.EH[lRun1 * 100 + 31];
acadoWorkspace.Q1[lRun1 * 64 + 26] = acadoWorkspace.EH[lRun1 * 100 + 32];
acadoWorkspace.Q1[lRun1 * 64 + 27] = acadoWorkspace.EH[lRun1 * 100 + 33];
acadoWorkspace.Q1[lRun1 * 64 + 28] = acadoWorkspace.EH[lRun1 * 100 + 34];
acadoWorkspace.Q1[lRun1 * 64 + 29] = acadoWorkspace.EH[lRun1 * 100 + 35];
acadoWorkspace.Q1[lRun1 * 64 + 30] = acadoWorkspace.EH[lRun1 * 100 + 36];
acadoWorkspace.Q1[lRun1 * 64 + 31] = acadoWorkspace.EH[lRun1 * 100 + 37];
acadoWorkspace.Q1[lRun1 * 64 + 32] = acadoWorkspace.EH[lRun1 * 100 + 40];
acadoWorkspace.Q1[lRun1 * 64 + 33] = acadoWorkspace.EH[lRun1 * 100 + 41];
acadoWorkspace.Q1[lRun1 * 64 + 34] = acadoWorkspace.EH[lRun1 * 100 + 42];
acadoWorkspace.Q1[lRun1 * 64 + 35] = acadoWorkspace.EH[lRun1 * 100 + 43];
acadoWorkspace.Q1[lRun1 * 64 + 36] = acadoWorkspace.EH[lRun1 * 100 + 44];
acadoWorkspace.Q1[lRun1 * 64 + 37] = acadoWorkspace.EH[lRun1 * 100 + 45];
acadoWorkspace.Q1[lRun1 * 64 + 38] = acadoWorkspace.EH[lRun1 * 100 + 46];
acadoWorkspace.Q1[lRun1 * 64 + 39] = acadoWorkspace.EH[lRun1 * 100 + 47];
acadoWorkspace.Q1[lRun1 * 64 + 40] = acadoWorkspace.EH[lRun1 * 100 + 50];
acadoWorkspace.Q1[lRun1 * 64 + 41] = acadoWorkspace.EH[lRun1 * 100 + 51];
acadoWorkspace.Q1[lRun1 * 64 + 42] = acadoWorkspace.EH[lRun1 * 100 + 52];
acadoWorkspace.Q1[lRun1 * 64 + 43] = acadoWorkspace.EH[lRun1 * 100 + 53];
acadoWorkspace.Q1[lRun1 * 64 + 44] = acadoWorkspace.EH[lRun1 * 100 + 54];
acadoWorkspace.Q1[lRun1 * 64 + 45] = acadoWorkspace.EH[lRun1 * 100 + 55];
acadoWorkspace.Q1[lRun1 * 64 + 46] = acadoWorkspace.EH[lRun1 * 100 + 56];
acadoWorkspace.Q1[lRun1 * 64 + 47] = acadoWorkspace.EH[lRun1 * 100 + 57];
acadoWorkspace.Q1[lRun1 * 64 + 48] = acadoWorkspace.EH[lRun1 * 100 + 60];
acadoWorkspace.Q1[lRun1 * 64 + 49] = acadoWorkspace.EH[lRun1 * 100 + 61];
acadoWorkspace.Q1[lRun1 * 64 + 50] = acadoWorkspace.EH[lRun1 * 100 + 62];
acadoWorkspace.Q1[lRun1 * 64 + 51] = acadoWorkspace.EH[lRun1 * 100 + 63];
acadoWorkspace.Q1[lRun1 * 64 + 52] = acadoWorkspace.EH[lRun1 * 100 + 64];
acadoWorkspace.Q1[lRun1 * 64 + 53] = acadoWorkspace.EH[lRun1 * 100 + 65];
acadoWorkspace.Q1[lRun1 * 64 + 54] = acadoWorkspace.EH[lRun1 * 100 + 66];
acadoWorkspace.Q1[lRun1 * 64 + 55] = acadoWorkspace.EH[lRun1 * 100 + 67];
acadoWorkspace.Q1[lRun1 * 64 + 56] = acadoWorkspace.EH[lRun1 * 100 + 70];
acadoWorkspace.Q1[lRun1 * 64 + 57] = acadoWorkspace.EH[lRun1 * 100 + 71];
acadoWorkspace.Q1[lRun1 * 64 + 58] = acadoWorkspace.EH[lRun1 * 100 + 72];
acadoWorkspace.Q1[lRun1 * 64 + 59] = acadoWorkspace.EH[lRun1 * 100 + 73];
acadoWorkspace.Q1[lRun1 * 64 + 60] = acadoWorkspace.EH[lRun1 * 100 + 74];
acadoWorkspace.Q1[lRun1 * 64 + 61] = acadoWorkspace.EH[lRun1 * 100 + 75];
acadoWorkspace.Q1[lRun1 * 64 + 62] = acadoWorkspace.EH[lRun1 * 100 + 76];
acadoWorkspace.Q1[lRun1 * 64 + 63] = acadoWorkspace.EH[lRun1 * 100 + 77];
acadoWorkspace.S1[lRun1 * 16] = acadoWorkspace.EH[lRun1 * 100 + 8];
acadoWorkspace.S1[lRun1 * 16 + 1] = acadoWorkspace.EH[lRun1 * 100 + 9];
acadoWorkspace.S1[lRun1 * 16 + 2] = acadoWorkspace.EH[lRun1 * 100 + 18];
acadoWorkspace.S1[lRun1 * 16 + 3] = acadoWorkspace.EH[lRun1 * 100 + 19];
acadoWorkspace.S1[lRun1 * 16 + 4] = acadoWorkspace.EH[lRun1 * 100 + 28];
acadoWorkspace.S1[lRun1 * 16 + 5] = acadoWorkspace.EH[lRun1 * 100 + 29];
acadoWorkspace.S1[lRun1 * 16 + 6] = acadoWorkspace.EH[lRun1 * 100 + 38];
acadoWorkspace.S1[lRun1 * 16 + 7] = acadoWorkspace.EH[lRun1 * 100 + 39];
acadoWorkspace.S1[lRun1 * 16 + 8] = acadoWorkspace.EH[lRun1 * 100 + 48];
acadoWorkspace.S1[lRun1 * 16 + 9] = acadoWorkspace.EH[lRun1 * 100 + 49];
acadoWorkspace.S1[lRun1 * 16 + 10] = acadoWorkspace.EH[lRun1 * 100 + 58];
acadoWorkspace.S1[lRun1 * 16 + 11] = acadoWorkspace.EH[lRun1 * 100 + 59];
acadoWorkspace.S1[lRun1 * 16 + 12] = acadoWorkspace.EH[lRun1 * 100 + 68];
acadoWorkspace.S1[lRun1 * 16 + 13] = acadoWorkspace.EH[lRun1 * 100 + 69];
acadoWorkspace.S1[lRun1 * 16 + 14] = acadoWorkspace.EH[lRun1 * 100 + 78];
acadoWorkspace.S1[lRun1 * 16 + 15] = acadoWorkspace.EH[lRun1 * 100 + 79];
acadoWorkspace.R1[lRun1 * 4] = acadoWorkspace.EH[lRun1 * 100 + 88];
acadoWorkspace.R1[lRun1 * 4 + 1] = acadoWorkspace.EH[lRun1 * 100 + 89];
acadoWorkspace.R1[lRun1 * 4 + 2] = acadoWorkspace.EH[lRun1 * 100 + 98];
acadoWorkspace.R1[lRun1 * 4 + 3] = acadoWorkspace.EH[lRun1 * 100 + 99];
}
acadoWorkspace.QN1[0] = acadoWorkspace.EH_N[0];
acadoWorkspace.QN1[1] = acadoWorkspace.EH_N[1];
acadoWorkspace.QN1[2] = acadoWorkspace.EH_N[2];
acadoWorkspace.QN1[3] = acadoWorkspace.EH_N[3];
acadoWorkspace.QN1[4] = acadoWorkspace.EH_N[4];
acadoWorkspace.QN1[5] = acadoWorkspace.EH_N[5];
acadoWorkspace.QN1[6] = acadoWorkspace.EH_N[6];
acadoWorkspace.QN1[7] = acadoWorkspace.EH_N[7];
acadoWorkspace.QN1[8] = acadoWorkspace.EH_N[8];
acadoWorkspace.QN1[9] = acadoWorkspace.EH_N[9];
acadoWorkspace.QN1[10] = acadoWorkspace.EH_N[10];
acadoWorkspace.QN1[11] = acadoWorkspace.EH_N[11];
acadoWorkspace.QN1[12] = acadoWorkspace.EH_N[12];
acadoWorkspace.QN1[13] = acadoWorkspace.EH_N[13];
acadoWorkspace.QN1[14] = acadoWorkspace.EH_N[14];
acadoWorkspace.QN1[15] = acadoWorkspace.EH_N[15];
acadoWorkspace.QN1[16] = acadoWorkspace.EH_N[16];
acadoWorkspace.QN1[17] = acadoWorkspace.EH_N[17];
acadoWorkspace.QN1[18] = acadoWorkspace.EH_N[18];
acadoWorkspace.QN1[19] = acadoWorkspace.EH_N[19];
acadoWorkspace.QN1[20] = acadoWorkspace.EH_N[20];
acadoWorkspace.QN1[21] = acadoWorkspace.EH_N[21];
acadoWorkspace.QN1[22] = acadoWorkspace.EH_N[22];
acadoWorkspace.QN1[23] = acadoWorkspace.EH_N[23];
acadoWorkspace.QN1[24] = acadoWorkspace.EH_N[24];
acadoWorkspace.QN1[25] = acadoWorkspace.EH_N[25];
acadoWorkspace.QN1[26] = acadoWorkspace.EH_N[26];
acadoWorkspace.QN1[27] = acadoWorkspace.EH_N[27];
acadoWorkspace.QN1[28] = acadoWorkspace.EH_N[28];
acadoWorkspace.QN1[29] = acadoWorkspace.EH_N[29];
acadoWorkspace.QN1[30] = acadoWorkspace.EH_N[30];
acadoWorkspace.QN1[31] = acadoWorkspace.EH_N[31];
acadoWorkspace.QN1[32] = acadoWorkspace.EH_N[32];
acadoWorkspace.QN1[33] = acadoWorkspace.EH_N[33];
acadoWorkspace.QN1[34] = acadoWorkspace.EH_N[34];
acadoWorkspace.QN1[35] = acadoWorkspace.EH_N[35];
acadoWorkspace.QN1[36] = acadoWorkspace.EH_N[36];
acadoWorkspace.QN1[37] = acadoWorkspace.EH_N[37];
acadoWorkspace.QN1[38] = acadoWorkspace.EH_N[38];
acadoWorkspace.QN1[39] = acadoWorkspace.EH_N[39];
acadoWorkspace.QN1[40] = acadoWorkspace.EH_N[40];
acadoWorkspace.QN1[41] = acadoWorkspace.EH_N[41];
acadoWorkspace.QN1[42] = acadoWorkspace.EH_N[42];
acadoWorkspace.QN1[43] = acadoWorkspace.EH_N[43];
acadoWorkspace.QN1[44] = acadoWorkspace.EH_N[44];
acadoWorkspace.QN1[45] = acadoWorkspace.EH_N[45];
acadoWorkspace.QN1[46] = acadoWorkspace.EH_N[46];
acadoWorkspace.QN1[47] = acadoWorkspace.EH_N[47];
acadoWorkspace.QN1[48] = acadoWorkspace.EH_N[48];
acadoWorkspace.QN1[49] = acadoWorkspace.EH_N[49];
acadoWorkspace.QN1[50] = acadoWorkspace.EH_N[50];
acadoWorkspace.QN1[51] = acadoWorkspace.EH_N[51];
acadoWorkspace.QN1[52] = acadoWorkspace.EH_N[52];
acadoWorkspace.QN1[53] = acadoWorkspace.EH_N[53];
acadoWorkspace.QN1[54] = acadoWorkspace.EH_N[54];
acadoWorkspace.QN1[55] = acadoWorkspace.EH_N[55];
acadoWorkspace.QN1[56] = acadoWorkspace.EH_N[56];
acadoWorkspace.QN1[57] = acadoWorkspace.EH_N[57];
acadoWorkspace.QN1[58] = acadoWorkspace.EH_N[58];
acadoWorkspace.QN1[59] = acadoWorkspace.EH_N[59];
acadoWorkspace.QN1[60] = acadoWorkspace.EH_N[60];
acadoWorkspace.QN1[61] = acadoWorkspace.EH_N[61];
acadoWorkspace.QN1[62] = acadoWorkspace.EH_N[62];
acadoWorkspace.QN1[63] = acadoWorkspace.EH_N[63];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
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

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
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

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
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

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
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

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10] + Gu1[12]*Gu2[12] + Gu1[14]*Gu2[14];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11] + Gu1[12]*Gu2[13] + Gu1[14]*Gu2[15];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10] + Gu1[13]*Gu2[12] + Gu1[15]*Gu2[14];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11] + Gu1[13]*Gu2[13] + Gu1[15]*Gu2[15];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10] + Gu1[12]*Gu2[12] + Gu1[14]*Gu2[14];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11] + Gu1[12]*Gu2[13] + Gu1[14]*Gu2[15];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10] + Gu1[13]*Gu2[12] + Gu1[15]*Gu2[14];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11] + Gu1[13]*Gu2[13] + Gu1[15]*Gu2[15];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 42] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10] + Gu1[12]*Gu2[12] + Gu1[14]*Gu2[14] + R11[0];
acadoWorkspace.H[iRow * 42 + 1] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11] + Gu1[12]*Gu2[13] + Gu1[14]*Gu2[15] + R11[1];
acadoWorkspace.H[iRow * 42 + 20] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10] + Gu1[13]*Gu2[12] + Gu1[15]*Gu2[14] + R11[2];
acadoWorkspace.H[iRow * 42 + 21] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11] + Gu1[13]*Gu2[13] + Gu1[15]*Gu2[15] + R11[3];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
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

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
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

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
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

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2] + Gu1[6]*w11[3] + Gu1[8]*w11[4] + Gu1[10]*w11[5] + Gu1[12]*w11[6] + Gu1[14]*w11[7];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2] + Gu1[7]*w11[3] + Gu1[9]*w11[4] + Gu1[11]*w11[5] + Gu1[13]*w11[6] + Gu1[15]*w11[7];
}

void acado_macS1TSbar( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2] + Gu1[6]*w11[3] + Gu1[8]*w11[4] + Gu1[10]*w11[5] + Gu1[12]*w11[6] + Gu1[14]*w11[7];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2] + Gu1[7]*w11[3] + Gu1[9]*w11[4] + Gu1[11]*w11[5] + Gu1[13]*w11[6] + Gu1[15]*w11[7];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
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

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
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

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
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

void acado_expansionStep2( real_t* const QDy1, real_t* const Q11, real_t* const w11, real_t* const Gu1, real_t* const U1, real_t* const Gx1, real_t* const mu1, real_t* const mu2 )
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

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2 + 1)];
}

void acado_multRDy( real_t* const RDy1 )
{
}

void acado_multQDy( real_t* const QDy1 )
{
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 20 + 1600) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4] + Hx[3]*E[6] + Hx[4]*E[8] + Hx[5]*E[10] + Hx[6]*E[12] + Hx[7]*E[14];
acadoWorkspace.A[(row * 20 + 1600) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5] + Hx[3]*E[7] + Hx[4]*E[9] + Hx[5]*E[11] + Hx[6]*E[13] + Hx[7]*E[15];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4] + Hx[5]*tmpd[5] + Hx[6]*tmpd[6] + Hx[7]*tmpd[7];
lbA[0] -= acadoWorkspace.evHxd[0];
ubA[0] -= acadoWorkspace.evHxd[0];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 80 */
static const int xBoundIndices[ 80 ] = 
{ 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87 };
acado_moveGxT( acadoWorkspace.evGx, acadoWorkspace.C );
acado_multGxGx( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.C, &(acadoWorkspace.C[ 64 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.C[ 64 ]), &(acadoWorkspace.C[ 128 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.C[ 128 ]), &(acadoWorkspace.C[ 192 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.C[ 192 ]), &(acadoWorkspace.C[ 256 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.C[ 256 ]), &(acadoWorkspace.C[ 320 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.C[ 320 ]), &(acadoWorkspace.C[ 384 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.C[ 384 ]), &(acadoWorkspace.C[ 448 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.C[ 448 ]), &(acadoWorkspace.C[ 512 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.C[ 512 ]), &(acadoWorkspace.C[ 576 ]) );
/* Column: 0 */
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_multGxGu( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.E, &(acadoWorkspace.E[ 16 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.E[ 32 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 64 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.E[ 80 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 112 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.E[ 128 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.E[ 144 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 144 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 9, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 128 ]), 9, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 128 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.W1, 8, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.E[ 112 ]), 8, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.E[ 112 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.W1, 7, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 112 ]), &(acadoWorkspace.E[ 96 ]), 7, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.E[ 96 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.W1, 6, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.E[ 80 ]), 6, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.E[ 80 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 5, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 80 ]), &(acadoWorkspace.E[ 64 ]), 5, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.E[ 64 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 4, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 64 ]), &(acadoWorkspace.E[ 48 ]), 4, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 48 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 3, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.E[ 32 ]), 3, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 32 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 2, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 32 ]), &(acadoWorkspace.E[ 16 ]), 2, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 16 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 1, 0 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 16 ]), acadoWorkspace.E, 1, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 64 ]), acadoWorkspace.E, acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( acadoWorkspace.R1, acadoWorkspace.evGu, acadoWorkspace.W1, 0 );

/* Column: 1 */
acado_moveGuE( &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.E[ 160 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.E[ 176 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 208 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.E[ 224 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 256 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.E[ 272 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.E[ 288 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 288 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 9, 1 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 272 ]), 9, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 272 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.W1, 8, 1 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.E[ 256 ]), 8, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.E[ 256 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.W1, 7, 1 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 112 ]), &(acadoWorkspace.E[ 240 ]), 7, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.E[ 240 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.W1, 6, 1 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.E[ 224 ]), 6, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.E[ 224 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 5, 1 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 80 ]), &(acadoWorkspace.E[ 208 ]), 5, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.E[ 208 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 4, 1 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 64 ]), &(acadoWorkspace.E[ 192 ]), 4, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 192 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 3, 1 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.E[ 176 ]), 3, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 176 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 2, 1 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 32 ]), &(acadoWorkspace.E[ 160 ]), 2, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.E[ 160 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 4 ]), &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 1 );

/* Column: 2 */
acado_moveGuE( &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.E[ 304 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.E[ 320 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.E[ 336 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.E[ 352 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.E[ 368 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.E[ 384 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.E[ 400 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.E[ 416 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 416 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 9, 2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 400 ]), 9, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 400 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.W1, 8, 2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.E[ 384 ]), 8, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.E[ 384 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.W1, 7, 2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 112 ]), &(acadoWorkspace.E[ 368 ]), 7, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.E[ 368 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.W1, 6, 2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.E[ 352 ]), 6, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.E[ 352 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 5, 2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 80 ]), &(acadoWorkspace.E[ 336 ]), 5, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.E[ 336 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 4, 2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 64 ]), &(acadoWorkspace.E[ 320 ]), 4, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 320 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 3, 2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.E[ 304 ]), 3, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.E[ 304 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 8 ]), &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 2 );

/* Column: 3 */
acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 432 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 448 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.E[ 464 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 496 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.E[ 512 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.E[ 528 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 528 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 9, 3 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 512 ]), 9, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 512 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.W1, 8, 3 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.E[ 496 ]), 8, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.E[ 496 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.W1, 7, 3 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 112 ]), &(acadoWorkspace.E[ 480 ]), 7, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.E[ 480 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.W1, 6, 3 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.E[ 464 ]), 6, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.E[ 464 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 5, 3 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 80 ]), &(acadoWorkspace.E[ 448 ]), 5, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.E[ 448 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 4, 3 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 64 ]), &(acadoWorkspace.E[ 432 ]), 4, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.E[ 432 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 12 ]), &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.W1, 3 );

/* Column: 4 */
acado_moveGuE( &(acadoWorkspace.evGu[ 64 ]), &(acadoWorkspace.E[ 544 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.E[ 560 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.E[ 576 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.E[ 592 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.E[ 608 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.E[ 624 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 624 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 9, 4 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 608 ]), 9, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 608 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.W1, 8, 4 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.E[ 592 ]), 8, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.E[ 592 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.W1, 7, 4 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 112 ]), &(acadoWorkspace.E[ 576 ]), 7, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.E[ 576 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.W1, 6, 4 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.E[ 560 ]), 6, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.E[ 560 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 5, 4 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 80 ]), &(acadoWorkspace.E[ 544 ]), 5, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.E[ 544 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 16 ]), &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.W1, 4 );

/* Column: 5 */
acado_moveGuE( &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.E[ 640 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.E[ 656 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.E[ 672 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.E[ 688 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.E[ 704 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 704 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 9, 5 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 688 ]), 9, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 688 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.W1, 8, 5 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.E[ 672 ]), 8, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.E[ 672 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.W1, 7, 5 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 112 ]), &(acadoWorkspace.E[ 656 ]), 7, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.E[ 656 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.W1, 6, 5 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.E[ 640 ]), 6, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.E[ 640 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 20 ]), &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 5 );

/* Column: 6 */
acado_moveGuE( &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.E[ 720 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.E[ 736 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.E[ 752 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.E[ 768 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 768 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 9, 6 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 752 ]), 9, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 752 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.W1, 8, 6 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.E[ 736 ]), 8, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.E[ 736 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.W1, 7, 6 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 112 ]), &(acadoWorkspace.E[ 720 ]), 7, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.E[ 720 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 24 ]), &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.W1, 6 );

/* Column: 7 */
acado_moveGuE( &(acadoWorkspace.evGu[ 112 ]), &(acadoWorkspace.E[ 784 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.E[ 800 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.E[ 816 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 816 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 9, 7 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 800 ]), 9, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 800 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.W1, 8, 7 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.E[ 784 ]), 8, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.E[ 784 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 28 ]), &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.W1, 7 );

/* Column: 8 */
acado_moveGuE( &(acadoWorkspace.evGu[ 128 ]), &(acadoWorkspace.E[ 832 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.E[ 848 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 848 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 9, 8 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.E[ 832 ]), 9, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.E[ 832 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 32 ]), &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.W1, 8 );

/* Column: 9 */
acado_moveGuE( &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.E[ 864 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 864 ]), acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 36 ]), &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.W1, 9 );

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );

acadoWorkspace.sbar[8] = acadoWorkspace.d[0];
acadoWorkspace.sbar[9] = acadoWorkspace.d[1];
acadoWorkspace.sbar[10] = acadoWorkspace.d[2];
acadoWorkspace.sbar[11] = acadoWorkspace.d[3];
acadoWorkspace.sbar[12] = acadoWorkspace.d[4];
acadoWorkspace.sbar[13] = acadoWorkspace.d[5];
acadoWorkspace.sbar[14] = acadoWorkspace.d[6];
acadoWorkspace.sbar[15] = acadoWorkspace.d[7];
acadoWorkspace.sbar[16] = acadoWorkspace.d[8];
acadoWorkspace.sbar[17] = acadoWorkspace.d[9];
acadoWorkspace.sbar[18] = acadoWorkspace.d[10];
acadoWorkspace.sbar[19] = acadoWorkspace.d[11];
acadoWorkspace.sbar[20] = acadoWorkspace.d[12];
acadoWorkspace.sbar[21] = acadoWorkspace.d[13];
acadoWorkspace.sbar[22] = acadoWorkspace.d[14];
acadoWorkspace.sbar[23] = acadoWorkspace.d[15];
acadoWorkspace.sbar[24] = acadoWorkspace.d[16];
acadoWorkspace.sbar[25] = acadoWorkspace.d[17];
acadoWorkspace.sbar[26] = acadoWorkspace.d[18];
acadoWorkspace.sbar[27] = acadoWorkspace.d[19];
acadoWorkspace.sbar[28] = acadoWorkspace.d[20];
acadoWorkspace.sbar[29] = acadoWorkspace.d[21];
acadoWorkspace.sbar[30] = acadoWorkspace.d[22];
acadoWorkspace.sbar[31] = acadoWorkspace.d[23];
acadoWorkspace.sbar[32] = acadoWorkspace.d[24];
acadoWorkspace.sbar[33] = acadoWorkspace.d[25];
acadoWorkspace.sbar[34] = acadoWorkspace.d[26];
acadoWorkspace.sbar[35] = acadoWorkspace.d[27];
acadoWorkspace.sbar[36] = acadoWorkspace.d[28];
acadoWorkspace.sbar[37] = acadoWorkspace.d[29];
acadoWorkspace.sbar[38] = acadoWorkspace.d[30];
acadoWorkspace.sbar[39] = acadoWorkspace.d[31];
acadoWorkspace.sbar[40] = acadoWorkspace.d[32];
acadoWorkspace.sbar[41] = acadoWorkspace.d[33];
acadoWorkspace.sbar[42] = acadoWorkspace.d[34];
acadoWorkspace.sbar[43] = acadoWorkspace.d[35];
acadoWorkspace.sbar[44] = acadoWorkspace.d[36];
acadoWorkspace.sbar[45] = acadoWorkspace.d[37];
acadoWorkspace.sbar[46] = acadoWorkspace.d[38];
acadoWorkspace.sbar[47] = acadoWorkspace.d[39];
acadoWorkspace.sbar[48] = acadoWorkspace.d[40];
acadoWorkspace.sbar[49] = acadoWorkspace.d[41];
acadoWorkspace.sbar[50] = acadoWorkspace.d[42];
acadoWorkspace.sbar[51] = acadoWorkspace.d[43];
acadoWorkspace.sbar[52] = acadoWorkspace.d[44];
acadoWorkspace.sbar[53] = acadoWorkspace.d[45];
acadoWorkspace.sbar[54] = acadoWorkspace.d[46];
acadoWorkspace.sbar[55] = acadoWorkspace.d[47];
acadoWorkspace.sbar[56] = acadoWorkspace.d[48];
acadoWorkspace.sbar[57] = acadoWorkspace.d[49];
acadoWorkspace.sbar[58] = acadoWorkspace.d[50];
acadoWorkspace.sbar[59] = acadoWorkspace.d[51];
acadoWorkspace.sbar[60] = acadoWorkspace.d[52];
acadoWorkspace.sbar[61] = acadoWorkspace.d[53];
acadoWorkspace.sbar[62] = acadoWorkspace.d[54];
acadoWorkspace.sbar[63] = acadoWorkspace.d[55];
acadoWorkspace.sbar[64] = acadoWorkspace.d[56];
acadoWorkspace.sbar[65] = acadoWorkspace.d[57];
acadoWorkspace.sbar[66] = acadoWorkspace.d[58];
acadoWorkspace.sbar[67] = acadoWorkspace.d[59];
acadoWorkspace.sbar[68] = acadoWorkspace.d[60];
acadoWorkspace.sbar[69] = acadoWorkspace.d[61];
acadoWorkspace.sbar[70] = acadoWorkspace.d[62];
acadoWorkspace.sbar[71] = acadoWorkspace.d[63];
acadoWorkspace.sbar[72] = acadoWorkspace.d[64];
acadoWorkspace.sbar[73] = acadoWorkspace.d[65];
acadoWorkspace.sbar[74] = acadoWorkspace.d[66];
acadoWorkspace.sbar[75] = acadoWorkspace.d[67];
acadoWorkspace.sbar[76] = acadoWorkspace.d[68];
acadoWorkspace.sbar[77] = acadoWorkspace.d[69];
acadoWorkspace.sbar[78] = acadoWorkspace.d[70];
acadoWorkspace.sbar[79] = acadoWorkspace.d[71];
acadoWorkspace.sbar[80] = acadoWorkspace.d[72];
acadoWorkspace.sbar[81] = acadoWorkspace.d[73];
acadoWorkspace.sbar[82] = acadoWorkspace.d[74];
acadoWorkspace.sbar[83] = acadoWorkspace.d[75];
acadoWorkspace.sbar[84] = acadoWorkspace.d[76];
acadoWorkspace.sbar[85] = acadoWorkspace.d[77];
acadoWorkspace.sbar[86] = acadoWorkspace.d[78];
acadoWorkspace.sbar[87] = acadoWorkspace.d[79];

for (lRun1 = 0; lRun1 < 80; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 8;
lRun4 = ((lRun3) / (8)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 19)) / (2)) + (lRun4)) - (1)) * (8)) + ((lRun3) % (8));
acadoWorkspace.A[(lRun1 * 20) + (lRun2 * 2)] = acadoWorkspace.E[lRun5 * 2];
acadoWorkspace.A[(lRun1 * 20) + (lRun2 * 2 + 1)] = acadoWorkspace.E[lRun5 * 2 + 1];
}
}



acado_multHxE( &(acadoWorkspace.evHx[ 8 ]), acadoWorkspace.E, 1, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 16 ]), &(acadoWorkspace.E[ 16 ]), 2, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 16 ]), &(acadoWorkspace.E[ 160 ]), 2, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.E[ 32 ]), 3, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.E[ 176 ]), 3, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.E[ 304 ]), 3, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 32 ]), &(acadoWorkspace.E[ 48 ]), 4, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 32 ]), &(acadoWorkspace.E[ 192 ]), 4, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 32 ]), &(acadoWorkspace.E[ 320 ]), 4, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 32 ]), &(acadoWorkspace.E[ 432 ]), 4, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 64 ]), 5, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 208 ]), 5, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 336 ]), 5, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 448 ]), 5, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 544 ]), 5, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 80 ]), 6, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 224 ]), 6, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 352 ]), 6, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 464 ]), 6, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 560 ]), 6, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 640 ]), 6, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.E[ 96 ]), 7, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.E[ 240 ]), 7, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.E[ 368 ]), 7, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.E[ 480 ]), 7, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.E[ 576 ]), 7, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.E[ 656 ]), 7, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.E[ 720 ]), 7, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.E[ 112 ]), 8, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.E[ 256 ]), 8, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.E[ 384 ]), 8, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.E[ 496 ]), 8, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.E[ 592 ]), 8, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.E[ 672 ]), 8, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.E[ 736 ]), 8, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.E[ 784 ]), 8, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 128 ]), 9, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 272 ]), 9, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 400 ]), 9, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 512 ]), 9, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 608 ]), 9, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 688 ]), 9, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 752 ]), 9, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 800 ]), 9, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 832 ]), 9, 8 );

acadoWorkspace.A[1600] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1601] = acadoWorkspace.evHu[1];
acadoWorkspace.A[1622] = acadoWorkspace.evHu[2];
acadoWorkspace.A[1623] = acadoWorkspace.evHu[3];
acadoWorkspace.A[1644] = acadoWorkspace.evHu[4];
acadoWorkspace.A[1645] = acadoWorkspace.evHu[5];
acadoWorkspace.A[1666] = acadoWorkspace.evHu[6];
acadoWorkspace.A[1667] = acadoWorkspace.evHu[7];
acadoWorkspace.A[1688] = acadoWorkspace.evHu[8];
acadoWorkspace.A[1689] = acadoWorkspace.evHu[9];
acadoWorkspace.A[1710] = acadoWorkspace.evHu[10];
acadoWorkspace.A[1711] = acadoWorkspace.evHu[11];
acadoWorkspace.A[1732] = acadoWorkspace.evHu[12];
acadoWorkspace.A[1733] = acadoWorkspace.evHu[13];
acadoWorkspace.A[1754] = acadoWorkspace.evHu[14];
acadoWorkspace.A[1755] = acadoWorkspace.evHu[15];
acadoWorkspace.A[1776] = acadoWorkspace.evHu[16];
acadoWorkspace.A[1777] = acadoWorkspace.evHu[17];
acadoWorkspace.A[1798] = acadoWorkspace.evHu[18];
acadoWorkspace.A[1799] = acadoWorkspace.evHu[19];
acadoWorkspace.lbA[80] = acadoVariables.lbAValues[80] - acadoWorkspace.evH[0];
acadoWorkspace.lbA[81] = acadoVariables.lbAValues[81] - acadoWorkspace.evH[1];
acadoWorkspace.lbA[82] = acadoVariables.lbAValues[82] - acadoWorkspace.evH[2];
acadoWorkspace.lbA[83] = acadoVariables.lbAValues[83] - acadoWorkspace.evH[3];
acadoWorkspace.lbA[84] = acadoVariables.lbAValues[84] - acadoWorkspace.evH[4];
acadoWorkspace.lbA[85] = acadoVariables.lbAValues[85] - acadoWorkspace.evH[5];
acadoWorkspace.lbA[86] = acadoVariables.lbAValues[86] - acadoWorkspace.evH[6];
acadoWorkspace.lbA[87] = acadoVariables.lbAValues[87] - acadoWorkspace.evH[7];
acadoWorkspace.lbA[88] = acadoVariables.lbAValues[88] - acadoWorkspace.evH[8];
acadoWorkspace.lbA[89] = acadoVariables.lbAValues[89] - acadoWorkspace.evH[9];

acadoWorkspace.ubA[80] = acadoVariables.ubAValues[80] - acadoWorkspace.evH[0];
acadoWorkspace.ubA[81] = acadoVariables.ubAValues[81] - acadoWorkspace.evH[1];
acadoWorkspace.ubA[82] = acadoVariables.ubAValues[82] - acadoWorkspace.evH[2];
acadoWorkspace.ubA[83] = acadoVariables.ubAValues[83] - acadoWorkspace.evH[3];
acadoWorkspace.ubA[84] = acadoVariables.ubAValues[84] - acadoWorkspace.evH[4];
acadoWorkspace.ubA[85] = acadoVariables.ubAValues[85] - acadoWorkspace.evH[5];
acadoWorkspace.ubA[86] = acadoVariables.ubAValues[86] - acadoWorkspace.evH[6];
acadoWorkspace.ubA[87] = acadoVariables.ubAValues[87] - acadoWorkspace.evH[7];
acadoWorkspace.ubA[88] = acadoVariables.ubAValues[88] - acadoWorkspace.evH[8];
acadoWorkspace.ubA[89] = acadoVariables.ubAValues[89] - acadoWorkspace.evH[9];

}

void acado_condenseFdb(  )
{
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 8 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 64 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 80 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[80] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[81] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[82] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[83] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[84] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[85] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[86] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[87] + acadoWorkspace.QDy[80];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[80] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[81] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[82] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[83] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[84] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[85] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[86] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[87] + acadoWorkspace.QDy[81];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[80] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[81] + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[82] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[83] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[84] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[85] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[86] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[87] + acadoWorkspace.QDy[82];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[80] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[81] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[82] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[83] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[84] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[85] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[86] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[87] + acadoWorkspace.QDy[83];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[80] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[81] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[82] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[83] + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[84] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[85] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[86] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[87] + acadoWorkspace.QDy[84];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[80] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[81] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[82] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[83] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[84] + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[85] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[86] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[87] + acadoWorkspace.QDy[85];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[80] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[81] + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[82] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[83] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[84] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[85] + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[86] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[87] + acadoWorkspace.QDy[86];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[80] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[81] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[82] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[83] + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[84] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[85] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[86] + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[87] + acadoWorkspace.QDy[87];
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 64 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.sbar[ 64 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 112 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 56 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.sbar[ 56 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 80 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.sbar[ 40 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 64 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.sbar[ 32 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 32 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.sbar[ 16 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 16 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );
acado_macS1TSbar( acadoWorkspace.S1, acadoWorkspace.sbar, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];

tmp = acadoWorkspace.sbar[8] + acadoVariables.x[8];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - tmp;
acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - tmp;
tmp = acadoWorkspace.sbar[9] + acadoVariables.x[9];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - tmp;
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - tmp;
tmp = acadoWorkspace.sbar[10] + acadoVariables.x[10];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - tmp;
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - tmp;
tmp = acadoWorkspace.sbar[11] + acadoVariables.x[11];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - tmp;
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - tmp;
tmp = acadoWorkspace.sbar[12] + acadoVariables.x[12];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - tmp;
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - tmp;
tmp = acadoWorkspace.sbar[13] + acadoVariables.x[13];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - tmp;
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - tmp;
tmp = acadoWorkspace.sbar[14] + acadoVariables.x[14];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - tmp;
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - tmp;
tmp = acadoWorkspace.sbar[15] + acadoVariables.x[15];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - tmp;
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - tmp;
tmp = acadoWorkspace.sbar[16] + acadoVariables.x[16];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - tmp;
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - tmp;
tmp = acadoWorkspace.sbar[17] + acadoVariables.x[17];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - tmp;
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - tmp;
tmp = acadoWorkspace.sbar[18] + acadoVariables.x[18];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - tmp;
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - tmp;
tmp = acadoWorkspace.sbar[19] + acadoVariables.x[19];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - tmp;
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - tmp;
tmp = acadoWorkspace.sbar[20] + acadoVariables.x[20];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - tmp;
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - tmp;
tmp = acadoWorkspace.sbar[21] + acadoVariables.x[21];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - tmp;
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - tmp;
tmp = acadoWorkspace.sbar[22] + acadoVariables.x[22];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - tmp;
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - tmp;
tmp = acadoWorkspace.sbar[23] + acadoVariables.x[23];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - tmp;
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - tmp;
tmp = acadoWorkspace.sbar[24] + acadoVariables.x[24];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - tmp;
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - tmp;
tmp = acadoWorkspace.sbar[25] + acadoVariables.x[25];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - tmp;
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - tmp;
tmp = acadoWorkspace.sbar[26] + acadoVariables.x[26];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - tmp;
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - tmp;
tmp = acadoWorkspace.sbar[27] + acadoVariables.x[27];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - tmp;
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - tmp;
tmp = acadoWorkspace.sbar[28] + acadoVariables.x[28];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - tmp;
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - tmp;
tmp = acadoWorkspace.sbar[29] + acadoVariables.x[29];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - tmp;
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - tmp;
tmp = acadoWorkspace.sbar[30] + acadoVariables.x[30];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - tmp;
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - tmp;
tmp = acadoWorkspace.sbar[31] + acadoVariables.x[31];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - tmp;
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - tmp;
tmp = acadoWorkspace.sbar[32] + acadoVariables.x[32];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - tmp;
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - tmp;
tmp = acadoWorkspace.sbar[33] + acadoVariables.x[33];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - tmp;
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - tmp;
tmp = acadoWorkspace.sbar[34] + acadoVariables.x[34];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - tmp;
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - tmp;
tmp = acadoWorkspace.sbar[35] + acadoVariables.x[35];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - tmp;
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - tmp;
tmp = acadoWorkspace.sbar[36] + acadoVariables.x[36];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - tmp;
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - tmp;
tmp = acadoWorkspace.sbar[37] + acadoVariables.x[37];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - tmp;
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - tmp;
tmp = acadoWorkspace.sbar[38] + acadoVariables.x[38];
acadoWorkspace.lbA[30] = acadoVariables.lbAValues[30] - tmp;
acadoWorkspace.ubA[30] = acadoVariables.ubAValues[30] - tmp;
tmp = acadoWorkspace.sbar[39] + acadoVariables.x[39];
acadoWorkspace.lbA[31] = acadoVariables.lbAValues[31] - tmp;
acadoWorkspace.ubA[31] = acadoVariables.ubAValues[31] - tmp;
tmp = acadoWorkspace.sbar[40] + acadoVariables.x[40];
acadoWorkspace.lbA[32] = acadoVariables.lbAValues[32] - tmp;
acadoWorkspace.ubA[32] = acadoVariables.ubAValues[32] - tmp;
tmp = acadoWorkspace.sbar[41] + acadoVariables.x[41];
acadoWorkspace.lbA[33] = acadoVariables.lbAValues[33] - tmp;
acadoWorkspace.ubA[33] = acadoVariables.ubAValues[33] - tmp;
tmp = acadoWorkspace.sbar[42] + acadoVariables.x[42];
acadoWorkspace.lbA[34] = acadoVariables.lbAValues[34] - tmp;
acadoWorkspace.ubA[34] = acadoVariables.ubAValues[34] - tmp;
tmp = acadoWorkspace.sbar[43] + acadoVariables.x[43];
acadoWorkspace.lbA[35] = acadoVariables.lbAValues[35] - tmp;
acadoWorkspace.ubA[35] = acadoVariables.ubAValues[35] - tmp;
tmp = acadoWorkspace.sbar[44] + acadoVariables.x[44];
acadoWorkspace.lbA[36] = acadoVariables.lbAValues[36] - tmp;
acadoWorkspace.ubA[36] = acadoVariables.ubAValues[36] - tmp;
tmp = acadoWorkspace.sbar[45] + acadoVariables.x[45];
acadoWorkspace.lbA[37] = acadoVariables.lbAValues[37] - tmp;
acadoWorkspace.ubA[37] = acadoVariables.ubAValues[37] - tmp;
tmp = acadoWorkspace.sbar[46] + acadoVariables.x[46];
acadoWorkspace.lbA[38] = acadoVariables.lbAValues[38] - tmp;
acadoWorkspace.ubA[38] = acadoVariables.ubAValues[38] - tmp;
tmp = acadoWorkspace.sbar[47] + acadoVariables.x[47];
acadoWorkspace.lbA[39] = acadoVariables.lbAValues[39] - tmp;
acadoWorkspace.ubA[39] = acadoVariables.ubAValues[39] - tmp;
tmp = acadoWorkspace.sbar[48] + acadoVariables.x[48];
acadoWorkspace.lbA[40] = acadoVariables.lbAValues[40] - tmp;
acadoWorkspace.ubA[40] = acadoVariables.ubAValues[40] - tmp;
tmp = acadoWorkspace.sbar[49] + acadoVariables.x[49];
acadoWorkspace.lbA[41] = acadoVariables.lbAValues[41] - tmp;
acadoWorkspace.ubA[41] = acadoVariables.ubAValues[41] - tmp;
tmp = acadoWorkspace.sbar[50] + acadoVariables.x[50];
acadoWorkspace.lbA[42] = acadoVariables.lbAValues[42] - tmp;
acadoWorkspace.ubA[42] = acadoVariables.ubAValues[42] - tmp;
tmp = acadoWorkspace.sbar[51] + acadoVariables.x[51];
acadoWorkspace.lbA[43] = acadoVariables.lbAValues[43] - tmp;
acadoWorkspace.ubA[43] = acadoVariables.ubAValues[43] - tmp;
tmp = acadoWorkspace.sbar[52] + acadoVariables.x[52];
acadoWorkspace.lbA[44] = acadoVariables.lbAValues[44] - tmp;
acadoWorkspace.ubA[44] = acadoVariables.ubAValues[44] - tmp;
tmp = acadoWorkspace.sbar[53] + acadoVariables.x[53];
acadoWorkspace.lbA[45] = acadoVariables.lbAValues[45] - tmp;
acadoWorkspace.ubA[45] = acadoVariables.ubAValues[45] - tmp;
tmp = acadoWorkspace.sbar[54] + acadoVariables.x[54];
acadoWorkspace.lbA[46] = acadoVariables.lbAValues[46] - tmp;
acadoWorkspace.ubA[46] = acadoVariables.ubAValues[46] - tmp;
tmp = acadoWorkspace.sbar[55] + acadoVariables.x[55];
acadoWorkspace.lbA[47] = acadoVariables.lbAValues[47] - tmp;
acadoWorkspace.ubA[47] = acadoVariables.ubAValues[47] - tmp;
tmp = acadoWorkspace.sbar[56] + acadoVariables.x[56];
acadoWorkspace.lbA[48] = acadoVariables.lbAValues[48] - tmp;
acadoWorkspace.ubA[48] = acadoVariables.ubAValues[48] - tmp;
tmp = acadoWorkspace.sbar[57] + acadoVariables.x[57];
acadoWorkspace.lbA[49] = acadoVariables.lbAValues[49] - tmp;
acadoWorkspace.ubA[49] = acadoVariables.ubAValues[49] - tmp;
tmp = acadoWorkspace.sbar[58] + acadoVariables.x[58];
acadoWorkspace.lbA[50] = acadoVariables.lbAValues[50] - tmp;
acadoWorkspace.ubA[50] = acadoVariables.ubAValues[50] - tmp;
tmp = acadoWorkspace.sbar[59] + acadoVariables.x[59];
acadoWorkspace.lbA[51] = acadoVariables.lbAValues[51] - tmp;
acadoWorkspace.ubA[51] = acadoVariables.ubAValues[51] - tmp;
tmp = acadoWorkspace.sbar[60] + acadoVariables.x[60];
acadoWorkspace.lbA[52] = acadoVariables.lbAValues[52] - tmp;
acadoWorkspace.ubA[52] = acadoVariables.ubAValues[52] - tmp;
tmp = acadoWorkspace.sbar[61] + acadoVariables.x[61];
acadoWorkspace.lbA[53] = acadoVariables.lbAValues[53] - tmp;
acadoWorkspace.ubA[53] = acadoVariables.ubAValues[53] - tmp;
tmp = acadoWorkspace.sbar[62] + acadoVariables.x[62];
acadoWorkspace.lbA[54] = acadoVariables.lbAValues[54] - tmp;
acadoWorkspace.ubA[54] = acadoVariables.ubAValues[54] - tmp;
tmp = acadoWorkspace.sbar[63] + acadoVariables.x[63];
acadoWorkspace.lbA[55] = acadoVariables.lbAValues[55] - tmp;
acadoWorkspace.ubA[55] = acadoVariables.ubAValues[55] - tmp;
tmp = acadoWorkspace.sbar[64] + acadoVariables.x[64];
acadoWorkspace.lbA[56] = acadoVariables.lbAValues[56] - tmp;
acadoWorkspace.ubA[56] = acadoVariables.ubAValues[56] - tmp;
tmp = acadoWorkspace.sbar[65] + acadoVariables.x[65];
acadoWorkspace.lbA[57] = acadoVariables.lbAValues[57] - tmp;
acadoWorkspace.ubA[57] = acadoVariables.ubAValues[57] - tmp;
tmp = acadoWorkspace.sbar[66] + acadoVariables.x[66];
acadoWorkspace.lbA[58] = acadoVariables.lbAValues[58] - tmp;
acadoWorkspace.ubA[58] = acadoVariables.ubAValues[58] - tmp;
tmp = acadoWorkspace.sbar[67] + acadoVariables.x[67];
acadoWorkspace.lbA[59] = acadoVariables.lbAValues[59] - tmp;
acadoWorkspace.ubA[59] = acadoVariables.ubAValues[59] - tmp;
tmp = acadoWorkspace.sbar[68] + acadoVariables.x[68];
acadoWorkspace.lbA[60] = acadoVariables.lbAValues[60] - tmp;
acadoWorkspace.ubA[60] = acadoVariables.ubAValues[60] - tmp;
tmp = acadoWorkspace.sbar[69] + acadoVariables.x[69];
acadoWorkspace.lbA[61] = acadoVariables.lbAValues[61] - tmp;
acadoWorkspace.ubA[61] = acadoVariables.ubAValues[61] - tmp;
tmp = acadoWorkspace.sbar[70] + acadoVariables.x[70];
acadoWorkspace.lbA[62] = acadoVariables.lbAValues[62] - tmp;
acadoWorkspace.ubA[62] = acadoVariables.ubAValues[62] - tmp;
tmp = acadoWorkspace.sbar[71] + acadoVariables.x[71];
acadoWorkspace.lbA[63] = acadoVariables.lbAValues[63] - tmp;
acadoWorkspace.ubA[63] = acadoVariables.ubAValues[63] - tmp;
tmp = acadoWorkspace.sbar[72] + acadoVariables.x[72];
acadoWorkspace.lbA[64] = acadoVariables.lbAValues[64] - tmp;
acadoWorkspace.ubA[64] = acadoVariables.ubAValues[64] - tmp;
tmp = acadoWorkspace.sbar[73] + acadoVariables.x[73];
acadoWorkspace.lbA[65] = acadoVariables.lbAValues[65] - tmp;
acadoWorkspace.ubA[65] = acadoVariables.ubAValues[65] - tmp;
tmp = acadoWorkspace.sbar[74] + acadoVariables.x[74];
acadoWorkspace.lbA[66] = acadoVariables.lbAValues[66] - tmp;
acadoWorkspace.ubA[66] = acadoVariables.ubAValues[66] - tmp;
tmp = acadoWorkspace.sbar[75] + acadoVariables.x[75];
acadoWorkspace.lbA[67] = acadoVariables.lbAValues[67] - tmp;
acadoWorkspace.ubA[67] = acadoVariables.ubAValues[67] - tmp;
tmp = acadoWorkspace.sbar[76] + acadoVariables.x[76];
acadoWorkspace.lbA[68] = acadoVariables.lbAValues[68] - tmp;
acadoWorkspace.ubA[68] = acadoVariables.ubAValues[68] - tmp;
tmp = acadoWorkspace.sbar[77] + acadoVariables.x[77];
acadoWorkspace.lbA[69] = acadoVariables.lbAValues[69] - tmp;
acadoWorkspace.ubA[69] = acadoVariables.ubAValues[69] - tmp;
tmp = acadoWorkspace.sbar[78] + acadoVariables.x[78];
acadoWorkspace.lbA[70] = acadoVariables.lbAValues[70] - tmp;
acadoWorkspace.ubA[70] = acadoVariables.ubAValues[70] - tmp;
tmp = acadoWorkspace.sbar[79] + acadoVariables.x[79];
acadoWorkspace.lbA[71] = acadoVariables.lbAValues[71] - tmp;
acadoWorkspace.ubA[71] = acadoVariables.ubAValues[71] - tmp;
tmp = acadoWorkspace.sbar[80] + acadoVariables.x[80];
acadoWorkspace.lbA[72] = acadoVariables.lbAValues[72] - tmp;
acadoWorkspace.ubA[72] = acadoVariables.ubAValues[72] - tmp;
tmp = acadoWorkspace.sbar[81] + acadoVariables.x[81];
acadoWorkspace.lbA[73] = acadoVariables.lbAValues[73] - tmp;
acadoWorkspace.ubA[73] = acadoVariables.ubAValues[73] - tmp;
tmp = acadoWorkspace.sbar[82] + acadoVariables.x[82];
acadoWorkspace.lbA[74] = acadoVariables.lbAValues[74] - tmp;
acadoWorkspace.ubA[74] = acadoVariables.ubAValues[74] - tmp;
tmp = acadoWorkspace.sbar[83] + acadoVariables.x[83];
acadoWorkspace.lbA[75] = acadoVariables.lbAValues[75] - tmp;
acadoWorkspace.ubA[75] = acadoVariables.ubAValues[75] - tmp;
tmp = acadoWorkspace.sbar[84] + acadoVariables.x[84];
acadoWorkspace.lbA[76] = acadoVariables.lbAValues[76] - tmp;
acadoWorkspace.ubA[76] = acadoVariables.ubAValues[76] - tmp;
tmp = acadoWorkspace.sbar[85] + acadoVariables.x[85];
acadoWorkspace.lbA[77] = acadoVariables.lbAValues[77] - tmp;
acadoWorkspace.ubA[77] = acadoVariables.ubAValues[77] - tmp;
tmp = acadoWorkspace.sbar[86] + acadoVariables.x[86];
acadoWorkspace.lbA[78] = acadoVariables.lbAValues[78] - tmp;
acadoWorkspace.ubA[78] = acadoVariables.ubAValues[78] - tmp;
tmp = acadoWorkspace.sbar[87] + acadoVariables.x[87];
acadoWorkspace.lbA[79] = acadoVariables.lbAValues[79] - tmp;
acadoWorkspace.ubA[79] = acadoVariables.ubAValues[79] - tmp;

acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, &(acadoWorkspace.lbA[ 80 ]), &(acadoWorkspace.ubA[ 80 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 8 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.lbA[ 81 ]), &(acadoWorkspace.ubA[ 81 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 16 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.lbA[ 82 ]), &(acadoWorkspace.ubA[ 82 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.lbA[ 83 ]), &(acadoWorkspace.ubA[ 83 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 32 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.lbA[ 84 ]), &(acadoWorkspace.ubA[ 84 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.lbA[ 85 ]), &(acadoWorkspace.ubA[ 85 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.lbA[ 86 ]), &(acadoWorkspace.ubA[ 86 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.lbA[ 87 ]), &(acadoWorkspace.ubA[ 87 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.lbA[ 88 ]), &(acadoWorkspace.ubA[ 88 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.lbA[ 89 ]), &(acadoWorkspace.ubA[ 89 ]) );

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.d[0];
acadoWorkspace.sbar[9] = acadoWorkspace.d[1];
acadoWorkspace.sbar[10] = acadoWorkspace.d[2];
acadoWorkspace.sbar[11] = acadoWorkspace.d[3];
acadoWorkspace.sbar[12] = acadoWorkspace.d[4];
acadoWorkspace.sbar[13] = acadoWorkspace.d[5];
acadoWorkspace.sbar[14] = acadoWorkspace.d[6];
acadoWorkspace.sbar[15] = acadoWorkspace.d[7];
acadoWorkspace.sbar[16] = acadoWorkspace.d[8];
acadoWorkspace.sbar[17] = acadoWorkspace.d[9];
acadoWorkspace.sbar[18] = acadoWorkspace.d[10];
acadoWorkspace.sbar[19] = acadoWorkspace.d[11];
acadoWorkspace.sbar[20] = acadoWorkspace.d[12];
acadoWorkspace.sbar[21] = acadoWorkspace.d[13];
acadoWorkspace.sbar[22] = acadoWorkspace.d[14];
acadoWorkspace.sbar[23] = acadoWorkspace.d[15];
acadoWorkspace.sbar[24] = acadoWorkspace.d[16];
acadoWorkspace.sbar[25] = acadoWorkspace.d[17];
acadoWorkspace.sbar[26] = acadoWorkspace.d[18];
acadoWorkspace.sbar[27] = acadoWorkspace.d[19];
acadoWorkspace.sbar[28] = acadoWorkspace.d[20];
acadoWorkspace.sbar[29] = acadoWorkspace.d[21];
acadoWorkspace.sbar[30] = acadoWorkspace.d[22];
acadoWorkspace.sbar[31] = acadoWorkspace.d[23];
acadoWorkspace.sbar[32] = acadoWorkspace.d[24];
acadoWorkspace.sbar[33] = acadoWorkspace.d[25];
acadoWorkspace.sbar[34] = acadoWorkspace.d[26];
acadoWorkspace.sbar[35] = acadoWorkspace.d[27];
acadoWorkspace.sbar[36] = acadoWorkspace.d[28];
acadoWorkspace.sbar[37] = acadoWorkspace.d[29];
acadoWorkspace.sbar[38] = acadoWorkspace.d[30];
acadoWorkspace.sbar[39] = acadoWorkspace.d[31];
acadoWorkspace.sbar[40] = acadoWorkspace.d[32];
acadoWorkspace.sbar[41] = acadoWorkspace.d[33];
acadoWorkspace.sbar[42] = acadoWorkspace.d[34];
acadoWorkspace.sbar[43] = acadoWorkspace.d[35];
acadoWorkspace.sbar[44] = acadoWorkspace.d[36];
acadoWorkspace.sbar[45] = acadoWorkspace.d[37];
acadoWorkspace.sbar[46] = acadoWorkspace.d[38];
acadoWorkspace.sbar[47] = acadoWorkspace.d[39];
acadoWorkspace.sbar[48] = acadoWorkspace.d[40];
acadoWorkspace.sbar[49] = acadoWorkspace.d[41];
acadoWorkspace.sbar[50] = acadoWorkspace.d[42];
acadoWorkspace.sbar[51] = acadoWorkspace.d[43];
acadoWorkspace.sbar[52] = acadoWorkspace.d[44];
acadoWorkspace.sbar[53] = acadoWorkspace.d[45];
acadoWorkspace.sbar[54] = acadoWorkspace.d[46];
acadoWorkspace.sbar[55] = acadoWorkspace.d[47];
acadoWorkspace.sbar[56] = acadoWorkspace.d[48];
acadoWorkspace.sbar[57] = acadoWorkspace.d[49];
acadoWorkspace.sbar[58] = acadoWorkspace.d[50];
acadoWorkspace.sbar[59] = acadoWorkspace.d[51];
acadoWorkspace.sbar[60] = acadoWorkspace.d[52];
acadoWorkspace.sbar[61] = acadoWorkspace.d[53];
acadoWorkspace.sbar[62] = acadoWorkspace.d[54];
acadoWorkspace.sbar[63] = acadoWorkspace.d[55];
acadoWorkspace.sbar[64] = acadoWorkspace.d[56];
acadoWorkspace.sbar[65] = acadoWorkspace.d[57];
acadoWorkspace.sbar[66] = acadoWorkspace.d[58];
acadoWorkspace.sbar[67] = acadoWorkspace.d[59];
acadoWorkspace.sbar[68] = acadoWorkspace.d[60];
acadoWorkspace.sbar[69] = acadoWorkspace.d[61];
acadoWorkspace.sbar[70] = acadoWorkspace.d[62];
acadoWorkspace.sbar[71] = acadoWorkspace.d[63];
acadoWorkspace.sbar[72] = acadoWorkspace.d[64];
acadoWorkspace.sbar[73] = acadoWorkspace.d[65];
acadoWorkspace.sbar[74] = acadoWorkspace.d[66];
acadoWorkspace.sbar[75] = acadoWorkspace.d[67];
acadoWorkspace.sbar[76] = acadoWorkspace.d[68];
acadoWorkspace.sbar[77] = acadoWorkspace.d[69];
acadoWorkspace.sbar[78] = acadoWorkspace.d[70];
acadoWorkspace.sbar[79] = acadoWorkspace.d[71];
acadoWorkspace.sbar[80] = acadoWorkspace.d[72];
acadoWorkspace.sbar[81] = acadoWorkspace.d[73];
acadoWorkspace.sbar[82] = acadoWorkspace.d[74];
acadoWorkspace.sbar[83] = acadoWorkspace.d[75];
acadoWorkspace.sbar[84] = acadoWorkspace.d[76];
acadoWorkspace.sbar[85] = acadoWorkspace.d[77];
acadoWorkspace.sbar[86] = acadoWorkspace.d[78];
acadoWorkspace.sbar[87] = acadoWorkspace.d[79];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.evGu[ 64 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.evGu[ 112 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 64 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.evGu[ 128 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 80 ]) );
acadoVariables.x[0] += acadoWorkspace.sbar[0];
acadoVariables.x[1] += acadoWorkspace.sbar[1];
acadoVariables.x[2] += acadoWorkspace.sbar[2];
acadoVariables.x[3] += acadoWorkspace.sbar[3];
acadoVariables.x[4] += acadoWorkspace.sbar[4];
acadoVariables.x[5] += acadoWorkspace.sbar[5];
acadoVariables.x[6] += acadoWorkspace.sbar[6];
acadoVariables.x[7] += acadoWorkspace.sbar[7];
acadoVariables.x[8] += acadoWorkspace.sbar[8];
acadoVariables.x[9] += acadoWorkspace.sbar[9];
acadoVariables.x[10] += acadoWorkspace.sbar[10];
acadoVariables.x[11] += acadoWorkspace.sbar[11];
acadoVariables.x[12] += acadoWorkspace.sbar[12];
acadoVariables.x[13] += acadoWorkspace.sbar[13];
acadoVariables.x[14] += acadoWorkspace.sbar[14];
acadoVariables.x[15] += acadoWorkspace.sbar[15];
acadoVariables.x[16] += acadoWorkspace.sbar[16];
acadoVariables.x[17] += acadoWorkspace.sbar[17];
acadoVariables.x[18] += acadoWorkspace.sbar[18];
acadoVariables.x[19] += acadoWorkspace.sbar[19];
acadoVariables.x[20] += acadoWorkspace.sbar[20];
acadoVariables.x[21] += acadoWorkspace.sbar[21];
acadoVariables.x[22] += acadoWorkspace.sbar[22];
acadoVariables.x[23] += acadoWorkspace.sbar[23];
acadoVariables.x[24] += acadoWorkspace.sbar[24];
acadoVariables.x[25] += acadoWorkspace.sbar[25];
acadoVariables.x[26] += acadoWorkspace.sbar[26];
acadoVariables.x[27] += acadoWorkspace.sbar[27];
acadoVariables.x[28] += acadoWorkspace.sbar[28];
acadoVariables.x[29] += acadoWorkspace.sbar[29];
acadoVariables.x[30] += acadoWorkspace.sbar[30];
acadoVariables.x[31] += acadoWorkspace.sbar[31];
acadoVariables.x[32] += acadoWorkspace.sbar[32];
acadoVariables.x[33] += acadoWorkspace.sbar[33];
acadoVariables.x[34] += acadoWorkspace.sbar[34];
acadoVariables.x[35] += acadoWorkspace.sbar[35];
acadoVariables.x[36] += acadoWorkspace.sbar[36];
acadoVariables.x[37] += acadoWorkspace.sbar[37];
acadoVariables.x[38] += acadoWorkspace.sbar[38];
acadoVariables.x[39] += acadoWorkspace.sbar[39];
acadoVariables.x[40] += acadoWorkspace.sbar[40];
acadoVariables.x[41] += acadoWorkspace.sbar[41];
acadoVariables.x[42] += acadoWorkspace.sbar[42];
acadoVariables.x[43] += acadoWorkspace.sbar[43];
acadoVariables.x[44] += acadoWorkspace.sbar[44];
acadoVariables.x[45] += acadoWorkspace.sbar[45];
acadoVariables.x[46] += acadoWorkspace.sbar[46];
acadoVariables.x[47] += acadoWorkspace.sbar[47];
acadoVariables.x[48] += acadoWorkspace.sbar[48];
acadoVariables.x[49] += acadoWorkspace.sbar[49];
acadoVariables.x[50] += acadoWorkspace.sbar[50];
acadoVariables.x[51] += acadoWorkspace.sbar[51];
acadoVariables.x[52] += acadoWorkspace.sbar[52];
acadoVariables.x[53] += acadoWorkspace.sbar[53];
acadoVariables.x[54] += acadoWorkspace.sbar[54];
acadoVariables.x[55] += acadoWorkspace.sbar[55];
acadoVariables.x[56] += acadoWorkspace.sbar[56];
acadoVariables.x[57] += acadoWorkspace.sbar[57];
acadoVariables.x[58] += acadoWorkspace.sbar[58];
acadoVariables.x[59] += acadoWorkspace.sbar[59];
acadoVariables.x[60] += acadoWorkspace.sbar[60];
acadoVariables.x[61] += acadoWorkspace.sbar[61];
acadoVariables.x[62] += acadoWorkspace.sbar[62];
acadoVariables.x[63] += acadoWorkspace.sbar[63];
acadoVariables.x[64] += acadoWorkspace.sbar[64];
acadoVariables.x[65] += acadoWorkspace.sbar[65];
acadoVariables.x[66] += acadoWorkspace.sbar[66];
acadoVariables.x[67] += acadoWorkspace.sbar[67];
acadoVariables.x[68] += acadoWorkspace.sbar[68];
acadoVariables.x[69] += acadoWorkspace.sbar[69];
acadoVariables.x[70] += acadoWorkspace.sbar[70];
acadoVariables.x[71] += acadoWorkspace.sbar[71];
acadoVariables.x[72] += acadoWorkspace.sbar[72];
acadoVariables.x[73] += acadoWorkspace.sbar[73];
acadoVariables.x[74] += acadoWorkspace.sbar[74];
acadoVariables.x[75] += acadoWorkspace.sbar[75];
acadoVariables.x[76] += acadoWorkspace.sbar[76];
acadoVariables.x[77] += acadoWorkspace.sbar[77];
acadoVariables.x[78] += acadoWorkspace.sbar[78];
acadoVariables.x[79] += acadoWorkspace.sbar[79];
acadoVariables.x[80] += acadoWorkspace.sbar[80];
acadoVariables.x[81] += acadoWorkspace.sbar[81];
acadoVariables.x[82] += acadoWorkspace.sbar[82];
acadoVariables.x[83] += acadoWorkspace.sbar[83];
acadoVariables.x[84] += acadoWorkspace.sbar[84];
acadoVariables.x[85] += acadoWorkspace.sbar[85];
acadoVariables.x[86] += acadoWorkspace.sbar[86];
acadoVariables.x[87] += acadoWorkspace.sbar[87];
acadoVariables.mu[72] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[92];
acadoVariables.mu[73] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[93];
acadoVariables.mu[74] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[94];
acadoVariables.mu[75] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[95];
acadoVariables.mu[76] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[96];
acadoVariables.mu[77] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[97];
acadoVariables.mu[78] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[98];
acadoVariables.mu[79] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[99];
acadoVariables.mu[72] += + acadoWorkspace.sbar[80]*acadoWorkspace.QN1[0] + acadoWorkspace.sbar[81]*acadoWorkspace.QN1[8] + acadoWorkspace.sbar[82]*acadoWorkspace.QN1[16] + acadoWorkspace.sbar[83]*acadoWorkspace.QN1[24] + acadoWorkspace.sbar[84]*acadoWorkspace.QN1[32] + acadoWorkspace.sbar[85]*acadoWorkspace.QN1[40] + acadoWorkspace.sbar[86]*acadoWorkspace.QN1[48] + acadoWorkspace.sbar[87]*acadoWorkspace.QN1[56];
acadoVariables.mu[73] += + acadoWorkspace.sbar[80]*acadoWorkspace.QN1[1] + acadoWorkspace.sbar[81]*acadoWorkspace.QN1[9] + acadoWorkspace.sbar[82]*acadoWorkspace.QN1[17] + acadoWorkspace.sbar[83]*acadoWorkspace.QN1[25] + acadoWorkspace.sbar[84]*acadoWorkspace.QN1[33] + acadoWorkspace.sbar[85]*acadoWorkspace.QN1[41] + acadoWorkspace.sbar[86]*acadoWorkspace.QN1[49] + acadoWorkspace.sbar[87]*acadoWorkspace.QN1[57];
acadoVariables.mu[74] += + acadoWorkspace.sbar[80]*acadoWorkspace.QN1[2] + acadoWorkspace.sbar[81]*acadoWorkspace.QN1[10] + acadoWorkspace.sbar[82]*acadoWorkspace.QN1[18] + acadoWorkspace.sbar[83]*acadoWorkspace.QN1[26] + acadoWorkspace.sbar[84]*acadoWorkspace.QN1[34] + acadoWorkspace.sbar[85]*acadoWorkspace.QN1[42] + acadoWorkspace.sbar[86]*acadoWorkspace.QN1[50] + acadoWorkspace.sbar[87]*acadoWorkspace.QN1[58];
acadoVariables.mu[75] += + acadoWorkspace.sbar[80]*acadoWorkspace.QN1[3] + acadoWorkspace.sbar[81]*acadoWorkspace.QN1[11] + acadoWorkspace.sbar[82]*acadoWorkspace.QN1[19] + acadoWorkspace.sbar[83]*acadoWorkspace.QN1[27] + acadoWorkspace.sbar[84]*acadoWorkspace.QN1[35] + acadoWorkspace.sbar[85]*acadoWorkspace.QN1[43] + acadoWorkspace.sbar[86]*acadoWorkspace.QN1[51] + acadoWorkspace.sbar[87]*acadoWorkspace.QN1[59];
acadoVariables.mu[76] += + acadoWorkspace.sbar[80]*acadoWorkspace.QN1[4] + acadoWorkspace.sbar[81]*acadoWorkspace.QN1[12] + acadoWorkspace.sbar[82]*acadoWorkspace.QN1[20] + acadoWorkspace.sbar[83]*acadoWorkspace.QN1[28] + acadoWorkspace.sbar[84]*acadoWorkspace.QN1[36] + acadoWorkspace.sbar[85]*acadoWorkspace.QN1[44] + acadoWorkspace.sbar[86]*acadoWorkspace.QN1[52] + acadoWorkspace.sbar[87]*acadoWorkspace.QN1[60];
acadoVariables.mu[77] += + acadoWorkspace.sbar[80]*acadoWorkspace.QN1[5] + acadoWorkspace.sbar[81]*acadoWorkspace.QN1[13] + acadoWorkspace.sbar[82]*acadoWorkspace.QN1[21] + acadoWorkspace.sbar[83]*acadoWorkspace.QN1[29] + acadoWorkspace.sbar[84]*acadoWorkspace.QN1[37] + acadoWorkspace.sbar[85]*acadoWorkspace.QN1[45] + acadoWorkspace.sbar[86]*acadoWorkspace.QN1[53] + acadoWorkspace.sbar[87]*acadoWorkspace.QN1[61];
acadoVariables.mu[78] += + acadoWorkspace.sbar[80]*acadoWorkspace.QN1[6] + acadoWorkspace.sbar[81]*acadoWorkspace.QN1[14] + acadoWorkspace.sbar[82]*acadoWorkspace.QN1[22] + acadoWorkspace.sbar[83]*acadoWorkspace.QN1[30] + acadoWorkspace.sbar[84]*acadoWorkspace.QN1[38] + acadoWorkspace.sbar[85]*acadoWorkspace.QN1[46] + acadoWorkspace.sbar[86]*acadoWorkspace.QN1[54] + acadoWorkspace.sbar[87]*acadoWorkspace.QN1[62];
acadoVariables.mu[79] += + acadoWorkspace.sbar[80]*acadoWorkspace.QN1[7] + acadoWorkspace.sbar[81]*acadoWorkspace.QN1[15] + acadoWorkspace.sbar[82]*acadoWorkspace.QN1[23] + acadoWorkspace.sbar[83]*acadoWorkspace.QN1[31] + acadoWorkspace.sbar[84]*acadoWorkspace.QN1[39] + acadoWorkspace.sbar[85]*acadoWorkspace.QN1[47] + acadoWorkspace.sbar[86]*acadoWorkspace.QN1[55] + acadoWorkspace.sbar[87]*acadoWorkspace.QN1[63];
acadoVariables.mu[72] += acadoWorkspace.QDy[80];
acadoVariables.mu[73] += acadoWorkspace.QDy[81];
acadoVariables.mu[74] += acadoWorkspace.QDy[82];
acadoVariables.mu[75] += acadoWorkspace.QDy[83];
acadoVariables.mu[76] += acadoWorkspace.QDy[84];
acadoVariables.mu[77] += acadoWorkspace.QDy[85];
acadoVariables.mu[78] += acadoWorkspace.QDy[86];
acadoVariables.mu[79] += acadoWorkspace.QDy[87];
acadoVariables.mu[64] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[84];
acadoVariables.mu[64] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[72];
acadoVariables.mu[65] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[85];
acadoVariables.mu[65] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[73];
acadoVariables.mu[66] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[86];
acadoVariables.mu[66] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[74];
acadoVariables.mu[67] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[87];
acadoVariables.mu[67] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[75];
acadoVariables.mu[68] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[88];
acadoVariables.mu[68] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[76];
acadoVariables.mu[69] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[89];
acadoVariables.mu[69] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[77];
acadoVariables.mu[70] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[90];
acadoVariables.mu[70] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[78];
acadoVariables.mu[71] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[91];
acadoVariables.mu[71] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[79];
acado_expansionStep2( &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoVariables.mu[ 64 ]), &(acadoVariables.mu[ 72 ]) );
acadoVariables.mu[56] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[76];
acadoVariables.mu[56] -= + acadoWorkspace.y[108]*acadoWorkspace.evHx[64];
acadoVariables.mu[57] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[77];
acadoVariables.mu[57] -= + acadoWorkspace.y[108]*acadoWorkspace.evHx[65];
acadoVariables.mu[58] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[78];
acadoVariables.mu[58] -= + acadoWorkspace.y[108]*acadoWorkspace.evHx[66];
acadoVariables.mu[59] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[79];
acadoVariables.mu[59] -= + acadoWorkspace.y[108]*acadoWorkspace.evHx[67];
acadoVariables.mu[60] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[80];
acadoVariables.mu[60] -= + acadoWorkspace.y[108]*acadoWorkspace.evHx[68];
acadoVariables.mu[61] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[81];
acadoVariables.mu[61] -= + acadoWorkspace.y[108]*acadoWorkspace.evHx[69];
acadoVariables.mu[62] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[82];
acadoVariables.mu[62] -= + acadoWorkspace.y[108]*acadoWorkspace.evHx[70];
acadoVariables.mu[63] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[83];
acadoVariables.mu[63] -= + acadoWorkspace.y[108]*acadoWorkspace.evHx[71];
acado_expansionStep2( &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.S1[ 128 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoVariables.mu[ 56 ]), &(acadoVariables.mu[ 64 ]) );
acadoVariables.mu[48] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[68];
acadoVariables.mu[48] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[56];
acadoVariables.mu[49] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[69];
acadoVariables.mu[49] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[57];
acadoVariables.mu[50] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[70];
acadoVariables.mu[50] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[58];
acadoVariables.mu[51] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[71];
acadoVariables.mu[51] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[59];
acadoVariables.mu[52] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[72];
acadoVariables.mu[52] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[60];
acadoVariables.mu[53] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[73];
acadoVariables.mu[53] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[61];
acadoVariables.mu[54] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[74];
acadoVariables.mu[54] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[62];
acadoVariables.mu[55] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[75];
acadoVariables.mu[55] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[63];
acado_expansionStep2( &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.S1[ 112 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoVariables.mu[ 48 ]), &(acadoVariables.mu[ 56 ]) );
acadoVariables.mu[40] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[60];
acadoVariables.mu[40] -= + acadoWorkspace.y[106]*acadoWorkspace.evHx[48];
acadoVariables.mu[41] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[61];
acadoVariables.mu[41] -= + acadoWorkspace.y[106]*acadoWorkspace.evHx[49];
acadoVariables.mu[42] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[62];
acadoVariables.mu[42] -= + acadoWorkspace.y[106]*acadoWorkspace.evHx[50];
acadoVariables.mu[43] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[63];
acadoVariables.mu[43] -= + acadoWorkspace.y[106]*acadoWorkspace.evHx[51];
acadoVariables.mu[44] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[64];
acadoVariables.mu[44] -= + acadoWorkspace.y[106]*acadoWorkspace.evHx[52];
acadoVariables.mu[45] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[65];
acadoVariables.mu[45] -= + acadoWorkspace.y[106]*acadoWorkspace.evHx[53];
acadoVariables.mu[46] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[66];
acadoVariables.mu[46] -= + acadoWorkspace.y[106]*acadoWorkspace.evHx[54];
acadoVariables.mu[47] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[67];
acadoVariables.mu[47] -= + acadoWorkspace.y[106]*acadoWorkspace.evHx[55];
acado_expansionStep2( &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoVariables.mu[ 40 ]), &(acadoVariables.mu[ 48 ]) );
acadoVariables.mu[32] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[52];
acadoVariables.mu[32] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[40];
acadoVariables.mu[33] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[53];
acadoVariables.mu[33] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[41];
acadoVariables.mu[34] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[54];
acadoVariables.mu[34] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[42];
acadoVariables.mu[35] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[55];
acadoVariables.mu[35] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[43];
acadoVariables.mu[36] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[56];
acadoVariables.mu[36] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[44];
acadoVariables.mu[37] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[57];
acadoVariables.mu[37] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[45];
acadoVariables.mu[38] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[58];
acadoVariables.mu[38] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[46];
acadoVariables.mu[39] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[59];
acadoVariables.mu[39] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[47];
acado_expansionStep2( &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.S1[ 80 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoVariables.mu[ 32 ]), &(acadoVariables.mu[ 40 ]) );
acadoVariables.mu[24] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[44];
acadoVariables.mu[24] -= + acadoWorkspace.y[104]*acadoWorkspace.evHx[32];
acadoVariables.mu[25] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[45];
acadoVariables.mu[25] -= + acadoWorkspace.y[104]*acadoWorkspace.evHx[33];
acadoVariables.mu[26] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[46];
acadoVariables.mu[26] -= + acadoWorkspace.y[104]*acadoWorkspace.evHx[34];
acadoVariables.mu[27] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[47];
acadoVariables.mu[27] -= + acadoWorkspace.y[104]*acadoWorkspace.evHx[35];
acadoVariables.mu[28] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[48];
acadoVariables.mu[28] -= + acadoWorkspace.y[104]*acadoWorkspace.evHx[36];
acadoVariables.mu[29] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[49];
acadoVariables.mu[29] -= + acadoWorkspace.y[104]*acadoWorkspace.evHx[37];
acadoVariables.mu[30] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[50];
acadoVariables.mu[30] -= + acadoWorkspace.y[104]*acadoWorkspace.evHx[38];
acadoVariables.mu[31] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[51];
acadoVariables.mu[31] -= + acadoWorkspace.y[104]*acadoWorkspace.evHx[39];
acado_expansionStep2( &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.S1[ 64 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoVariables.mu[ 24 ]), &(acadoVariables.mu[ 32 ]) );
acadoVariables.mu[16] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[36];
acadoVariables.mu[16] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[24];
acadoVariables.mu[17] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[37];
acadoVariables.mu[17] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[25];
acadoVariables.mu[18] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[38];
acadoVariables.mu[18] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[26];
acadoVariables.mu[19] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[39];
acadoVariables.mu[19] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[27];
acadoVariables.mu[20] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[40];
acadoVariables.mu[20] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[28];
acadoVariables.mu[21] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[41];
acadoVariables.mu[21] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[29];
acadoVariables.mu[22] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[42];
acadoVariables.mu[22] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[30];
acadoVariables.mu[23] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[43];
acadoVariables.mu[23] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[31];
acado_expansionStep2( &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoVariables.mu[ 16 ]), &(acadoVariables.mu[ 24 ]) );
acadoVariables.mu[8] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[28];
acadoVariables.mu[8] -= + acadoWorkspace.y[102]*acadoWorkspace.evHx[16];
acadoVariables.mu[9] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[29];
acadoVariables.mu[9] -= + acadoWorkspace.y[102]*acadoWorkspace.evHx[17];
acadoVariables.mu[10] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[30];
acadoVariables.mu[10] -= + acadoWorkspace.y[102]*acadoWorkspace.evHx[18];
acadoVariables.mu[11] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[31];
acadoVariables.mu[11] -= + acadoWorkspace.y[102]*acadoWorkspace.evHx[19];
acadoVariables.mu[12] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[32];
acadoVariables.mu[12] -= + acadoWorkspace.y[102]*acadoWorkspace.evHx[20];
acadoVariables.mu[13] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[33];
acadoVariables.mu[13] -= + acadoWorkspace.y[102]*acadoWorkspace.evHx[21];
acadoVariables.mu[14] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[34];
acadoVariables.mu[14] -= + acadoWorkspace.y[102]*acadoWorkspace.evHx[22];
acadoVariables.mu[15] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[35];
acadoVariables.mu[15] -= + acadoWorkspace.y[102]*acadoWorkspace.evHx[23];
acado_expansionStep2( &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.S1[ 32 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoVariables.mu[ 8 ]), &(acadoVariables.mu[ 16 ]) );
acadoVariables.mu[0] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[20];
acadoVariables.mu[0] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[8];
acadoVariables.mu[1] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[21];
acadoVariables.mu[1] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[9];
acadoVariables.mu[2] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[22];
acadoVariables.mu[2] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[10];
acadoVariables.mu[3] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[23];
acadoVariables.mu[3] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[11];
acadoVariables.mu[4] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[24];
acadoVariables.mu[4] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[12];
acadoVariables.mu[5] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[25];
acadoVariables.mu[5] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[13];
acadoVariables.mu[6] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[26];
acadoVariables.mu[6] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[14];
acadoVariables.mu[7] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.y[27];
acadoVariables.mu[7] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[15];
acado_expansionStep2( &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.S1[ 16 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.evGx[ 64 ]), acadoVariables.mu, &(acadoVariables.mu[ 8 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_regularizeHessian(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -8.0000000000000000e+00;
acadoVariables.lbValues[1] = -3.0000000000000000e+00;
acadoVariables.lbValues[2] = -8.0000000000000000e+00;
acadoVariables.lbValues[3] = -3.0000000000000000e+00;
acadoVariables.lbValues[4] = -8.0000000000000000e+00;
acadoVariables.lbValues[5] = -3.0000000000000000e+00;
acadoVariables.lbValues[6] = -8.0000000000000000e+00;
acadoVariables.lbValues[7] = -3.0000000000000000e+00;
acadoVariables.lbValues[8] = -8.0000000000000000e+00;
acadoVariables.lbValues[9] = -3.0000000000000000e+00;
acadoVariables.lbValues[10] = -8.0000000000000000e+00;
acadoVariables.lbValues[11] = -3.0000000000000000e+00;
acadoVariables.lbValues[12] = -8.0000000000000000e+00;
acadoVariables.lbValues[13] = -3.0000000000000000e+00;
acadoVariables.lbValues[14] = -8.0000000000000000e+00;
acadoVariables.lbValues[15] = -3.0000000000000000e+00;
acadoVariables.lbValues[16] = -8.0000000000000000e+00;
acadoVariables.lbValues[17] = -3.0000000000000000e+00;
acadoVariables.lbValues[18] = -8.0000000000000000e+00;
acadoVariables.lbValues[19] = -3.0000000000000000e+00;
acadoVariables.ubValues[0] = 8.0000000000000000e+00;
acadoVariables.ubValues[1] = 3.0000000000000000e+00;
acadoVariables.ubValues[2] = 8.0000000000000000e+00;
acadoVariables.ubValues[3] = 3.0000000000000000e+00;
acadoVariables.ubValues[4] = 8.0000000000000000e+00;
acadoVariables.ubValues[5] = 3.0000000000000000e+00;
acadoVariables.ubValues[6] = 8.0000000000000000e+00;
acadoVariables.ubValues[7] = 3.0000000000000000e+00;
acadoVariables.ubValues[8] = 8.0000000000000000e+00;
acadoVariables.ubValues[9] = 3.0000000000000000e+00;
acadoVariables.ubValues[10] = 8.0000000000000000e+00;
acadoVariables.ubValues[11] = 3.0000000000000000e+00;
acadoVariables.ubValues[12] = 8.0000000000000000e+00;
acadoVariables.ubValues[13] = 3.0000000000000000e+00;
acadoVariables.ubValues[14] = 8.0000000000000000e+00;
acadoVariables.ubValues[15] = 3.0000000000000000e+00;
acadoVariables.ubValues[16] = 8.0000000000000000e+00;
acadoVariables.ubValues[17] = 3.0000000000000000e+00;
acadoVariables.ubValues[18] = 8.0000000000000000e+00;
acadoVariables.ubValues[19] = 3.0000000000000000e+00;
acadoVariables.lbAValues[0] = -1.2999999999999998e+00;
acadoVariables.lbAValues[1] = -7.5000000000000000e-01;
acadoVariables.lbAValues[2] = 5.0000000000000003e-02;
acadoVariables.lbAValues[3] = -5.0000000000000000e-01;
acadoVariables.lbAValues[4] = -4.0000000000000000e+00;
acadoVariables.lbAValues[5] = -4.0999999999999998e-01;
acadoVariables.lbAValues[6] = 0.0000000000000000e+00;
acadoVariables.lbAValues[7] = 0.0000000000000000e+00;
acadoVariables.lbAValues[8] = -1.2999999999999998e+00;
acadoVariables.lbAValues[9] = -7.5000000000000000e-01;
acadoVariables.lbAValues[10] = 5.0000000000000003e-02;
acadoVariables.lbAValues[11] = -5.0000000000000000e-01;
acadoVariables.lbAValues[12] = -4.0000000000000000e+00;
acadoVariables.lbAValues[13] = -4.0999999999999998e-01;
acadoVariables.lbAValues[14] = 0.0000000000000000e+00;
acadoVariables.lbAValues[15] = 0.0000000000000000e+00;
acadoVariables.lbAValues[16] = -1.2999999999999998e+00;
acadoVariables.lbAValues[17] = -7.5000000000000000e-01;
acadoVariables.lbAValues[18] = 5.0000000000000003e-02;
acadoVariables.lbAValues[19] = -5.0000000000000000e-01;
acadoVariables.lbAValues[20] = -4.0000000000000000e+00;
acadoVariables.lbAValues[21] = -4.0999999999999998e-01;
acadoVariables.lbAValues[22] = 0.0000000000000000e+00;
acadoVariables.lbAValues[23] = 0.0000000000000000e+00;
acadoVariables.lbAValues[24] = -1.2999999999999998e+00;
acadoVariables.lbAValues[25] = -7.5000000000000000e-01;
acadoVariables.lbAValues[26] = 5.0000000000000003e-02;
acadoVariables.lbAValues[27] = -5.0000000000000000e-01;
acadoVariables.lbAValues[28] = -4.0000000000000000e+00;
acadoVariables.lbAValues[29] = -4.0999999999999998e-01;
acadoVariables.lbAValues[30] = 0.0000000000000000e+00;
acadoVariables.lbAValues[31] = 0.0000000000000000e+00;
acadoVariables.lbAValues[32] = -1.2999999999999998e+00;
acadoVariables.lbAValues[33] = -7.5000000000000000e-01;
acadoVariables.lbAValues[34] = 5.0000000000000003e-02;
acadoVariables.lbAValues[35] = -5.0000000000000000e-01;
acadoVariables.lbAValues[36] = -4.0000000000000000e+00;
acadoVariables.lbAValues[37] = -4.0999999999999998e-01;
acadoVariables.lbAValues[38] = 0.0000000000000000e+00;
acadoVariables.lbAValues[39] = 0.0000000000000000e+00;
acadoVariables.lbAValues[40] = -1.2999999999999998e+00;
acadoVariables.lbAValues[41] = -7.5000000000000000e-01;
acadoVariables.lbAValues[42] = 5.0000000000000003e-02;
acadoVariables.lbAValues[43] = -5.0000000000000000e-01;
acadoVariables.lbAValues[44] = -4.0000000000000000e+00;
acadoVariables.lbAValues[45] = -4.0999999999999998e-01;
acadoVariables.lbAValues[46] = 0.0000000000000000e+00;
acadoVariables.lbAValues[47] = 0.0000000000000000e+00;
acadoVariables.lbAValues[48] = -1.2999999999999998e+00;
acadoVariables.lbAValues[49] = -7.5000000000000000e-01;
acadoVariables.lbAValues[50] = 5.0000000000000003e-02;
acadoVariables.lbAValues[51] = -5.0000000000000000e-01;
acadoVariables.lbAValues[52] = -4.0000000000000000e+00;
acadoVariables.lbAValues[53] = -4.0999999999999998e-01;
acadoVariables.lbAValues[54] = 0.0000000000000000e+00;
acadoVariables.lbAValues[55] = 0.0000000000000000e+00;
acadoVariables.lbAValues[56] = -1.2999999999999998e+00;
acadoVariables.lbAValues[57] = -7.5000000000000000e-01;
acadoVariables.lbAValues[58] = 5.0000000000000003e-02;
acadoVariables.lbAValues[59] = -5.0000000000000000e-01;
acadoVariables.lbAValues[60] = -4.0000000000000000e+00;
acadoVariables.lbAValues[61] = -4.0999999999999998e-01;
acadoVariables.lbAValues[62] = 0.0000000000000000e+00;
acadoVariables.lbAValues[63] = 0.0000000000000000e+00;
acadoVariables.lbAValues[64] = -1.2999999999999998e+00;
acadoVariables.lbAValues[65] = -7.5000000000000000e-01;
acadoVariables.lbAValues[66] = 5.0000000000000003e-02;
acadoVariables.lbAValues[67] = -5.0000000000000000e-01;
acadoVariables.lbAValues[68] = -4.0000000000000000e+00;
acadoVariables.lbAValues[69] = -4.0999999999999998e-01;
acadoVariables.lbAValues[70] = 0.0000000000000000e+00;
acadoVariables.lbAValues[71] = 0.0000000000000000e+00;
acadoVariables.lbAValues[72] = -1.2999999999999998e+00;
acadoVariables.lbAValues[73] = -7.5000000000000000e-01;
acadoVariables.lbAValues[74] = 5.0000000000000003e-02;
acadoVariables.lbAValues[75] = -5.0000000000000000e-01;
acadoVariables.lbAValues[76] = -4.0000000000000000e+00;
acadoVariables.lbAValues[77] = -4.0999999999999998e-01;
acadoVariables.lbAValues[78] = 0.0000000000000000e+00;
acadoVariables.lbAValues[79] = 0.0000000000000000e+00;
acadoVariables.lbAValues[80] = -1.0000000000000000e+12;
acadoVariables.lbAValues[81] = -1.0000000000000000e+12;
acadoVariables.lbAValues[82] = -1.0000000000000000e+12;
acadoVariables.lbAValues[83] = -1.0000000000000000e+12;
acadoVariables.lbAValues[84] = -1.0000000000000000e+12;
acadoVariables.lbAValues[85] = -1.0000000000000000e+12;
acadoVariables.lbAValues[86] = -1.0000000000000000e+12;
acadoVariables.lbAValues[87] = -1.0000000000000000e+12;
acadoVariables.lbAValues[88] = -1.0000000000000000e+12;
acadoVariables.lbAValues[89] = -1.0000000000000000e+12;
acadoVariables.ubAValues[0] = 1.2999999999999998e+00;
acadoVariables.ubAValues[1] = 7.5000000000000000e-01;
acadoVariables.ubAValues[2] = 2.0000000000000000e+00;
acadoVariables.ubAValues[3] = 5.0000000000000000e-01;
acadoVariables.ubAValues[4] = 4.0000000000000000e+00;
acadoVariables.ubAValues[5] = 4.0999999999999998e-01;
acadoVariables.ubAValues[6] = 1.0000000000000000e+03;
acadoVariables.ubAValues[7] = 1.0000000000000000e+03;
acadoVariables.ubAValues[8] = 1.2999999999999998e+00;
acadoVariables.ubAValues[9] = 7.5000000000000000e-01;
acadoVariables.ubAValues[10] = 2.0000000000000000e+00;
acadoVariables.ubAValues[11] = 5.0000000000000000e-01;
acadoVariables.ubAValues[12] = 4.0000000000000000e+00;
acadoVariables.ubAValues[13] = 4.0999999999999998e-01;
acadoVariables.ubAValues[14] = 1.0000000000000000e+03;
acadoVariables.ubAValues[15] = 1.0000000000000000e+03;
acadoVariables.ubAValues[16] = 1.2999999999999998e+00;
acadoVariables.ubAValues[17] = 7.5000000000000000e-01;
acadoVariables.ubAValues[18] = 2.0000000000000000e+00;
acadoVariables.ubAValues[19] = 5.0000000000000000e-01;
acadoVariables.ubAValues[20] = 4.0000000000000000e+00;
acadoVariables.ubAValues[21] = 4.0999999999999998e-01;
acadoVariables.ubAValues[22] = 1.0000000000000000e+03;
acadoVariables.ubAValues[23] = 1.0000000000000000e+03;
acadoVariables.ubAValues[24] = 1.2999999999999998e+00;
acadoVariables.ubAValues[25] = 7.5000000000000000e-01;
acadoVariables.ubAValues[26] = 2.0000000000000000e+00;
acadoVariables.ubAValues[27] = 5.0000000000000000e-01;
acadoVariables.ubAValues[28] = 4.0000000000000000e+00;
acadoVariables.ubAValues[29] = 4.0999999999999998e-01;
acadoVariables.ubAValues[30] = 1.0000000000000000e+03;
acadoVariables.ubAValues[31] = 1.0000000000000000e+03;
acadoVariables.ubAValues[32] = 1.2999999999999998e+00;
acadoVariables.ubAValues[33] = 7.5000000000000000e-01;
acadoVariables.ubAValues[34] = 2.0000000000000000e+00;
acadoVariables.ubAValues[35] = 5.0000000000000000e-01;
acadoVariables.ubAValues[36] = 4.0000000000000000e+00;
acadoVariables.ubAValues[37] = 4.0999999999999998e-01;
acadoVariables.ubAValues[38] = 1.0000000000000000e+03;
acadoVariables.ubAValues[39] = 1.0000000000000000e+03;
acadoVariables.ubAValues[40] = 1.2999999999999998e+00;
acadoVariables.ubAValues[41] = 7.5000000000000000e-01;
acadoVariables.ubAValues[42] = 2.0000000000000000e+00;
acadoVariables.ubAValues[43] = 5.0000000000000000e-01;
acadoVariables.ubAValues[44] = 4.0000000000000000e+00;
acadoVariables.ubAValues[45] = 4.0999999999999998e-01;
acadoVariables.ubAValues[46] = 1.0000000000000000e+03;
acadoVariables.ubAValues[47] = 1.0000000000000000e+03;
acadoVariables.ubAValues[48] = 1.2999999999999998e+00;
acadoVariables.ubAValues[49] = 7.5000000000000000e-01;
acadoVariables.ubAValues[50] = 2.0000000000000000e+00;
acadoVariables.ubAValues[51] = 5.0000000000000000e-01;
acadoVariables.ubAValues[52] = 4.0000000000000000e+00;
acadoVariables.ubAValues[53] = 4.0999999999999998e-01;
acadoVariables.ubAValues[54] = 1.0000000000000000e+03;
acadoVariables.ubAValues[55] = 1.0000000000000000e+03;
acadoVariables.ubAValues[56] = 1.2999999999999998e+00;
acadoVariables.ubAValues[57] = 7.5000000000000000e-01;
acadoVariables.ubAValues[58] = 2.0000000000000000e+00;
acadoVariables.ubAValues[59] = 5.0000000000000000e-01;
acadoVariables.ubAValues[60] = 4.0000000000000000e+00;
acadoVariables.ubAValues[61] = 4.0999999999999998e-01;
acadoVariables.ubAValues[62] = 1.0000000000000000e+03;
acadoVariables.ubAValues[63] = 1.0000000000000000e+03;
acadoVariables.ubAValues[64] = 1.2999999999999998e+00;
acadoVariables.ubAValues[65] = 7.5000000000000000e-01;
acadoVariables.ubAValues[66] = 2.0000000000000000e+00;
acadoVariables.ubAValues[67] = 5.0000000000000000e-01;
acadoVariables.ubAValues[68] = 4.0000000000000000e+00;
acadoVariables.ubAValues[69] = 4.0999999999999998e-01;
acadoVariables.ubAValues[70] = 1.0000000000000000e+03;
acadoVariables.ubAValues[71] = 1.0000000000000000e+03;
acadoVariables.ubAValues[72] = 1.2999999999999998e+00;
acadoVariables.ubAValues[73] = 7.5000000000000000e-01;
acadoVariables.ubAValues[74] = 2.0000000000000000e+00;
acadoVariables.ubAValues[75] = 5.0000000000000000e-01;
acadoVariables.ubAValues[76] = 4.0000000000000000e+00;
acadoVariables.ubAValues[77] = 4.0999999999999998e-01;
acadoVariables.ubAValues[78] = 1.0000000000000000e+03;
acadoVariables.ubAValues[79] = 1.0000000000000000e+03;
acadoVariables.ubAValues[80] = 4.0000000000000000e+00;
acadoVariables.ubAValues[81] = 4.0000000000000000e+00;
acadoVariables.ubAValues[82] = 4.0000000000000000e+00;
acadoVariables.ubAValues[83] = 4.0000000000000000e+00;
acadoVariables.ubAValues[84] = 4.0000000000000000e+00;
acadoVariables.ubAValues[85] = 4.0000000000000000e+00;
acadoVariables.ubAValues[86] = 4.0000000000000000e+00;
acadoVariables.ubAValues[87] = 4.0000000000000000e+00;
acadoVariables.ubAValues[88] = 4.0000000000000000e+00;
acadoVariables.ubAValues[89] = 4.0000000000000000e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 8];
acadoWorkspace.state[1] = acadoVariables.x[index * 8 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 8 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 8 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 8 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 8 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 8 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 8 + 7];
acadoWorkspace.state[151] = acadoVariables.u[index * 2];
acadoWorkspace.state[152] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[153] = acadoVariables.od[index];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 8 + 8] = acadoWorkspace.state[0];
acadoVariables.x[index * 8 + 9] = acadoWorkspace.state[1];
acadoVariables.x[index * 8 + 10] = acadoWorkspace.state[2];
acadoVariables.x[index * 8 + 11] = acadoWorkspace.state[3];
acadoVariables.x[index * 8 + 12] = acadoWorkspace.state[4];
acadoVariables.x[index * 8 + 13] = acadoWorkspace.state[5];
acadoVariables.x[index * 8 + 14] = acadoWorkspace.state[6];
acadoVariables.x[index * 8 + 15] = acadoWorkspace.state[7];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 8] = acadoVariables.x[index * 8 + 8];
acadoVariables.x[index * 8 + 1] = acadoVariables.x[index * 8 + 9];
acadoVariables.x[index * 8 + 2] = acadoVariables.x[index * 8 + 10];
acadoVariables.x[index * 8 + 3] = acadoVariables.x[index * 8 + 11];
acadoVariables.x[index * 8 + 4] = acadoVariables.x[index * 8 + 12];
acadoVariables.x[index * 8 + 5] = acadoVariables.x[index * 8 + 13];
acadoVariables.x[index * 8 + 6] = acadoVariables.x[index * 8 + 14];
acadoVariables.x[index * 8 + 7] = acadoVariables.x[index * 8 + 15];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[80] = xEnd[0];
acadoVariables.x[81] = xEnd[1];
acadoVariables.x[82] = xEnd[2];
acadoVariables.x[83] = xEnd[3];
acadoVariables.x[84] = xEnd[4];
acadoVariables.x[85] = xEnd[5];
acadoVariables.x[86] = xEnd[6];
acadoVariables.x[87] = xEnd[7];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[80];
acadoWorkspace.state[1] = acadoVariables.x[81];
acadoWorkspace.state[2] = acadoVariables.x[82];
acadoWorkspace.state[3] = acadoVariables.x[83];
acadoWorkspace.state[4] = acadoVariables.x[84];
acadoWorkspace.state[5] = acadoVariables.x[85];
acadoWorkspace.state[6] = acadoVariables.x[86];
acadoWorkspace.state[7] = acadoVariables.x[87];
if (uEnd != 0)
{
acadoWorkspace.state[151] = uEnd[0];
acadoWorkspace.state[152] = uEnd[1];
}
else
{
acadoWorkspace.state[151] = acadoVariables.u[18];
acadoWorkspace.state[152] = acadoVariables.u[19];
}
acadoWorkspace.state[153] = acadoVariables.od[10];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[80] = acadoWorkspace.state[0];
acadoVariables.x[81] = acadoWorkspace.state[1];
acadoVariables.x[82] = acadoWorkspace.state[2];
acadoVariables.x[83] = acadoWorkspace.state[3];
acadoVariables.x[84] = acadoWorkspace.state[4];
acadoVariables.x[85] = acadoWorkspace.state[5];
acadoVariables.x[86] = acadoWorkspace.state[6];
acadoVariables.x[87] = acadoWorkspace.state[7];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[18] = uEnd[0];
acadoVariables.u[19] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 90; ++index)
{
prd = acadoWorkspace.y[index + 20];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
objVal = 0.0000000000000000e+00;
acadoWorkspace.objValueIn[0] = acadoVariables.x[80];
acadoWorkspace.objValueIn[1] = acadoVariables.x[81];
acadoWorkspace.objValueIn[2] = acadoVariables.x[82];
acadoWorkspace.objValueIn[3] = acadoVariables.x[83];
acadoWorkspace.objValueIn[4] = acadoVariables.x[84];
acadoWorkspace.objValueIn[5] = acadoVariables.x[85];
acadoWorkspace.objValueIn[6] = acadoVariables.x[86];
acadoWorkspace.objValueIn[7] = acadoVariables.x[87];
acadoWorkspace.objValueIn[8] = acadoVariables.od[10];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
return objVal;
}

