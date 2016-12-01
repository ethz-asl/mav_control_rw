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


/** Row vector of size: 129 */
real_t state[ 129 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 9];
state[1] = acadoVariables.x[lRun1 * 9 + 1];
state[2] = acadoVariables.x[lRun1 * 9 + 2];
state[3] = acadoVariables.x[lRun1 * 9 + 3];
state[4] = acadoVariables.x[lRun1 * 9 + 4];
state[5] = acadoVariables.x[lRun1 * 9 + 5];
state[6] = acadoVariables.x[lRun1 * 9 + 6];
state[7] = acadoVariables.x[lRun1 * 9 + 7];
state[8] = acadoVariables.x[lRun1 * 9 + 8];

state[117] = acadoVariables.u[lRun1 * 3];
state[118] = acadoVariables.u[lRun1 * 3 + 1];
state[119] = acadoVariables.u[lRun1 * 3 + 2];
state[120] = acadoVariables.od[lRun1 * 9];
state[121] = acadoVariables.od[lRun1 * 9 + 1];
state[122] = acadoVariables.od[lRun1 * 9 + 2];
state[123] = acadoVariables.od[lRun1 * 9 + 3];
state[124] = acadoVariables.od[lRun1 * 9 + 4];
state[125] = acadoVariables.od[lRun1 * 9 + 5];
state[126] = acadoVariables.od[lRun1 * 9 + 6];
state[127] = acadoVariables.od[lRun1 * 9 + 7];
state[128] = acadoVariables.od[lRun1 * 9 + 8];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 9] = state[0] - acadoVariables.x[lRun1 * 9 + 9];
acadoWorkspace.d[lRun1 * 9 + 1] = state[1] - acadoVariables.x[lRun1 * 9 + 10];
acadoWorkspace.d[lRun1 * 9 + 2] = state[2] - acadoVariables.x[lRun1 * 9 + 11];
acadoWorkspace.d[lRun1 * 9 + 3] = state[3] - acadoVariables.x[lRun1 * 9 + 12];
acadoWorkspace.d[lRun1 * 9 + 4] = state[4] - acadoVariables.x[lRun1 * 9 + 13];
acadoWorkspace.d[lRun1 * 9 + 5] = state[5] - acadoVariables.x[lRun1 * 9 + 14];
acadoWorkspace.d[lRun1 * 9 + 6] = state[6] - acadoVariables.x[lRun1 * 9 + 15];
acadoWorkspace.d[lRun1 * 9 + 7] = state[7] - acadoVariables.x[lRun1 * 9 + 16];
acadoWorkspace.d[lRun1 * 9 + 8] = state[8] - acadoVariables.x[lRun1 * 9 + 17];

acadoWorkspace.evGx[lRun1 * 81] = state[9];
acadoWorkspace.evGx[lRun1 * 81 + 1] = state[10];
acadoWorkspace.evGx[lRun1 * 81 + 2] = state[11];
acadoWorkspace.evGx[lRun1 * 81 + 3] = state[12];
acadoWorkspace.evGx[lRun1 * 81 + 4] = state[13];
acadoWorkspace.evGx[lRun1 * 81 + 5] = state[14];
acadoWorkspace.evGx[lRun1 * 81 + 6] = state[15];
acadoWorkspace.evGx[lRun1 * 81 + 7] = state[16];
acadoWorkspace.evGx[lRun1 * 81 + 8] = state[17];
acadoWorkspace.evGx[lRun1 * 81 + 9] = state[18];
acadoWorkspace.evGx[lRun1 * 81 + 10] = state[19];
acadoWorkspace.evGx[lRun1 * 81 + 11] = state[20];
acadoWorkspace.evGx[lRun1 * 81 + 12] = state[21];
acadoWorkspace.evGx[lRun1 * 81 + 13] = state[22];
acadoWorkspace.evGx[lRun1 * 81 + 14] = state[23];
acadoWorkspace.evGx[lRun1 * 81 + 15] = state[24];
acadoWorkspace.evGx[lRun1 * 81 + 16] = state[25];
acadoWorkspace.evGx[lRun1 * 81 + 17] = state[26];
acadoWorkspace.evGx[lRun1 * 81 + 18] = state[27];
acadoWorkspace.evGx[lRun1 * 81 + 19] = state[28];
acadoWorkspace.evGx[lRun1 * 81 + 20] = state[29];
acadoWorkspace.evGx[lRun1 * 81 + 21] = state[30];
acadoWorkspace.evGx[lRun1 * 81 + 22] = state[31];
acadoWorkspace.evGx[lRun1 * 81 + 23] = state[32];
acadoWorkspace.evGx[lRun1 * 81 + 24] = state[33];
acadoWorkspace.evGx[lRun1 * 81 + 25] = state[34];
acadoWorkspace.evGx[lRun1 * 81 + 26] = state[35];
acadoWorkspace.evGx[lRun1 * 81 + 27] = state[36];
acadoWorkspace.evGx[lRun1 * 81 + 28] = state[37];
acadoWorkspace.evGx[lRun1 * 81 + 29] = state[38];
acadoWorkspace.evGx[lRun1 * 81 + 30] = state[39];
acadoWorkspace.evGx[lRun1 * 81 + 31] = state[40];
acadoWorkspace.evGx[lRun1 * 81 + 32] = state[41];
acadoWorkspace.evGx[lRun1 * 81 + 33] = state[42];
acadoWorkspace.evGx[lRun1 * 81 + 34] = state[43];
acadoWorkspace.evGx[lRun1 * 81 + 35] = state[44];
acadoWorkspace.evGx[lRun1 * 81 + 36] = state[45];
acadoWorkspace.evGx[lRun1 * 81 + 37] = state[46];
acadoWorkspace.evGx[lRun1 * 81 + 38] = state[47];
acadoWorkspace.evGx[lRun1 * 81 + 39] = state[48];
acadoWorkspace.evGx[lRun1 * 81 + 40] = state[49];
acadoWorkspace.evGx[lRun1 * 81 + 41] = state[50];
acadoWorkspace.evGx[lRun1 * 81 + 42] = state[51];
acadoWorkspace.evGx[lRun1 * 81 + 43] = state[52];
acadoWorkspace.evGx[lRun1 * 81 + 44] = state[53];
acadoWorkspace.evGx[lRun1 * 81 + 45] = state[54];
acadoWorkspace.evGx[lRun1 * 81 + 46] = state[55];
acadoWorkspace.evGx[lRun1 * 81 + 47] = state[56];
acadoWorkspace.evGx[lRun1 * 81 + 48] = state[57];
acadoWorkspace.evGx[lRun1 * 81 + 49] = state[58];
acadoWorkspace.evGx[lRun1 * 81 + 50] = state[59];
acadoWorkspace.evGx[lRun1 * 81 + 51] = state[60];
acadoWorkspace.evGx[lRun1 * 81 + 52] = state[61];
acadoWorkspace.evGx[lRun1 * 81 + 53] = state[62];
acadoWorkspace.evGx[lRun1 * 81 + 54] = state[63];
acadoWorkspace.evGx[lRun1 * 81 + 55] = state[64];
acadoWorkspace.evGx[lRun1 * 81 + 56] = state[65];
acadoWorkspace.evGx[lRun1 * 81 + 57] = state[66];
acadoWorkspace.evGx[lRun1 * 81 + 58] = state[67];
acadoWorkspace.evGx[lRun1 * 81 + 59] = state[68];
acadoWorkspace.evGx[lRun1 * 81 + 60] = state[69];
acadoWorkspace.evGx[lRun1 * 81 + 61] = state[70];
acadoWorkspace.evGx[lRun1 * 81 + 62] = state[71];
acadoWorkspace.evGx[lRun1 * 81 + 63] = state[72];
acadoWorkspace.evGx[lRun1 * 81 + 64] = state[73];
acadoWorkspace.evGx[lRun1 * 81 + 65] = state[74];
acadoWorkspace.evGx[lRun1 * 81 + 66] = state[75];
acadoWorkspace.evGx[lRun1 * 81 + 67] = state[76];
acadoWorkspace.evGx[lRun1 * 81 + 68] = state[77];
acadoWorkspace.evGx[lRun1 * 81 + 69] = state[78];
acadoWorkspace.evGx[lRun1 * 81 + 70] = state[79];
acadoWorkspace.evGx[lRun1 * 81 + 71] = state[80];
acadoWorkspace.evGx[lRun1 * 81 + 72] = state[81];
acadoWorkspace.evGx[lRun1 * 81 + 73] = state[82];
acadoWorkspace.evGx[lRun1 * 81 + 74] = state[83];
acadoWorkspace.evGx[lRun1 * 81 + 75] = state[84];
acadoWorkspace.evGx[lRun1 * 81 + 76] = state[85];
acadoWorkspace.evGx[lRun1 * 81 + 77] = state[86];
acadoWorkspace.evGx[lRun1 * 81 + 78] = state[87];
acadoWorkspace.evGx[lRun1 * 81 + 79] = state[88];
acadoWorkspace.evGx[lRun1 * 81 + 80] = state[89];

acadoWorkspace.evGu[lRun1 * 27] = state[90];
acadoWorkspace.evGu[lRun1 * 27 + 1] = state[91];
acadoWorkspace.evGu[lRun1 * 27 + 2] = state[92];
acadoWorkspace.evGu[lRun1 * 27 + 3] = state[93];
acadoWorkspace.evGu[lRun1 * 27 + 4] = state[94];
acadoWorkspace.evGu[lRun1 * 27 + 5] = state[95];
acadoWorkspace.evGu[lRun1 * 27 + 6] = state[96];
acadoWorkspace.evGu[lRun1 * 27 + 7] = state[97];
acadoWorkspace.evGu[lRun1 * 27 + 8] = state[98];
acadoWorkspace.evGu[lRun1 * 27 + 9] = state[99];
acadoWorkspace.evGu[lRun1 * 27 + 10] = state[100];
acadoWorkspace.evGu[lRun1 * 27 + 11] = state[101];
acadoWorkspace.evGu[lRun1 * 27 + 12] = state[102];
acadoWorkspace.evGu[lRun1 * 27 + 13] = state[103];
acadoWorkspace.evGu[lRun1 * 27 + 14] = state[104];
acadoWorkspace.evGu[lRun1 * 27 + 15] = state[105];
acadoWorkspace.evGu[lRun1 * 27 + 16] = state[106];
acadoWorkspace.evGu[lRun1 * 27 + 17] = state[107];
acadoWorkspace.evGu[lRun1 * 27 + 18] = state[108];
acadoWorkspace.evGu[lRun1 * 27 + 19] = state[109];
acadoWorkspace.evGu[lRun1 * 27 + 20] = state[110];
acadoWorkspace.evGu[lRun1 * 27 + 21] = state[111];
acadoWorkspace.evGu[lRun1 * 27 + 22] = state[112];
acadoWorkspace.evGu[lRun1 * 27 + 23] = state[113];
acadoWorkspace.evGu[lRun1 * 27 + 24] = state[114];
acadoWorkspace.evGu[lRun1 * 27 + 25] = state[115];
acadoWorkspace.evGu[lRun1 * 27 + 26] = state[116];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;
/* Vector of auxiliary variables; number of elements: 4. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[4]));
a[1] = (cos(xd[3]));
a[2] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[3] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));

/* Compute outputs: */
out[0] = xd[6];
out[1] = xd[7];
out[2] = xd[8];
out[3] = xd[0];
out[4] = xd[1];
out[5] = xd[2];
out[6] = xd[3];
out[7] = xd[4];
out[8] = u[0];
out[9] = u[1];
out[10] = ((real_t)(-9.8065999999999995e+00)+((a[0]*a[1])*u[2]));
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(1.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(1.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(1.0000000000000000e+00);
out[38] = (real_t)(1.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(1.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(1.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(1.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(1.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = ((a[0]*a[2])*u[2]);
out[105] = ((a[3]*a[1])*u[2]);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(1.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(1.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (a[0]*a[1]);
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[6];
out[1] = xd[7];
out[2] = xd[8];
out[3] = xd[0];
out[4] = xd[1];
out[5] = xd[2];
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[9]*tmpObjS[11] + tmpFx[18]*tmpObjS[22] + tmpFx[27]*tmpObjS[33] + tmpFx[36]*tmpObjS[44] + tmpFx[45]*tmpObjS[55] + tmpFx[54]*tmpObjS[66] + tmpFx[63]*tmpObjS[77] + tmpFx[72]*tmpObjS[88] + tmpFx[81]*tmpObjS[99] + tmpFx[90]*tmpObjS[110];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[9]*tmpObjS[12] + tmpFx[18]*tmpObjS[23] + tmpFx[27]*tmpObjS[34] + tmpFx[36]*tmpObjS[45] + tmpFx[45]*tmpObjS[56] + tmpFx[54]*tmpObjS[67] + tmpFx[63]*tmpObjS[78] + tmpFx[72]*tmpObjS[89] + tmpFx[81]*tmpObjS[100] + tmpFx[90]*tmpObjS[111];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[9]*tmpObjS[13] + tmpFx[18]*tmpObjS[24] + tmpFx[27]*tmpObjS[35] + tmpFx[36]*tmpObjS[46] + tmpFx[45]*tmpObjS[57] + tmpFx[54]*tmpObjS[68] + tmpFx[63]*tmpObjS[79] + tmpFx[72]*tmpObjS[90] + tmpFx[81]*tmpObjS[101] + tmpFx[90]*tmpObjS[112];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[9]*tmpObjS[14] + tmpFx[18]*tmpObjS[25] + tmpFx[27]*tmpObjS[36] + tmpFx[36]*tmpObjS[47] + tmpFx[45]*tmpObjS[58] + tmpFx[54]*tmpObjS[69] + tmpFx[63]*tmpObjS[80] + tmpFx[72]*tmpObjS[91] + tmpFx[81]*tmpObjS[102] + tmpFx[90]*tmpObjS[113];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[9]*tmpObjS[15] + tmpFx[18]*tmpObjS[26] + tmpFx[27]*tmpObjS[37] + tmpFx[36]*tmpObjS[48] + tmpFx[45]*tmpObjS[59] + tmpFx[54]*tmpObjS[70] + tmpFx[63]*tmpObjS[81] + tmpFx[72]*tmpObjS[92] + tmpFx[81]*tmpObjS[103] + tmpFx[90]*tmpObjS[114];
tmpQ2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[9]*tmpObjS[16] + tmpFx[18]*tmpObjS[27] + tmpFx[27]*tmpObjS[38] + tmpFx[36]*tmpObjS[49] + tmpFx[45]*tmpObjS[60] + tmpFx[54]*tmpObjS[71] + tmpFx[63]*tmpObjS[82] + tmpFx[72]*tmpObjS[93] + tmpFx[81]*tmpObjS[104] + tmpFx[90]*tmpObjS[115];
tmpQ2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[9]*tmpObjS[17] + tmpFx[18]*tmpObjS[28] + tmpFx[27]*tmpObjS[39] + tmpFx[36]*tmpObjS[50] + tmpFx[45]*tmpObjS[61] + tmpFx[54]*tmpObjS[72] + tmpFx[63]*tmpObjS[83] + tmpFx[72]*tmpObjS[94] + tmpFx[81]*tmpObjS[105] + tmpFx[90]*tmpObjS[116];
tmpQ2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[9]*tmpObjS[18] + tmpFx[18]*tmpObjS[29] + tmpFx[27]*tmpObjS[40] + tmpFx[36]*tmpObjS[51] + tmpFx[45]*tmpObjS[62] + tmpFx[54]*tmpObjS[73] + tmpFx[63]*tmpObjS[84] + tmpFx[72]*tmpObjS[95] + tmpFx[81]*tmpObjS[106] + tmpFx[90]*tmpObjS[117];
tmpQ2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[9]*tmpObjS[19] + tmpFx[18]*tmpObjS[30] + tmpFx[27]*tmpObjS[41] + tmpFx[36]*tmpObjS[52] + tmpFx[45]*tmpObjS[63] + tmpFx[54]*tmpObjS[74] + tmpFx[63]*tmpObjS[85] + tmpFx[72]*tmpObjS[96] + tmpFx[81]*tmpObjS[107] + tmpFx[90]*tmpObjS[118];
tmpQ2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[9]*tmpObjS[20] + tmpFx[18]*tmpObjS[31] + tmpFx[27]*tmpObjS[42] + tmpFx[36]*tmpObjS[53] + tmpFx[45]*tmpObjS[64] + tmpFx[54]*tmpObjS[75] + tmpFx[63]*tmpObjS[86] + tmpFx[72]*tmpObjS[97] + tmpFx[81]*tmpObjS[108] + tmpFx[90]*tmpObjS[119];
tmpQ2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[9]*tmpObjS[21] + tmpFx[18]*tmpObjS[32] + tmpFx[27]*tmpObjS[43] + tmpFx[36]*tmpObjS[54] + tmpFx[45]*tmpObjS[65] + tmpFx[54]*tmpObjS[76] + tmpFx[63]*tmpObjS[87] + tmpFx[72]*tmpObjS[98] + tmpFx[81]*tmpObjS[109] + tmpFx[90]*tmpObjS[120];
tmpQ2[11] = + tmpFx[1]*tmpObjS[0] + tmpFx[10]*tmpObjS[11] + tmpFx[19]*tmpObjS[22] + tmpFx[28]*tmpObjS[33] + tmpFx[37]*tmpObjS[44] + tmpFx[46]*tmpObjS[55] + tmpFx[55]*tmpObjS[66] + tmpFx[64]*tmpObjS[77] + tmpFx[73]*tmpObjS[88] + tmpFx[82]*tmpObjS[99] + tmpFx[91]*tmpObjS[110];
tmpQ2[12] = + tmpFx[1]*tmpObjS[1] + tmpFx[10]*tmpObjS[12] + tmpFx[19]*tmpObjS[23] + tmpFx[28]*tmpObjS[34] + tmpFx[37]*tmpObjS[45] + tmpFx[46]*tmpObjS[56] + tmpFx[55]*tmpObjS[67] + tmpFx[64]*tmpObjS[78] + tmpFx[73]*tmpObjS[89] + tmpFx[82]*tmpObjS[100] + tmpFx[91]*tmpObjS[111];
tmpQ2[13] = + tmpFx[1]*tmpObjS[2] + tmpFx[10]*tmpObjS[13] + tmpFx[19]*tmpObjS[24] + tmpFx[28]*tmpObjS[35] + tmpFx[37]*tmpObjS[46] + tmpFx[46]*tmpObjS[57] + tmpFx[55]*tmpObjS[68] + tmpFx[64]*tmpObjS[79] + tmpFx[73]*tmpObjS[90] + tmpFx[82]*tmpObjS[101] + tmpFx[91]*tmpObjS[112];
tmpQ2[14] = + tmpFx[1]*tmpObjS[3] + tmpFx[10]*tmpObjS[14] + tmpFx[19]*tmpObjS[25] + tmpFx[28]*tmpObjS[36] + tmpFx[37]*tmpObjS[47] + tmpFx[46]*tmpObjS[58] + tmpFx[55]*tmpObjS[69] + tmpFx[64]*tmpObjS[80] + tmpFx[73]*tmpObjS[91] + tmpFx[82]*tmpObjS[102] + tmpFx[91]*tmpObjS[113];
tmpQ2[15] = + tmpFx[1]*tmpObjS[4] + tmpFx[10]*tmpObjS[15] + tmpFx[19]*tmpObjS[26] + tmpFx[28]*tmpObjS[37] + tmpFx[37]*tmpObjS[48] + tmpFx[46]*tmpObjS[59] + tmpFx[55]*tmpObjS[70] + tmpFx[64]*tmpObjS[81] + tmpFx[73]*tmpObjS[92] + tmpFx[82]*tmpObjS[103] + tmpFx[91]*tmpObjS[114];
tmpQ2[16] = + tmpFx[1]*tmpObjS[5] + tmpFx[10]*tmpObjS[16] + tmpFx[19]*tmpObjS[27] + tmpFx[28]*tmpObjS[38] + tmpFx[37]*tmpObjS[49] + tmpFx[46]*tmpObjS[60] + tmpFx[55]*tmpObjS[71] + tmpFx[64]*tmpObjS[82] + tmpFx[73]*tmpObjS[93] + tmpFx[82]*tmpObjS[104] + tmpFx[91]*tmpObjS[115];
tmpQ2[17] = + tmpFx[1]*tmpObjS[6] + tmpFx[10]*tmpObjS[17] + tmpFx[19]*tmpObjS[28] + tmpFx[28]*tmpObjS[39] + tmpFx[37]*tmpObjS[50] + tmpFx[46]*tmpObjS[61] + tmpFx[55]*tmpObjS[72] + tmpFx[64]*tmpObjS[83] + tmpFx[73]*tmpObjS[94] + tmpFx[82]*tmpObjS[105] + tmpFx[91]*tmpObjS[116];
tmpQ2[18] = + tmpFx[1]*tmpObjS[7] + tmpFx[10]*tmpObjS[18] + tmpFx[19]*tmpObjS[29] + tmpFx[28]*tmpObjS[40] + tmpFx[37]*tmpObjS[51] + tmpFx[46]*tmpObjS[62] + tmpFx[55]*tmpObjS[73] + tmpFx[64]*tmpObjS[84] + tmpFx[73]*tmpObjS[95] + tmpFx[82]*tmpObjS[106] + tmpFx[91]*tmpObjS[117];
tmpQ2[19] = + tmpFx[1]*tmpObjS[8] + tmpFx[10]*tmpObjS[19] + tmpFx[19]*tmpObjS[30] + tmpFx[28]*tmpObjS[41] + tmpFx[37]*tmpObjS[52] + tmpFx[46]*tmpObjS[63] + tmpFx[55]*tmpObjS[74] + tmpFx[64]*tmpObjS[85] + tmpFx[73]*tmpObjS[96] + tmpFx[82]*tmpObjS[107] + tmpFx[91]*tmpObjS[118];
tmpQ2[20] = + tmpFx[1]*tmpObjS[9] + tmpFx[10]*tmpObjS[20] + tmpFx[19]*tmpObjS[31] + tmpFx[28]*tmpObjS[42] + tmpFx[37]*tmpObjS[53] + tmpFx[46]*tmpObjS[64] + tmpFx[55]*tmpObjS[75] + tmpFx[64]*tmpObjS[86] + tmpFx[73]*tmpObjS[97] + tmpFx[82]*tmpObjS[108] + tmpFx[91]*tmpObjS[119];
tmpQ2[21] = + tmpFx[1]*tmpObjS[10] + tmpFx[10]*tmpObjS[21] + tmpFx[19]*tmpObjS[32] + tmpFx[28]*tmpObjS[43] + tmpFx[37]*tmpObjS[54] + tmpFx[46]*tmpObjS[65] + tmpFx[55]*tmpObjS[76] + tmpFx[64]*tmpObjS[87] + tmpFx[73]*tmpObjS[98] + tmpFx[82]*tmpObjS[109] + tmpFx[91]*tmpObjS[120];
tmpQ2[22] = + tmpFx[2]*tmpObjS[0] + tmpFx[11]*tmpObjS[11] + tmpFx[20]*tmpObjS[22] + tmpFx[29]*tmpObjS[33] + tmpFx[38]*tmpObjS[44] + tmpFx[47]*tmpObjS[55] + tmpFx[56]*tmpObjS[66] + tmpFx[65]*tmpObjS[77] + tmpFx[74]*tmpObjS[88] + tmpFx[83]*tmpObjS[99] + tmpFx[92]*tmpObjS[110];
tmpQ2[23] = + tmpFx[2]*tmpObjS[1] + tmpFx[11]*tmpObjS[12] + tmpFx[20]*tmpObjS[23] + tmpFx[29]*tmpObjS[34] + tmpFx[38]*tmpObjS[45] + tmpFx[47]*tmpObjS[56] + tmpFx[56]*tmpObjS[67] + tmpFx[65]*tmpObjS[78] + tmpFx[74]*tmpObjS[89] + tmpFx[83]*tmpObjS[100] + tmpFx[92]*tmpObjS[111];
tmpQ2[24] = + tmpFx[2]*tmpObjS[2] + tmpFx[11]*tmpObjS[13] + tmpFx[20]*tmpObjS[24] + tmpFx[29]*tmpObjS[35] + tmpFx[38]*tmpObjS[46] + tmpFx[47]*tmpObjS[57] + tmpFx[56]*tmpObjS[68] + tmpFx[65]*tmpObjS[79] + tmpFx[74]*tmpObjS[90] + tmpFx[83]*tmpObjS[101] + tmpFx[92]*tmpObjS[112];
tmpQ2[25] = + tmpFx[2]*tmpObjS[3] + tmpFx[11]*tmpObjS[14] + tmpFx[20]*tmpObjS[25] + tmpFx[29]*tmpObjS[36] + tmpFx[38]*tmpObjS[47] + tmpFx[47]*tmpObjS[58] + tmpFx[56]*tmpObjS[69] + tmpFx[65]*tmpObjS[80] + tmpFx[74]*tmpObjS[91] + tmpFx[83]*tmpObjS[102] + tmpFx[92]*tmpObjS[113];
tmpQ2[26] = + tmpFx[2]*tmpObjS[4] + tmpFx[11]*tmpObjS[15] + tmpFx[20]*tmpObjS[26] + tmpFx[29]*tmpObjS[37] + tmpFx[38]*tmpObjS[48] + tmpFx[47]*tmpObjS[59] + tmpFx[56]*tmpObjS[70] + tmpFx[65]*tmpObjS[81] + tmpFx[74]*tmpObjS[92] + tmpFx[83]*tmpObjS[103] + tmpFx[92]*tmpObjS[114];
tmpQ2[27] = + tmpFx[2]*tmpObjS[5] + tmpFx[11]*tmpObjS[16] + tmpFx[20]*tmpObjS[27] + tmpFx[29]*tmpObjS[38] + tmpFx[38]*tmpObjS[49] + tmpFx[47]*tmpObjS[60] + tmpFx[56]*tmpObjS[71] + tmpFx[65]*tmpObjS[82] + tmpFx[74]*tmpObjS[93] + tmpFx[83]*tmpObjS[104] + tmpFx[92]*tmpObjS[115];
tmpQ2[28] = + tmpFx[2]*tmpObjS[6] + tmpFx[11]*tmpObjS[17] + tmpFx[20]*tmpObjS[28] + tmpFx[29]*tmpObjS[39] + tmpFx[38]*tmpObjS[50] + tmpFx[47]*tmpObjS[61] + tmpFx[56]*tmpObjS[72] + tmpFx[65]*tmpObjS[83] + tmpFx[74]*tmpObjS[94] + tmpFx[83]*tmpObjS[105] + tmpFx[92]*tmpObjS[116];
tmpQ2[29] = + tmpFx[2]*tmpObjS[7] + tmpFx[11]*tmpObjS[18] + tmpFx[20]*tmpObjS[29] + tmpFx[29]*tmpObjS[40] + tmpFx[38]*tmpObjS[51] + tmpFx[47]*tmpObjS[62] + tmpFx[56]*tmpObjS[73] + tmpFx[65]*tmpObjS[84] + tmpFx[74]*tmpObjS[95] + tmpFx[83]*tmpObjS[106] + tmpFx[92]*tmpObjS[117];
tmpQ2[30] = + tmpFx[2]*tmpObjS[8] + tmpFx[11]*tmpObjS[19] + tmpFx[20]*tmpObjS[30] + tmpFx[29]*tmpObjS[41] + tmpFx[38]*tmpObjS[52] + tmpFx[47]*tmpObjS[63] + tmpFx[56]*tmpObjS[74] + tmpFx[65]*tmpObjS[85] + tmpFx[74]*tmpObjS[96] + tmpFx[83]*tmpObjS[107] + tmpFx[92]*tmpObjS[118];
tmpQ2[31] = + tmpFx[2]*tmpObjS[9] + tmpFx[11]*tmpObjS[20] + tmpFx[20]*tmpObjS[31] + tmpFx[29]*tmpObjS[42] + tmpFx[38]*tmpObjS[53] + tmpFx[47]*tmpObjS[64] + tmpFx[56]*tmpObjS[75] + tmpFx[65]*tmpObjS[86] + tmpFx[74]*tmpObjS[97] + tmpFx[83]*tmpObjS[108] + tmpFx[92]*tmpObjS[119];
tmpQ2[32] = + tmpFx[2]*tmpObjS[10] + tmpFx[11]*tmpObjS[21] + tmpFx[20]*tmpObjS[32] + tmpFx[29]*tmpObjS[43] + tmpFx[38]*tmpObjS[54] + tmpFx[47]*tmpObjS[65] + tmpFx[56]*tmpObjS[76] + tmpFx[65]*tmpObjS[87] + tmpFx[74]*tmpObjS[98] + tmpFx[83]*tmpObjS[109] + tmpFx[92]*tmpObjS[120];
tmpQ2[33] = + tmpFx[3]*tmpObjS[0] + tmpFx[12]*tmpObjS[11] + tmpFx[21]*tmpObjS[22] + tmpFx[30]*tmpObjS[33] + tmpFx[39]*tmpObjS[44] + tmpFx[48]*tmpObjS[55] + tmpFx[57]*tmpObjS[66] + tmpFx[66]*tmpObjS[77] + tmpFx[75]*tmpObjS[88] + tmpFx[84]*tmpObjS[99] + tmpFx[93]*tmpObjS[110];
tmpQ2[34] = + tmpFx[3]*tmpObjS[1] + tmpFx[12]*tmpObjS[12] + tmpFx[21]*tmpObjS[23] + tmpFx[30]*tmpObjS[34] + tmpFx[39]*tmpObjS[45] + tmpFx[48]*tmpObjS[56] + tmpFx[57]*tmpObjS[67] + tmpFx[66]*tmpObjS[78] + tmpFx[75]*tmpObjS[89] + tmpFx[84]*tmpObjS[100] + tmpFx[93]*tmpObjS[111];
tmpQ2[35] = + tmpFx[3]*tmpObjS[2] + tmpFx[12]*tmpObjS[13] + tmpFx[21]*tmpObjS[24] + tmpFx[30]*tmpObjS[35] + tmpFx[39]*tmpObjS[46] + tmpFx[48]*tmpObjS[57] + tmpFx[57]*tmpObjS[68] + tmpFx[66]*tmpObjS[79] + tmpFx[75]*tmpObjS[90] + tmpFx[84]*tmpObjS[101] + tmpFx[93]*tmpObjS[112];
tmpQ2[36] = + tmpFx[3]*tmpObjS[3] + tmpFx[12]*tmpObjS[14] + tmpFx[21]*tmpObjS[25] + tmpFx[30]*tmpObjS[36] + tmpFx[39]*tmpObjS[47] + tmpFx[48]*tmpObjS[58] + tmpFx[57]*tmpObjS[69] + tmpFx[66]*tmpObjS[80] + tmpFx[75]*tmpObjS[91] + tmpFx[84]*tmpObjS[102] + tmpFx[93]*tmpObjS[113];
tmpQ2[37] = + tmpFx[3]*tmpObjS[4] + tmpFx[12]*tmpObjS[15] + tmpFx[21]*tmpObjS[26] + tmpFx[30]*tmpObjS[37] + tmpFx[39]*tmpObjS[48] + tmpFx[48]*tmpObjS[59] + tmpFx[57]*tmpObjS[70] + tmpFx[66]*tmpObjS[81] + tmpFx[75]*tmpObjS[92] + tmpFx[84]*tmpObjS[103] + tmpFx[93]*tmpObjS[114];
tmpQ2[38] = + tmpFx[3]*tmpObjS[5] + tmpFx[12]*tmpObjS[16] + tmpFx[21]*tmpObjS[27] + tmpFx[30]*tmpObjS[38] + tmpFx[39]*tmpObjS[49] + tmpFx[48]*tmpObjS[60] + tmpFx[57]*tmpObjS[71] + tmpFx[66]*tmpObjS[82] + tmpFx[75]*tmpObjS[93] + tmpFx[84]*tmpObjS[104] + tmpFx[93]*tmpObjS[115];
tmpQ2[39] = + tmpFx[3]*tmpObjS[6] + tmpFx[12]*tmpObjS[17] + tmpFx[21]*tmpObjS[28] + tmpFx[30]*tmpObjS[39] + tmpFx[39]*tmpObjS[50] + tmpFx[48]*tmpObjS[61] + tmpFx[57]*tmpObjS[72] + tmpFx[66]*tmpObjS[83] + tmpFx[75]*tmpObjS[94] + tmpFx[84]*tmpObjS[105] + tmpFx[93]*tmpObjS[116];
tmpQ2[40] = + tmpFx[3]*tmpObjS[7] + tmpFx[12]*tmpObjS[18] + tmpFx[21]*tmpObjS[29] + tmpFx[30]*tmpObjS[40] + tmpFx[39]*tmpObjS[51] + tmpFx[48]*tmpObjS[62] + tmpFx[57]*tmpObjS[73] + tmpFx[66]*tmpObjS[84] + tmpFx[75]*tmpObjS[95] + tmpFx[84]*tmpObjS[106] + tmpFx[93]*tmpObjS[117];
tmpQ2[41] = + tmpFx[3]*tmpObjS[8] + tmpFx[12]*tmpObjS[19] + tmpFx[21]*tmpObjS[30] + tmpFx[30]*tmpObjS[41] + tmpFx[39]*tmpObjS[52] + tmpFx[48]*tmpObjS[63] + tmpFx[57]*tmpObjS[74] + tmpFx[66]*tmpObjS[85] + tmpFx[75]*tmpObjS[96] + tmpFx[84]*tmpObjS[107] + tmpFx[93]*tmpObjS[118];
tmpQ2[42] = + tmpFx[3]*tmpObjS[9] + tmpFx[12]*tmpObjS[20] + tmpFx[21]*tmpObjS[31] + tmpFx[30]*tmpObjS[42] + tmpFx[39]*tmpObjS[53] + tmpFx[48]*tmpObjS[64] + tmpFx[57]*tmpObjS[75] + tmpFx[66]*tmpObjS[86] + tmpFx[75]*tmpObjS[97] + tmpFx[84]*tmpObjS[108] + tmpFx[93]*tmpObjS[119];
tmpQ2[43] = + tmpFx[3]*tmpObjS[10] + tmpFx[12]*tmpObjS[21] + tmpFx[21]*tmpObjS[32] + tmpFx[30]*tmpObjS[43] + tmpFx[39]*tmpObjS[54] + tmpFx[48]*tmpObjS[65] + tmpFx[57]*tmpObjS[76] + tmpFx[66]*tmpObjS[87] + tmpFx[75]*tmpObjS[98] + tmpFx[84]*tmpObjS[109] + tmpFx[93]*tmpObjS[120];
tmpQ2[44] = + tmpFx[4]*tmpObjS[0] + tmpFx[13]*tmpObjS[11] + tmpFx[22]*tmpObjS[22] + tmpFx[31]*tmpObjS[33] + tmpFx[40]*tmpObjS[44] + tmpFx[49]*tmpObjS[55] + tmpFx[58]*tmpObjS[66] + tmpFx[67]*tmpObjS[77] + tmpFx[76]*tmpObjS[88] + tmpFx[85]*tmpObjS[99] + tmpFx[94]*tmpObjS[110];
tmpQ2[45] = + tmpFx[4]*tmpObjS[1] + tmpFx[13]*tmpObjS[12] + tmpFx[22]*tmpObjS[23] + tmpFx[31]*tmpObjS[34] + tmpFx[40]*tmpObjS[45] + tmpFx[49]*tmpObjS[56] + tmpFx[58]*tmpObjS[67] + tmpFx[67]*tmpObjS[78] + tmpFx[76]*tmpObjS[89] + tmpFx[85]*tmpObjS[100] + tmpFx[94]*tmpObjS[111];
tmpQ2[46] = + tmpFx[4]*tmpObjS[2] + tmpFx[13]*tmpObjS[13] + tmpFx[22]*tmpObjS[24] + tmpFx[31]*tmpObjS[35] + tmpFx[40]*tmpObjS[46] + tmpFx[49]*tmpObjS[57] + tmpFx[58]*tmpObjS[68] + tmpFx[67]*tmpObjS[79] + tmpFx[76]*tmpObjS[90] + tmpFx[85]*tmpObjS[101] + tmpFx[94]*tmpObjS[112];
tmpQ2[47] = + tmpFx[4]*tmpObjS[3] + tmpFx[13]*tmpObjS[14] + tmpFx[22]*tmpObjS[25] + tmpFx[31]*tmpObjS[36] + tmpFx[40]*tmpObjS[47] + tmpFx[49]*tmpObjS[58] + tmpFx[58]*tmpObjS[69] + tmpFx[67]*tmpObjS[80] + tmpFx[76]*tmpObjS[91] + tmpFx[85]*tmpObjS[102] + tmpFx[94]*tmpObjS[113];
tmpQ2[48] = + tmpFx[4]*tmpObjS[4] + tmpFx[13]*tmpObjS[15] + tmpFx[22]*tmpObjS[26] + tmpFx[31]*tmpObjS[37] + tmpFx[40]*tmpObjS[48] + tmpFx[49]*tmpObjS[59] + tmpFx[58]*tmpObjS[70] + tmpFx[67]*tmpObjS[81] + tmpFx[76]*tmpObjS[92] + tmpFx[85]*tmpObjS[103] + tmpFx[94]*tmpObjS[114];
tmpQ2[49] = + tmpFx[4]*tmpObjS[5] + tmpFx[13]*tmpObjS[16] + tmpFx[22]*tmpObjS[27] + tmpFx[31]*tmpObjS[38] + tmpFx[40]*tmpObjS[49] + tmpFx[49]*tmpObjS[60] + tmpFx[58]*tmpObjS[71] + tmpFx[67]*tmpObjS[82] + tmpFx[76]*tmpObjS[93] + tmpFx[85]*tmpObjS[104] + tmpFx[94]*tmpObjS[115];
tmpQ2[50] = + tmpFx[4]*tmpObjS[6] + tmpFx[13]*tmpObjS[17] + tmpFx[22]*tmpObjS[28] + tmpFx[31]*tmpObjS[39] + tmpFx[40]*tmpObjS[50] + tmpFx[49]*tmpObjS[61] + tmpFx[58]*tmpObjS[72] + tmpFx[67]*tmpObjS[83] + tmpFx[76]*tmpObjS[94] + tmpFx[85]*tmpObjS[105] + tmpFx[94]*tmpObjS[116];
tmpQ2[51] = + tmpFx[4]*tmpObjS[7] + tmpFx[13]*tmpObjS[18] + tmpFx[22]*tmpObjS[29] + tmpFx[31]*tmpObjS[40] + tmpFx[40]*tmpObjS[51] + tmpFx[49]*tmpObjS[62] + tmpFx[58]*tmpObjS[73] + tmpFx[67]*tmpObjS[84] + tmpFx[76]*tmpObjS[95] + tmpFx[85]*tmpObjS[106] + tmpFx[94]*tmpObjS[117];
tmpQ2[52] = + tmpFx[4]*tmpObjS[8] + tmpFx[13]*tmpObjS[19] + tmpFx[22]*tmpObjS[30] + tmpFx[31]*tmpObjS[41] + tmpFx[40]*tmpObjS[52] + tmpFx[49]*tmpObjS[63] + tmpFx[58]*tmpObjS[74] + tmpFx[67]*tmpObjS[85] + tmpFx[76]*tmpObjS[96] + tmpFx[85]*tmpObjS[107] + tmpFx[94]*tmpObjS[118];
tmpQ2[53] = + tmpFx[4]*tmpObjS[9] + tmpFx[13]*tmpObjS[20] + tmpFx[22]*tmpObjS[31] + tmpFx[31]*tmpObjS[42] + tmpFx[40]*tmpObjS[53] + tmpFx[49]*tmpObjS[64] + tmpFx[58]*tmpObjS[75] + tmpFx[67]*tmpObjS[86] + tmpFx[76]*tmpObjS[97] + tmpFx[85]*tmpObjS[108] + tmpFx[94]*tmpObjS[119];
tmpQ2[54] = + tmpFx[4]*tmpObjS[10] + tmpFx[13]*tmpObjS[21] + tmpFx[22]*tmpObjS[32] + tmpFx[31]*tmpObjS[43] + tmpFx[40]*tmpObjS[54] + tmpFx[49]*tmpObjS[65] + tmpFx[58]*tmpObjS[76] + tmpFx[67]*tmpObjS[87] + tmpFx[76]*tmpObjS[98] + tmpFx[85]*tmpObjS[109] + tmpFx[94]*tmpObjS[120];
tmpQ2[55] = + tmpFx[5]*tmpObjS[0] + tmpFx[14]*tmpObjS[11] + tmpFx[23]*tmpObjS[22] + tmpFx[32]*tmpObjS[33] + tmpFx[41]*tmpObjS[44] + tmpFx[50]*tmpObjS[55] + tmpFx[59]*tmpObjS[66] + tmpFx[68]*tmpObjS[77] + tmpFx[77]*tmpObjS[88] + tmpFx[86]*tmpObjS[99] + tmpFx[95]*tmpObjS[110];
tmpQ2[56] = + tmpFx[5]*tmpObjS[1] + tmpFx[14]*tmpObjS[12] + tmpFx[23]*tmpObjS[23] + tmpFx[32]*tmpObjS[34] + tmpFx[41]*tmpObjS[45] + tmpFx[50]*tmpObjS[56] + tmpFx[59]*tmpObjS[67] + tmpFx[68]*tmpObjS[78] + tmpFx[77]*tmpObjS[89] + tmpFx[86]*tmpObjS[100] + tmpFx[95]*tmpObjS[111];
tmpQ2[57] = + tmpFx[5]*tmpObjS[2] + tmpFx[14]*tmpObjS[13] + tmpFx[23]*tmpObjS[24] + tmpFx[32]*tmpObjS[35] + tmpFx[41]*tmpObjS[46] + tmpFx[50]*tmpObjS[57] + tmpFx[59]*tmpObjS[68] + tmpFx[68]*tmpObjS[79] + tmpFx[77]*tmpObjS[90] + tmpFx[86]*tmpObjS[101] + tmpFx[95]*tmpObjS[112];
tmpQ2[58] = + tmpFx[5]*tmpObjS[3] + tmpFx[14]*tmpObjS[14] + tmpFx[23]*tmpObjS[25] + tmpFx[32]*tmpObjS[36] + tmpFx[41]*tmpObjS[47] + tmpFx[50]*tmpObjS[58] + tmpFx[59]*tmpObjS[69] + tmpFx[68]*tmpObjS[80] + tmpFx[77]*tmpObjS[91] + tmpFx[86]*tmpObjS[102] + tmpFx[95]*tmpObjS[113];
tmpQ2[59] = + tmpFx[5]*tmpObjS[4] + tmpFx[14]*tmpObjS[15] + tmpFx[23]*tmpObjS[26] + tmpFx[32]*tmpObjS[37] + tmpFx[41]*tmpObjS[48] + tmpFx[50]*tmpObjS[59] + tmpFx[59]*tmpObjS[70] + tmpFx[68]*tmpObjS[81] + tmpFx[77]*tmpObjS[92] + tmpFx[86]*tmpObjS[103] + tmpFx[95]*tmpObjS[114];
tmpQ2[60] = + tmpFx[5]*tmpObjS[5] + tmpFx[14]*tmpObjS[16] + tmpFx[23]*tmpObjS[27] + tmpFx[32]*tmpObjS[38] + tmpFx[41]*tmpObjS[49] + tmpFx[50]*tmpObjS[60] + tmpFx[59]*tmpObjS[71] + tmpFx[68]*tmpObjS[82] + tmpFx[77]*tmpObjS[93] + tmpFx[86]*tmpObjS[104] + tmpFx[95]*tmpObjS[115];
tmpQ2[61] = + tmpFx[5]*tmpObjS[6] + tmpFx[14]*tmpObjS[17] + tmpFx[23]*tmpObjS[28] + tmpFx[32]*tmpObjS[39] + tmpFx[41]*tmpObjS[50] + tmpFx[50]*tmpObjS[61] + tmpFx[59]*tmpObjS[72] + tmpFx[68]*tmpObjS[83] + tmpFx[77]*tmpObjS[94] + tmpFx[86]*tmpObjS[105] + tmpFx[95]*tmpObjS[116];
tmpQ2[62] = + tmpFx[5]*tmpObjS[7] + tmpFx[14]*tmpObjS[18] + tmpFx[23]*tmpObjS[29] + tmpFx[32]*tmpObjS[40] + tmpFx[41]*tmpObjS[51] + tmpFx[50]*tmpObjS[62] + tmpFx[59]*tmpObjS[73] + tmpFx[68]*tmpObjS[84] + tmpFx[77]*tmpObjS[95] + tmpFx[86]*tmpObjS[106] + tmpFx[95]*tmpObjS[117];
tmpQ2[63] = + tmpFx[5]*tmpObjS[8] + tmpFx[14]*tmpObjS[19] + tmpFx[23]*tmpObjS[30] + tmpFx[32]*tmpObjS[41] + tmpFx[41]*tmpObjS[52] + tmpFx[50]*tmpObjS[63] + tmpFx[59]*tmpObjS[74] + tmpFx[68]*tmpObjS[85] + tmpFx[77]*tmpObjS[96] + tmpFx[86]*tmpObjS[107] + tmpFx[95]*tmpObjS[118];
tmpQ2[64] = + tmpFx[5]*tmpObjS[9] + tmpFx[14]*tmpObjS[20] + tmpFx[23]*tmpObjS[31] + tmpFx[32]*tmpObjS[42] + tmpFx[41]*tmpObjS[53] + tmpFx[50]*tmpObjS[64] + tmpFx[59]*tmpObjS[75] + tmpFx[68]*tmpObjS[86] + tmpFx[77]*tmpObjS[97] + tmpFx[86]*tmpObjS[108] + tmpFx[95]*tmpObjS[119];
tmpQ2[65] = + tmpFx[5]*tmpObjS[10] + tmpFx[14]*tmpObjS[21] + tmpFx[23]*tmpObjS[32] + tmpFx[32]*tmpObjS[43] + tmpFx[41]*tmpObjS[54] + tmpFx[50]*tmpObjS[65] + tmpFx[59]*tmpObjS[76] + tmpFx[68]*tmpObjS[87] + tmpFx[77]*tmpObjS[98] + tmpFx[86]*tmpObjS[109] + tmpFx[95]*tmpObjS[120];
tmpQ2[66] = + tmpFx[6]*tmpObjS[0] + tmpFx[15]*tmpObjS[11] + tmpFx[24]*tmpObjS[22] + tmpFx[33]*tmpObjS[33] + tmpFx[42]*tmpObjS[44] + tmpFx[51]*tmpObjS[55] + tmpFx[60]*tmpObjS[66] + tmpFx[69]*tmpObjS[77] + tmpFx[78]*tmpObjS[88] + tmpFx[87]*tmpObjS[99] + tmpFx[96]*tmpObjS[110];
tmpQ2[67] = + tmpFx[6]*tmpObjS[1] + tmpFx[15]*tmpObjS[12] + tmpFx[24]*tmpObjS[23] + tmpFx[33]*tmpObjS[34] + tmpFx[42]*tmpObjS[45] + tmpFx[51]*tmpObjS[56] + tmpFx[60]*tmpObjS[67] + tmpFx[69]*tmpObjS[78] + tmpFx[78]*tmpObjS[89] + tmpFx[87]*tmpObjS[100] + tmpFx[96]*tmpObjS[111];
tmpQ2[68] = + tmpFx[6]*tmpObjS[2] + tmpFx[15]*tmpObjS[13] + tmpFx[24]*tmpObjS[24] + tmpFx[33]*tmpObjS[35] + tmpFx[42]*tmpObjS[46] + tmpFx[51]*tmpObjS[57] + tmpFx[60]*tmpObjS[68] + tmpFx[69]*tmpObjS[79] + tmpFx[78]*tmpObjS[90] + tmpFx[87]*tmpObjS[101] + tmpFx[96]*tmpObjS[112];
tmpQ2[69] = + tmpFx[6]*tmpObjS[3] + tmpFx[15]*tmpObjS[14] + tmpFx[24]*tmpObjS[25] + tmpFx[33]*tmpObjS[36] + tmpFx[42]*tmpObjS[47] + tmpFx[51]*tmpObjS[58] + tmpFx[60]*tmpObjS[69] + tmpFx[69]*tmpObjS[80] + tmpFx[78]*tmpObjS[91] + tmpFx[87]*tmpObjS[102] + tmpFx[96]*tmpObjS[113];
tmpQ2[70] = + tmpFx[6]*tmpObjS[4] + tmpFx[15]*tmpObjS[15] + tmpFx[24]*tmpObjS[26] + tmpFx[33]*tmpObjS[37] + tmpFx[42]*tmpObjS[48] + tmpFx[51]*tmpObjS[59] + tmpFx[60]*tmpObjS[70] + tmpFx[69]*tmpObjS[81] + tmpFx[78]*tmpObjS[92] + tmpFx[87]*tmpObjS[103] + tmpFx[96]*tmpObjS[114];
tmpQ2[71] = + tmpFx[6]*tmpObjS[5] + tmpFx[15]*tmpObjS[16] + tmpFx[24]*tmpObjS[27] + tmpFx[33]*tmpObjS[38] + tmpFx[42]*tmpObjS[49] + tmpFx[51]*tmpObjS[60] + tmpFx[60]*tmpObjS[71] + tmpFx[69]*tmpObjS[82] + tmpFx[78]*tmpObjS[93] + tmpFx[87]*tmpObjS[104] + tmpFx[96]*tmpObjS[115];
tmpQ2[72] = + tmpFx[6]*tmpObjS[6] + tmpFx[15]*tmpObjS[17] + tmpFx[24]*tmpObjS[28] + tmpFx[33]*tmpObjS[39] + tmpFx[42]*tmpObjS[50] + tmpFx[51]*tmpObjS[61] + tmpFx[60]*tmpObjS[72] + tmpFx[69]*tmpObjS[83] + tmpFx[78]*tmpObjS[94] + tmpFx[87]*tmpObjS[105] + tmpFx[96]*tmpObjS[116];
tmpQ2[73] = + tmpFx[6]*tmpObjS[7] + tmpFx[15]*tmpObjS[18] + tmpFx[24]*tmpObjS[29] + tmpFx[33]*tmpObjS[40] + tmpFx[42]*tmpObjS[51] + tmpFx[51]*tmpObjS[62] + tmpFx[60]*tmpObjS[73] + tmpFx[69]*tmpObjS[84] + tmpFx[78]*tmpObjS[95] + tmpFx[87]*tmpObjS[106] + tmpFx[96]*tmpObjS[117];
tmpQ2[74] = + tmpFx[6]*tmpObjS[8] + tmpFx[15]*tmpObjS[19] + tmpFx[24]*tmpObjS[30] + tmpFx[33]*tmpObjS[41] + tmpFx[42]*tmpObjS[52] + tmpFx[51]*tmpObjS[63] + tmpFx[60]*tmpObjS[74] + tmpFx[69]*tmpObjS[85] + tmpFx[78]*tmpObjS[96] + tmpFx[87]*tmpObjS[107] + tmpFx[96]*tmpObjS[118];
tmpQ2[75] = + tmpFx[6]*tmpObjS[9] + tmpFx[15]*tmpObjS[20] + tmpFx[24]*tmpObjS[31] + tmpFx[33]*tmpObjS[42] + tmpFx[42]*tmpObjS[53] + tmpFx[51]*tmpObjS[64] + tmpFx[60]*tmpObjS[75] + tmpFx[69]*tmpObjS[86] + tmpFx[78]*tmpObjS[97] + tmpFx[87]*tmpObjS[108] + tmpFx[96]*tmpObjS[119];
tmpQ2[76] = + tmpFx[6]*tmpObjS[10] + tmpFx[15]*tmpObjS[21] + tmpFx[24]*tmpObjS[32] + tmpFx[33]*tmpObjS[43] + tmpFx[42]*tmpObjS[54] + tmpFx[51]*tmpObjS[65] + tmpFx[60]*tmpObjS[76] + tmpFx[69]*tmpObjS[87] + tmpFx[78]*tmpObjS[98] + tmpFx[87]*tmpObjS[109] + tmpFx[96]*tmpObjS[120];
tmpQ2[77] = + tmpFx[7]*tmpObjS[0] + tmpFx[16]*tmpObjS[11] + tmpFx[25]*tmpObjS[22] + tmpFx[34]*tmpObjS[33] + tmpFx[43]*tmpObjS[44] + tmpFx[52]*tmpObjS[55] + tmpFx[61]*tmpObjS[66] + tmpFx[70]*tmpObjS[77] + tmpFx[79]*tmpObjS[88] + tmpFx[88]*tmpObjS[99] + tmpFx[97]*tmpObjS[110];
tmpQ2[78] = + tmpFx[7]*tmpObjS[1] + tmpFx[16]*tmpObjS[12] + tmpFx[25]*tmpObjS[23] + tmpFx[34]*tmpObjS[34] + tmpFx[43]*tmpObjS[45] + tmpFx[52]*tmpObjS[56] + tmpFx[61]*tmpObjS[67] + tmpFx[70]*tmpObjS[78] + tmpFx[79]*tmpObjS[89] + tmpFx[88]*tmpObjS[100] + tmpFx[97]*tmpObjS[111];
tmpQ2[79] = + tmpFx[7]*tmpObjS[2] + tmpFx[16]*tmpObjS[13] + tmpFx[25]*tmpObjS[24] + tmpFx[34]*tmpObjS[35] + tmpFx[43]*tmpObjS[46] + tmpFx[52]*tmpObjS[57] + tmpFx[61]*tmpObjS[68] + tmpFx[70]*tmpObjS[79] + tmpFx[79]*tmpObjS[90] + tmpFx[88]*tmpObjS[101] + tmpFx[97]*tmpObjS[112];
tmpQ2[80] = + tmpFx[7]*tmpObjS[3] + tmpFx[16]*tmpObjS[14] + tmpFx[25]*tmpObjS[25] + tmpFx[34]*tmpObjS[36] + tmpFx[43]*tmpObjS[47] + tmpFx[52]*tmpObjS[58] + tmpFx[61]*tmpObjS[69] + tmpFx[70]*tmpObjS[80] + tmpFx[79]*tmpObjS[91] + tmpFx[88]*tmpObjS[102] + tmpFx[97]*tmpObjS[113];
tmpQ2[81] = + tmpFx[7]*tmpObjS[4] + tmpFx[16]*tmpObjS[15] + tmpFx[25]*tmpObjS[26] + tmpFx[34]*tmpObjS[37] + tmpFx[43]*tmpObjS[48] + tmpFx[52]*tmpObjS[59] + tmpFx[61]*tmpObjS[70] + tmpFx[70]*tmpObjS[81] + tmpFx[79]*tmpObjS[92] + tmpFx[88]*tmpObjS[103] + tmpFx[97]*tmpObjS[114];
tmpQ2[82] = + tmpFx[7]*tmpObjS[5] + tmpFx[16]*tmpObjS[16] + tmpFx[25]*tmpObjS[27] + tmpFx[34]*tmpObjS[38] + tmpFx[43]*tmpObjS[49] + tmpFx[52]*tmpObjS[60] + tmpFx[61]*tmpObjS[71] + tmpFx[70]*tmpObjS[82] + tmpFx[79]*tmpObjS[93] + tmpFx[88]*tmpObjS[104] + tmpFx[97]*tmpObjS[115];
tmpQ2[83] = + tmpFx[7]*tmpObjS[6] + tmpFx[16]*tmpObjS[17] + tmpFx[25]*tmpObjS[28] + tmpFx[34]*tmpObjS[39] + tmpFx[43]*tmpObjS[50] + tmpFx[52]*tmpObjS[61] + tmpFx[61]*tmpObjS[72] + tmpFx[70]*tmpObjS[83] + tmpFx[79]*tmpObjS[94] + tmpFx[88]*tmpObjS[105] + tmpFx[97]*tmpObjS[116];
tmpQ2[84] = + tmpFx[7]*tmpObjS[7] + tmpFx[16]*tmpObjS[18] + tmpFx[25]*tmpObjS[29] + tmpFx[34]*tmpObjS[40] + tmpFx[43]*tmpObjS[51] + tmpFx[52]*tmpObjS[62] + tmpFx[61]*tmpObjS[73] + tmpFx[70]*tmpObjS[84] + tmpFx[79]*tmpObjS[95] + tmpFx[88]*tmpObjS[106] + tmpFx[97]*tmpObjS[117];
tmpQ2[85] = + tmpFx[7]*tmpObjS[8] + tmpFx[16]*tmpObjS[19] + tmpFx[25]*tmpObjS[30] + tmpFx[34]*tmpObjS[41] + tmpFx[43]*tmpObjS[52] + tmpFx[52]*tmpObjS[63] + tmpFx[61]*tmpObjS[74] + tmpFx[70]*tmpObjS[85] + tmpFx[79]*tmpObjS[96] + tmpFx[88]*tmpObjS[107] + tmpFx[97]*tmpObjS[118];
tmpQ2[86] = + tmpFx[7]*tmpObjS[9] + tmpFx[16]*tmpObjS[20] + tmpFx[25]*tmpObjS[31] + tmpFx[34]*tmpObjS[42] + tmpFx[43]*tmpObjS[53] + tmpFx[52]*tmpObjS[64] + tmpFx[61]*tmpObjS[75] + tmpFx[70]*tmpObjS[86] + tmpFx[79]*tmpObjS[97] + tmpFx[88]*tmpObjS[108] + tmpFx[97]*tmpObjS[119];
tmpQ2[87] = + tmpFx[7]*tmpObjS[10] + tmpFx[16]*tmpObjS[21] + tmpFx[25]*tmpObjS[32] + tmpFx[34]*tmpObjS[43] + tmpFx[43]*tmpObjS[54] + tmpFx[52]*tmpObjS[65] + tmpFx[61]*tmpObjS[76] + tmpFx[70]*tmpObjS[87] + tmpFx[79]*tmpObjS[98] + tmpFx[88]*tmpObjS[109] + tmpFx[97]*tmpObjS[120];
tmpQ2[88] = + tmpFx[8]*tmpObjS[0] + tmpFx[17]*tmpObjS[11] + tmpFx[26]*tmpObjS[22] + tmpFx[35]*tmpObjS[33] + tmpFx[44]*tmpObjS[44] + tmpFx[53]*tmpObjS[55] + tmpFx[62]*tmpObjS[66] + tmpFx[71]*tmpObjS[77] + tmpFx[80]*tmpObjS[88] + tmpFx[89]*tmpObjS[99] + tmpFx[98]*tmpObjS[110];
tmpQ2[89] = + tmpFx[8]*tmpObjS[1] + tmpFx[17]*tmpObjS[12] + tmpFx[26]*tmpObjS[23] + tmpFx[35]*tmpObjS[34] + tmpFx[44]*tmpObjS[45] + tmpFx[53]*tmpObjS[56] + tmpFx[62]*tmpObjS[67] + tmpFx[71]*tmpObjS[78] + tmpFx[80]*tmpObjS[89] + tmpFx[89]*tmpObjS[100] + tmpFx[98]*tmpObjS[111];
tmpQ2[90] = + tmpFx[8]*tmpObjS[2] + tmpFx[17]*tmpObjS[13] + tmpFx[26]*tmpObjS[24] + tmpFx[35]*tmpObjS[35] + tmpFx[44]*tmpObjS[46] + tmpFx[53]*tmpObjS[57] + tmpFx[62]*tmpObjS[68] + tmpFx[71]*tmpObjS[79] + tmpFx[80]*tmpObjS[90] + tmpFx[89]*tmpObjS[101] + tmpFx[98]*tmpObjS[112];
tmpQ2[91] = + tmpFx[8]*tmpObjS[3] + tmpFx[17]*tmpObjS[14] + tmpFx[26]*tmpObjS[25] + tmpFx[35]*tmpObjS[36] + tmpFx[44]*tmpObjS[47] + tmpFx[53]*tmpObjS[58] + tmpFx[62]*tmpObjS[69] + tmpFx[71]*tmpObjS[80] + tmpFx[80]*tmpObjS[91] + tmpFx[89]*tmpObjS[102] + tmpFx[98]*tmpObjS[113];
tmpQ2[92] = + tmpFx[8]*tmpObjS[4] + tmpFx[17]*tmpObjS[15] + tmpFx[26]*tmpObjS[26] + tmpFx[35]*tmpObjS[37] + tmpFx[44]*tmpObjS[48] + tmpFx[53]*tmpObjS[59] + tmpFx[62]*tmpObjS[70] + tmpFx[71]*tmpObjS[81] + tmpFx[80]*tmpObjS[92] + tmpFx[89]*tmpObjS[103] + tmpFx[98]*tmpObjS[114];
tmpQ2[93] = + tmpFx[8]*tmpObjS[5] + tmpFx[17]*tmpObjS[16] + tmpFx[26]*tmpObjS[27] + tmpFx[35]*tmpObjS[38] + tmpFx[44]*tmpObjS[49] + tmpFx[53]*tmpObjS[60] + tmpFx[62]*tmpObjS[71] + tmpFx[71]*tmpObjS[82] + tmpFx[80]*tmpObjS[93] + tmpFx[89]*tmpObjS[104] + tmpFx[98]*tmpObjS[115];
tmpQ2[94] = + tmpFx[8]*tmpObjS[6] + tmpFx[17]*tmpObjS[17] + tmpFx[26]*tmpObjS[28] + tmpFx[35]*tmpObjS[39] + tmpFx[44]*tmpObjS[50] + tmpFx[53]*tmpObjS[61] + tmpFx[62]*tmpObjS[72] + tmpFx[71]*tmpObjS[83] + tmpFx[80]*tmpObjS[94] + tmpFx[89]*tmpObjS[105] + tmpFx[98]*tmpObjS[116];
tmpQ2[95] = + tmpFx[8]*tmpObjS[7] + tmpFx[17]*tmpObjS[18] + tmpFx[26]*tmpObjS[29] + tmpFx[35]*tmpObjS[40] + tmpFx[44]*tmpObjS[51] + tmpFx[53]*tmpObjS[62] + tmpFx[62]*tmpObjS[73] + tmpFx[71]*tmpObjS[84] + tmpFx[80]*tmpObjS[95] + tmpFx[89]*tmpObjS[106] + tmpFx[98]*tmpObjS[117];
tmpQ2[96] = + tmpFx[8]*tmpObjS[8] + tmpFx[17]*tmpObjS[19] + tmpFx[26]*tmpObjS[30] + tmpFx[35]*tmpObjS[41] + tmpFx[44]*tmpObjS[52] + tmpFx[53]*tmpObjS[63] + tmpFx[62]*tmpObjS[74] + tmpFx[71]*tmpObjS[85] + tmpFx[80]*tmpObjS[96] + tmpFx[89]*tmpObjS[107] + tmpFx[98]*tmpObjS[118];
tmpQ2[97] = + tmpFx[8]*tmpObjS[9] + tmpFx[17]*tmpObjS[20] + tmpFx[26]*tmpObjS[31] + tmpFx[35]*tmpObjS[42] + tmpFx[44]*tmpObjS[53] + tmpFx[53]*tmpObjS[64] + tmpFx[62]*tmpObjS[75] + tmpFx[71]*tmpObjS[86] + tmpFx[80]*tmpObjS[97] + tmpFx[89]*tmpObjS[108] + tmpFx[98]*tmpObjS[119];
tmpQ2[98] = + tmpFx[8]*tmpObjS[10] + tmpFx[17]*tmpObjS[21] + tmpFx[26]*tmpObjS[32] + tmpFx[35]*tmpObjS[43] + tmpFx[44]*tmpObjS[54] + tmpFx[53]*tmpObjS[65] + tmpFx[62]*tmpObjS[76] + tmpFx[71]*tmpObjS[87] + tmpFx[80]*tmpObjS[98] + tmpFx[89]*tmpObjS[109] + tmpFx[98]*tmpObjS[120];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[9] + tmpQ2[2]*tmpFx[18] + tmpQ2[3]*tmpFx[27] + tmpQ2[4]*tmpFx[36] + tmpQ2[5]*tmpFx[45] + tmpQ2[6]*tmpFx[54] + tmpQ2[7]*tmpFx[63] + tmpQ2[8]*tmpFx[72] + tmpQ2[9]*tmpFx[81] + tmpQ2[10]*tmpFx[90];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[10] + tmpQ2[2]*tmpFx[19] + tmpQ2[3]*tmpFx[28] + tmpQ2[4]*tmpFx[37] + tmpQ2[5]*tmpFx[46] + tmpQ2[6]*tmpFx[55] + tmpQ2[7]*tmpFx[64] + tmpQ2[8]*tmpFx[73] + tmpQ2[9]*tmpFx[82] + tmpQ2[10]*tmpFx[91];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[11] + tmpQ2[2]*tmpFx[20] + tmpQ2[3]*tmpFx[29] + tmpQ2[4]*tmpFx[38] + tmpQ2[5]*tmpFx[47] + tmpQ2[6]*tmpFx[56] + tmpQ2[7]*tmpFx[65] + tmpQ2[8]*tmpFx[74] + tmpQ2[9]*tmpFx[83] + tmpQ2[10]*tmpFx[92];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[12] + tmpQ2[2]*tmpFx[21] + tmpQ2[3]*tmpFx[30] + tmpQ2[4]*tmpFx[39] + tmpQ2[5]*tmpFx[48] + tmpQ2[6]*tmpFx[57] + tmpQ2[7]*tmpFx[66] + tmpQ2[8]*tmpFx[75] + tmpQ2[9]*tmpFx[84] + tmpQ2[10]*tmpFx[93];
tmpQ1[4] = + tmpQ2[0]*tmpFx[4] + tmpQ2[1]*tmpFx[13] + tmpQ2[2]*tmpFx[22] + tmpQ2[3]*tmpFx[31] + tmpQ2[4]*tmpFx[40] + tmpQ2[5]*tmpFx[49] + tmpQ2[6]*tmpFx[58] + tmpQ2[7]*tmpFx[67] + tmpQ2[8]*tmpFx[76] + tmpQ2[9]*tmpFx[85] + tmpQ2[10]*tmpFx[94];
tmpQ1[5] = + tmpQ2[0]*tmpFx[5] + tmpQ2[1]*tmpFx[14] + tmpQ2[2]*tmpFx[23] + tmpQ2[3]*tmpFx[32] + tmpQ2[4]*tmpFx[41] + tmpQ2[5]*tmpFx[50] + tmpQ2[6]*tmpFx[59] + tmpQ2[7]*tmpFx[68] + tmpQ2[8]*tmpFx[77] + tmpQ2[9]*tmpFx[86] + tmpQ2[10]*tmpFx[95];
tmpQ1[6] = + tmpQ2[0]*tmpFx[6] + tmpQ2[1]*tmpFx[15] + tmpQ2[2]*tmpFx[24] + tmpQ2[3]*tmpFx[33] + tmpQ2[4]*tmpFx[42] + tmpQ2[5]*tmpFx[51] + tmpQ2[6]*tmpFx[60] + tmpQ2[7]*tmpFx[69] + tmpQ2[8]*tmpFx[78] + tmpQ2[9]*tmpFx[87] + tmpQ2[10]*tmpFx[96];
tmpQ1[7] = + tmpQ2[0]*tmpFx[7] + tmpQ2[1]*tmpFx[16] + tmpQ2[2]*tmpFx[25] + tmpQ2[3]*tmpFx[34] + tmpQ2[4]*tmpFx[43] + tmpQ2[5]*tmpFx[52] + tmpQ2[6]*tmpFx[61] + tmpQ2[7]*tmpFx[70] + tmpQ2[8]*tmpFx[79] + tmpQ2[9]*tmpFx[88] + tmpQ2[10]*tmpFx[97];
tmpQ1[8] = + tmpQ2[0]*tmpFx[8] + tmpQ2[1]*tmpFx[17] + tmpQ2[2]*tmpFx[26] + tmpQ2[3]*tmpFx[35] + tmpQ2[4]*tmpFx[44] + tmpQ2[5]*tmpFx[53] + tmpQ2[6]*tmpFx[62] + tmpQ2[7]*tmpFx[71] + tmpQ2[8]*tmpFx[80] + tmpQ2[9]*tmpFx[89] + tmpQ2[10]*tmpFx[98];
tmpQ1[9] = + tmpQ2[11]*tmpFx[0] + tmpQ2[12]*tmpFx[9] + tmpQ2[13]*tmpFx[18] + tmpQ2[14]*tmpFx[27] + tmpQ2[15]*tmpFx[36] + tmpQ2[16]*tmpFx[45] + tmpQ2[17]*tmpFx[54] + tmpQ2[18]*tmpFx[63] + tmpQ2[19]*tmpFx[72] + tmpQ2[20]*tmpFx[81] + tmpQ2[21]*tmpFx[90];
tmpQ1[10] = + tmpQ2[11]*tmpFx[1] + tmpQ2[12]*tmpFx[10] + tmpQ2[13]*tmpFx[19] + tmpQ2[14]*tmpFx[28] + tmpQ2[15]*tmpFx[37] + tmpQ2[16]*tmpFx[46] + tmpQ2[17]*tmpFx[55] + tmpQ2[18]*tmpFx[64] + tmpQ2[19]*tmpFx[73] + tmpQ2[20]*tmpFx[82] + tmpQ2[21]*tmpFx[91];
tmpQ1[11] = + tmpQ2[11]*tmpFx[2] + tmpQ2[12]*tmpFx[11] + tmpQ2[13]*tmpFx[20] + tmpQ2[14]*tmpFx[29] + tmpQ2[15]*tmpFx[38] + tmpQ2[16]*tmpFx[47] + tmpQ2[17]*tmpFx[56] + tmpQ2[18]*tmpFx[65] + tmpQ2[19]*tmpFx[74] + tmpQ2[20]*tmpFx[83] + tmpQ2[21]*tmpFx[92];
tmpQ1[12] = + tmpQ2[11]*tmpFx[3] + tmpQ2[12]*tmpFx[12] + tmpQ2[13]*tmpFx[21] + tmpQ2[14]*tmpFx[30] + tmpQ2[15]*tmpFx[39] + tmpQ2[16]*tmpFx[48] + tmpQ2[17]*tmpFx[57] + tmpQ2[18]*tmpFx[66] + tmpQ2[19]*tmpFx[75] + tmpQ2[20]*tmpFx[84] + tmpQ2[21]*tmpFx[93];
tmpQ1[13] = + tmpQ2[11]*tmpFx[4] + tmpQ2[12]*tmpFx[13] + tmpQ2[13]*tmpFx[22] + tmpQ2[14]*tmpFx[31] + tmpQ2[15]*tmpFx[40] + tmpQ2[16]*tmpFx[49] + tmpQ2[17]*tmpFx[58] + tmpQ2[18]*tmpFx[67] + tmpQ2[19]*tmpFx[76] + tmpQ2[20]*tmpFx[85] + tmpQ2[21]*tmpFx[94];
tmpQ1[14] = + tmpQ2[11]*tmpFx[5] + tmpQ2[12]*tmpFx[14] + tmpQ2[13]*tmpFx[23] + tmpQ2[14]*tmpFx[32] + tmpQ2[15]*tmpFx[41] + tmpQ2[16]*tmpFx[50] + tmpQ2[17]*tmpFx[59] + tmpQ2[18]*tmpFx[68] + tmpQ2[19]*tmpFx[77] + tmpQ2[20]*tmpFx[86] + tmpQ2[21]*tmpFx[95];
tmpQ1[15] = + tmpQ2[11]*tmpFx[6] + tmpQ2[12]*tmpFx[15] + tmpQ2[13]*tmpFx[24] + tmpQ2[14]*tmpFx[33] + tmpQ2[15]*tmpFx[42] + tmpQ2[16]*tmpFx[51] + tmpQ2[17]*tmpFx[60] + tmpQ2[18]*tmpFx[69] + tmpQ2[19]*tmpFx[78] + tmpQ2[20]*tmpFx[87] + tmpQ2[21]*tmpFx[96];
tmpQ1[16] = + tmpQ2[11]*tmpFx[7] + tmpQ2[12]*tmpFx[16] + tmpQ2[13]*tmpFx[25] + tmpQ2[14]*tmpFx[34] + tmpQ2[15]*tmpFx[43] + tmpQ2[16]*tmpFx[52] + tmpQ2[17]*tmpFx[61] + tmpQ2[18]*tmpFx[70] + tmpQ2[19]*tmpFx[79] + tmpQ2[20]*tmpFx[88] + tmpQ2[21]*tmpFx[97];
tmpQ1[17] = + tmpQ2[11]*tmpFx[8] + tmpQ2[12]*tmpFx[17] + tmpQ2[13]*tmpFx[26] + tmpQ2[14]*tmpFx[35] + tmpQ2[15]*tmpFx[44] + tmpQ2[16]*tmpFx[53] + tmpQ2[17]*tmpFx[62] + tmpQ2[18]*tmpFx[71] + tmpQ2[19]*tmpFx[80] + tmpQ2[20]*tmpFx[89] + tmpQ2[21]*tmpFx[98];
tmpQ1[18] = + tmpQ2[22]*tmpFx[0] + tmpQ2[23]*tmpFx[9] + tmpQ2[24]*tmpFx[18] + tmpQ2[25]*tmpFx[27] + tmpQ2[26]*tmpFx[36] + tmpQ2[27]*tmpFx[45] + tmpQ2[28]*tmpFx[54] + tmpQ2[29]*tmpFx[63] + tmpQ2[30]*tmpFx[72] + tmpQ2[31]*tmpFx[81] + tmpQ2[32]*tmpFx[90];
tmpQ1[19] = + tmpQ2[22]*tmpFx[1] + tmpQ2[23]*tmpFx[10] + tmpQ2[24]*tmpFx[19] + tmpQ2[25]*tmpFx[28] + tmpQ2[26]*tmpFx[37] + tmpQ2[27]*tmpFx[46] + tmpQ2[28]*tmpFx[55] + tmpQ2[29]*tmpFx[64] + tmpQ2[30]*tmpFx[73] + tmpQ2[31]*tmpFx[82] + tmpQ2[32]*tmpFx[91];
tmpQ1[20] = + tmpQ2[22]*tmpFx[2] + tmpQ2[23]*tmpFx[11] + tmpQ2[24]*tmpFx[20] + tmpQ2[25]*tmpFx[29] + tmpQ2[26]*tmpFx[38] + tmpQ2[27]*tmpFx[47] + tmpQ2[28]*tmpFx[56] + tmpQ2[29]*tmpFx[65] + tmpQ2[30]*tmpFx[74] + tmpQ2[31]*tmpFx[83] + tmpQ2[32]*tmpFx[92];
tmpQ1[21] = + tmpQ2[22]*tmpFx[3] + tmpQ2[23]*tmpFx[12] + tmpQ2[24]*tmpFx[21] + tmpQ2[25]*tmpFx[30] + tmpQ2[26]*tmpFx[39] + tmpQ2[27]*tmpFx[48] + tmpQ2[28]*tmpFx[57] + tmpQ2[29]*tmpFx[66] + tmpQ2[30]*tmpFx[75] + tmpQ2[31]*tmpFx[84] + tmpQ2[32]*tmpFx[93];
tmpQ1[22] = + tmpQ2[22]*tmpFx[4] + tmpQ2[23]*tmpFx[13] + tmpQ2[24]*tmpFx[22] + tmpQ2[25]*tmpFx[31] + tmpQ2[26]*tmpFx[40] + tmpQ2[27]*tmpFx[49] + tmpQ2[28]*tmpFx[58] + tmpQ2[29]*tmpFx[67] + tmpQ2[30]*tmpFx[76] + tmpQ2[31]*tmpFx[85] + tmpQ2[32]*tmpFx[94];
tmpQ1[23] = + tmpQ2[22]*tmpFx[5] + tmpQ2[23]*tmpFx[14] + tmpQ2[24]*tmpFx[23] + tmpQ2[25]*tmpFx[32] + tmpQ2[26]*tmpFx[41] + tmpQ2[27]*tmpFx[50] + tmpQ2[28]*tmpFx[59] + tmpQ2[29]*tmpFx[68] + tmpQ2[30]*tmpFx[77] + tmpQ2[31]*tmpFx[86] + tmpQ2[32]*tmpFx[95];
tmpQ1[24] = + tmpQ2[22]*tmpFx[6] + tmpQ2[23]*tmpFx[15] + tmpQ2[24]*tmpFx[24] + tmpQ2[25]*tmpFx[33] + tmpQ2[26]*tmpFx[42] + tmpQ2[27]*tmpFx[51] + tmpQ2[28]*tmpFx[60] + tmpQ2[29]*tmpFx[69] + tmpQ2[30]*tmpFx[78] + tmpQ2[31]*tmpFx[87] + tmpQ2[32]*tmpFx[96];
tmpQ1[25] = + tmpQ2[22]*tmpFx[7] + tmpQ2[23]*tmpFx[16] + tmpQ2[24]*tmpFx[25] + tmpQ2[25]*tmpFx[34] + tmpQ2[26]*tmpFx[43] + tmpQ2[27]*tmpFx[52] + tmpQ2[28]*tmpFx[61] + tmpQ2[29]*tmpFx[70] + tmpQ2[30]*tmpFx[79] + tmpQ2[31]*tmpFx[88] + tmpQ2[32]*tmpFx[97];
tmpQ1[26] = + tmpQ2[22]*tmpFx[8] + tmpQ2[23]*tmpFx[17] + tmpQ2[24]*tmpFx[26] + tmpQ2[25]*tmpFx[35] + tmpQ2[26]*tmpFx[44] + tmpQ2[27]*tmpFx[53] + tmpQ2[28]*tmpFx[62] + tmpQ2[29]*tmpFx[71] + tmpQ2[30]*tmpFx[80] + tmpQ2[31]*tmpFx[89] + tmpQ2[32]*tmpFx[98];
tmpQ1[27] = + tmpQ2[33]*tmpFx[0] + tmpQ2[34]*tmpFx[9] + tmpQ2[35]*tmpFx[18] + tmpQ2[36]*tmpFx[27] + tmpQ2[37]*tmpFx[36] + tmpQ2[38]*tmpFx[45] + tmpQ2[39]*tmpFx[54] + tmpQ2[40]*tmpFx[63] + tmpQ2[41]*tmpFx[72] + tmpQ2[42]*tmpFx[81] + tmpQ2[43]*tmpFx[90];
tmpQ1[28] = + tmpQ2[33]*tmpFx[1] + tmpQ2[34]*tmpFx[10] + tmpQ2[35]*tmpFx[19] + tmpQ2[36]*tmpFx[28] + tmpQ2[37]*tmpFx[37] + tmpQ2[38]*tmpFx[46] + tmpQ2[39]*tmpFx[55] + tmpQ2[40]*tmpFx[64] + tmpQ2[41]*tmpFx[73] + tmpQ2[42]*tmpFx[82] + tmpQ2[43]*tmpFx[91];
tmpQ1[29] = + tmpQ2[33]*tmpFx[2] + tmpQ2[34]*tmpFx[11] + tmpQ2[35]*tmpFx[20] + tmpQ2[36]*tmpFx[29] + tmpQ2[37]*tmpFx[38] + tmpQ2[38]*tmpFx[47] + tmpQ2[39]*tmpFx[56] + tmpQ2[40]*tmpFx[65] + tmpQ2[41]*tmpFx[74] + tmpQ2[42]*tmpFx[83] + tmpQ2[43]*tmpFx[92];
tmpQ1[30] = + tmpQ2[33]*tmpFx[3] + tmpQ2[34]*tmpFx[12] + tmpQ2[35]*tmpFx[21] + tmpQ2[36]*tmpFx[30] + tmpQ2[37]*tmpFx[39] + tmpQ2[38]*tmpFx[48] + tmpQ2[39]*tmpFx[57] + tmpQ2[40]*tmpFx[66] + tmpQ2[41]*tmpFx[75] + tmpQ2[42]*tmpFx[84] + tmpQ2[43]*tmpFx[93];
tmpQ1[31] = + tmpQ2[33]*tmpFx[4] + tmpQ2[34]*tmpFx[13] + tmpQ2[35]*tmpFx[22] + tmpQ2[36]*tmpFx[31] + tmpQ2[37]*tmpFx[40] + tmpQ2[38]*tmpFx[49] + tmpQ2[39]*tmpFx[58] + tmpQ2[40]*tmpFx[67] + tmpQ2[41]*tmpFx[76] + tmpQ2[42]*tmpFx[85] + tmpQ2[43]*tmpFx[94];
tmpQ1[32] = + tmpQ2[33]*tmpFx[5] + tmpQ2[34]*tmpFx[14] + tmpQ2[35]*tmpFx[23] + tmpQ2[36]*tmpFx[32] + tmpQ2[37]*tmpFx[41] + tmpQ2[38]*tmpFx[50] + tmpQ2[39]*tmpFx[59] + tmpQ2[40]*tmpFx[68] + tmpQ2[41]*tmpFx[77] + tmpQ2[42]*tmpFx[86] + tmpQ2[43]*tmpFx[95];
tmpQ1[33] = + tmpQ2[33]*tmpFx[6] + tmpQ2[34]*tmpFx[15] + tmpQ2[35]*tmpFx[24] + tmpQ2[36]*tmpFx[33] + tmpQ2[37]*tmpFx[42] + tmpQ2[38]*tmpFx[51] + tmpQ2[39]*tmpFx[60] + tmpQ2[40]*tmpFx[69] + tmpQ2[41]*tmpFx[78] + tmpQ2[42]*tmpFx[87] + tmpQ2[43]*tmpFx[96];
tmpQ1[34] = + tmpQ2[33]*tmpFx[7] + tmpQ2[34]*tmpFx[16] + tmpQ2[35]*tmpFx[25] + tmpQ2[36]*tmpFx[34] + tmpQ2[37]*tmpFx[43] + tmpQ2[38]*tmpFx[52] + tmpQ2[39]*tmpFx[61] + tmpQ2[40]*tmpFx[70] + tmpQ2[41]*tmpFx[79] + tmpQ2[42]*tmpFx[88] + tmpQ2[43]*tmpFx[97];
tmpQ1[35] = + tmpQ2[33]*tmpFx[8] + tmpQ2[34]*tmpFx[17] + tmpQ2[35]*tmpFx[26] + tmpQ2[36]*tmpFx[35] + tmpQ2[37]*tmpFx[44] + tmpQ2[38]*tmpFx[53] + tmpQ2[39]*tmpFx[62] + tmpQ2[40]*tmpFx[71] + tmpQ2[41]*tmpFx[80] + tmpQ2[42]*tmpFx[89] + tmpQ2[43]*tmpFx[98];
tmpQ1[36] = + tmpQ2[44]*tmpFx[0] + tmpQ2[45]*tmpFx[9] + tmpQ2[46]*tmpFx[18] + tmpQ2[47]*tmpFx[27] + tmpQ2[48]*tmpFx[36] + tmpQ2[49]*tmpFx[45] + tmpQ2[50]*tmpFx[54] + tmpQ2[51]*tmpFx[63] + tmpQ2[52]*tmpFx[72] + tmpQ2[53]*tmpFx[81] + tmpQ2[54]*tmpFx[90];
tmpQ1[37] = + tmpQ2[44]*tmpFx[1] + tmpQ2[45]*tmpFx[10] + tmpQ2[46]*tmpFx[19] + tmpQ2[47]*tmpFx[28] + tmpQ2[48]*tmpFx[37] + tmpQ2[49]*tmpFx[46] + tmpQ2[50]*tmpFx[55] + tmpQ2[51]*tmpFx[64] + tmpQ2[52]*tmpFx[73] + tmpQ2[53]*tmpFx[82] + tmpQ2[54]*tmpFx[91];
tmpQ1[38] = + tmpQ2[44]*tmpFx[2] + tmpQ2[45]*tmpFx[11] + tmpQ2[46]*tmpFx[20] + tmpQ2[47]*tmpFx[29] + tmpQ2[48]*tmpFx[38] + tmpQ2[49]*tmpFx[47] + tmpQ2[50]*tmpFx[56] + tmpQ2[51]*tmpFx[65] + tmpQ2[52]*tmpFx[74] + tmpQ2[53]*tmpFx[83] + tmpQ2[54]*tmpFx[92];
tmpQ1[39] = + tmpQ2[44]*tmpFx[3] + tmpQ2[45]*tmpFx[12] + tmpQ2[46]*tmpFx[21] + tmpQ2[47]*tmpFx[30] + tmpQ2[48]*tmpFx[39] + tmpQ2[49]*tmpFx[48] + tmpQ2[50]*tmpFx[57] + tmpQ2[51]*tmpFx[66] + tmpQ2[52]*tmpFx[75] + tmpQ2[53]*tmpFx[84] + tmpQ2[54]*tmpFx[93];
tmpQ1[40] = + tmpQ2[44]*tmpFx[4] + tmpQ2[45]*tmpFx[13] + tmpQ2[46]*tmpFx[22] + tmpQ2[47]*tmpFx[31] + tmpQ2[48]*tmpFx[40] + tmpQ2[49]*tmpFx[49] + tmpQ2[50]*tmpFx[58] + tmpQ2[51]*tmpFx[67] + tmpQ2[52]*tmpFx[76] + tmpQ2[53]*tmpFx[85] + tmpQ2[54]*tmpFx[94];
tmpQ1[41] = + tmpQ2[44]*tmpFx[5] + tmpQ2[45]*tmpFx[14] + tmpQ2[46]*tmpFx[23] + tmpQ2[47]*tmpFx[32] + tmpQ2[48]*tmpFx[41] + tmpQ2[49]*tmpFx[50] + tmpQ2[50]*tmpFx[59] + tmpQ2[51]*tmpFx[68] + tmpQ2[52]*tmpFx[77] + tmpQ2[53]*tmpFx[86] + tmpQ2[54]*tmpFx[95];
tmpQ1[42] = + tmpQ2[44]*tmpFx[6] + tmpQ2[45]*tmpFx[15] + tmpQ2[46]*tmpFx[24] + tmpQ2[47]*tmpFx[33] + tmpQ2[48]*tmpFx[42] + tmpQ2[49]*tmpFx[51] + tmpQ2[50]*tmpFx[60] + tmpQ2[51]*tmpFx[69] + tmpQ2[52]*tmpFx[78] + tmpQ2[53]*tmpFx[87] + tmpQ2[54]*tmpFx[96];
tmpQ1[43] = + tmpQ2[44]*tmpFx[7] + tmpQ2[45]*tmpFx[16] + tmpQ2[46]*tmpFx[25] + tmpQ2[47]*tmpFx[34] + tmpQ2[48]*tmpFx[43] + tmpQ2[49]*tmpFx[52] + tmpQ2[50]*tmpFx[61] + tmpQ2[51]*tmpFx[70] + tmpQ2[52]*tmpFx[79] + tmpQ2[53]*tmpFx[88] + tmpQ2[54]*tmpFx[97];
tmpQ1[44] = + tmpQ2[44]*tmpFx[8] + tmpQ2[45]*tmpFx[17] + tmpQ2[46]*tmpFx[26] + tmpQ2[47]*tmpFx[35] + tmpQ2[48]*tmpFx[44] + tmpQ2[49]*tmpFx[53] + tmpQ2[50]*tmpFx[62] + tmpQ2[51]*tmpFx[71] + tmpQ2[52]*tmpFx[80] + tmpQ2[53]*tmpFx[89] + tmpQ2[54]*tmpFx[98];
tmpQ1[45] = + tmpQ2[55]*tmpFx[0] + tmpQ2[56]*tmpFx[9] + tmpQ2[57]*tmpFx[18] + tmpQ2[58]*tmpFx[27] + tmpQ2[59]*tmpFx[36] + tmpQ2[60]*tmpFx[45] + tmpQ2[61]*tmpFx[54] + tmpQ2[62]*tmpFx[63] + tmpQ2[63]*tmpFx[72] + tmpQ2[64]*tmpFx[81] + tmpQ2[65]*tmpFx[90];
tmpQ1[46] = + tmpQ2[55]*tmpFx[1] + tmpQ2[56]*tmpFx[10] + tmpQ2[57]*tmpFx[19] + tmpQ2[58]*tmpFx[28] + tmpQ2[59]*tmpFx[37] + tmpQ2[60]*tmpFx[46] + tmpQ2[61]*tmpFx[55] + tmpQ2[62]*tmpFx[64] + tmpQ2[63]*tmpFx[73] + tmpQ2[64]*tmpFx[82] + tmpQ2[65]*tmpFx[91];
tmpQ1[47] = + tmpQ2[55]*tmpFx[2] + tmpQ2[56]*tmpFx[11] + tmpQ2[57]*tmpFx[20] + tmpQ2[58]*tmpFx[29] + tmpQ2[59]*tmpFx[38] + tmpQ2[60]*tmpFx[47] + tmpQ2[61]*tmpFx[56] + tmpQ2[62]*tmpFx[65] + tmpQ2[63]*tmpFx[74] + tmpQ2[64]*tmpFx[83] + tmpQ2[65]*tmpFx[92];
tmpQ1[48] = + tmpQ2[55]*tmpFx[3] + tmpQ2[56]*tmpFx[12] + tmpQ2[57]*tmpFx[21] + tmpQ2[58]*tmpFx[30] + tmpQ2[59]*tmpFx[39] + tmpQ2[60]*tmpFx[48] + tmpQ2[61]*tmpFx[57] + tmpQ2[62]*tmpFx[66] + tmpQ2[63]*tmpFx[75] + tmpQ2[64]*tmpFx[84] + tmpQ2[65]*tmpFx[93];
tmpQ1[49] = + tmpQ2[55]*tmpFx[4] + tmpQ2[56]*tmpFx[13] + tmpQ2[57]*tmpFx[22] + tmpQ2[58]*tmpFx[31] + tmpQ2[59]*tmpFx[40] + tmpQ2[60]*tmpFx[49] + tmpQ2[61]*tmpFx[58] + tmpQ2[62]*tmpFx[67] + tmpQ2[63]*tmpFx[76] + tmpQ2[64]*tmpFx[85] + tmpQ2[65]*tmpFx[94];
tmpQ1[50] = + tmpQ2[55]*tmpFx[5] + tmpQ2[56]*tmpFx[14] + tmpQ2[57]*tmpFx[23] + tmpQ2[58]*tmpFx[32] + tmpQ2[59]*tmpFx[41] + tmpQ2[60]*tmpFx[50] + tmpQ2[61]*tmpFx[59] + tmpQ2[62]*tmpFx[68] + tmpQ2[63]*tmpFx[77] + tmpQ2[64]*tmpFx[86] + tmpQ2[65]*tmpFx[95];
tmpQ1[51] = + tmpQ2[55]*tmpFx[6] + tmpQ2[56]*tmpFx[15] + tmpQ2[57]*tmpFx[24] + tmpQ2[58]*tmpFx[33] + tmpQ2[59]*tmpFx[42] + tmpQ2[60]*tmpFx[51] + tmpQ2[61]*tmpFx[60] + tmpQ2[62]*tmpFx[69] + tmpQ2[63]*tmpFx[78] + tmpQ2[64]*tmpFx[87] + tmpQ2[65]*tmpFx[96];
tmpQ1[52] = + tmpQ2[55]*tmpFx[7] + tmpQ2[56]*tmpFx[16] + tmpQ2[57]*tmpFx[25] + tmpQ2[58]*tmpFx[34] + tmpQ2[59]*tmpFx[43] + tmpQ2[60]*tmpFx[52] + tmpQ2[61]*tmpFx[61] + tmpQ2[62]*tmpFx[70] + tmpQ2[63]*tmpFx[79] + tmpQ2[64]*tmpFx[88] + tmpQ2[65]*tmpFx[97];
tmpQ1[53] = + tmpQ2[55]*tmpFx[8] + tmpQ2[56]*tmpFx[17] + tmpQ2[57]*tmpFx[26] + tmpQ2[58]*tmpFx[35] + tmpQ2[59]*tmpFx[44] + tmpQ2[60]*tmpFx[53] + tmpQ2[61]*tmpFx[62] + tmpQ2[62]*tmpFx[71] + tmpQ2[63]*tmpFx[80] + tmpQ2[64]*tmpFx[89] + tmpQ2[65]*tmpFx[98];
tmpQ1[54] = + tmpQ2[66]*tmpFx[0] + tmpQ2[67]*tmpFx[9] + tmpQ2[68]*tmpFx[18] + tmpQ2[69]*tmpFx[27] + tmpQ2[70]*tmpFx[36] + tmpQ2[71]*tmpFx[45] + tmpQ2[72]*tmpFx[54] + tmpQ2[73]*tmpFx[63] + tmpQ2[74]*tmpFx[72] + tmpQ2[75]*tmpFx[81] + tmpQ2[76]*tmpFx[90];
tmpQ1[55] = + tmpQ2[66]*tmpFx[1] + tmpQ2[67]*tmpFx[10] + tmpQ2[68]*tmpFx[19] + tmpQ2[69]*tmpFx[28] + tmpQ2[70]*tmpFx[37] + tmpQ2[71]*tmpFx[46] + tmpQ2[72]*tmpFx[55] + tmpQ2[73]*tmpFx[64] + tmpQ2[74]*tmpFx[73] + tmpQ2[75]*tmpFx[82] + tmpQ2[76]*tmpFx[91];
tmpQ1[56] = + tmpQ2[66]*tmpFx[2] + tmpQ2[67]*tmpFx[11] + tmpQ2[68]*tmpFx[20] + tmpQ2[69]*tmpFx[29] + tmpQ2[70]*tmpFx[38] + tmpQ2[71]*tmpFx[47] + tmpQ2[72]*tmpFx[56] + tmpQ2[73]*tmpFx[65] + tmpQ2[74]*tmpFx[74] + tmpQ2[75]*tmpFx[83] + tmpQ2[76]*tmpFx[92];
tmpQ1[57] = + tmpQ2[66]*tmpFx[3] + tmpQ2[67]*tmpFx[12] + tmpQ2[68]*tmpFx[21] + tmpQ2[69]*tmpFx[30] + tmpQ2[70]*tmpFx[39] + tmpQ2[71]*tmpFx[48] + tmpQ2[72]*tmpFx[57] + tmpQ2[73]*tmpFx[66] + tmpQ2[74]*tmpFx[75] + tmpQ2[75]*tmpFx[84] + tmpQ2[76]*tmpFx[93];
tmpQ1[58] = + tmpQ2[66]*tmpFx[4] + tmpQ2[67]*tmpFx[13] + tmpQ2[68]*tmpFx[22] + tmpQ2[69]*tmpFx[31] + tmpQ2[70]*tmpFx[40] + tmpQ2[71]*tmpFx[49] + tmpQ2[72]*tmpFx[58] + tmpQ2[73]*tmpFx[67] + tmpQ2[74]*tmpFx[76] + tmpQ2[75]*tmpFx[85] + tmpQ2[76]*tmpFx[94];
tmpQ1[59] = + tmpQ2[66]*tmpFx[5] + tmpQ2[67]*tmpFx[14] + tmpQ2[68]*tmpFx[23] + tmpQ2[69]*tmpFx[32] + tmpQ2[70]*tmpFx[41] + tmpQ2[71]*tmpFx[50] + tmpQ2[72]*tmpFx[59] + tmpQ2[73]*tmpFx[68] + tmpQ2[74]*tmpFx[77] + tmpQ2[75]*tmpFx[86] + tmpQ2[76]*tmpFx[95];
tmpQ1[60] = + tmpQ2[66]*tmpFx[6] + tmpQ2[67]*tmpFx[15] + tmpQ2[68]*tmpFx[24] + tmpQ2[69]*tmpFx[33] + tmpQ2[70]*tmpFx[42] + tmpQ2[71]*tmpFx[51] + tmpQ2[72]*tmpFx[60] + tmpQ2[73]*tmpFx[69] + tmpQ2[74]*tmpFx[78] + tmpQ2[75]*tmpFx[87] + tmpQ2[76]*tmpFx[96];
tmpQ1[61] = + tmpQ2[66]*tmpFx[7] + tmpQ2[67]*tmpFx[16] + tmpQ2[68]*tmpFx[25] + tmpQ2[69]*tmpFx[34] + tmpQ2[70]*tmpFx[43] + tmpQ2[71]*tmpFx[52] + tmpQ2[72]*tmpFx[61] + tmpQ2[73]*tmpFx[70] + tmpQ2[74]*tmpFx[79] + tmpQ2[75]*tmpFx[88] + tmpQ2[76]*tmpFx[97];
tmpQ1[62] = + tmpQ2[66]*tmpFx[8] + tmpQ2[67]*tmpFx[17] + tmpQ2[68]*tmpFx[26] + tmpQ2[69]*tmpFx[35] + tmpQ2[70]*tmpFx[44] + tmpQ2[71]*tmpFx[53] + tmpQ2[72]*tmpFx[62] + tmpQ2[73]*tmpFx[71] + tmpQ2[74]*tmpFx[80] + tmpQ2[75]*tmpFx[89] + tmpQ2[76]*tmpFx[98];
tmpQ1[63] = + tmpQ2[77]*tmpFx[0] + tmpQ2[78]*tmpFx[9] + tmpQ2[79]*tmpFx[18] + tmpQ2[80]*tmpFx[27] + tmpQ2[81]*tmpFx[36] + tmpQ2[82]*tmpFx[45] + tmpQ2[83]*tmpFx[54] + tmpQ2[84]*tmpFx[63] + tmpQ2[85]*tmpFx[72] + tmpQ2[86]*tmpFx[81] + tmpQ2[87]*tmpFx[90];
tmpQ1[64] = + tmpQ2[77]*tmpFx[1] + tmpQ2[78]*tmpFx[10] + tmpQ2[79]*tmpFx[19] + tmpQ2[80]*tmpFx[28] + tmpQ2[81]*tmpFx[37] + tmpQ2[82]*tmpFx[46] + tmpQ2[83]*tmpFx[55] + tmpQ2[84]*tmpFx[64] + tmpQ2[85]*tmpFx[73] + tmpQ2[86]*tmpFx[82] + tmpQ2[87]*tmpFx[91];
tmpQ1[65] = + tmpQ2[77]*tmpFx[2] + tmpQ2[78]*tmpFx[11] + tmpQ2[79]*tmpFx[20] + tmpQ2[80]*tmpFx[29] + tmpQ2[81]*tmpFx[38] + tmpQ2[82]*tmpFx[47] + tmpQ2[83]*tmpFx[56] + tmpQ2[84]*tmpFx[65] + tmpQ2[85]*tmpFx[74] + tmpQ2[86]*tmpFx[83] + tmpQ2[87]*tmpFx[92];
tmpQ1[66] = + tmpQ2[77]*tmpFx[3] + tmpQ2[78]*tmpFx[12] + tmpQ2[79]*tmpFx[21] + tmpQ2[80]*tmpFx[30] + tmpQ2[81]*tmpFx[39] + tmpQ2[82]*tmpFx[48] + tmpQ2[83]*tmpFx[57] + tmpQ2[84]*tmpFx[66] + tmpQ2[85]*tmpFx[75] + tmpQ2[86]*tmpFx[84] + tmpQ2[87]*tmpFx[93];
tmpQ1[67] = + tmpQ2[77]*tmpFx[4] + tmpQ2[78]*tmpFx[13] + tmpQ2[79]*tmpFx[22] + tmpQ2[80]*tmpFx[31] + tmpQ2[81]*tmpFx[40] + tmpQ2[82]*tmpFx[49] + tmpQ2[83]*tmpFx[58] + tmpQ2[84]*tmpFx[67] + tmpQ2[85]*tmpFx[76] + tmpQ2[86]*tmpFx[85] + tmpQ2[87]*tmpFx[94];
tmpQ1[68] = + tmpQ2[77]*tmpFx[5] + tmpQ2[78]*tmpFx[14] + tmpQ2[79]*tmpFx[23] + tmpQ2[80]*tmpFx[32] + tmpQ2[81]*tmpFx[41] + tmpQ2[82]*tmpFx[50] + tmpQ2[83]*tmpFx[59] + tmpQ2[84]*tmpFx[68] + tmpQ2[85]*tmpFx[77] + tmpQ2[86]*tmpFx[86] + tmpQ2[87]*tmpFx[95];
tmpQ1[69] = + tmpQ2[77]*tmpFx[6] + tmpQ2[78]*tmpFx[15] + tmpQ2[79]*tmpFx[24] + tmpQ2[80]*tmpFx[33] + tmpQ2[81]*tmpFx[42] + tmpQ2[82]*tmpFx[51] + tmpQ2[83]*tmpFx[60] + tmpQ2[84]*tmpFx[69] + tmpQ2[85]*tmpFx[78] + tmpQ2[86]*tmpFx[87] + tmpQ2[87]*tmpFx[96];
tmpQ1[70] = + tmpQ2[77]*tmpFx[7] + tmpQ2[78]*tmpFx[16] + tmpQ2[79]*tmpFx[25] + tmpQ2[80]*tmpFx[34] + tmpQ2[81]*tmpFx[43] + tmpQ2[82]*tmpFx[52] + tmpQ2[83]*tmpFx[61] + tmpQ2[84]*tmpFx[70] + tmpQ2[85]*tmpFx[79] + tmpQ2[86]*tmpFx[88] + tmpQ2[87]*tmpFx[97];
tmpQ1[71] = + tmpQ2[77]*tmpFx[8] + tmpQ2[78]*tmpFx[17] + tmpQ2[79]*tmpFx[26] + tmpQ2[80]*tmpFx[35] + tmpQ2[81]*tmpFx[44] + tmpQ2[82]*tmpFx[53] + tmpQ2[83]*tmpFx[62] + tmpQ2[84]*tmpFx[71] + tmpQ2[85]*tmpFx[80] + tmpQ2[86]*tmpFx[89] + tmpQ2[87]*tmpFx[98];
tmpQ1[72] = + tmpQ2[88]*tmpFx[0] + tmpQ2[89]*tmpFx[9] + tmpQ2[90]*tmpFx[18] + tmpQ2[91]*tmpFx[27] + tmpQ2[92]*tmpFx[36] + tmpQ2[93]*tmpFx[45] + tmpQ2[94]*tmpFx[54] + tmpQ2[95]*tmpFx[63] + tmpQ2[96]*tmpFx[72] + tmpQ2[97]*tmpFx[81] + tmpQ2[98]*tmpFx[90];
tmpQ1[73] = + tmpQ2[88]*tmpFx[1] + tmpQ2[89]*tmpFx[10] + tmpQ2[90]*tmpFx[19] + tmpQ2[91]*tmpFx[28] + tmpQ2[92]*tmpFx[37] + tmpQ2[93]*tmpFx[46] + tmpQ2[94]*tmpFx[55] + tmpQ2[95]*tmpFx[64] + tmpQ2[96]*tmpFx[73] + tmpQ2[97]*tmpFx[82] + tmpQ2[98]*tmpFx[91];
tmpQ1[74] = + tmpQ2[88]*tmpFx[2] + tmpQ2[89]*tmpFx[11] + tmpQ2[90]*tmpFx[20] + tmpQ2[91]*tmpFx[29] + tmpQ2[92]*tmpFx[38] + tmpQ2[93]*tmpFx[47] + tmpQ2[94]*tmpFx[56] + tmpQ2[95]*tmpFx[65] + tmpQ2[96]*tmpFx[74] + tmpQ2[97]*tmpFx[83] + tmpQ2[98]*tmpFx[92];
tmpQ1[75] = + tmpQ2[88]*tmpFx[3] + tmpQ2[89]*tmpFx[12] + tmpQ2[90]*tmpFx[21] + tmpQ2[91]*tmpFx[30] + tmpQ2[92]*tmpFx[39] + tmpQ2[93]*tmpFx[48] + tmpQ2[94]*tmpFx[57] + tmpQ2[95]*tmpFx[66] + tmpQ2[96]*tmpFx[75] + tmpQ2[97]*tmpFx[84] + tmpQ2[98]*tmpFx[93];
tmpQ1[76] = + tmpQ2[88]*tmpFx[4] + tmpQ2[89]*tmpFx[13] + tmpQ2[90]*tmpFx[22] + tmpQ2[91]*tmpFx[31] + tmpQ2[92]*tmpFx[40] + tmpQ2[93]*tmpFx[49] + tmpQ2[94]*tmpFx[58] + tmpQ2[95]*tmpFx[67] + tmpQ2[96]*tmpFx[76] + tmpQ2[97]*tmpFx[85] + tmpQ2[98]*tmpFx[94];
tmpQ1[77] = + tmpQ2[88]*tmpFx[5] + tmpQ2[89]*tmpFx[14] + tmpQ2[90]*tmpFx[23] + tmpQ2[91]*tmpFx[32] + tmpQ2[92]*tmpFx[41] + tmpQ2[93]*tmpFx[50] + tmpQ2[94]*tmpFx[59] + tmpQ2[95]*tmpFx[68] + tmpQ2[96]*tmpFx[77] + tmpQ2[97]*tmpFx[86] + tmpQ2[98]*tmpFx[95];
tmpQ1[78] = + tmpQ2[88]*tmpFx[6] + tmpQ2[89]*tmpFx[15] + tmpQ2[90]*tmpFx[24] + tmpQ2[91]*tmpFx[33] + tmpQ2[92]*tmpFx[42] + tmpQ2[93]*tmpFx[51] + tmpQ2[94]*tmpFx[60] + tmpQ2[95]*tmpFx[69] + tmpQ2[96]*tmpFx[78] + tmpQ2[97]*tmpFx[87] + tmpQ2[98]*tmpFx[96];
tmpQ1[79] = + tmpQ2[88]*tmpFx[7] + tmpQ2[89]*tmpFx[16] + tmpQ2[90]*tmpFx[25] + tmpQ2[91]*tmpFx[34] + tmpQ2[92]*tmpFx[43] + tmpQ2[93]*tmpFx[52] + tmpQ2[94]*tmpFx[61] + tmpQ2[95]*tmpFx[70] + tmpQ2[96]*tmpFx[79] + tmpQ2[97]*tmpFx[88] + tmpQ2[98]*tmpFx[97];
tmpQ1[80] = + tmpQ2[88]*tmpFx[8] + tmpQ2[89]*tmpFx[17] + tmpQ2[90]*tmpFx[26] + tmpQ2[91]*tmpFx[35] + tmpQ2[92]*tmpFx[44] + tmpQ2[93]*tmpFx[53] + tmpQ2[94]*tmpFx[62] + tmpQ2[95]*tmpFx[71] + tmpQ2[96]*tmpFx[80] + tmpQ2[97]*tmpFx[89] + tmpQ2[98]*tmpFx[98];
}

void acado_setObjR1R2( real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = + tmpFu[0]*tmpObjS[0] + tmpFu[3]*tmpObjS[11] + tmpFu[6]*tmpObjS[22] + tmpFu[9]*tmpObjS[33] + tmpFu[12]*tmpObjS[44] + tmpFu[15]*tmpObjS[55] + tmpFu[18]*tmpObjS[66] + tmpFu[21]*tmpObjS[77] + tmpFu[24]*tmpObjS[88] + tmpFu[27]*tmpObjS[99] + tmpFu[30]*tmpObjS[110];
tmpR2[1] = + tmpFu[0]*tmpObjS[1] + tmpFu[3]*tmpObjS[12] + tmpFu[6]*tmpObjS[23] + tmpFu[9]*tmpObjS[34] + tmpFu[12]*tmpObjS[45] + tmpFu[15]*tmpObjS[56] + tmpFu[18]*tmpObjS[67] + tmpFu[21]*tmpObjS[78] + tmpFu[24]*tmpObjS[89] + tmpFu[27]*tmpObjS[100] + tmpFu[30]*tmpObjS[111];
tmpR2[2] = + tmpFu[0]*tmpObjS[2] + tmpFu[3]*tmpObjS[13] + tmpFu[6]*tmpObjS[24] + tmpFu[9]*tmpObjS[35] + tmpFu[12]*tmpObjS[46] + tmpFu[15]*tmpObjS[57] + tmpFu[18]*tmpObjS[68] + tmpFu[21]*tmpObjS[79] + tmpFu[24]*tmpObjS[90] + tmpFu[27]*tmpObjS[101] + tmpFu[30]*tmpObjS[112];
tmpR2[3] = + tmpFu[0]*tmpObjS[3] + tmpFu[3]*tmpObjS[14] + tmpFu[6]*tmpObjS[25] + tmpFu[9]*tmpObjS[36] + tmpFu[12]*tmpObjS[47] + tmpFu[15]*tmpObjS[58] + tmpFu[18]*tmpObjS[69] + tmpFu[21]*tmpObjS[80] + tmpFu[24]*tmpObjS[91] + tmpFu[27]*tmpObjS[102] + tmpFu[30]*tmpObjS[113];
tmpR2[4] = + tmpFu[0]*tmpObjS[4] + tmpFu[3]*tmpObjS[15] + tmpFu[6]*tmpObjS[26] + tmpFu[9]*tmpObjS[37] + tmpFu[12]*tmpObjS[48] + tmpFu[15]*tmpObjS[59] + tmpFu[18]*tmpObjS[70] + tmpFu[21]*tmpObjS[81] + tmpFu[24]*tmpObjS[92] + tmpFu[27]*tmpObjS[103] + tmpFu[30]*tmpObjS[114];
tmpR2[5] = + tmpFu[0]*tmpObjS[5] + tmpFu[3]*tmpObjS[16] + tmpFu[6]*tmpObjS[27] + tmpFu[9]*tmpObjS[38] + tmpFu[12]*tmpObjS[49] + tmpFu[15]*tmpObjS[60] + tmpFu[18]*tmpObjS[71] + tmpFu[21]*tmpObjS[82] + tmpFu[24]*tmpObjS[93] + tmpFu[27]*tmpObjS[104] + tmpFu[30]*tmpObjS[115];
tmpR2[6] = + tmpFu[0]*tmpObjS[6] + tmpFu[3]*tmpObjS[17] + tmpFu[6]*tmpObjS[28] + tmpFu[9]*tmpObjS[39] + tmpFu[12]*tmpObjS[50] + tmpFu[15]*tmpObjS[61] + tmpFu[18]*tmpObjS[72] + tmpFu[21]*tmpObjS[83] + tmpFu[24]*tmpObjS[94] + tmpFu[27]*tmpObjS[105] + tmpFu[30]*tmpObjS[116];
tmpR2[7] = + tmpFu[0]*tmpObjS[7] + tmpFu[3]*tmpObjS[18] + tmpFu[6]*tmpObjS[29] + tmpFu[9]*tmpObjS[40] + tmpFu[12]*tmpObjS[51] + tmpFu[15]*tmpObjS[62] + tmpFu[18]*tmpObjS[73] + tmpFu[21]*tmpObjS[84] + tmpFu[24]*tmpObjS[95] + tmpFu[27]*tmpObjS[106] + tmpFu[30]*tmpObjS[117];
tmpR2[8] = + tmpFu[0]*tmpObjS[8] + tmpFu[3]*tmpObjS[19] + tmpFu[6]*tmpObjS[30] + tmpFu[9]*tmpObjS[41] + tmpFu[12]*tmpObjS[52] + tmpFu[15]*tmpObjS[63] + tmpFu[18]*tmpObjS[74] + tmpFu[21]*tmpObjS[85] + tmpFu[24]*tmpObjS[96] + tmpFu[27]*tmpObjS[107] + tmpFu[30]*tmpObjS[118];
tmpR2[9] = + tmpFu[0]*tmpObjS[9] + tmpFu[3]*tmpObjS[20] + tmpFu[6]*tmpObjS[31] + tmpFu[9]*tmpObjS[42] + tmpFu[12]*tmpObjS[53] + tmpFu[15]*tmpObjS[64] + tmpFu[18]*tmpObjS[75] + tmpFu[21]*tmpObjS[86] + tmpFu[24]*tmpObjS[97] + tmpFu[27]*tmpObjS[108] + tmpFu[30]*tmpObjS[119];
tmpR2[10] = + tmpFu[0]*tmpObjS[10] + tmpFu[3]*tmpObjS[21] + tmpFu[6]*tmpObjS[32] + tmpFu[9]*tmpObjS[43] + tmpFu[12]*tmpObjS[54] + tmpFu[15]*tmpObjS[65] + tmpFu[18]*tmpObjS[76] + tmpFu[21]*tmpObjS[87] + tmpFu[24]*tmpObjS[98] + tmpFu[27]*tmpObjS[109] + tmpFu[30]*tmpObjS[120];
tmpR2[11] = + tmpFu[1]*tmpObjS[0] + tmpFu[4]*tmpObjS[11] + tmpFu[7]*tmpObjS[22] + tmpFu[10]*tmpObjS[33] + tmpFu[13]*tmpObjS[44] + tmpFu[16]*tmpObjS[55] + tmpFu[19]*tmpObjS[66] + tmpFu[22]*tmpObjS[77] + tmpFu[25]*tmpObjS[88] + tmpFu[28]*tmpObjS[99] + tmpFu[31]*tmpObjS[110];
tmpR2[12] = + tmpFu[1]*tmpObjS[1] + tmpFu[4]*tmpObjS[12] + tmpFu[7]*tmpObjS[23] + tmpFu[10]*tmpObjS[34] + tmpFu[13]*tmpObjS[45] + tmpFu[16]*tmpObjS[56] + tmpFu[19]*tmpObjS[67] + tmpFu[22]*tmpObjS[78] + tmpFu[25]*tmpObjS[89] + tmpFu[28]*tmpObjS[100] + tmpFu[31]*tmpObjS[111];
tmpR2[13] = + tmpFu[1]*tmpObjS[2] + tmpFu[4]*tmpObjS[13] + tmpFu[7]*tmpObjS[24] + tmpFu[10]*tmpObjS[35] + tmpFu[13]*tmpObjS[46] + tmpFu[16]*tmpObjS[57] + tmpFu[19]*tmpObjS[68] + tmpFu[22]*tmpObjS[79] + tmpFu[25]*tmpObjS[90] + tmpFu[28]*tmpObjS[101] + tmpFu[31]*tmpObjS[112];
tmpR2[14] = + tmpFu[1]*tmpObjS[3] + tmpFu[4]*tmpObjS[14] + tmpFu[7]*tmpObjS[25] + tmpFu[10]*tmpObjS[36] + tmpFu[13]*tmpObjS[47] + tmpFu[16]*tmpObjS[58] + tmpFu[19]*tmpObjS[69] + tmpFu[22]*tmpObjS[80] + tmpFu[25]*tmpObjS[91] + tmpFu[28]*tmpObjS[102] + tmpFu[31]*tmpObjS[113];
tmpR2[15] = + tmpFu[1]*tmpObjS[4] + tmpFu[4]*tmpObjS[15] + tmpFu[7]*tmpObjS[26] + tmpFu[10]*tmpObjS[37] + tmpFu[13]*tmpObjS[48] + tmpFu[16]*tmpObjS[59] + tmpFu[19]*tmpObjS[70] + tmpFu[22]*tmpObjS[81] + tmpFu[25]*tmpObjS[92] + tmpFu[28]*tmpObjS[103] + tmpFu[31]*tmpObjS[114];
tmpR2[16] = + tmpFu[1]*tmpObjS[5] + tmpFu[4]*tmpObjS[16] + tmpFu[7]*tmpObjS[27] + tmpFu[10]*tmpObjS[38] + tmpFu[13]*tmpObjS[49] + tmpFu[16]*tmpObjS[60] + tmpFu[19]*tmpObjS[71] + tmpFu[22]*tmpObjS[82] + tmpFu[25]*tmpObjS[93] + tmpFu[28]*tmpObjS[104] + tmpFu[31]*tmpObjS[115];
tmpR2[17] = + tmpFu[1]*tmpObjS[6] + tmpFu[4]*tmpObjS[17] + tmpFu[7]*tmpObjS[28] + tmpFu[10]*tmpObjS[39] + tmpFu[13]*tmpObjS[50] + tmpFu[16]*tmpObjS[61] + tmpFu[19]*tmpObjS[72] + tmpFu[22]*tmpObjS[83] + tmpFu[25]*tmpObjS[94] + tmpFu[28]*tmpObjS[105] + tmpFu[31]*tmpObjS[116];
tmpR2[18] = + tmpFu[1]*tmpObjS[7] + tmpFu[4]*tmpObjS[18] + tmpFu[7]*tmpObjS[29] + tmpFu[10]*tmpObjS[40] + tmpFu[13]*tmpObjS[51] + tmpFu[16]*tmpObjS[62] + tmpFu[19]*tmpObjS[73] + tmpFu[22]*tmpObjS[84] + tmpFu[25]*tmpObjS[95] + tmpFu[28]*tmpObjS[106] + tmpFu[31]*tmpObjS[117];
tmpR2[19] = + tmpFu[1]*tmpObjS[8] + tmpFu[4]*tmpObjS[19] + tmpFu[7]*tmpObjS[30] + tmpFu[10]*tmpObjS[41] + tmpFu[13]*tmpObjS[52] + tmpFu[16]*tmpObjS[63] + tmpFu[19]*tmpObjS[74] + tmpFu[22]*tmpObjS[85] + tmpFu[25]*tmpObjS[96] + tmpFu[28]*tmpObjS[107] + tmpFu[31]*tmpObjS[118];
tmpR2[20] = + tmpFu[1]*tmpObjS[9] + tmpFu[4]*tmpObjS[20] + tmpFu[7]*tmpObjS[31] + tmpFu[10]*tmpObjS[42] + tmpFu[13]*tmpObjS[53] + tmpFu[16]*tmpObjS[64] + tmpFu[19]*tmpObjS[75] + tmpFu[22]*tmpObjS[86] + tmpFu[25]*tmpObjS[97] + tmpFu[28]*tmpObjS[108] + tmpFu[31]*tmpObjS[119];
tmpR2[21] = + tmpFu[1]*tmpObjS[10] + tmpFu[4]*tmpObjS[21] + tmpFu[7]*tmpObjS[32] + tmpFu[10]*tmpObjS[43] + tmpFu[13]*tmpObjS[54] + tmpFu[16]*tmpObjS[65] + tmpFu[19]*tmpObjS[76] + tmpFu[22]*tmpObjS[87] + tmpFu[25]*tmpObjS[98] + tmpFu[28]*tmpObjS[109] + tmpFu[31]*tmpObjS[120];
tmpR2[22] = + tmpFu[2]*tmpObjS[0] + tmpFu[5]*tmpObjS[11] + tmpFu[8]*tmpObjS[22] + tmpFu[11]*tmpObjS[33] + tmpFu[14]*tmpObjS[44] + tmpFu[17]*tmpObjS[55] + tmpFu[20]*tmpObjS[66] + tmpFu[23]*tmpObjS[77] + tmpFu[26]*tmpObjS[88] + tmpFu[29]*tmpObjS[99] + tmpFu[32]*tmpObjS[110];
tmpR2[23] = + tmpFu[2]*tmpObjS[1] + tmpFu[5]*tmpObjS[12] + tmpFu[8]*tmpObjS[23] + tmpFu[11]*tmpObjS[34] + tmpFu[14]*tmpObjS[45] + tmpFu[17]*tmpObjS[56] + tmpFu[20]*tmpObjS[67] + tmpFu[23]*tmpObjS[78] + tmpFu[26]*tmpObjS[89] + tmpFu[29]*tmpObjS[100] + tmpFu[32]*tmpObjS[111];
tmpR2[24] = + tmpFu[2]*tmpObjS[2] + tmpFu[5]*tmpObjS[13] + tmpFu[8]*tmpObjS[24] + tmpFu[11]*tmpObjS[35] + tmpFu[14]*tmpObjS[46] + tmpFu[17]*tmpObjS[57] + tmpFu[20]*tmpObjS[68] + tmpFu[23]*tmpObjS[79] + tmpFu[26]*tmpObjS[90] + tmpFu[29]*tmpObjS[101] + tmpFu[32]*tmpObjS[112];
tmpR2[25] = + tmpFu[2]*tmpObjS[3] + tmpFu[5]*tmpObjS[14] + tmpFu[8]*tmpObjS[25] + tmpFu[11]*tmpObjS[36] + tmpFu[14]*tmpObjS[47] + tmpFu[17]*tmpObjS[58] + tmpFu[20]*tmpObjS[69] + tmpFu[23]*tmpObjS[80] + tmpFu[26]*tmpObjS[91] + tmpFu[29]*tmpObjS[102] + tmpFu[32]*tmpObjS[113];
tmpR2[26] = + tmpFu[2]*tmpObjS[4] + tmpFu[5]*tmpObjS[15] + tmpFu[8]*tmpObjS[26] + tmpFu[11]*tmpObjS[37] + tmpFu[14]*tmpObjS[48] + tmpFu[17]*tmpObjS[59] + tmpFu[20]*tmpObjS[70] + tmpFu[23]*tmpObjS[81] + tmpFu[26]*tmpObjS[92] + tmpFu[29]*tmpObjS[103] + tmpFu[32]*tmpObjS[114];
tmpR2[27] = + tmpFu[2]*tmpObjS[5] + tmpFu[5]*tmpObjS[16] + tmpFu[8]*tmpObjS[27] + tmpFu[11]*tmpObjS[38] + tmpFu[14]*tmpObjS[49] + tmpFu[17]*tmpObjS[60] + tmpFu[20]*tmpObjS[71] + tmpFu[23]*tmpObjS[82] + tmpFu[26]*tmpObjS[93] + tmpFu[29]*tmpObjS[104] + tmpFu[32]*tmpObjS[115];
tmpR2[28] = + tmpFu[2]*tmpObjS[6] + tmpFu[5]*tmpObjS[17] + tmpFu[8]*tmpObjS[28] + tmpFu[11]*tmpObjS[39] + tmpFu[14]*tmpObjS[50] + tmpFu[17]*tmpObjS[61] + tmpFu[20]*tmpObjS[72] + tmpFu[23]*tmpObjS[83] + tmpFu[26]*tmpObjS[94] + tmpFu[29]*tmpObjS[105] + tmpFu[32]*tmpObjS[116];
tmpR2[29] = + tmpFu[2]*tmpObjS[7] + tmpFu[5]*tmpObjS[18] + tmpFu[8]*tmpObjS[29] + tmpFu[11]*tmpObjS[40] + tmpFu[14]*tmpObjS[51] + tmpFu[17]*tmpObjS[62] + tmpFu[20]*tmpObjS[73] + tmpFu[23]*tmpObjS[84] + tmpFu[26]*tmpObjS[95] + tmpFu[29]*tmpObjS[106] + tmpFu[32]*tmpObjS[117];
tmpR2[30] = + tmpFu[2]*tmpObjS[8] + tmpFu[5]*tmpObjS[19] + tmpFu[8]*tmpObjS[30] + tmpFu[11]*tmpObjS[41] + tmpFu[14]*tmpObjS[52] + tmpFu[17]*tmpObjS[63] + tmpFu[20]*tmpObjS[74] + tmpFu[23]*tmpObjS[85] + tmpFu[26]*tmpObjS[96] + tmpFu[29]*tmpObjS[107] + tmpFu[32]*tmpObjS[118];
tmpR2[31] = + tmpFu[2]*tmpObjS[9] + tmpFu[5]*tmpObjS[20] + tmpFu[8]*tmpObjS[31] + tmpFu[11]*tmpObjS[42] + tmpFu[14]*tmpObjS[53] + tmpFu[17]*tmpObjS[64] + tmpFu[20]*tmpObjS[75] + tmpFu[23]*tmpObjS[86] + tmpFu[26]*tmpObjS[97] + tmpFu[29]*tmpObjS[108] + tmpFu[32]*tmpObjS[119];
tmpR2[32] = + tmpFu[2]*tmpObjS[10] + tmpFu[5]*tmpObjS[21] + tmpFu[8]*tmpObjS[32] + tmpFu[11]*tmpObjS[43] + tmpFu[14]*tmpObjS[54] + tmpFu[17]*tmpObjS[65] + tmpFu[20]*tmpObjS[76] + tmpFu[23]*tmpObjS[87] + tmpFu[26]*tmpObjS[98] + tmpFu[29]*tmpObjS[109] + tmpFu[32]*tmpObjS[120];
tmpR1[0] = + tmpR2[0]*tmpFu[0] + tmpR2[1]*tmpFu[3] + tmpR2[2]*tmpFu[6] + tmpR2[3]*tmpFu[9] + tmpR2[4]*tmpFu[12] + tmpR2[5]*tmpFu[15] + tmpR2[6]*tmpFu[18] + tmpR2[7]*tmpFu[21] + tmpR2[8]*tmpFu[24] + tmpR2[9]*tmpFu[27] + tmpR2[10]*tmpFu[30];
tmpR1[1] = + tmpR2[0]*tmpFu[1] + tmpR2[1]*tmpFu[4] + tmpR2[2]*tmpFu[7] + tmpR2[3]*tmpFu[10] + tmpR2[4]*tmpFu[13] + tmpR2[5]*tmpFu[16] + tmpR2[6]*tmpFu[19] + tmpR2[7]*tmpFu[22] + tmpR2[8]*tmpFu[25] + tmpR2[9]*tmpFu[28] + tmpR2[10]*tmpFu[31];
tmpR1[2] = + tmpR2[0]*tmpFu[2] + tmpR2[1]*tmpFu[5] + tmpR2[2]*tmpFu[8] + tmpR2[3]*tmpFu[11] + tmpR2[4]*tmpFu[14] + tmpR2[5]*tmpFu[17] + tmpR2[6]*tmpFu[20] + tmpR2[7]*tmpFu[23] + tmpR2[8]*tmpFu[26] + tmpR2[9]*tmpFu[29] + tmpR2[10]*tmpFu[32];
tmpR1[3] = + tmpR2[11]*tmpFu[0] + tmpR2[12]*tmpFu[3] + tmpR2[13]*tmpFu[6] + tmpR2[14]*tmpFu[9] + tmpR2[15]*tmpFu[12] + tmpR2[16]*tmpFu[15] + tmpR2[17]*tmpFu[18] + tmpR2[18]*tmpFu[21] + tmpR2[19]*tmpFu[24] + tmpR2[20]*tmpFu[27] + tmpR2[21]*tmpFu[30];
tmpR1[4] = + tmpR2[11]*tmpFu[1] + tmpR2[12]*tmpFu[4] + tmpR2[13]*tmpFu[7] + tmpR2[14]*tmpFu[10] + tmpR2[15]*tmpFu[13] + tmpR2[16]*tmpFu[16] + tmpR2[17]*tmpFu[19] + tmpR2[18]*tmpFu[22] + tmpR2[19]*tmpFu[25] + tmpR2[20]*tmpFu[28] + tmpR2[21]*tmpFu[31];
tmpR1[5] = + tmpR2[11]*tmpFu[2] + tmpR2[12]*tmpFu[5] + tmpR2[13]*tmpFu[8] + tmpR2[14]*tmpFu[11] + tmpR2[15]*tmpFu[14] + tmpR2[16]*tmpFu[17] + tmpR2[17]*tmpFu[20] + tmpR2[18]*tmpFu[23] + tmpR2[19]*tmpFu[26] + tmpR2[20]*tmpFu[29] + tmpR2[21]*tmpFu[32];
tmpR1[6] = + tmpR2[22]*tmpFu[0] + tmpR2[23]*tmpFu[3] + tmpR2[24]*tmpFu[6] + tmpR2[25]*tmpFu[9] + tmpR2[26]*tmpFu[12] + tmpR2[27]*tmpFu[15] + tmpR2[28]*tmpFu[18] + tmpR2[29]*tmpFu[21] + tmpR2[30]*tmpFu[24] + tmpR2[31]*tmpFu[27] + tmpR2[32]*tmpFu[30];
tmpR1[7] = + tmpR2[22]*tmpFu[1] + tmpR2[23]*tmpFu[4] + tmpR2[24]*tmpFu[7] + tmpR2[25]*tmpFu[10] + tmpR2[26]*tmpFu[13] + tmpR2[27]*tmpFu[16] + tmpR2[28]*tmpFu[19] + tmpR2[29]*tmpFu[22] + tmpR2[30]*tmpFu[25] + tmpR2[31]*tmpFu[28] + tmpR2[32]*tmpFu[31];
tmpR1[8] = + tmpR2[22]*tmpFu[2] + tmpR2[23]*tmpFu[5] + tmpR2[24]*tmpFu[8] + tmpR2[25]*tmpFu[11] + tmpR2[26]*tmpFu[14] + tmpR2[27]*tmpFu[17] + tmpR2[28]*tmpFu[20] + tmpR2[29]*tmpFu[23] + tmpR2[30]*tmpFu[26] + tmpR2[31]*tmpFu[29] + tmpR2[32]*tmpFu[32];
}

void acado_setObjS1( real_t* const tmpFx, real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpS1 )
{
/** Matrix of size: 9 x 11 (row major format) */
real_t tmpS2[ 99 ];

tmpS2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[9]*tmpObjS[11] + tmpFx[18]*tmpObjS[22] + tmpFx[27]*tmpObjS[33] + tmpFx[36]*tmpObjS[44] + tmpFx[45]*tmpObjS[55] + tmpFx[54]*tmpObjS[66] + tmpFx[63]*tmpObjS[77] + tmpFx[72]*tmpObjS[88] + tmpFx[81]*tmpObjS[99] + tmpFx[90]*tmpObjS[110];
tmpS2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[9]*tmpObjS[12] + tmpFx[18]*tmpObjS[23] + tmpFx[27]*tmpObjS[34] + tmpFx[36]*tmpObjS[45] + tmpFx[45]*tmpObjS[56] + tmpFx[54]*tmpObjS[67] + tmpFx[63]*tmpObjS[78] + tmpFx[72]*tmpObjS[89] + tmpFx[81]*tmpObjS[100] + tmpFx[90]*tmpObjS[111];
tmpS2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[9]*tmpObjS[13] + tmpFx[18]*tmpObjS[24] + tmpFx[27]*tmpObjS[35] + tmpFx[36]*tmpObjS[46] + tmpFx[45]*tmpObjS[57] + tmpFx[54]*tmpObjS[68] + tmpFx[63]*tmpObjS[79] + tmpFx[72]*tmpObjS[90] + tmpFx[81]*tmpObjS[101] + tmpFx[90]*tmpObjS[112];
tmpS2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[9]*tmpObjS[14] + tmpFx[18]*tmpObjS[25] + tmpFx[27]*tmpObjS[36] + tmpFx[36]*tmpObjS[47] + tmpFx[45]*tmpObjS[58] + tmpFx[54]*tmpObjS[69] + tmpFx[63]*tmpObjS[80] + tmpFx[72]*tmpObjS[91] + tmpFx[81]*tmpObjS[102] + tmpFx[90]*tmpObjS[113];
tmpS2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[9]*tmpObjS[15] + tmpFx[18]*tmpObjS[26] + tmpFx[27]*tmpObjS[37] + tmpFx[36]*tmpObjS[48] + tmpFx[45]*tmpObjS[59] + tmpFx[54]*tmpObjS[70] + tmpFx[63]*tmpObjS[81] + tmpFx[72]*tmpObjS[92] + tmpFx[81]*tmpObjS[103] + tmpFx[90]*tmpObjS[114];
tmpS2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[9]*tmpObjS[16] + tmpFx[18]*tmpObjS[27] + tmpFx[27]*tmpObjS[38] + tmpFx[36]*tmpObjS[49] + tmpFx[45]*tmpObjS[60] + tmpFx[54]*tmpObjS[71] + tmpFx[63]*tmpObjS[82] + tmpFx[72]*tmpObjS[93] + tmpFx[81]*tmpObjS[104] + tmpFx[90]*tmpObjS[115];
tmpS2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[9]*tmpObjS[17] + tmpFx[18]*tmpObjS[28] + tmpFx[27]*tmpObjS[39] + tmpFx[36]*tmpObjS[50] + tmpFx[45]*tmpObjS[61] + tmpFx[54]*tmpObjS[72] + tmpFx[63]*tmpObjS[83] + tmpFx[72]*tmpObjS[94] + tmpFx[81]*tmpObjS[105] + tmpFx[90]*tmpObjS[116];
tmpS2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[9]*tmpObjS[18] + tmpFx[18]*tmpObjS[29] + tmpFx[27]*tmpObjS[40] + tmpFx[36]*tmpObjS[51] + tmpFx[45]*tmpObjS[62] + tmpFx[54]*tmpObjS[73] + tmpFx[63]*tmpObjS[84] + tmpFx[72]*tmpObjS[95] + tmpFx[81]*tmpObjS[106] + tmpFx[90]*tmpObjS[117];
tmpS2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[9]*tmpObjS[19] + tmpFx[18]*tmpObjS[30] + tmpFx[27]*tmpObjS[41] + tmpFx[36]*tmpObjS[52] + tmpFx[45]*tmpObjS[63] + tmpFx[54]*tmpObjS[74] + tmpFx[63]*tmpObjS[85] + tmpFx[72]*tmpObjS[96] + tmpFx[81]*tmpObjS[107] + tmpFx[90]*tmpObjS[118];
tmpS2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[9]*tmpObjS[20] + tmpFx[18]*tmpObjS[31] + tmpFx[27]*tmpObjS[42] + tmpFx[36]*tmpObjS[53] + tmpFx[45]*tmpObjS[64] + tmpFx[54]*tmpObjS[75] + tmpFx[63]*tmpObjS[86] + tmpFx[72]*tmpObjS[97] + tmpFx[81]*tmpObjS[108] + tmpFx[90]*tmpObjS[119];
tmpS2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[9]*tmpObjS[21] + tmpFx[18]*tmpObjS[32] + tmpFx[27]*tmpObjS[43] + tmpFx[36]*tmpObjS[54] + tmpFx[45]*tmpObjS[65] + tmpFx[54]*tmpObjS[76] + tmpFx[63]*tmpObjS[87] + tmpFx[72]*tmpObjS[98] + tmpFx[81]*tmpObjS[109] + tmpFx[90]*tmpObjS[120];
tmpS2[11] = + tmpFx[1]*tmpObjS[0] + tmpFx[10]*tmpObjS[11] + tmpFx[19]*tmpObjS[22] + tmpFx[28]*tmpObjS[33] + tmpFx[37]*tmpObjS[44] + tmpFx[46]*tmpObjS[55] + tmpFx[55]*tmpObjS[66] + tmpFx[64]*tmpObjS[77] + tmpFx[73]*tmpObjS[88] + tmpFx[82]*tmpObjS[99] + tmpFx[91]*tmpObjS[110];
tmpS2[12] = + tmpFx[1]*tmpObjS[1] + tmpFx[10]*tmpObjS[12] + tmpFx[19]*tmpObjS[23] + tmpFx[28]*tmpObjS[34] + tmpFx[37]*tmpObjS[45] + tmpFx[46]*tmpObjS[56] + tmpFx[55]*tmpObjS[67] + tmpFx[64]*tmpObjS[78] + tmpFx[73]*tmpObjS[89] + tmpFx[82]*tmpObjS[100] + tmpFx[91]*tmpObjS[111];
tmpS2[13] = + tmpFx[1]*tmpObjS[2] + tmpFx[10]*tmpObjS[13] + tmpFx[19]*tmpObjS[24] + tmpFx[28]*tmpObjS[35] + tmpFx[37]*tmpObjS[46] + tmpFx[46]*tmpObjS[57] + tmpFx[55]*tmpObjS[68] + tmpFx[64]*tmpObjS[79] + tmpFx[73]*tmpObjS[90] + tmpFx[82]*tmpObjS[101] + tmpFx[91]*tmpObjS[112];
tmpS2[14] = + tmpFx[1]*tmpObjS[3] + tmpFx[10]*tmpObjS[14] + tmpFx[19]*tmpObjS[25] + tmpFx[28]*tmpObjS[36] + tmpFx[37]*tmpObjS[47] + tmpFx[46]*tmpObjS[58] + tmpFx[55]*tmpObjS[69] + tmpFx[64]*tmpObjS[80] + tmpFx[73]*tmpObjS[91] + tmpFx[82]*tmpObjS[102] + tmpFx[91]*tmpObjS[113];
tmpS2[15] = + tmpFx[1]*tmpObjS[4] + tmpFx[10]*tmpObjS[15] + tmpFx[19]*tmpObjS[26] + tmpFx[28]*tmpObjS[37] + tmpFx[37]*tmpObjS[48] + tmpFx[46]*tmpObjS[59] + tmpFx[55]*tmpObjS[70] + tmpFx[64]*tmpObjS[81] + tmpFx[73]*tmpObjS[92] + tmpFx[82]*tmpObjS[103] + tmpFx[91]*tmpObjS[114];
tmpS2[16] = + tmpFx[1]*tmpObjS[5] + tmpFx[10]*tmpObjS[16] + tmpFx[19]*tmpObjS[27] + tmpFx[28]*tmpObjS[38] + tmpFx[37]*tmpObjS[49] + tmpFx[46]*tmpObjS[60] + tmpFx[55]*tmpObjS[71] + tmpFx[64]*tmpObjS[82] + tmpFx[73]*tmpObjS[93] + tmpFx[82]*tmpObjS[104] + tmpFx[91]*tmpObjS[115];
tmpS2[17] = + tmpFx[1]*tmpObjS[6] + tmpFx[10]*tmpObjS[17] + tmpFx[19]*tmpObjS[28] + tmpFx[28]*tmpObjS[39] + tmpFx[37]*tmpObjS[50] + tmpFx[46]*tmpObjS[61] + tmpFx[55]*tmpObjS[72] + tmpFx[64]*tmpObjS[83] + tmpFx[73]*tmpObjS[94] + tmpFx[82]*tmpObjS[105] + tmpFx[91]*tmpObjS[116];
tmpS2[18] = + tmpFx[1]*tmpObjS[7] + tmpFx[10]*tmpObjS[18] + tmpFx[19]*tmpObjS[29] + tmpFx[28]*tmpObjS[40] + tmpFx[37]*tmpObjS[51] + tmpFx[46]*tmpObjS[62] + tmpFx[55]*tmpObjS[73] + tmpFx[64]*tmpObjS[84] + tmpFx[73]*tmpObjS[95] + tmpFx[82]*tmpObjS[106] + tmpFx[91]*tmpObjS[117];
tmpS2[19] = + tmpFx[1]*tmpObjS[8] + tmpFx[10]*tmpObjS[19] + tmpFx[19]*tmpObjS[30] + tmpFx[28]*tmpObjS[41] + tmpFx[37]*tmpObjS[52] + tmpFx[46]*tmpObjS[63] + tmpFx[55]*tmpObjS[74] + tmpFx[64]*tmpObjS[85] + tmpFx[73]*tmpObjS[96] + tmpFx[82]*tmpObjS[107] + tmpFx[91]*tmpObjS[118];
tmpS2[20] = + tmpFx[1]*tmpObjS[9] + tmpFx[10]*tmpObjS[20] + tmpFx[19]*tmpObjS[31] + tmpFx[28]*tmpObjS[42] + tmpFx[37]*tmpObjS[53] + tmpFx[46]*tmpObjS[64] + tmpFx[55]*tmpObjS[75] + tmpFx[64]*tmpObjS[86] + tmpFx[73]*tmpObjS[97] + tmpFx[82]*tmpObjS[108] + tmpFx[91]*tmpObjS[119];
tmpS2[21] = + tmpFx[1]*tmpObjS[10] + tmpFx[10]*tmpObjS[21] + tmpFx[19]*tmpObjS[32] + tmpFx[28]*tmpObjS[43] + tmpFx[37]*tmpObjS[54] + tmpFx[46]*tmpObjS[65] + tmpFx[55]*tmpObjS[76] + tmpFx[64]*tmpObjS[87] + tmpFx[73]*tmpObjS[98] + tmpFx[82]*tmpObjS[109] + tmpFx[91]*tmpObjS[120];
tmpS2[22] = + tmpFx[2]*tmpObjS[0] + tmpFx[11]*tmpObjS[11] + tmpFx[20]*tmpObjS[22] + tmpFx[29]*tmpObjS[33] + tmpFx[38]*tmpObjS[44] + tmpFx[47]*tmpObjS[55] + tmpFx[56]*tmpObjS[66] + tmpFx[65]*tmpObjS[77] + tmpFx[74]*tmpObjS[88] + tmpFx[83]*tmpObjS[99] + tmpFx[92]*tmpObjS[110];
tmpS2[23] = + tmpFx[2]*tmpObjS[1] + tmpFx[11]*tmpObjS[12] + tmpFx[20]*tmpObjS[23] + tmpFx[29]*tmpObjS[34] + tmpFx[38]*tmpObjS[45] + tmpFx[47]*tmpObjS[56] + tmpFx[56]*tmpObjS[67] + tmpFx[65]*tmpObjS[78] + tmpFx[74]*tmpObjS[89] + tmpFx[83]*tmpObjS[100] + tmpFx[92]*tmpObjS[111];
tmpS2[24] = + tmpFx[2]*tmpObjS[2] + tmpFx[11]*tmpObjS[13] + tmpFx[20]*tmpObjS[24] + tmpFx[29]*tmpObjS[35] + tmpFx[38]*tmpObjS[46] + tmpFx[47]*tmpObjS[57] + tmpFx[56]*tmpObjS[68] + tmpFx[65]*tmpObjS[79] + tmpFx[74]*tmpObjS[90] + tmpFx[83]*tmpObjS[101] + tmpFx[92]*tmpObjS[112];
tmpS2[25] = + tmpFx[2]*tmpObjS[3] + tmpFx[11]*tmpObjS[14] + tmpFx[20]*tmpObjS[25] + tmpFx[29]*tmpObjS[36] + tmpFx[38]*tmpObjS[47] + tmpFx[47]*tmpObjS[58] + tmpFx[56]*tmpObjS[69] + tmpFx[65]*tmpObjS[80] + tmpFx[74]*tmpObjS[91] + tmpFx[83]*tmpObjS[102] + tmpFx[92]*tmpObjS[113];
tmpS2[26] = + tmpFx[2]*tmpObjS[4] + tmpFx[11]*tmpObjS[15] + tmpFx[20]*tmpObjS[26] + tmpFx[29]*tmpObjS[37] + tmpFx[38]*tmpObjS[48] + tmpFx[47]*tmpObjS[59] + tmpFx[56]*tmpObjS[70] + tmpFx[65]*tmpObjS[81] + tmpFx[74]*tmpObjS[92] + tmpFx[83]*tmpObjS[103] + tmpFx[92]*tmpObjS[114];
tmpS2[27] = + tmpFx[2]*tmpObjS[5] + tmpFx[11]*tmpObjS[16] + tmpFx[20]*tmpObjS[27] + tmpFx[29]*tmpObjS[38] + tmpFx[38]*tmpObjS[49] + tmpFx[47]*tmpObjS[60] + tmpFx[56]*tmpObjS[71] + tmpFx[65]*tmpObjS[82] + tmpFx[74]*tmpObjS[93] + tmpFx[83]*tmpObjS[104] + tmpFx[92]*tmpObjS[115];
tmpS2[28] = + tmpFx[2]*tmpObjS[6] + tmpFx[11]*tmpObjS[17] + tmpFx[20]*tmpObjS[28] + tmpFx[29]*tmpObjS[39] + tmpFx[38]*tmpObjS[50] + tmpFx[47]*tmpObjS[61] + tmpFx[56]*tmpObjS[72] + tmpFx[65]*tmpObjS[83] + tmpFx[74]*tmpObjS[94] + tmpFx[83]*tmpObjS[105] + tmpFx[92]*tmpObjS[116];
tmpS2[29] = + tmpFx[2]*tmpObjS[7] + tmpFx[11]*tmpObjS[18] + tmpFx[20]*tmpObjS[29] + tmpFx[29]*tmpObjS[40] + tmpFx[38]*tmpObjS[51] + tmpFx[47]*tmpObjS[62] + tmpFx[56]*tmpObjS[73] + tmpFx[65]*tmpObjS[84] + tmpFx[74]*tmpObjS[95] + tmpFx[83]*tmpObjS[106] + tmpFx[92]*tmpObjS[117];
tmpS2[30] = + tmpFx[2]*tmpObjS[8] + tmpFx[11]*tmpObjS[19] + tmpFx[20]*tmpObjS[30] + tmpFx[29]*tmpObjS[41] + tmpFx[38]*tmpObjS[52] + tmpFx[47]*tmpObjS[63] + tmpFx[56]*tmpObjS[74] + tmpFx[65]*tmpObjS[85] + tmpFx[74]*tmpObjS[96] + tmpFx[83]*tmpObjS[107] + tmpFx[92]*tmpObjS[118];
tmpS2[31] = + tmpFx[2]*tmpObjS[9] + tmpFx[11]*tmpObjS[20] + tmpFx[20]*tmpObjS[31] + tmpFx[29]*tmpObjS[42] + tmpFx[38]*tmpObjS[53] + tmpFx[47]*tmpObjS[64] + tmpFx[56]*tmpObjS[75] + tmpFx[65]*tmpObjS[86] + tmpFx[74]*tmpObjS[97] + tmpFx[83]*tmpObjS[108] + tmpFx[92]*tmpObjS[119];
tmpS2[32] = + tmpFx[2]*tmpObjS[10] + tmpFx[11]*tmpObjS[21] + tmpFx[20]*tmpObjS[32] + tmpFx[29]*tmpObjS[43] + tmpFx[38]*tmpObjS[54] + tmpFx[47]*tmpObjS[65] + tmpFx[56]*tmpObjS[76] + tmpFx[65]*tmpObjS[87] + tmpFx[74]*tmpObjS[98] + tmpFx[83]*tmpObjS[109] + tmpFx[92]*tmpObjS[120];
tmpS2[33] = + tmpFx[3]*tmpObjS[0] + tmpFx[12]*tmpObjS[11] + tmpFx[21]*tmpObjS[22] + tmpFx[30]*tmpObjS[33] + tmpFx[39]*tmpObjS[44] + tmpFx[48]*tmpObjS[55] + tmpFx[57]*tmpObjS[66] + tmpFx[66]*tmpObjS[77] + tmpFx[75]*tmpObjS[88] + tmpFx[84]*tmpObjS[99] + tmpFx[93]*tmpObjS[110];
tmpS2[34] = + tmpFx[3]*tmpObjS[1] + tmpFx[12]*tmpObjS[12] + tmpFx[21]*tmpObjS[23] + tmpFx[30]*tmpObjS[34] + tmpFx[39]*tmpObjS[45] + tmpFx[48]*tmpObjS[56] + tmpFx[57]*tmpObjS[67] + tmpFx[66]*tmpObjS[78] + tmpFx[75]*tmpObjS[89] + tmpFx[84]*tmpObjS[100] + tmpFx[93]*tmpObjS[111];
tmpS2[35] = + tmpFx[3]*tmpObjS[2] + tmpFx[12]*tmpObjS[13] + tmpFx[21]*tmpObjS[24] + tmpFx[30]*tmpObjS[35] + tmpFx[39]*tmpObjS[46] + tmpFx[48]*tmpObjS[57] + tmpFx[57]*tmpObjS[68] + tmpFx[66]*tmpObjS[79] + tmpFx[75]*tmpObjS[90] + tmpFx[84]*tmpObjS[101] + tmpFx[93]*tmpObjS[112];
tmpS2[36] = + tmpFx[3]*tmpObjS[3] + tmpFx[12]*tmpObjS[14] + tmpFx[21]*tmpObjS[25] + tmpFx[30]*tmpObjS[36] + tmpFx[39]*tmpObjS[47] + tmpFx[48]*tmpObjS[58] + tmpFx[57]*tmpObjS[69] + tmpFx[66]*tmpObjS[80] + tmpFx[75]*tmpObjS[91] + tmpFx[84]*tmpObjS[102] + tmpFx[93]*tmpObjS[113];
tmpS2[37] = + tmpFx[3]*tmpObjS[4] + tmpFx[12]*tmpObjS[15] + tmpFx[21]*tmpObjS[26] + tmpFx[30]*tmpObjS[37] + tmpFx[39]*tmpObjS[48] + tmpFx[48]*tmpObjS[59] + tmpFx[57]*tmpObjS[70] + tmpFx[66]*tmpObjS[81] + tmpFx[75]*tmpObjS[92] + tmpFx[84]*tmpObjS[103] + tmpFx[93]*tmpObjS[114];
tmpS2[38] = + tmpFx[3]*tmpObjS[5] + tmpFx[12]*tmpObjS[16] + tmpFx[21]*tmpObjS[27] + tmpFx[30]*tmpObjS[38] + tmpFx[39]*tmpObjS[49] + tmpFx[48]*tmpObjS[60] + tmpFx[57]*tmpObjS[71] + tmpFx[66]*tmpObjS[82] + tmpFx[75]*tmpObjS[93] + tmpFx[84]*tmpObjS[104] + tmpFx[93]*tmpObjS[115];
tmpS2[39] = + tmpFx[3]*tmpObjS[6] + tmpFx[12]*tmpObjS[17] + tmpFx[21]*tmpObjS[28] + tmpFx[30]*tmpObjS[39] + tmpFx[39]*tmpObjS[50] + tmpFx[48]*tmpObjS[61] + tmpFx[57]*tmpObjS[72] + tmpFx[66]*tmpObjS[83] + tmpFx[75]*tmpObjS[94] + tmpFx[84]*tmpObjS[105] + tmpFx[93]*tmpObjS[116];
tmpS2[40] = + tmpFx[3]*tmpObjS[7] + tmpFx[12]*tmpObjS[18] + tmpFx[21]*tmpObjS[29] + tmpFx[30]*tmpObjS[40] + tmpFx[39]*tmpObjS[51] + tmpFx[48]*tmpObjS[62] + tmpFx[57]*tmpObjS[73] + tmpFx[66]*tmpObjS[84] + tmpFx[75]*tmpObjS[95] + tmpFx[84]*tmpObjS[106] + tmpFx[93]*tmpObjS[117];
tmpS2[41] = + tmpFx[3]*tmpObjS[8] + tmpFx[12]*tmpObjS[19] + tmpFx[21]*tmpObjS[30] + tmpFx[30]*tmpObjS[41] + tmpFx[39]*tmpObjS[52] + tmpFx[48]*tmpObjS[63] + tmpFx[57]*tmpObjS[74] + tmpFx[66]*tmpObjS[85] + tmpFx[75]*tmpObjS[96] + tmpFx[84]*tmpObjS[107] + tmpFx[93]*tmpObjS[118];
tmpS2[42] = + tmpFx[3]*tmpObjS[9] + tmpFx[12]*tmpObjS[20] + tmpFx[21]*tmpObjS[31] + tmpFx[30]*tmpObjS[42] + tmpFx[39]*tmpObjS[53] + tmpFx[48]*tmpObjS[64] + tmpFx[57]*tmpObjS[75] + tmpFx[66]*tmpObjS[86] + tmpFx[75]*tmpObjS[97] + tmpFx[84]*tmpObjS[108] + tmpFx[93]*tmpObjS[119];
tmpS2[43] = + tmpFx[3]*tmpObjS[10] + tmpFx[12]*tmpObjS[21] + tmpFx[21]*tmpObjS[32] + tmpFx[30]*tmpObjS[43] + tmpFx[39]*tmpObjS[54] + tmpFx[48]*tmpObjS[65] + tmpFx[57]*tmpObjS[76] + tmpFx[66]*tmpObjS[87] + tmpFx[75]*tmpObjS[98] + tmpFx[84]*tmpObjS[109] + tmpFx[93]*tmpObjS[120];
tmpS2[44] = + tmpFx[4]*tmpObjS[0] + tmpFx[13]*tmpObjS[11] + tmpFx[22]*tmpObjS[22] + tmpFx[31]*tmpObjS[33] + tmpFx[40]*tmpObjS[44] + tmpFx[49]*tmpObjS[55] + tmpFx[58]*tmpObjS[66] + tmpFx[67]*tmpObjS[77] + tmpFx[76]*tmpObjS[88] + tmpFx[85]*tmpObjS[99] + tmpFx[94]*tmpObjS[110];
tmpS2[45] = + tmpFx[4]*tmpObjS[1] + tmpFx[13]*tmpObjS[12] + tmpFx[22]*tmpObjS[23] + tmpFx[31]*tmpObjS[34] + tmpFx[40]*tmpObjS[45] + tmpFx[49]*tmpObjS[56] + tmpFx[58]*tmpObjS[67] + tmpFx[67]*tmpObjS[78] + tmpFx[76]*tmpObjS[89] + tmpFx[85]*tmpObjS[100] + tmpFx[94]*tmpObjS[111];
tmpS2[46] = + tmpFx[4]*tmpObjS[2] + tmpFx[13]*tmpObjS[13] + tmpFx[22]*tmpObjS[24] + tmpFx[31]*tmpObjS[35] + tmpFx[40]*tmpObjS[46] + tmpFx[49]*tmpObjS[57] + tmpFx[58]*tmpObjS[68] + tmpFx[67]*tmpObjS[79] + tmpFx[76]*tmpObjS[90] + tmpFx[85]*tmpObjS[101] + tmpFx[94]*tmpObjS[112];
tmpS2[47] = + tmpFx[4]*tmpObjS[3] + tmpFx[13]*tmpObjS[14] + tmpFx[22]*tmpObjS[25] + tmpFx[31]*tmpObjS[36] + tmpFx[40]*tmpObjS[47] + tmpFx[49]*tmpObjS[58] + tmpFx[58]*tmpObjS[69] + tmpFx[67]*tmpObjS[80] + tmpFx[76]*tmpObjS[91] + tmpFx[85]*tmpObjS[102] + tmpFx[94]*tmpObjS[113];
tmpS2[48] = + tmpFx[4]*tmpObjS[4] + tmpFx[13]*tmpObjS[15] + tmpFx[22]*tmpObjS[26] + tmpFx[31]*tmpObjS[37] + tmpFx[40]*tmpObjS[48] + tmpFx[49]*tmpObjS[59] + tmpFx[58]*tmpObjS[70] + tmpFx[67]*tmpObjS[81] + tmpFx[76]*tmpObjS[92] + tmpFx[85]*tmpObjS[103] + tmpFx[94]*tmpObjS[114];
tmpS2[49] = + tmpFx[4]*tmpObjS[5] + tmpFx[13]*tmpObjS[16] + tmpFx[22]*tmpObjS[27] + tmpFx[31]*tmpObjS[38] + tmpFx[40]*tmpObjS[49] + tmpFx[49]*tmpObjS[60] + tmpFx[58]*tmpObjS[71] + tmpFx[67]*tmpObjS[82] + tmpFx[76]*tmpObjS[93] + tmpFx[85]*tmpObjS[104] + tmpFx[94]*tmpObjS[115];
tmpS2[50] = + tmpFx[4]*tmpObjS[6] + tmpFx[13]*tmpObjS[17] + tmpFx[22]*tmpObjS[28] + tmpFx[31]*tmpObjS[39] + tmpFx[40]*tmpObjS[50] + tmpFx[49]*tmpObjS[61] + tmpFx[58]*tmpObjS[72] + tmpFx[67]*tmpObjS[83] + tmpFx[76]*tmpObjS[94] + tmpFx[85]*tmpObjS[105] + tmpFx[94]*tmpObjS[116];
tmpS2[51] = + tmpFx[4]*tmpObjS[7] + tmpFx[13]*tmpObjS[18] + tmpFx[22]*tmpObjS[29] + tmpFx[31]*tmpObjS[40] + tmpFx[40]*tmpObjS[51] + tmpFx[49]*tmpObjS[62] + tmpFx[58]*tmpObjS[73] + tmpFx[67]*tmpObjS[84] + tmpFx[76]*tmpObjS[95] + tmpFx[85]*tmpObjS[106] + tmpFx[94]*tmpObjS[117];
tmpS2[52] = + tmpFx[4]*tmpObjS[8] + tmpFx[13]*tmpObjS[19] + tmpFx[22]*tmpObjS[30] + tmpFx[31]*tmpObjS[41] + tmpFx[40]*tmpObjS[52] + tmpFx[49]*tmpObjS[63] + tmpFx[58]*tmpObjS[74] + tmpFx[67]*tmpObjS[85] + tmpFx[76]*tmpObjS[96] + tmpFx[85]*tmpObjS[107] + tmpFx[94]*tmpObjS[118];
tmpS2[53] = + tmpFx[4]*tmpObjS[9] + tmpFx[13]*tmpObjS[20] + tmpFx[22]*tmpObjS[31] + tmpFx[31]*tmpObjS[42] + tmpFx[40]*tmpObjS[53] + tmpFx[49]*tmpObjS[64] + tmpFx[58]*tmpObjS[75] + tmpFx[67]*tmpObjS[86] + tmpFx[76]*tmpObjS[97] + tmpFx[85]*tmpObjS[108] + tmpFx[94]*tmpObjS[119];
tmpS2[54] = + tmpFx[4]*tmpObjS[10] + tmpFx[13]*tmpObjS[21] + tmpFx[22]*tmpObjS[32] + tmpFx[31]*tmpObjS[43] + tmpFx[40]*tmpObjS[54] + tmpFx[49]*tmpObjS[65] + tmpFx[58]*tmpObjS[76] + tmpFx[67]*tmpObjS[87] + tmpFx[76]*tmpObjS[98] + tmpFx[85]*tmpObjS[109] + tmpFx[94]*tmpObjS[120];
tmpS2[55] = + tmpFx[5]*tmpObjS[0] + tmpFx[14]*tmpObjS[11] + tmpFx[23]*tmpObjS[22] + tmpFx[32]*tmpObjS[33] + tmpFx[41]*tmpObjS[44] + tmpFx[50]*tmpObjS[55] + tmpFx[59]*tmpObjS[66] + tmpFx[68]*tmpObjS[77] + tmpFx[77]*tmpObjS[88] + tmpFx[86]*tmpObjS[99] + tmpFx[95]*tmpObjS[110];
tmpS2[56] = + tmpFx[5]*tmpObjS[1] + tmpFx[14]*tmpObjS[12] + tmpFx[23]*tmpObjS[23] + tmpFx[32]*tmpObjS[34] + tmpFx[41]*tmpObjS[45] + tmpFx[50]*tmpObjS[56] + tmpFx[59]*tmpObjS[67] + tmpFx[68]*tmpObjS[78] + tmpFx[77]*tmpObjS[89] + tmpFx[86]*tmpObjS[100] + tmpFx[95]*tmpObjS[111];
tmpS2[57] = + tmpFx[5]*tmpObjS[2] + tmpFx[14]*tmpObjS[13] + tmpFx[23]*tmpObjS[24] + tmpFx[32]*tmpObjS[35] + tmpFx[41]*tmpObjS[46] + tmpFx[50]*tmpObjS[57] + tmpFx[59]*tmpObjS[68] + tmpFx[68]*tmpObjS[79] + tmpFx[77]*tmpObjS[90] + tmpFx[86]*tmpObjS[101] + tmpFx[95]*tmpObjS[112];
tmpS2[58] = + tmpFx[5]*tmpObjS[3] + tmpFx[14]*tmpObjS[14] + tmpFx[23]*tmpObjS[25] + tmpFx[32]*tmpObjS[36] + tmpFx[41]*tmpObjS[47] + tmpFx[50]*tmpObjS[58] + tmpFx[59]*tmpObjS[69] + tmpFx[68]*tmpObjS[80] + tmpFx[77]*tmpObjS[91] + tmpFx[86]*tmpObjS[102] + tmpFx[95]*tmpObjS[113];
tmpS2[59] = + tmpFx[5]*tmpObjS[4] + tmpFx[14]*tmpObjS[15] + tmpFx[23]*tmpObjS[26] + tmpFx[32]*tmpObjS[37] + tmpFx[41]*tmpObjS[48] + tmpFx[50]*tmpObjS[59] + tmpFx[59]*tmpObjS[70] + tmpFx[68]*tmpObjS[81] + tmpFx[77]*tmpObjS[92] + tmpFx[86]*tmpObjS[103] + tmpFx[95]*tmpObjS[114];
tmpS2[60] = + tmpFx[5]*tmpObjS[5] + tmpFx[14]*tmpObjS[16] + tmpFx[23]*tmpObjS[27] + tmpFx[32]*tmpObjS[38] + tmpFx[41]*tmpObjS[49] + tmpFx[50]*tmpObjS[60] + tmpFx[59]*tmpObjS[71] + tmpFx[68]*tmpObjS[82] + tmpFx[77]*tmpObjS[93] + tmpFx[86]*tmpObjS[104] + tmpFx[95]*tmpObjS[115];
tmpS2[61] = + tmpFx[5]*tmpObjS[6] + tmpFx[14]*tmpObjS[17] + tmpFx[23]*tmpObjS[28] + tmpFx[32]*tmpObjS[39] + tmpFx[41]*tmpObjS[50] + tmpFx[50]*tmpObjS[61] + tmpFx[59]*tmpObjS[72] + tmpFx[68]*tmpObjS[83] + tmpFx[77]*tmpObjS[94] + tmpFx[86]*tmpObjS[105] + tmpFx[95]*tmpObjS[116];
tmpS2[62] = + tmpFx[5]*tmpObjS[7] + tmpFx[14]*tmpObjS[18] + tmpFx[23]*tmpObjS[29] + tmpFx[32]*tmpObjS[40] + tmpFx[41]*tmpObjS[51] + tmpFx[50]*tmpObjS[62] + tmpFx[59]*tmpObjS[73] + tmpFx[68]*tmpObjS[84] + tmpFx[77]*tmpObjS[95] + tmpFx[86]*tmpObjS[106] + tmpFx[95]*tmpObjS[117];
tmpS2[63] = + tmpFx[5]*tmpObjS[8] + tmpFx[14]*tmpObjS[19] + tmpFx[23]*tmpObjS[30] + tmpFx[32]*tmpObjS[41] + tmpFx[41]*tmpObjS[52] + tmpFx[50]*tmpObjS[63] + tmpFx[59]*tmpObjS[74] + tmpFx[68]*tmpObjS[85] + tmpFx[77]*tmpObjS[96] + tmpFx[86]*tmpObjS[107] + tmpFx[95]*tmpObjS[118];
tmpS2[64] = + tmpFx[5]*tmpObjS[9] + tmpFx[14]*tmpObjS[20] + tmpFx[23]*tmpObjS[31] + tmpFx[32]*tmpObjS[42] + tmpFx[41]*tmpObjS[53] + tmpFx[50]*tmpObjS[64] + tmpFx[59]*tmpObjS[75] + tmpFx[68]*tmpObjS[86] + tmpFx[77]*tmpObjS[97] + tmpFx[86]*tmpObjS[108] + tmpFx[95]*tmpObjS[119];
tmpS2[65] = + tmpFx[5]*tmpObjS[10] + tmpFx[14]*tmpObjS[21] + tmpFx[23]*tmpObjS[32] + tmpFx[32]*tmpObjS[43] + tmpFx[41]*tmpObjS[54] + tmpFx[50]*tmpObjS[65] + tmpFx[59]*tmpObjS[76] + tmpFx[68]*tmpObjS[87] + tmpFx[77]*tmpObjS[98] + tmpFx[86]*tmpObjS[109] + tmpFx[95]*tmpObjS[120];
tmpS2[66] = + tmpFx[6]*tmpObjS[0] + tmpFx[15]*tmpObjS[11] + tmpFx[24]*tmpObjS[22] + tmpFx[33]*tmpObjS[33] + tmpFx[42]*tmpObjS[44] + tmpFx[51]*tmpObjS[55] + tmpFx[60]*tmpObjS[66] + tmpFx[69]*tmpObjS[77] + tmpFx[78]*tmpObjS[88] + tmpFx[87]*tmpObjS[99] + tmpFx[96]*tmpObjS[110];
tmpS2[67] = + tmpFx[6]*tmpObjS[1] + tmpFx[15]*tmpObjS[12] + tmpFx[24]*tmpObjS[23] + tmpFx[33]*tmpObjS[34] + tmpFx[42]*tmpObjS[45] + tmpFx[51]*tmpObjS[56] + tmpFx[60]*tmpObjS[67] + tmpFx[69]*tmpObjS[78] + tmpFx[78]*tmpObjS[89] + tmpFx[87]*tmpObjS[100] + tmpFx[96]*tmpObjS[111];
tmpS2[68] = + tmpFx[6]*tmpObjS[2] + tmpFx[15]*tmpObjS[13] + tmpFx[24]*tmpObjS[24] + tmpFx[33]*tmpObjS[35] + tmpFx[42]*tmpObjS[46] + tmpFx[51]*tmpObjS[57] + tmpFx[60]*tmpObjS[68] + tmpFx[69]*tmpObjS[79] + tmpFx[78]*tmpObjS[90] + tmpFx[87]*tmpObjS[101] + tmpFx[96]*tmpObjS[112];
tmpS2[69] = + tmpFx[6]*tmpObjS[3] + tmpFx[15]*tmpObjS[14] + tmpFx[24]*tmpObjS[25] + tmpFx[33]*tmpObjS[36] + tmpFx[42]*tmpObjS[47] + tmpFx[51]*tmpObjS[58] + tmpFx[60]*tmpObjS[69] + tmpFx[69]*tmpObjS[80] + tmpFx[78]*tmpObjS[91] + tmpFx[87]*tmpObjS[102] + tmpFx[96]*tmpObjS[113];
tmpS2[70] = + tmpFx[6]*tmpObjS[4] + tmpFx[15]*tmpObjS[15] + tmpFx[24]*tmpObjS[26] + tmpFx[33]*tmpObjS[37] + tmpFx[42]*tmpObjS[48] + tmpFx[51]*tmpObjS[59] + tmpFx[60]*tmpObjS[70] + tmpFx[69]*tmpObjS[81] + tmpFx[78]*tmpObjS[92] + tmpFx[87]*tmpObjS[103] + tmpFx[96]*tmpObjS[114];
tmpS2[71] = + tmpFx[6]*tmpObjS[5] + tmpFx[15]*tmpObjS[16] + tmpFx[24]*tmpObjS[27] + tmpFx[33]*tmpObjS[38] + tmpFx[42]*tmpObjS[49] + tmpFx[51]*tmpObjS[60] + tmpFx[60]*tmpObjS[71] + tmpFx[69]*tmpObjS[82] + tmpFx[78]*tmpObjS[93] + tmpFx[87]*tmpObjS[104] + tmpFx[96]*tmpObjS[115];
tmpS2[72] = + tmpFx[6]*tmpObjS[6] + tmpFx[15]*tmpObjS[17] + tmpFx[24]*tmpObjS[28] + tmpFx[33]*tmpObjS[39] + tmpFx[42]*tmpObjS[50] + tmpFx[51]*tmpObjS[61] + tmpFx[60]*tmpObjS[72] + tmpFx[69]*tmpObjS[83] + tmpFx[78]*tmpObjS[94] + tmpFx[87]*tmpObjS[105] + tmpFx[96]*tmpObjS[116];
tmpS2[73] = + tmpFx[6]*tmpObjS[7] + tmpFx[15]*tmpObjS[18] + tmpFx[24]*tmpObjS[29] + tmpFx[33]*tmpObjS[40] + tmpFx[42]*tmpObjS[51] + tmpFx[51]*tmpObjS[62] + tmpFx[60]*tmpObjS[73] + tmpFx[69]*tmpObjS[84] + tmpFx[78]*tmpObjS[95] + tmpFx[87]*tmpObjS[106] + tmpFx[96]*tmpObjS[117];
tmpS2[74] = + tmpFx[6]*tmpObjS[8] + tmpFx[15]*tmpObjS[19] + tmpFx[24]*tmpObjS[30] + tmpFx[33]*tmpObjS[41] + tmpFx[42]*tmpObjS[52] + tmpFx[51]*tmpObjS[63] + tmpFx[60]*tmpObjS[74] + tmpFx[69]*tmpObjS[85] + tmpFx[78]*tmpObjS[96] + tmpFx[87]*tmpObjS[107] + tmpFx[96]*tmpObjS[118];
tmpS2[75] = + tmpFx[6]*tmpObjS[9] + tmpFx[15]*tmpObjS[20] + tmpFx[24]*tmpObjS[31] + tmpFx[33]*tmpObjS[42] + tmpFx[42]*tmpObjS[53] + tmpFx[51]*tmpObjS[64] + tmpFx[60]*tmpObjS[75] + tmpFx[69]*tmpObjS[86] + tmpFx[78]*tmpObjS[97] + tmpFx[87]*tmpObjS[108] + tmpFx[96]*tmpObjS[119];
tmpS2[76] = + tmpFx[6]*tmpObjS[10] + tmpFx[15]*tmpObjS[21] + tmpFx[24]*tmpObjS[32] + tmpFx[33]*tmpObjS[43] + tmpFx[42]*tmpObjS[54] + tmpFx[51]*tmpObjS[65] + tmpFx[60]*tmpObjS[76] + tmpFx[69]*tmpObjS[87] + tmpFx[78]*tmpObjS[98] + tmpFx[87]*tmpObjS[109] + tmpFx[96]*tmpObjS[120];
tmpS2[77] = + tmpFx[7]*tmpObjS[0] + tmpFx[16]*tmpObjS[11] + tmpFx[25]*tmpObjS[22] + tmpFx[34]*tmpObjS[33] + tmpFx[43]*tmpObjS[44] + tmpFx[52]*tmpObjS[55] + tmpFx[61]*tmpObjS[66] + tmpFx[70]*tmpObjS[77] + tmpFx[79]*tmpObjS[88] + tmpFx[88]*tmpObjS[99] + tmpFx[97]*tmpObjS[110];
tmpS2[78] = + tmpFx[7]*tmpObjS[1] + tmpFx[16]*tmpObjS[12] + tmpFx[25]*tmpObjS[23] + tmpFx[34]*tmpObjS[34] + tmpFx[43]*tmpObjS[45] + tmpFx[52]*tmpObjS[56] + tmpFx[61]*tmpObjS[67] + tmpFx[70]*tmpObjS[78] + tmpFx[79]*tmpObjS[89] + tmpFx[88]*tmpObjS[100] + tmpFx[97]*tmpObjS[111];
tmpS2[79] = + tmpFx[7]*tmpObjS[2] + tmpFx[16]*tmpObjS[13] + tmpFx[25]*tmpObjS[24] + tmpFx[34]*tmpObjS[35] + tmpFx[43]*tmpObjS[46] + tmpFx[52]*tmpObjS[57] + tmpFx[61]*tmpObjS[68] + tmpFx[70]*tmpObjS[79] + tmpFx[79]*tmpObjS[90] + tmpFx[88]*tmpObjS[101] + tmpFx[97]*tmpObjS[112];
tmpS2[80] = + tmpFx[7]*tmpObjS[3] + tmpFx[16]*tmpObjS[14] + tmpFx[25]*tmpObjS[25] + tmpFx[34]*tmpObjS[36] + tmpFx[43]*tmpObjS[47] + tmpFx[52]*tmpObjS[58] + tmpFx[61]*tmpObjS[69] + tmpFx[70]*tmpObjS[80] + tmpFx[79]*tmpObjS[91] + tmpFx[88]*tmpObjS[102] + tmpFx[97]*tmpObjS[113];
tmpS2[81] = + tmpFx[7]*tmpObjS[4] + tmpFx[16]*tmpObjS[15] + tmpFx[25]*tmpObjS[26] + tmpFx[34]*tmpObjS[37] + tmpFx[43]*tmpObjS[48] + tmpFx[52]*tmpObjS[59] + tmpFx[61]*tmpObjS[70] + tmpFx[70]*tmpObjS[81] + tmpFx[79]*tmpObjS[92] + tmpFx[88]*tmpObjS[103] + tmpFx[97]*tmpObjS[114];
tmpS2[82] = + tmpFx[7]*tmpObjS[5] + tmpFx[16]*tmpObjS[16] + tmpFx[25]*tmpObjS[27] + tmpFx[34]*tmpObjS[38] + tmpFx[43]*tmpObjS[49] + tmpFx[52]*tmpObjS[60] + tmpFx[61]*tmpObjS[71] + tmpFx[70]*tmpObjS[82] + tmpFx[79]*tmpObjS[93] + tmpFx[88]*tmpObjS[104] + tmpFx[97]*tmpObjS[115];
tmpS2[83] = + tmpFx[7]*tmpObjS[6] + tmpFx[16]*tmpObjS[17] + tmpFx[25]*tmpObjS[28] + tmpFx[34]*tmpObjS[39] + tmpFx[43]*tmpObjS[50] + tmpFx[52]*tmpObjS[61] + tmpFx[61]*tmpObjS[72] + tmpFx[70]*tmpObjS[83] + tmpFx[79]*tmpObjS[94] + tmpFx[88]*tmpObjS[105] + tmpFx[97]*tmpObjS[116];
tmpS2[84] = + tmpFx[7]*tmpObjS[7] + tmpFx[16]*tmpObjS[18] + tmpFx[25]*tmpObjS[29] + tmpFx[34]*tmpObjS[40] + tmpFx[43]*tmpObjS[51] + tmpFx[52]*tmpObjS[62] + tmpFx[61]*tmpObjS[73] + tmpFx[70]*tmpObjS[84] + tmpFx[79]*tmpObjS[95] + tmpFx[88]*tmpObjS[106] + tmpFx[97]*tmpObjS[117];
tmpS2[85] = + tmpFx[7]*tmpObjS[8] + tmpFx[16]*tmpObjS[19] + tmpFx[25]*tmpObjS[30] + tmpFx[34]*tmpObjS[41] + tmpFx[43]*tmpObjS[52] + tmpFx[52]*tmpObjS[63] + tmpFx[61]*tmpObjS[74] + tmpFx[70]*tmpObjS[85] + tmpFx[79]*tmpObjS[96] + tmpFx[88]*tmpObjS[107] + tmpFx[97]*tmpObjS[118];
tmpS2[86] = + tmpFx[7]*tmpObjS[9] + tmpFx[16]*tmpObjS[20] + tmpFx[25]*tmpObjS[31] + tmpFx[34]*tmpObjS[42] + tmpFx[43]*tmpObjS[53] + tmpFx[52]*tmpObjS[64] + tmpFx[61]*tmpObjS[75] + tmpFx[70]*tmpObjS[86] + tmpFx[79]*tmpObjS[97] + tmpFx[88]*tmpObjS[108] + tmpFx[97]*tmpObjS[119];
tmpS2[87] = + tmpFx[7]*tmpObjS[10] + tmpFx[16]*tmpObjS[21] + tmpFx[25]*tmpObjS[32] + tmpFx[34]*tmpObjS[43] + tmpFx[43]*tmpObjS[54] + tmpFx[52]*tmpObjS[65] + tmpFx[61]*tmpObjS[76] + tmpFx[70]*tmpObjS[87] + tmpFx[79]*tmpObjS[98] + tmpFx[88]*tmpObjS[109] + tmpFx[97]*tmpObjS[120];
tmpS2[88] = + tmpFx[8]*tmpObjS[0] + tmpFx[17]*tmpObjS[11] + tmpFx[26]*tmpObjS[22] + tmpFx[35]*tmpObjS[33] + tmpFx[44]*tmpObjS[44] + tmpFx[53]*tmpObjS[55] + tmpFx[62]*tmpObjS[66] + tmpFx[71]*tmpObjS[77] + tmpFx[80]*tmpObjS[88] + tmpFx[89]*tmpObjS[99] + tmpFx[98]*tmpObjS[110];
tmpS2[89] = + tmpFx[8]*tmpObjS[1] + tmpFx[17]*tmpObjS[12] + tmpFx[26]*tmpObjS[23] + tmpFx[35]*tmpObjS[34] + tmpFx[44]*tmpObjS[45] + tmpFx[53]*tmpObjS[56] + tmpFx[62]*tmpObjS[67] + tmpFx[71]*tmpObjS[78] + tmpFx[80]*tmpObjS[89] + tmpFx[89]*tmpObjS[100] + tmpFx[98]*tmpObjS[111];
tmpS2[90] = + tmpFx[8]*tmpObjS[2] + tmpFx[17]*tmpObjS[13] + tmpFx[26]*tmpObjS[24] + tmpFx[35]*tmpObjS[35] + tmpFx[44]*tmpObjS[46] + tmpFx[53]*tmpObjS[57] + tmpFx[62]*tmpObjS[68] + tmpFx[71]*tmpObjS[79] + tmpFx[80]*tmpObjS[90] + tmpFx[89]*tmpObjS[101] + tmpFx[98]*tmpObjS[112];
tmpS2[91] = + tmpFx[8]*tmpObjS[3] + tmpFx[17]*tmpObjS[14] + tmpFx[26]*tmpObjS[25] + tmpFx[35]*tmpObjS[36] + tmpFx[44]*tmpObjS[47] + tmpFx[53]*tmpObjS[58] + tmpFx[62]*tmpObjS[69] + tmpFx[71]*tmpObjS[80] + tmpFx[80]*tmpObjS[91] + tmpFx[89]*tmpObjS[102] + tmpFx[98]*tmpObjS[113];
tmpS2[92] = + tmpFx[8]*tmpObjS[4] + tmpFx[17]*tmpObjS[15] + tmpFx[26]*tmpObjS[26] + tmpFx[35]*tmpObjS[37] + tmpFx[44]*tmpObjS[48] + tmpFx[53]*tmpObjS[59] + tmpFx[62]*tmpObjS[70] + tmpFx[71]*tmpObjS[81] + tmpFx[80]*tmpObjS[92] + tmpFx[89]*tmpObjS[103] + tmpFx[98]*tmpObjS[114];
tmpS2[93] = + tmpFx[8]*tmpObjS[5] + tmpFx[17]*tmpObjS[16] + tmpFx[26]*tmpObjS[27] + tmpFx[35]*tmpObjS[38] + tmpFx[44]*tmpObjS[49] + tmpFx[53]*tmpObjS[60] + tmpFx[62]*tmpObjS[71] + tmpFx[71]*tmpObjS[82] + tmpFx[80]*tmpObjS[93] + tmpFx[89]*tmpObjS[104] + tmpFx[98]*tmpObjS[115];
tmpS2[94] = + tmpFx[8]*tmpObjS[6] + tmpFx[17]*tmpObjS[17] + tmpFx[26]*tmpObjS[28] + tmpFx[35]*tmpObjS[39] + tmpFx[44]*tmpObjS[50] + tmpFx[53]*tmpObjS[61] + tmpFx[62]*tmpObjS[72] + tmpFx[71]*tmpObjS[83] + tmpFx[80]*tmpObjS[94] + tmpFx[89]*tmpObjS[105] + tmpFx[98]*tmpObjS[116];
tmpS2[95] = + tmpFx[8]*tmpObjS[7] + tmpFx[17]*tmpObjS[18] + tmpFx[26]*tmpObjS[29] + tmpFx[35]*tmpObjS[40] + tmpFx[44]*tmpObjS[51] + tmpFx[53]*tmpObjS[62] + tmpFx[62]*tmpObjS[73] + tmpFx[71]*tmpObjS[84] + tmpFx[80]*tmpObjS[95] + tmpFx[89]*tmpObjS[106] + tmpFx[98]*tmpObjS[117];
tmpS2[96] = + tmpFx[8]*tmpObjS[8] + tmpFx[17]*tmpObjS[19] + tmpFx[26]*tmpObjS[30] + tmpFx[35]*tmpObjS[41] + tmpFx[44]*tmpObjS[52] + tmpFx[53]*tmpObjS[63] + tmpFx[62]*tmpObjS[74] + tmpFx[71]*tmpObjS[85] + tmpFx[80]*tmpObjS[96] + tmpFx[89]*tmpObjS[107] + tmpFx[98]*tmpObjS[118];
tmpS2[97] = + tmpFx[8]*tmpObjS[9] + tmpFx[17]*tmpObjS[20] + tmpFx[26]*tmpObjS[31] + tmpFx[35]*tmpObjS[42] + tmpFx[44]*tmpObjS[53] + tmpFx[53]*tmpObjS[64] + tmpFx[62]*tmpObjS[75] + tmpFx[71]*tmpObjS[86] + tmpFx[80]*tmpObjS[97] + tmpFx[89]*tmpObjS[108] + tmpFx[98]*tmpObjS[119];
tmpS2[98] = + tmpFx[8]*tmpObjS[10] + tmpFx[17]*tmpObjS[21] + tmpFx[26]*tmpObjS[32] + tmpFx[35]*tmpObjS[43] + tmpFx[44]*tmpObjS[54] + tmpFx[53]*tmpObjS[65] + tmpFx[62]*tmpObjS[76] + tmpFx[71]*tmpObjS[87] + tmpFx[80]*tmpObjS[98] + tmpFx[89]*tmpObjS[109] + tmpFx[98]*tmpObjS[120];
tmpS1[0] = + tmpS2[0]*tmpFu[0] + tmpS2[1]*tmpFu[3] + tmpS2[2]*tmpFu[6] + tmpS2[3]*tmpFu[9] + tmpS2[4]*tmpFu[12] + tmpS2[5]*tmpFu[15] + tmpS2[6]*tmpFu[18] + tmpS2[7]*tmpFu[21] + tmpS2[8]*tmpFu[24] + tmpS2[9]*tmpFu[27] + tmpS2[10]*tmpFu[30];
tmpS1[1] = + tmpS2[0]*tmpFu[1] + tmpS2[1]*tmpFu[4] + tmpS2[2]*tmpFu[7] + tmpS2[3]*tmpFu[10] + tmpS2[4]*tmpFu[13] + tmpS2[5]*tmpFu[16] + tmpS2[6]*tmpFu[19] + tmpS2[7]*tmpFu[22] + tmpS2[8]*tmpFu[25] + tmpS2[9]*tmpFu[28] + tmpS2[10]*tmpFu[31];
tmpS1[2] = + tmpS2[0]*tmpFu[2] + tmpS2[1]*tmpFu[5] + tmpS2[2]*tmpFu[8] + tmpS2[3]*tmpFu[11] + tmpS2[4]*tmpFu[14] + tmpS2[5]*tmpFu[17] + tmpS2[6]*tmpFu[20] + tmpS2[7]*tmpFu[23] + tmpS2[8]*tmpFu[26] + tmpS2[9]*tmpFu[29] + tmpS2[10]*tmpFu[32];
tmpS1[3] = + tmpS2[11]*tmpFu[0] + tmpS2[12]*tmpFu[3] + tmpS2[13]*tmpFu[6] + tmpS2[14]*tmpFu[9] + tmpS2[15]*tmpFu[12] + tmpS2[16]*tmpFu[15] + tmpS2[17]*tmpFu[18] + tmpS2[18]*tmpFu[21] + tmpS2[19]*tmpFu[24] + tmpS2[20]*tmpFu[27] + tmpS2[21]*tmpFu[30];
tmpS1[4] = + tmpS2[11]*tmpFu[1] + tmpS2[12]*tmpFu[4] + tmpS2[13]*tmpFu[7] + tmpS2[14]*tmpFu[10] + tmpS2[15]*tmpFu[13] + tmpS2[16]*tmpFu[16] + tmpS2[17]*tmpFu[19] + tmpS2[18]*tmpFu[22] + tmpS2[19]*tmpFu[25] + tmpS2[20]*tmpFu[28] + tmpS2[21]*tmpFu[31];
tmpS1[5] = + tmpS2[11]*tmpFu[2] + tmpS2[12]*tmpFu[5] + tmpS2[13]*tmpFu[8] + tmpS2[14]*tmpFu[11] + tmpS2[15]*tmpFu[14] + tmpS2[16]*tmpFu[17] + tmpS2[17]*tmpFu[20] + tmpS2[18]*tmpFu[23] + tmpS2[19]*tmpFu[26] + tmpS2[20]*tmpFu[29] + tmpS2[21]*tmpFu[32];
tmpS1[6] = + tmpS2[22]*tmpFu[0] + tmpS2[23]*tmpFu[3] + tmpS2[24]*tmpFu[6] + tmpS2[25]*tmpFu[9] + tmpS2[26]*tmpFu[12] + tmpS2[27]*tmpFu[15] + tmpS2[28]*tmpFu[18] + tmpS2[29]*tmpFu[21] + tmpS2[30]*tmpFu[24] + tmpS2[31]*tmpFu[27] + tmpS2[32]*tmpFu[30];
tmpS1[7] = + tmpS2[22]*tmpFu[1] + tmpS2[23]*tmpFu[4] + tmpS2[24]*tmpFu[7] + tmpS2[25]*tmpFu[10] + tmpS2[26]*tmpFu[13] + tmpS2[27]*tmpFu[16] + tmpS2[28]*tmpFu[19] + tmpS2[29]*tmpFu[22] + tmpS2[30]*tmpFu[25] + tmpS2[31]*tmpFu[28] + tmpS2[32]*tmpFu[31];
tmpS1[8] = + tmpS2[22]*tmpFu[2] + tmpS2[23]*tmpFu[5] + tmpS2[24]*tmpFu[8] + tmpS2[25]*tmpFu[11] + tmpS2[26]*tmpFu[14] + tmpS2[27]*tmpFu[17] + tmpS2[28]*tmpFu[20] + tmpS2[29]*tmpFu[23] + tmpS2[30]*tmpFu[26] + tmpS2[31]*tmpFu[29] + tmpS2[32]*tmpFu[32];
tmpS1[9] = + tmpS2[33]*tmpFu[0] + tmpS2[34]*tmpFu[3] + tmpS2[35]*tmpFu[6] + tmpS2[36]*tmpFu[9] + tmpS2[37]*tmpFu[12] + tmpS2[38]*tmpFu[15] + tmpS2[39]*tmpFu[18] + tmpS2[40]*tmpFu[21] + tmpS2[41]*tmpFu[24] + tmpS2[42]*tmpFu[27] + tmpS2[43]*tmpFu[30];
tmpS1[10] = + tmpS2[33]*tmpFu[1] + tmpS2[34]*tmpFu[4] + tmpS2[35]*tmpFu[7] + tmpS2[36]*tmpFu[10] + tmpS2[37]*tmpFu[13] + tmpS2[38]*tmpFu[16] + tmpS2[39]*tmpFu[19] + tmpS2[40]*tmpFu[22] + tmpS2[41]*tmpFu[25] + tmpS2[42]*tmpFu[28] + tmpS2[43]*tmpFu[31];
tmpS1[11] = + tmpS2[33]*tmpFu[2] + tmpS2[34]*tmpFu[5] + tmpS2[35]*tmpFu[8] + tmpS2[36]*tmpFu[11] + tmpS2[37]*tmpFu[14] + tmpS2[38]*tmpFu[17] + tmpS2[39]*tmpFu[20] + tmpS2[40]*tmpFu[23] + tmpS2[41]*tmpFu[26] + tmpS2[42]*tmpFu[29] + tmpS2[43]*tmpFu[32];
tmpS1[12] = + tmpS2[44]*tmpFu[0] + tmpS2[45]*tmpFu[3] + tmpS2[46]*tmpFu[6] + tmpS2[47]*tmpFu[9] + tmpS2[48]*tmpFu[12] + tmpS2[49]*tmpFu[15] + tmpS2[50]*tmpFu[18] + tmpS2[51]*tmpFu[21] + tmpS2[52]*tmpFu[24] + tmpS2[53]*tmpFu[27] + tmpS2[54]*tmpFu[30];
tmpS1[13] = + tmpS2[44]*tmpFu[1] + tmpS2[45]*tmpFu[4] + tmpS2[46]*tmpFu[7] + tmpS2[47]*tmpFu[10] + tmpS2[48]*tmpFu[13] + tmpS2[49]*tmpFu[16] + tmpS2[50]*tmpFu[19] + tmpS2[51]*tmpFu[22] + tmpS2[52]*tmpFu[25] + tmpS2[53]*tmpFu[28] + tmpS2[54]*tmpFu[31];
tmpS1[14] = + tmpS2[44]*tmpFu[2] + tmpS2[45]*tmpFu[5] + tmpS2[46]*tmpFu[8] + tmpS2[47]*tmpFu[11] + tmpS2[48]*tmpFu[14] + tmpS2[49]*tmpFu[17] + tmpS2[50]*tmpFu[20] + tmpS2[51]*tmpFu[23] + tmpS2[52]*tmpFu[26] + tmpS2[53]*tmpFu[29] + tmpS2[54]*tmpFu[32];
tmpS1[15] = + tmpS2[55]*tmpFu[0] + tmpS2[56]*tmpFu[3] + tmpS2[57]*tmpFu[6] + tmpS2[58]*tmpFu[9] + tmpS2[59]*tmpFu[12] + tmpS2[60]*tmpFu[15] + tmpS2[61]*tmpFu[18] + tmpS2[62]*tmpFu[21] + tmpS2[63]*tmpFu[24] + tmpS2[64]*tmpFu[27] + tmpS2[65]*tmpFu[30];
tmpS1[16] = + tmpS2[55]*tmpFu[1] + tmpS2[56]*tmpFu[4] + tmpS2[57]*tmpFu[7] + tmpS2[58]*tmpFu[10] + tmpS2[59]*tmpFu[13] + tmpS2[60]*tmpFu[16] + tmpS2[61]*tmpFu[19] + tmpS2[62]*tmpFu[22] + tmpS2[63]*tmpFu[25] + tmpS2[64]*tmpFu[28] + tmpS2[65]*tmpFu[31];
tmpS1[17] = + tmpS2[55]*tmpFu[2] + tmpS2[56]*tmpFu[5] + tmpS2[57]*tmpFu[8] + tmpS2[58]*tmpFu[11] + tmpS2[59]*tmpFu[14] + tmpS2[60]*tmpFu[17] + tmpS2[61]*tmpFu[20] + tmpS2[62]*tmpFu[23] + tmpS2[63]*tmpFu[26] + tmpS2[64]*tmpFu[29] + tmpS2[65]*tmpFu[32];
tmpS1[18] = + tmpS2[66]*tmpFu[0] + tmpS2[67]*tmpFu[3] + tmpS2[68]*tmpFu[6] + tmpS2[69]*tmpFu[9] + tmpS2[70]*tmpFu[12] + tmpS2[71]*tmpFu[15] + tmpS2[72]*tmpFu[18] + tmpS2[73]*tmpFu[21] + tmpS2[74]*tmpFu[24] + tmpS2[75]*tmpFu[27] + tmpS2[76]*tmpFu[30];
tmpS1[19] = + tmpS2[66]*tmpFu[1] + tmpS2[67]*tmpFu[4] + tmpS2[68]*tmpFu[7] + tmpS2[69]*tmpFu[10] + tmpS2[70]*tmpFu[13] + tmpS2[71]*tmpFu[16] + tmpS2[72]*tmpFu[19] + tmpS2[73]*tmpFu[22] + tmpS2[74]*tmpFu[25] + tmpS2[75]*tmpFu[28] + tmpS2[76]*tmpFu[31];
tmpS1[20] = + tmpS2[66]*tmpFu[2] + tmpS2[67]*tmpFu[5] + tmpS2[68]*tmpFu[8] + tmpS2[69]*tmpFu[11] + tmpS2[70]*tmpFu[14] + tmpS2[71]*tmpFu[17] + tmpS2[72]*tmpFu[20] + tmpS2[73]*tmpFu[23] + tmpS2[74]*tmpFu[26] + tmpS2[75]*tmpFu[29] + tmpS2[76]*tmpFu[32];
tmpS1[21] = + tmpS2[77]*tmpFu[0] + tmpS2[78]*tmpFu[3] + tmpS2[79]*tmpFu[6] + tmpS2[80]*tmpFu[9] + tmpS2[81]*tmpFu[12] + tmpS2[82]*tmpFu[15] + tmpS2[83]*tmpFu[18] + tmpS2[84]*tmpFu[21] + tmpS2[85]*tmpFu[24] + tmpS2[86]*tmpFu[27] + tmpS2[87]*tmpFu[30];
tmpS1[22] = + tmpS2[77]*tmpFu[1] + tmpS2[78]*tmpFu[4] + tmpS2[79]*tmpFu[7] + tmpS2[80]*tmpFu[10] + tmpS2[81]*tmpFu[13] + tmpS2[82]*tmpFu[16] + tmpS2[83]*tmpFu[19] + tmpS2[84]*tmpFu[22] + tmpS2[85]*tmpFu[25] + tmpS2[86]*tmpFu[28] + tmpS2[87]*tmpFu[31];
tmpS1[23] = + tmpS2[77]*tmpFu[2] + tmpS2[78]*tmpFu[5] + tmpS2[79]*tmpFu[8] + tmpS2[80]*tmpFu[11] + tmpS2[81]*tmpFu[14] + tmpS2[82]*tmpFu[17] + tmpS2[83]*tmpFu[20] + tmpS2[84]*tmpFu[23] + tmpS2[85]*tmpFu[26] + tmpS2[86]*tmpFu[29] + tmpS2[87]*tmpFu[32];
tmpS1[24] = + tmpS2[88]*tmpFu[0] + tmpS2[89]*tmpFu[3] + tmpS2[90]*tmpFu[6] + tmpS2[91]*tmpFu[9] + tmpS2[92]*tmpFu[12] + tmpS2[93]*tmpFu[15] + tmpS2[94]*tmpFu[18] + tmpS2[95]*tmpFu[21] + tmpS2[96]*tmpFu[24] + tmpS2[97]*tmpFu[27] + tmpS2[98]*tmpFu[30];
tmpS1[25] = + tmpS2[88]*tmpFu[1] + tmpS2[89]*tmpFu[4] + tmpS2[90]*tmpFu[7] + tmpS2[91]*tmpFu[10] + tmpS2[92]*tmpFu[13] + tmpS2[93]*tmpFu[16] + tmpS2[94]*tmpFu[19] + tmpS2[95]*tmpFu[22] + tmpS2[96]*tmpFu[25] + tmpS2[97]*tmpFu[28] + tmpS2[98]*tmpFu[31];
tmpS1[26] = + tmpS2[88]*tmpFu[2] + tmpS2[89]*tmpFu[5] + tmpS2[90]*tmpFu[8] + tmpS2[91]*tmpFu[11] + tmpS2[92]*tmpFu[14] + tmpS2[93]*tmpFu[17] + tmpS2[94]*tmpFu[20] + tmpS2[95]*tmpFu[23] + tmpS2[96]*tmpFu[26] + tmpS2[97]*tmpFu[29] + tmpS2[98]*tmpFu[32];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[18];
tmpQN2[1] = +tmpObjSEndTerm[19];
tmpQN2[2] = +tmpObjSEndTerm[20];
tmpQN2[3] = +tmpObjSEndTerm[21];
tmpQN2[4] = +tmpObjSEndTerm[22];
tmpQN2[5] = +tmpObjSEndTerm[23];
tmpQN2[6] = +tmpObjSEndTerm[24];
tmpQN2[7] = +tmpObjSEndTerm[25];
tmpQN2[8] = +tmpObjSEndTerm[26];
tmpQN2[9] = +tmpObjSEndTerm[27];
tmpQN2[10] = +tmpObjSEndTerm[28];
tmpQN2[11] = +tmpObjSEndTerm[29];
tmpQN2[12] = +tmpObjSEndTerm[30];
tmpQN2[13] = +tmpObjSEndTerm[31];
tmpQN2[14] = +tmpObjSEndTerm[32];
tmpQN2[15] = +tmpObjSEndTerm[33];
tmpQN2[16] = +tmpObjSEndTerm[34];
tmpQN2[17] = +tmpObjSEndTerm[35];
tmpQN2[18] = 0.0;
;
tmpQN2[19] = 0.0;
;
tmpQN2[20] = 0.0;
;
tmpQN2[21] = 0.0;
;
tmpQN2[22] = 0.0;
;
tmpQN2[23] = 0.0;
;
tmpQN2[24] = 0.0;
;
tmpQN2[25] = 0.0;
;
tmpQN2[26] = 0.0;
;
tmpQN2[27] = 0.0;
;
tmpQN2[28] = 0.0;
;
tmpQN2[29] = 0.0;
;
tmpQN2[30] = 0.0;
;
tmpQN2[31] = 0.0;
;
tmpQN2[32] = 0.0;
;
tmpQN2[33] = 0.0;
;
tmpQN2[34] = 0.0;
;
tmpQN2[35] = 0.0;
;
tmpQN2[36] = +tmpObjSEndTerm[0];
tmpQN2[37] = +tmpObjSEndTerm[1];
tmpQN2[38] = +tmpObjSEndTerm[2];
tmpQN2[39] = +tmpObjSEndTerm[3];
tmpQN2[40] = +tmpObjSEndTerm[4];
tmpQN2[41] = +tmpObjSEndTerm[5];
tmpQN2[42] = +tmpObjSEndTerm[6];
tmpQN2[43] = +tmpObjSEndTerm[7];
tmpQN2[44] = +tmpObjSEndTerm[8];
tmpQN2[45] = +tmpObjSEndTerm[9];
tmpQN2[46] = +tmpObjSEndTerm[10];
tmpQN2[47] = +tmpObjSEndTerm[11];
tmpQN2[48] = +tmpObjSEndTerm[12];
tmpQN2[49] = +tmpObjSEndTerm[13];
tmpQN2[50] = +tmpObjSEndTerm[14];
tmpQN2[51] = +tmpObjSEndTerm[15];
tmpQN2[52] = +tmpObjSEndTerm[16];
tmpQN2[53] = +tmpObjSEndTerm[17];
tmpQN1[0] = + tmpQN2[3];
tmpQN1[1] = + tmpQN2[4];
tmpQN1[2] = + tmpQN2[5];
tmpQN1[3] = 0.0;
;
tmpQN1[4] = 0.0;
;
tmpQN1[5] = 0.0;
;
tmpQN1[6] = + tmpQN2[0];
tmpQN1[7] = + tmpQN2[1];
tmpQN1[8] = + tmpQN2[2];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = 0.0;
;
tmpQN1[13] = 0.0;
;
tmpQN1[14] = 0.0;
;
tmpQN1[15] = + tmpQN2[6];
tmpQN1[16] = + tmpQN2[7];
tmpQN1[17] = + tmpQN2[8];
tmpQN1[18] = + tmpQN2[15];
tmpQN1[19] = + tmpQN2[16];
tmpQN1[20] = + tmpQN2[17];
tmpQN1[21] = 0.0;
;
tmpQN1[22] = 0.0;
;
tmpQN1[23] = 0.0;
;
tmpQN1[24] = + tmpQN2[12];
tmpQN1[25] = + tmpQN2[13];
tmpQN1[26] = + tmpQN2[14];
tmpQN1[27] = + tmpQN2[21];
tmpQN1[28] = + tmpQN2[22];
tmpQN1[29] = + tmpQN2[23];
tmpQN1[30] = 0.0;
;
tmpQN1[31] = 0.0;
;
tmpQN1[32] = 0.0;
;
tmpQN1[33] = + tmpQN2[18];
tmpQN1[34] = + tmpQN2[19];
tmpQN1[35] = + tmpQN2[20];
tmpQN1[36] = + tmpQN2[27];
tmpQN1[37] = + tmpQN2[28];
tmpQN1[38] = + tmpQN2[29];
tmpQN1[39] = 0.0;
;
tmpQN1[40] = 0.0;
;
tmpQN1[41] = 0.0;
;
tmpQN1[42] = + tmpQN2[24];
tmpQN1[43] = + tmpQN2[25];
tmpQN1[44] = + tmpQN2[26];
tmpQN1[45] = + tmpQN2[33];
tmpQN1[46] = + tmpQN2[34];
tmpQN1[47] = + tmpQN2[35];
tmpQN1[48] = 0.0;
;
tmpQN1[49] = 0.0;
;
tmpQN1[50] = 0.0;
;
tmpQN1[51] = + tmpQN2[30];
tmpQN1[52] = + tmpQN2[31];
tmpQN1[53] = + tmpQN2[32];
tmpQN1[54] = + tmpQN2[39];
tmpQN1[55] = + tmpQN2[40];
tmpQN1[56] = + tmpQN2[41];
tmpQN1[57] = 0.0;
;
tmpQN1[58] = 0.0;
;
tmpQN1[59] = 0.0;
;
tmpQN1[60] = + tmpQN2[36];
tmpQN1[61] = + tmpQN2[37];
tmpQN1[62] = + tmpQN2[38];
tmpQN1[63] = + tmpQN2[45];
tmpQN1[64] = + tmpQN2[46];
tmpQN1[65] = + tmpQN2[47];
tmpQN1[66] = 0.0;
;
tmpQN1[67] = 0.0;
;
tmpQN1[68] = 0.0;
;
tmpQN1[69] = + tmpQN2[42];
tmpQN1[70] = + tmpQN2[43];
tmpQN1[71] = + tmpQN2[44];
tmpQN1[72] = + tmpQN2[51];
tmpQN1[73] = + tmpQN2[52];
tmpQN1[74] = + tmpQN2[53];
tmpQN1[75] = 0.0;
;
tmpQN1[76] = 0.0;
;
tmpQN1[77] = 0.0;
;
tmpQN1[78] = + tmpQN2[48];
tmpQN1[79] = + tmpQN2[49];
tmpQN1[80] = + tmpQN2[50];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[runObj * 3 + 2];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 9];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 9 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 9 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 9 + 3];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 9 + 4];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 9 + 5];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 9 + 6];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 9 + 7];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 9 + 8];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 11] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 11 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 11 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 11 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 11 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 11 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 11 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 11 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 11 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 11 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 11 + 10] = acadoWorkspace.objValueOut[10];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 11 ]), acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 81 ]), &(acadoWorkspace.Q2[ runObj * 99 ]) );

acado_setObjR1R2( &(acadoWorkspace.objValueOut[ 110 ]), acadoVariables.W, &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 33 ]) );

acado_setObjS1( &(acadoWorkspace.objValueOut[ 11 ]), &(acadoWorkspace.objValueOut[ 110 ]), acadoVariables.W, &(acadoWorkspace.S1[ runObj * 27 ]) );
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[180];
acadoWorkspace.objValueIn[1] = acadoVariables.x[181];
acadoWorkspace.objValueIn[2] = acadoVariables.x[182];
acadoWorkspace.objValueIn[3] = acadoVariables.x[183];
acadoWorkspace.objValueIn[4] = acadoVariables.x[184];
acadoWorkspace.objValueIn[5] = acadoVariables.x[185];
acadoWorkspace.objValueIn[6] = acadoVariables.x[186];
acadoWorkspace.objValueIn[7] = acadoVariables.x[187];
acadoWorkspace.objValueIn[8] = acadoVariables.x[188];
acadoWorkspace.objValueIn[9] = acadoVariables.od[180];
acadoWorkspace.objValueIn[10] = acadoVariables.od[181];
acadoWorkspace.objValueIn[11] = acadoVariables.od[182];
acadoWorkspace.objValueIn[12] = acadoVariables.od[183];
acadoWorkspace.objValueIn[13] = acadoVariables.od[184];
acadoWorkspace.objValueIn[14] = acadoVariables.od[185];
acadoWorkspace.objValueIn[15] = acadoVariables.od[186];
acadoWorkspace.objValueIn[16] = acadoVariables.od[187];
acadoWorkspace.objValueIn[17] = acadoVariables.od[188];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9] + Gx1[4]*Gu1[12] + Gx1[5]*Gu1[15] + Gx1[6]*Gu1[18] + Gx1[7]*Gu1[21] + Gx1[8]*Gu1[24];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10] + Gx1[4]*Gu1[13] + Gx1[5]*Gu1[16] + Gx1[6]*Gu1[19] + Gx1[7]*Gu1[22] + Gx1[8]*Gu1[25];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11] + Gx1[4]*Gu1[14] + Gx1[5]*Gu1[17] + Gx1[6]*Gu1[20] + Gx1[7]*Gu1[23] + Gx1[8]*Gu1[26];
Gu2[3] = + Gx1[9]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[12] + Gx1[14]*Gu1[15] + Gx1[15]*Gu1[18] + Gx1[16]*Gu1[21] + Gx1[17]*Gu1[24];
Gu2[4] = + Gx1[9]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[10] + Gx1[13]*Gu1[13] + Gx1[14]*Gu1[16] + Gx1[15]*Gu1[19] + Gx1[16]*Gu1[22] + Gx1[17]*Gu1[25];
Gu2[5] = + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[8] + Gx1[12]*Gu1[11] + Gx1[13]*Gu1[14] + Gx1[14]*Gu1[17] + Gx1[15]*Gu1[20] + Gx1[16]*Gu1[23] + Gx1[17]*Gu1[26];
Gu2[6] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[21]*Gu1[9] + Gx1[22]*Gu1[12] + Gx1[23]*Gu1[15] + Gx1[24]*Gu1[18] + Gx1[25]*Gu1[21] + Gx1[26]*Gu1[24];
Gu2[7] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[21]*Gu1[10] + Gx1[22]*Gu1[13] + Gx1[23]*Gu1[16] + Gx1[24]*Gu1[19] + Gx1[25]*Gu1[22] + Gx1[26]*Gu1[25];
Gu2[8] = + Gx1[18]*Gu1[2] + Gx1[19]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[11] + Gx1[22]*Gu1[14] + Gx1[23]*Gu1[17] + Gx1[24]*Gu1[20] + Gx1[25]*Gu1[23] + Gx1[26]*Gu1[26];
Gu2[9] = + Gx1[27]*Gu1[0] + Gx1[28]*Gu1[3] + Gx1[29]*Gu1[6] + Gx1[30]*Gu1[9] + Gx1[31]*Gu1[12] + Gx1[32]*Gu1[15] + Gx1[33]*Gu1[18] + Gx1[34]*Gu1[21] + Gx1[35]*Gu1[24];
Gu2[10] = + Gx1[27]*Gu1[1] + Gx1[28]*Gu1[4] + Gx1[29]*Gu1[7] + Gx1[30]*Gu1[10] + Gx1[31]*Gu1[13] + Gx1[32]*Gu1[16] + Gx1[33]*Gu1[19] + Gx1[34]*Gu1[22] + Gx1[35]*Gu1[25];
Gu2[11] = + Gx1[27]*Gu1[2] + Gx1[28]*Gu1[5] + Gx1[29]*Gu1[8] + Gx1[30]*Gu1[11] + Gx1[31]*Gu1[14] + Gx1[32]*Gu1[17] + Gx1[33]*Gu1[20] + Gx1[34]*Gu1[23] + Gx1[35]*Gu1[26];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[3] + Gx1[38]*Gu1[6] + Gx1[39]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[41]*Gu1[15] + Gx1[42]*Gu1[18] + Gx1[43]*Gu1[21] + Gx1[44]*Gu1[24];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[7] + Gx1[39]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[41]*Gu1[16] + Gx1[42]*Gu1[19] + Gx1[43]*Gu1[22] + Gx1[44]*Gu1[25];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[41]*Gu1[17] + Gx1[42]*Gu1[20] + Gx1[43]*Gu1[23] + Gx1[44]*Gu1[26];
Gu2[15] = + Gx1[45]*Gu1[0] + Gx1[46]*Gu1[3] + Gx1[47]*Gu1[6] + Gx1[48]*Gu1[9] + Gx1[49]*Gu1[12] + Gx1[50]*Gu1[15] + Gx1[51]*Gu1[18] + Gx1[52]*Gu1[21] + Gx1[53]*Gu1[24];
Gu2[16] = + Gx1[45]*Gu1[1] + Gx1[46]*Gu1[4] + Gx1[47]*Gu1[7] + Gx1[48]*Gu1[10] + Gx1[49]*Gu1[13] + Gx1[50]*Gu1[16] + Gx1[51]*Gu1[19] + Gx1[52]*Gu1[22] + Gx1[53]*Gu1[25];
Gu2[17] = + Gx1[45]*Gu1[2] + Gx1[46]*Gu1[5] + Gx1[47]*Gu1[8] + Gx1[48]*Gu1[11] + Gx1[49]*Gu1[14] + Gx1[50]*Gu1[17] + Gx1[51]*Gu1[20] + Gx1[52]*Gu1[23] + Gx1[53]*Gu1[26];
Gu2[18] = + Gx1[54]*Gu1[0] + Gx1[55]*Gu1[3] + Gx1[56]*Gu1[6] + Gx1[57]*Gu1[9] + Gx1[58]*Gu1[12] + Gx1[59]*Gu1[15] + Gx1[60]*Gu1[18] + Gx1[61]*Gu1[21] + Gx1[62]*Gu1[24];
Gu2[19] = + Gx1[54]*Gu1[1] + Gx1[55]*Gu1[4] + Gx1[56]*Gu1[7] + Gx1[57]*Gu1[10] + Gx1[58]*Gu1[13] + Gx1[59]*Gu1[16] + Gx1[60]*Gu1[19] + Gx1[61]*Gu1[22] + Gx1[62]*Gu1[25];
Gu2[20] = + Gx1[54]*Gu1[2] + Gx1[55]*Gu1[5] + Gx1[56]*Gu1[8] + Gx1[57]*Gu1[11] + Gx1[58]*Gu1[14] + Gx1[59]*Gu1[17] + Gx1[60]*Gu1[20] + Gx1[61]*Gu1[23] + Gx1[62]*Gu1[26];
Gu2[21] = + Gx1[63]*Gu1[0] + Gx1[64]*Gu1[3] + Gx1[65]*Gu1[6] + Gx1[66]*Gu1[9] + Gx1[67]*Gu1[12] + Gx1[68]*Gu1[15] + Gx1[69]*Gu1[18] + Gx1[70]*Gu1[21] + Gx1[71]*Gu1[24];
Gu2[22] = + Gx1[63]*Gu1[1] + Gx1[64]*Gu1[4] + Gx1[65]*Gu1[7] + Gx1[66]*Gu1[10] + Gx1[67]*Gu1[13] + Gx1[68]*Gu1[16] + Gx1[69]*Gu1[19] + Gx1[70]*Gu1[22] + Gx1[71]*Gu1[25];
Gu2[23] = + Gx1[63]*Gu1[2] + Gx1[64]*Gu1[5] + Gx1[65]*Gu1[8] + Gx1[66]*Gu1[11] + Gx1[67]*Gu1[14] + Gx1[68]*Gu1[17] + Gx1[69]*Gu1[20] + Gx1[70]*Gu1[23] + Gx1[71]*Gu1[26];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[3] + Gx1[74]*Gu1[6] + Gx1[75]*Gu1[9] + Gx1[76]*Gu1[12] + Gx1[77]*Gu1[15] + Gx1[78]*Gu1[18] + Gx1[79]*Gu1[21] + Gx1[80]*Gu1[24];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[7] + Gx1[75]*Gu1[10] + Gx1[76]*Gu1[13] + Gx1[77]*Gu1[16] + Gx1[78]*Gu1[19] + Gx1[79]*Gu1[22] + Gx1[80]*Gu1[25];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[11] + Gx1[76]*Gu1[14] + Gx1[77]*Gu1[17] + Gx1[78]*Gu1[20] + Gx1[79]*Gu1[23] + Gx1[80]*Gu1[26];
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
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] += + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] += + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] += + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] += + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] += + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] += + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] += + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] += + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] += + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 183] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + R11[0];
acadoWorkspace.H[iRow * 183 + 1] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + R11[1];
acadoWorkspace.H[iRow * 183 + 2] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + R11[2];
acadoWorkspace.H[iRow * 183 + 60] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + R11[3];
acadoWorkspace.H[iRow * 183 + 61] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + R11[4];
acadoWorkspace.H[iRow * 183 + 62] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + R11[5];
acadoWorkspace.H[iRow * 183 + 120] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + R11[6];
acadoWorkspace.H[iRow * 183 + 121] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + R11[7];
acadoWorkspace.H[iRow * 183 + 122] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + R11[8];
acadoWorkspace.H[iRow * 183] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 183 + 61] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 183 + 122] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[9]*Gu1[3] + Gx1[18]*Gu1[6] + Gx1[27]*Gu1[9] + Gx1[36]*Gu1[12] + Gx1[45]*Gu1[15] + Gx1[54]*Gu1[18] + Gx1[63]*Gu1[21] + Gx1[72]*Gu1[24];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[9]*Gu1[4] + Gx1[18]*Gu1[7] + Gx1[27]*Gu1[10] + Gx1[36]*Gu1[13] + Gx1[45]*Gu1[16] + Gx1[54]*Gu1[19] + Gx1[63]*Gu1[22] + Gx1[72]*Gu1[25];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[9]*Gu1[5] + Gx1[18]*Gu1[8] + Gx1[27]*Gu1[11] + Gx1[36]*Gu1[14] + Gx1[45]*Gu1[17] + Gx1[54]*Gu1[20] + Gx1[63]*Gu1[23] + Gx1[72]*Gu1[26];
Gu2[3] = + Gx1[1]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[19]*Gu1[6] + Gx1[28]*Gu1[9] + Gx1[37]*Gu1[12] + Gx1[46]*Gu1[15] + Gx1[55]*Gu1[18] + Gx1[64]*Gu1[21] + Gx1[73]*Gu1[24];
Gu2[4] = + Gx1[1]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[19]*Gu1[7] + Gx1[28]*Gu1[10] + Gx1[37]*Gu1[13] + Gx1[46]*Gu1[16] + Gx1[55]*Gu1[19] + Gx1[64]*Gu1[22] + Gx1[73]*Gu1[25];
Gu2[5] = + Gx1[1]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[19]*Gu1[8] + Gx1[28]*Gu1[11] + Gx1[37]*Gu1[14] + Gx1[46]*Gu1[17] + Gx1[55]*Gu1[20] + Gx1[64]*Gu1[23] + Gx1[73]*Gu1[26];
Gu2[6] = + Gx1[2]*Gu1[0] + Gx1[11]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[29]*Gu1[9] + Gx1[38]*Gu1[12] + Gx1[47]*Gu1[15] + Gx1[56]*Gu1[18] + Gx1[65]*Gu1[21] + Gx1[74]*Gu1[24];
Gu2[7] = + Gx1[2]*Gu1[1] + Gx1[11]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[29]*Gu1[10] + Gx1[38]*Gu1[13] + Gx1[47]*Gu1[16] + Gx1[56]*Gu1[19] + Gx1[65]*Gu1[22] + Gx1[74]*Gu1[25];
Gu2[8] = + Gx1[2]*Gu1[2] + Gx1[11]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[29]*Gu1[11] + Gx1[38]*Gu1[14] + Gx1[47]*Gu1[17] + Gx1[56]*Gu1[20] + Gx1[65]*Gu1[23] + Gx1[74]*Gu1[26];
Gu2[9] = + Gx1[3]*Gu1[0] + Gx1[12]*Gu1[3] + Gx1[21]*Gu1[6] + Gx1[30]*Gu1[9] + Gx1[39]*Gu1[12] + Gx1[48]*Gu1[15] + Gx1[57]*Gu1[18] + Gx1[66]*Gu1[21] + Gx1[75]*Gu1[24];
Gu2[10] = + Gx1[3]*Gu1[1] + Gx1[12]*Gu1[4] + Gx1[21]*Gu1[7] + Gx1[30]*Gu1[10] + Gx1[39]*Gu1[13] + Gx1[48]*Gu1[16] + Gx1[57]*Gu1[19] + Gx1[66]*Gu1[22] + Gx1[75]*Gu1[25];
Gu2[11] = + Gx1[3]*Gu1[2] + Gx1[12]*Gu1[5] + Gx1[21]*Gu1[8] + Gx1[30]*Gu1[11] + Gx1[39]*Gu1[14] + Gx1[48]*Gu1[17] + Gx1[57]*Gu1[20] + Gx1[66]*Gu1[23] + Gx1[75]*Gu1[26];
Gu2[12] = + Gx1[4]*Gu1[0] + Gx1[13]*Gu1[3] + Gx1[22]*Gu1[6] + Gx1[31]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[49]*Gu1[15] + Gx1[58]*Gu1[18] + Gx1[67]*Gu1[21] + Gx1[76]*Gu1[24];
Gu2[13] = + Gx1[4]*Gu1[1] + Gx1[13]*Gu1[4] + Gx1[22]*Gu1[7] + Gx1[31]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[49]*Gu1[16] + Gx1[58]*Gu1[19] + Gx1[67]*Gu1[22] + Gx1[76]*Gu1[25];
Gu2[14] = + Gx1[4]*Gu1[2] + Gx1[13]*Gu1[5] + Gx1[22]*Gu1[8] + Gx1[31]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[49]*Gu1[17] + Gx1[58]*Gu1[20] + Gx1[67]*Gu1[23] + Gx1[76]*Gu1[26];
Gu2[15] = + Gx1[5]*Gu1[0] + Gx1[14]*Gu1[3] + Gx1[23]*Gu1[6] + Gx1[32]*Gu1[9] + Gx1[41]*Gu1[12] + Gx1[50]*Gu1[15] + Gx1[59]*Gu1[18] + Gx1[68]*Gu1[21] + Gx1[77]*Gu1[24];
Gu2[16] = + Gx1[5]*Gu1[1] + Gx1[14]*Gu1[4] + Gx1[23]*Gu1[7] + Gx1[32]*Gu1[10] + Gx1[41]*Gu1[13] + Gx1[50]*Gu1[16] + Gx1[59]*Gu1[19] + Gx1[68]*Gu1[22] + Gx1[77]*Gu1[25];
Gu2[17] = + Gx1[5]*Gu1[2] + Gx1[14]*Gu1[5] + Gx1[23]*Gu1[8] + Gx1[32]*Gu1[11] + Gx1[41]*Gu1[14] + Gx1[50]*Gu1[17] + Gx1[59]*Gu1[20] + Gx1[68]*Gu1[23] + Gx1[77]*Gu1[26];
Gu2[18] = + Gx1[6]*Gu1[0] + Gx1[15]*Gu1[3] + Gx1[24]*Gu1[6] + Gx1[33]*Gu1[9] + Gx1[42]*Gu1[12] + Gx1[51]*Gu1[15] + Gx1[60]*Gu1[18] + Gx1[69]*Gu1[21] + Gx1[78]*Gu1[24];
Gu2[19] = + Gx1[6]*Gu1[1] + Gx1[15]*Gu1[4] + Gx1[24]*Gu1[7] + Gx1[33]*Gu1[10] + Gx1[42]*Gu1[13] + Gx1[51]*Gu1[16] + Gx1[60]*Gu1[19] + Gx1[69]*Gu1[22] + Gx1[78]*Gu1[25];
Gu2[20] = + Gx1[6]*Gu1[2] + Gx1[15]*Gu1[5] + Gx1[24]*Gu1[8] + Gx1[33]*Gu1[11] + Gx1[42]*Gu1[14] + Gx1[51]*Gu1[17] + Gx1[60]*Gu1[20] + Gx1[69]*Gu1[23] + Gx1[78]*Gu1[26];
Gu2[21] = + Gx1[7]*Gu1[0] + Gx1[16]*Gu1[3] + Gx1[25]*Gu1[6] + Gx1[34]*Gu1[9] + Gx1[43]*Gu1[12] + Gx1[52]*Gu1[15] + Gx1[61]*Gu1[18] + Gx1[70]*Gu1[21] + Gx1[79]*Gu1[24];
Gu2[22] = + Gx1[7]*Gu1[1] + Gx1[16]*Gu1[4] + Gx1[25]*Gu1[7] + Gx1[34]*Gu1[10] + Gx1[43]*Gu1[13] + Gx1[52]*Gu1[16] + Gx1[61]*Gu1[19] + Gx1[70]*Gu1[22] + Gx1[79]*Gu1[25];
Gu2[23] = + Gx1[7]*Gu1[2] + Gx1[16]*Gu1[5] + Gx1[25]*Gu1[8] + Gx1[34]*Gu1[11] + Gx1[43]*Gu1[14] + Gx1[52]*Gu1[17] + Gx1[61]*Gu1[20] + Gx1[70]*Gu1[23] + Gx1[79]*Gu1[26];
Gu2[24] = + Gx1[8]*Gu1[0] + Gx1[17]*Gu1[3] + Gx1[26]*Gu1[6] + Gx1[35]*Gu1[9] + Gx1[44]*Gu1[12] + Gx1[53]*Gu1[15] + Gx1[62]*Gu1[18] + Gx1[71]*Gu1[21] + Gx1[80]*Gu1[24];
Gu2[25] = + Gx1[8]*Gu1[1] + Gx1[17]*Gu1[4] + Gx1[26]*Gu1[7] + Gx1[35]*Gu1[10] + Gx1[44]*Gu1[13] + Gx1[53]*Gu1[16] + Gx1[62]*Gu1[19] + Gx1[71]*Gu1[22] + Gx1[80]*Gu1[25];
Gu2[26] = + Gx1[8]*Gu1[2] + Gx1[17]*Gu1[5] + Gx1[26]*Gu1[8] + Gx1[35]*Gu1[11] + Gx1[44]*Gu1[14] + Gx1[53]*Gu1[17] + Gx1[62]*Gu1[20] + Gx1[71]*Gu1[23] + Gx1[80]*Gu1[26];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[3] + Q11[2]*Gu1[6] + Q11[3]*Gu1[9] + Q11[4]*Gu1[12] + Q11[5]*Gu1[15] + Q11[6]*Gu1[18] + Q11[7]*Gu1[21] + Q11[8]*Gu1[24] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[4] + Q11[2]*Gu1[7] + Q11[3]*Gu1[10] + Q11[4]*Gu1[13] + Q11[5]*Gu1[16] + Q11[6]*Gu1[19] + Q11[7]*Gu1[22] + Q11[8]*Gu1[25] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[5] + Q11[2]*Gu1[8] + Q11[3]*Gu1[11] + Q11[4]*Gu1[14] + Q11[5]*Gu1[17] + Q11[6]*Gu1[20] + Q11[7]*Gu1[23] + Q11[8]*Gu1[26] + Gu2[2];
Gu3[3] = + Q11[9]*Gu1[0] + Q11[10]*Gu1[3] + Q11[11]*Gu1[6] + Q11[12]*Gu1[9] + Q11[13]*Gu1[12] + Q11[14]*Gu1[15] + Q11[15]*Gu1[18] + Q11[16]*Gu1[21] + Q11[17]*Gu1[24] + Gu2[3];
Gu3[4] = + Q11[9]*Gu1[1] + Q11[10]*Gu1[4] + Q11[11]*Gu1[7] + Q11[12]*Gu1[10] + Q11[13]*Gu1[13] + Q11[14]*Gu1[16] + Q11[15]*Gu1[19] + Q11[16]*Gu1[22] + Q11[17]*Gu1[25] + Gu2[4];
Gu3[5] = + Q11[9]*Gu1[2] + Q11[10]*Gu1[5] + Q11[11]*Gu1[8] + Q11[12]*Gu1[11] + Q11[13]*Gu1[14] + Q11[14]*Gu1[17] + Q11[15]*Gu1[20] + Q11[16]*Gu1[23] + Q11[17]*Gu1[26] + Gu2[5];
Gu3[6] = + Q11[18]*Gu1[0] + Q11[19]*Gu1[3] + Q11[20]*Gu1[6] + Q11[21]*Gu1[9] + Q11[22]*Gu1[12] + Q11[23]*Gu1[15] + Q11[24]*Gu1[18] + Q11[25]*Gu1[21] + Q11[26]*Gu1[24] + Gu2[6];
Gu3[7] = + Q11[18]*Gu1[1] + Q11[19]*Gu1[4] + Q11[20]*Gu1[7] + Q11[21]*Gu1[10] + Q11[22]*Gu1[13] + Q11[23]*Gu1[16] + Q11[24]*Gu1[19] + Q11[25]*Gu1[22] + Q11[26]*Gu1[25] + Gu2[7];
Gu3[8] = + Q11[18]*Gu1[2] + Q11[19]*Gu1[5] + Q11[20]*Gu1[8] + Q11[21]*Gu1[11] + Q11[22]*Gu1[14] + Q11[23]*Gu1[17] + Q11[24]*Gu1[20] + Q11[25]*Gu1[23] + Q11[26]*Gu1[26] + Gu2[8];
Gu3[9] = + Q11[27]*Gu1[0] + Q11[28]*Gu1[3] + Q11[29]*Gu1[6] + Q11[30]*Gu1[9] + Q11[31]*Gu1[12] + Q11[32]*Gu1[15] + Q11[33]*Gu1[18] + Q11[34]*Gu1[21] + Q11[35]*Gu1[24] + Gu2[9];
Gu3[10] = + Q11[27]*Gu1[1] + Q11[28]*Gu1[4] + Q11[29]*Gu1[7] + Q11[30]*Gu1[10] + Q11[31]*Gu1[13] + Q11[32]*Gu1[16] + Q11[33]*Gu1[19] + Q11[34]*Gu1[22] + Q11[35]*Gu1[25] + Gu2[10];
Gu3[11] = + Q11[27]*Gu1[2] + Q11[28]*Gu1[5] + Q11[29]*Gu1[8] + Q11[30]*Gu1[11] + Q11[31]*Gu1[14] + Q11[32]*Gu1[17] + Q11[33]*Gu1[20] + Q11[34]*Gu1[23] + Q11[35]*Gu1[26] + Gu2[11];
Gu3[12] = + Q11[36]*Gu1[0] + Q11[37]*Gu1[3] + Q11[38]*Gu1[6] + Q11[39]*Gu1[9] + Q11[40]*Gu1[12] + Q11[41]*Gu1[15] + Q11[42]*Gu1[18] + Q11[43]*Gu1[21] + Q11[44]*Gu1[24] + Gu2[12];
Gu3[13] = + Q11[36]*Gu1[1] + Q11[37]*Gu1[4] + Q11[38]*Gu1[7] + Q11[39]*Gu1[10] + Q11[40]*Gu1[13] + Q11[41]*Gu1[16] + Q11[42]*Gu1[19] + Q11[43]*Gu1[22] + Q11[44]*Gu1[25] + Gu2[13];
Gu3[14] = + Q11[36]*Gu1[2] + Q11[37]*Gu1[5] + Q11[38]*Gu1[8] + Q11[39]*Gu1[11] + Q11[40]*Gu1[14] + Q11[41]*Gu1[17] + Q11[42]*Gu1[20] + Q11[43]*Gu1[23] + Q11[44]*Gu1[26] + Gu2[14];
Gu3[15] = + Q11[45]*Gu1[0] + Q11[46]*Gu1[3] + Q11[47]*Gu1[6] + Q11[48]*Gu1[9] + Q11[49]*Gu1[12] + Q11[50]*Gu1[15] + Q11[51]*Gu1[18] + Q11[52]*Gu1[21] + Q11[53]*Gu1[24] + Gu2[15];
Gu3[16] = + Q11[45]*Gu1[1] + Q11[46]*Gu1[4] + Q11[47]*Gu1[7] + Q11[48]*Gu1[10] + Q11[49]*Gu1[13] + Q11[50]*Gu1[16] + Q11[51]*Gu1[19] + Q11[52]*Gu1[22] + Q11[53]*Gu1[25] + Gu2[16];
Gu3[17] = + Q11[45]*Gu1[2] + Q11[46]*Gu1[5] + Q11[47]*Gu1[8] + Q11[48]*Gu1[11] + Q11[49]*Gu1[14] + Q11[50]*Gu1[17] + Q11[51]*Gu1[20] + Q11[52]*Gu1[23] + Q11[53]*Gu1[26] + Gu2[17];
Gu3[18] = + Q11[54]*Gu1[0] + Q11[55]*Gu1[3] + Q11[56]*Gu1[6] + Q11[57]*Gu1[9] + Q11[58]*Gu1[12] + Q11[59]*Gu1[15] + Q11[60]*Gu1[18] + Q11[61]*Gu1[21] + Q11[62]*Gu1[24] + Gu2[18];
Gu3[19] = + Q11[54]*Gu1[1] + Q11[55]*Gu1[4] + Q11[56]*Gu1[7] + Q11[57]*Gu1[10] + Q11[58]*Gu1[13] + Q11[59]*Gu1[16] + Q11[60]*Gu1[19] + Q11[61]*Gu1[22] + Q11[62]*Gu1[25] + Gu2[19];
Gu3[20] = + Q11[54]*Gu1[2] + Q11[55]*Gu1[5] + Q11[56]*Gu1[8] + Q11[57]*Gu1[11] + Q11[58]*Gu1[14] + Q11[59]*Gu1[17] + Q11[60]*Gu1[20] + Q11[61]*Gu1[23] + Q11[62]*Gu1[26] + Gu2[20];
Gu3[21] = + Q11[63]*Gu1[0] + Q11[64]*Gu1[3] + Q11[65]*Gu1[6] + Q11[66]*Gu1[9] + Q11[67]*Gu1[12] + Q11[68]*Gu1[15] + Q11[69]*Gu1[18] + Q11[70]*Gu1[21] + Q11[71]*Gu1[24] + Gu2[21];
Gu3[22] = + Q11[63]*Gu1[1] + Q11[64]*Gu1[4] + Q11[65]*Gu1[7] + Q11[66]*Gu1[10] + Q11[67]*Gu1[13] + Q11[68]*Gu1[16] + Q11[69]*Gu1[19] + Q11[70]*Gu1[22] + Q11[71]*Gu1[25] + Gu2[22];
Gu3[23] = + Q11[63]*Gu1[2] + Q11[64]*Gu1[5] + Q11[65]*Gu1[8] + Q11[66]*Gu1[11] + Q11[67]*Gu1[14] + Q11[68]*Gu1[17] + Q11[69]*Gu1[20] + Q11[70]*Gu1[23] + Q11[71]*Gu1[26] + Gu2[23];
Gu3[24] = + Q11[72]*Gu1[0] + Q11[73]*Gu1[3] + Q11[74]*Gu1[6] + Q11[75]*Gu1[9] + Q11[76]*Gu1[12] + Q11[77]*Gu1[15] + Q11[78]*Gu1[18] + Q11[79]*Gu1[21] + Q11[80]*Gu1[24] + Gu2[24];
Gu3[25] = + Q11[72]*Gu1[1] + Q11[73]*Gu1[4] + Q11[74]*Gu1[7] + Q11[75]*Gu1[10] + Q11[76]*Gu1[13] + Q11[77]*Gu1[16] + Q11[78]*Gu1[19] + Q11[79]*Gu1[22] + Q11[80]*Gu1[25] + Gu2[25];
Gu3[26] = + Q11[72]*Gu1[2] + Q11[73]*Gu1[5] + Q11[74]*Gu1[8] + Q11[75]*Gu1[11] + Q11[76]*Gu1[14] + Q11[77]*Gu1[17] + Q11[78]*Gu1[20] + Q11[79]*Gu1[23] + Q11[80]*Gu1[26] + Gu2[26];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[9]*w11[1] + Gx1[18]*w11[2] + Gx1[27]*w11[3] + Gx1[36]*w11[4] + Gx1[45]*w11[5] + Gx1[54]*w11[6] + Gx1[63]*w11[7] + Gx1[72]*w11[8] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[10]*w11[1] + Gx1[19]*w11[2] + Gx1[28]*w11[3] + Gx1[37]*w11[4] + Gx1[46]*w11[5] + Gx1[55]*w11[6] + Gx1[64]*w11[7] + Gx1[73]*w11[8] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[11]*w11[1] + Gx1[20]*w11[2] + Gx1[29]*w11[3] + Gx1[38]*w11[4] + Gx1[47]*w11[5] + Gx1[56]*w11[6] + Gx1[65]*w11[7] + Gx1[74]*w11[8] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[12]*w11[1] + Gx1[21]*w11[2] + Gx1[30]*w11[3] + Gx1[39]*w11[4] + Gx1[48]*w11[5] + Gx1[57]*w11[6] + Gx1[66]*w11[7] + Gx1[75]*w11[8] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[13]*w11[1] + Gx1[22]*w11[2] + Gx1[31]*w11[3] + Gx1[40]*w11[4] + Gx1[49]*w11[5] + Gx1[58]*w11[6] + Gx1[67]*w11[7] + Gx1[76]*w11[8] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[14]*w11[1] + Gx1[23]*w11[2] + Gx1[32]*w11[3] + Gx1[41]*w11[4] + Gx1[50]*w11[5] + Gx1[59]*w11[6] + Gx1[68]*w11[7] + Gx1[77]*w11[8] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[15]*w11[1] + Gx1[24]*w11[2] + Gx1[33]*w11[3] + Gx1[42]*w11[4] + Gx1[51]*w11[5] + Gx1[60]*w11[6] + Gx1[69]*w11[7] + Gx1[78]*w11[8] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[16]*w11[1] + Gx1[25]*w11[2] + Gx1[34]*w11[3] + Gx1[43]*w11[4] + Gx1[52]*w11[5] + Gx1[61]*w11[6] + Gx1[70]*w11[7] + Gx1[79]*w11[8] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[17]*w11[1] + Gx1[26]*w11[2] + Gx1[35]*w11[3] + Gx1[44]*w11[4] + Gx1[53]*w11[5] + Gx1[62]*w11[6] + Gx1[71]*w11[7] + Gx1[80]*w11[8] + w12[8];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5] + Gu1[18]*w11[6] + Gu1[21]*w11[7] + Gu1[24]*w11[8];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5] + Gu1[19]*w11[6] + Gu1[22]*w11[7] + Gu1[25]*w11[8];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5] + Gu1[20]*w11[6] + Gu1[23]*w11[7] + Gu1[26]*w11[8];
}

void acado_macS1TSbar( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5] + Gu1[18]*w11[6] + Gu1[21]*w11[7] + Gu1[24]*w11[8];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5] + Gu1[19]*w11[6] + Gu1[22]*w11[7] + Gu1[25]*w11[8];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5] + Gu1[20]*w11[6] + Gu1[23]*w11[7] + Gu1[26]*w11[8];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + Q11[8]*w11[8] + w12[0];
w13[1] = + Q11[9]*w11[0] + Q11[10]*w11[1] + Q11[11]*w11[2] + Q11[12]*w11[3] + Q11[13]*w11[4] + Q11[14]*w11[5] + Q11[15]*w11[6] + Q11[16]*w11[7] + Q11[17]*w11[8] + w12[1];
w13[2] = + Q11[18]*w11[0] + Q11[19]*w11[1] + Q11[20]*w11[2] + Q11[21]*w11[3] + Q11[22]*w11[4] + Q11[23]*w11[5] + Q11[24]*w11[6] + Q11[25]*w11[7] + Q11[26]*w11[8] + w12[2];
w13[3] = + Q11[27]*w11[0] + Q11[28]*w11[1] + Q11[29]*w11[2] + Q11[30]*w11[3] + Q11[31]*w11[4] + Q11[32]*w11[5] + Q11[33]*w11[6] + Q11[34]*w11[7] + Q11[35]*w11[8] + w12[3];
w13[4] = + Q11[36]*w11[0] + Q11[37]*w11[1] + Q11[38]*w11[2] + Q11[39]*w11[3] + Q11[40]*w11[4] + Q11[41]*w11[5] + Q11[42]*w11[6] + Q11[43]*w11[7] + Q11[44]*w11[8] + w12[4];
w13[5] = + Q11[45]*w11[0] + Q11[46]*w11[1] + Q11[47]*w11[2] + Q11[48]*w11[3] + Q11[49]*w11[4] + Q11[50]*w11[5] + Q11[51]*w11[6] + Q11[52]*w11[7] + Q11[53]*w11[8] + w12[5];
w13[6] = + Q11[54]*w11[0] + Q11[55]*w11[1] + Q11[56]*w11[2] + Q11[57]*w11[3] + Q11[58]*w11[4] + Q11[59]*w11[5] + Q11[60]*w11[6] + Q11[61]*w11[7] + Q11[62]*w11[8] + w12[6];
w13[7] = + Q11[63]*w11[0] + Q11[64]*w11[1] + Q11[65]*w11[2] + Q11[66]*w11[3] + Q11[67]*w11[4] + Q11[68]*w11[5] + Q11[69]*w11[6] + Q11[70]*w11[7] + Q11[71]*w11[8] + w12[7];
w13[8] = + Q11[72]*w11[0] + Q11[73]*w11[1] + Q11[74]*w11[2] + Q11[75]*w11[3] + Q11[76]*w11[4] + Q11[77]*w11[5] + Q11[78]*w11[6] + Q11[79]*w11[7] + Q11[80]*w11[8] + w12[8];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8];
w12[1] += + Gx1[9]*w11[0] + Gx1[10]*w11[1] + Gx1[11]*w11[2] + Gx1[12]*w11[3] + Gx1[13]*w11[4] + Gx1[14]*w11[5] + Gx1[15]*w11[6] + Gx1[16]*w11[7] + Gx1[17]*w11[8];
w12[2] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8];
w12[3] += + Gx1[27]*w11[0] + Gx1[28]*w11[1] + Gx1[29]*w11[2] + Gx1[30]*w11[3] + Gx1[31]*w11[4] + Gx1[32]*w11[5] + Gx1[33]*w11[6] + Gx1[34]*w11[7] + Gx1[35]*w11[8];
w12[4] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8];
w12[5] += + Gx1[45]*w11[0] + Gx1[46]*w11[1] + Gx1[47]*w11[2] + Gx1[48]*w11[3] + Gx1[49]*w11[4] + Gx1[50]*w11[5] + Gx1[51]*w11[6] + Gx1[52]*w11[7] + Gx1[53]*w11[8];
w12[6] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8];
w12[7] += + Gx1[63]*w11[0] + Gx1[64]*w11[1] + Gx1[65]*w11[2] + Gx1[66]*w11[3] + Gx1[67]*w11[4] + Gx1[68]*w11[5] + Gx1[69]*w11[6] + Gx1[70]*w11[7] + Gx1[71]*w11[8];
w12[8] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8];
w12[1] += + Gx1[9]*w11[0] + Gx1[10]*w11[1] + Gx1[11]*w11[2] + Gx1[12]*w11[3] + Gx1[13]*w11[4] + Gx1[14]*w11[5] + Gx1[15]*w11[6] + Gx1[16]*w11[7] + Gx1[17]*w11[8];
w12[2] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8];
w12[3] += + Gx1[27]*w11[0] + Gx1[28]*w11[1] + Gx1[29]*w11[2] + Gx1[30]*w11[3] + Gx1[31]*w11[4] + Gx1[32]*w11[5] + Gx1[33]*w11[6] + Gx1[34]*w11[7] + Gx1[35]*w11[8];
w12[4] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8];
w12[5] += + Gx1[45]*w11[0] + Gx1[46]*w11[1] + Gx1[47]*w11[2] + Gx1[48]*w11[3] + Gx1[49]*w11[4] + Gx1[50]*w11[5] + Gx1[51]*w11[6] + Gx1[52]*w11[7] + Gx1[53]*w11[8];
w12[6] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8];
w12[7] += + Gx1[63]*w11[0] + Gx1[64]*w11[1] + Gx1[65]*w11[2] + Gx1[66]*w11[3] + Gx1[67]*w11[4] + Gx1[68]*w11[5] + Gx1[69]*w11[6] + Gx1[70]*w11[7] + Gx1[71]*w11[8];
w12[8] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2];
w12[1] += + Gu1[3]*U1[0] + Gu1[4]*U1[1] + Gu1[5]*U1[2];
w12[2] += + Gu1[6]*U1[0] + Gu1[7]*U1[1] + Gu1[8]*U1[2];
w12[3] += + Gu1[9]*U1[0] + Gu1[10]*U1[1] + Gu1[11]*U1[2];
w12[4] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2];
w12[5] += + Gu1[15]*U1[0] + Gu1[16]*U1[1] + Gu1[17]*U1[2];
w12[6] += + Gu1[18]*U1[0] + Gu1[19]*U1[1] + Gu1[20]*U1[2];
w12[7] += + Gu1[21]*U1[0] + Gu1[22]*U1[1] + Gu1[23]*U1[2];
w12[8] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3 + 2)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10];
RDy1[1] = + R2[11]*Dy1[0] + R2[12]*Dy1[1] + R2[13]*Dy1[2] + R2[14]*Dy1[3] + R2[15]*Dy1[4] + R2[16]*Dy1[5] + R2[17]*Dy1[6] + R2[18]*Dy1[7] + R2[19]*Dy1[8] + R2[20]*Dy1[9] + R2[21]*Dy1[10];
RDy1[2] = + R2[22]*Dy1[0] + R2[23]*Dy1[1] + R2[24]*Dy1[2] + R2[25]*Dy1[3] + R2[26]*Dy1[4] + R2[27]*Dy1[5] + R2[28]*Dy1[6] + R2[29]*Dy1[7] + R2[30]*Dy1[8] + R2[31]*Dy1[9] + R2[32]*Dy1[10];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10];
QDy1[1] = + Q2[11]*Dy1[0] + Q2[12]*Dy1[1] + Q2[13]*Dy1[2] + Q2[14]*Dy1[3] + Q2[15]*Dy1[4] + Q2[16]*Dy1[5] + Q2[17]*Dy1[6] + Q2[18]*Dy1[7] + Q2[19]*Dy1[8] + Q2[20]*Dy1[9] + Q2[21]*Dy1[10];
QDy1[2] = + Q2[22]*Dy1[0] + Q2[23]*Dy1[1] + Q2[24]*Dy1[2] + Q2[25]*Dy1[3] + Q2[26]*Dy1[4] + Q2[27]*Dy1[5] + Q2[28]*Dy1[6] + Q2[29]*Dy1[7] + Q2[30]*Dy1[8] + Q2[31]*Dy1[9] + Q2[32]*Dy1[10];
QDy1[3] = + Q2[33]*Dy1[0] + Q2[34]*Dy1[1] + Q2[35]*Dy1[2] + Q2[36]*Dy1[3] + Q2[37]*Dy1[4] + Q2[38]*Dy1[5] + Q2[39]*Dy1[6] + Q2[40]*Dy1[7] + Q2[41]*Dy1[8] + Q2[42]*Dy1[9] + Q2[43]*Dy1[10];
QDy1[4] = + Q2[44]*Dy1[0] + Q2[45]*Dy1[1] + Q2[46]*Dy1[2] + Q2[47]*Dy1[3] + Q2[48]*Dy1[4] + Q2[49]*Dy1[5] + Q2[50]*Dy1[6] + Q2[51]*Dy1[7] + Q2[52]*Dy1[8] + Q2[53]*Dy1[9] + Q2[54]*Dy1[10];
QDy1[5] = + Q2[55]*Dy1[0] + Q2[56]*Dy1[1] + Q2[57]*Dy1[2] + Q2[58]*Dy1[3] + Q2[59]*Dy1[4] + Q2[60]*Dy1[5] + Q2[61]*Dy1[6] + Q2[62]*Dy1[7] + Q2[63]*Dy1[8] + Q2[64]*Dy1[9] + Q2[65]*Dy1[10];
QDy1[6] = + Q2[66]*Dy1[0] + Q2[67]*Dy1[1] + Q2[68]*Dy1[2] + Q2[69]*Dy1[3] + Q2[70]*Dy1[4] + Q2[71]*Dy1[5] + Q2[72]*Dy1[6] + Q2[73]*Dy1[7] + Q2[74]*Dy1[8] + Q2[75]*Dy1[9] + Q2[76]*Dy1[10];
QDy1[7] = + Q2[77]*Dy1[0] + Q2[78]*Dy1[1] + Q2[79]*Dy1[2] + Q2[80]*Dy1[3] + Q2[81]*Dy1[4] + Q2[82]*Dy1[5] + Q2[83]*Dy1[6] + Q2[84]*Dy1[7] + Q2[85]*Dy1[8] + Q2[86]*Dy1[9] + Q2[87]*Dy1[10];
QDy1[8] = + Q2[88]*Dy1[0] + Q2[89]*Dy1[1] + Q2[90]*Dy1[2] + Q2[91]*Dy1[3] + Q2[92]*Dy1[4] + Q2[93]*Dy1[5] + Q2[94]*Dy1[6] + Q2[95]*Dy1[7] + Q2[96]*Dy1[8] + Q2[97]*Dy1[9] + Q2[98]*Dy1[10];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun2 = 0; lRun2 < 20; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 41)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 27 ]), &(acadoWorkspace.E[ lRun3 * 27 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 20; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (9)) * (9)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (9)) * (3)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (9)) * (3)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (20)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 19; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 27 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ lRun1 * 27 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (9)) * (3)) + (0) ]), lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 81 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 81 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 9 ]), &(acadoWorkspace.evGu[ lRun2 * 27 ]), acadoWorkspace.W1, lRun2 );
}

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
acado_copyHTH( 0, 10 );
acado_copyHTH( 1, 10 );
acado_copyHTH( 2, 10 );
acado_copyHTH( 3, 10 );
acado_copyHTH( 4, 10 );
acado_copyHTH( 5, 10 );
acado_copyHTH( 6, 10 );
acado_copyHTH( 7, 10 );
acado_copyHTH( 8, 10 );
acado_copyHTH( 9, 10 );
acado_copyHTH( 0, 11 );
acado_copyHTH( 1, 11 );
acado_copyHTH( 2, 11 );
acado_copyHTH( 3, 11 );
acado_copyHTH( 4, 11 );
acado_copyHTH( 5, 11 );
acado_copyHTH( 6, 11 );
acado_copyHTH( 7, 11 );
acado_copyHTH( 8, 11 );
acado_copyHTH( 9, 11 );
acado_copyHTH( 10, 11 );
acado_copyHTH( 0, 12 );
acado_copyHTH( 1, 12 );
acado_copyHTH( 2, 12 );
acado_copyHTH( 3, 12 );
acado_copyHTH( 4, 12 );
acado_copyHTH( 5, 12 );
acado_copyHTH( 6, 12 );
acado_copyHTH( 7, 12 );
acado_copyHTH( 8, 12 );
acado_copyHTH( 9, 12 );
acado_copyHTH( 10, 12 );
acado_copyHTH( 11, 12 );
acado_copyHTH( 0, 13 );
acado_copyHTH( 1, 13 );
acado_copyHTH( 2, 13 );
acado_copyHTH( 3, 13 );
acado_copyHTH( 4, 13 );
acado_copyHTH( 5, 13 );
acado_copyHTH( 6, 13 );
acado_copyHTH( 7, 13 );
acado_copyHTH( 8, 13 );
acado_copyHTH( 9, 13 );
acado_copyHTH( 10, 13 );
acado_copyHTH( 11, 13 );
acado_copyHTH( 12, 13 );
acado_copyHTH( 0, 14 );
acado_copyHTH( 1, 14 );
acado_copyHTH( 2, 14 );
acado_copyHTH( 3, 14 );
acado_copyHTH( 4, 14 );
acado_copyHTH( 5, 14 );
acado_copyHTH( 6, 14 );
acado_copyHTH( 7, 14 );
acado_copyHTH( 8, 14 );
acado_copyHTH( 9, 14 );
acado_copyHTH( 10, 14 );
acado_copyHTH( 11, 14 );
acado_copyHTH( 12, 14 );
acado_copyHTH( 13, 14 );
acado_copyHTH( 0, 15 );
acado_copyHTH( 1, 15 );
acado_copyHTH( 2, 15 );
acado_copyHTH( 3, 15 );
acado_copyHTH( 4, 15 );
acado_copyHTH( 5, 15 );
acado_copyHTH( 6, 15 );
acado_copyHTH( 7, 15 );
acado_copyHTH( 8, 15 );
acado_copyHTH( 9, 15 );
acado_copyHTH( 10, 15 );
acado_copyHTH( 11, 15 );
acado_copyHTH( 12, 15 );
acado_copyHTH( 13, 15 );
acado_copyHTH( 14, 15 );
acado_copyHTH( 0, 16 );
acado_copyHTH( 1, 16 );
acado_copyHTH( 2, 16 );
acado_copyHTH( 3, 16 );
acado_copyHTH( 4, 16 );
acado_copyHTH( 5, 16 );
acado_copyHTH( 6, 16 );
acado_copyHTH( 7, 16 );
acado_copyHTH( 8, 16 );
acado_copyHTH( 9, 16 );
acado_copyHTH( 10, 16 );
acado_copyHTH( 11, 16 );
acado_copyHTH( 12, 16 );
acado_copyHTH( 13, 16 );
acado_copyHTH( 14, 16 );
acado_copyHTH( 15, 16 );
acado_copyHTH( 0, 17 );
acado_copyHTH( 1, 17 );
acado_copyHTH( 2, 17 );
acado_copyHTH( 3, 17 );
acado_copyHTH( 4, 17 );
acado_copyHTH( 5, 17 );
acado_copyHTH( 6, 17 );
acado_copyHTH( 7, 17 );
acado_copyHTH( 8, 17 );
acado_copyHTH( 9, 17 );
acado_copyHTH( 10, 17 );
acado_copyHTH( 11, 17 );
acado_copyHTH( 12, 17 );
acado_copyHTH( 13, 17 );
acado_copyHTH( 14, 17 );
acado_copyHTH( 15, 17 );
acado_copyHTH( 16, 17 );
acado_copyHTH( 0, 18 );
acado_copyHTH( 1, 18 );
acado_copyHTH( 2, 18 );
acado_copyHTH( 3, 18 );
acado_copyHTH( 4, 18 );
acado_copyHTH( 5, 18 );
acado_copyHTH( 6, 18 );
acado_copyHTH( 7, 18 );
acado_copyHTH( 8, 18 );
acado_copyHTH( 9, 18 );
acado_copyHTH( 10, 18 );
acado_copyHTH( 11, 18 );
acado_copyHTH( 12, 18 );
acado_copyHTH( 13, 18 );
acado_copyHTH( 14, 18 );
acado_copyHTH( 15, 18 );
acado_copyHTH( 16, 18 );
acado_copyHTH( 17, 18 );
acado_copyHTH( 0, 19 );
acado_copyHTH( 1, 19 );
acado_copyHTH( 2, 19 );
acado_copyHTH( 3, 19 );
acado_copyHTH( 4, 19 );
acado_copyHTH( 5, 19 );
acado_copyHTH( 6, 19 );
acado_copyHTH( 7, 19 );
acado_copyHTH( 8, 19 );
acado_copyHTH( 9, 19 );
acado_copyHTH( 10, 19 );
acado_copyHTH( 11, 19 );
acado_copyHTH( 12, 19 );
acado_copyHTH( 13, 19 );
acado_copyHTH( 14, 19 );
acado_copyHTH( 15, 19 );
acado_copyHTH( 16, 19 );
acado_copyHTH( 17, 19 );
acado_copyHTH( 18, 19 );

for (lRun1 = 0; lRun1 < 180; ++lRun1)
acadoWorkspace.sbar[lRun1 + 9] = acadoWorkspace.d[lRun1];


}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
for (lRun1 = 0; lRun1 < 220; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 33 ]), &(acadoWorkspace.Dy[ 11 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 66 ]), &(acadoWorkspace.Dy[ 22 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 99 ]), &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 132 ]), &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 165 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 198 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 231 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 264 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 297 ]), &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 330 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 363 ]), &(acadoWorkspace.Dy[ 121 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 396 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 429 ]), &(acadoWorkspace.Dy[ 143 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 462 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 495 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 528 ]), &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 561 ]), &(acadoWorkspace.Dy[ 187 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 594 ]), &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 627 ]), &(acadoWorkspace.Dy[ 209 ]), &(acadoWorkspace.g[ 57 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 99 ]), &(acadoWorkspace.Dy[ 11 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 198 ]), &(acadoWorkspace.Dy[ 22 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 297 ]), &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 396 ]), &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 495 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 594 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 693 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 792 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 891 ]), &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.QDy[ 81 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 990 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1089 ]), &(acadoWorkspace.Dy[ 121 ]), &(acadoWorkspace.QDy[ 99 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1188 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1287 ]), &(acadoWorkspace.Dy[ 143 ]), &(acadoWorkspace.QDy[ 117 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1386 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1485 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1584 ]), &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1683 ]), &(acadoWorkspace.Dy[ 187 ]), &(acadoWorkspace.QDy[ 153 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1782 ]), &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.QDy[ 162 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1881 ]), &(acadoWorkspace.Dy[ 209 ]), &(acadoWorkspace.QDy[ 171 ]) );

acadoWorkspace.QDy[180] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[181] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[182] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[183] = + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[184] = + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[185] = + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[186] = + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[187] = + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[188] = + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[5];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 9 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1053 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1134 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1215 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1377 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1458 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1539 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 180 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[180];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[181];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[182];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[183];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[184];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[185];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[186];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[64]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[65]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[66]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[67]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[68]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[69]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[70]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[71]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[187];
acadoWorkspace.w1[8] = + acadoWorkspace.QN1[72]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[73]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[74]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[75]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[76]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[77]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[78]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[79]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[80]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[188];
acado_macBTw1( &(acadoWorkspace.evGu[ 513 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 57 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 513 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.g[ 57 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1539 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 171 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1539 ]), &(acadoWorkspace.sbar[ 171 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 486 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1458 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 162 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1458 ]), &(acadoWorkspace.sbar[ 162 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 459 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 51 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 459 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.g[ 51 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1377 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 153 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1377 ]), &(acadoWorkspace.sbar[ 153 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 45 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.g[ 45 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1215 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 135 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1215 ]), &(acadoWorkspace.sbar[ 135 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 378 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1134 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1134 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 351 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1053 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1053 ]), &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 972 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 972 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 297 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 891 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 99 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 891 ]), &(acadoWorkspace.sbar[ 99 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 810 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 810 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 729 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 81 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.sbar[ 81 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 567 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.sbar[ 63 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
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
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
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
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];

}

void acado_expand(  )
{
int lRun1;
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
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
for (lRun1 = 0; lRun1 < 180; ++lRun1)
acadoWorkspace.sbar[lRun1 + 9] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 9 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.evGu[ 27 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.evGu[ 81 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.evGu[ 135 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.evGu[ 162 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.evGu[ 189 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.evGu[ 243 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.evGu[ 270 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.evGu[ 297 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.evGu[ 324 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1053 ]), &(acadoWorkspace.evGu[ 351 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1134 ]), &(acadoWorkspace.evGu[ 378 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1215 ]), &(acadoWorkspace.evGu[ 405 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1377 ]), &(acadoWorkspace.evGu[ 459 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1458 ]), &(acadoWorkspace.evGu[ 486 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1539 ]), &(acadoWorkspace.evGu[ 513 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 180 ]) );
for (lRun1 = 0; lRun1 < 189; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
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
acadoVariables.lbValues[0] = -7.8539816339744828e-01;
acadoVariables.lbValues[1] = -7.8539816339744828e-01;
acadoVariables.lbValues[2] = 4.9032999999999998e+00;
acadoVariables.lbValues[3] = -7.8539816339744828e-01;
acadoVariables.lbValues[4] = -7.8539816339744828e-01;
acadoVariables.lbValues[5] = 4.9032999999999998e+00;
acadoVariables.lbValues[6] = -7.8539816339744828e-01;
acadoVariables.lbValues[7] = -7.8539816339744828e-01;
acadoVariables.lbValues[8] = 4.9032999999999998e+00;
acadoVariables.lbValues[9] = -7.8539816339744828e-01;
acadoVariables.lbValues[10] = -7.8539816339744828e-01;
acadoVariables.lbValues[11] = 4.9032999999999998e+00;
acadoVariables.lbValues[12] = -7.8539816339744828e-01;
acadoVariables.lbValues[13] = -7.8539816339744828e-01;
acadoVariables.lbValues[14] = 4.9032999999999998e+00;
acadoVariables.lbValues[15] = -7.8539816339744828e-01;
acadoVariables.lbValues[16] = -7.8539816339744828e-01;
acadoVariables.lbValues[17] = 4.9032999999999998e+00;
acadoVariables.lbValues[18] = -7.8539816339744828e-01;
acadoVariables.lbValues[19] = -7.8539816339744828e-01;
acadoVariables.lbValues[20] = 4.9032999999999998e+00;
acadoVariables.lbValues[21] = -7.8539816339744828e-01;
acadoVariables.lbValues[22] = -7.8539816339744828e-01;
acadoVariables.lbValues[23] = 4.9032999999999998e+00;
acadoVariables.lbValues[24] = -7.8539816339744828e-01;
acadoVariables.lbValues[25] = -7.8539816339744828e-01;
acadoVariables.lbValues[26] = 4.9032999999999998e+00;
acadoVariables.lbValues[27] = -7.8539816339744828e-01;
acadoVariables.lbValues[28] = -7.8539816339744828e-01;
acadoVariables.lbValues[29] = 4.9032999999999998e+00;
acadoVariables.lbValues[30] = -7.8539816339744828e-01;
acadoVariables.lbValues[31] = -7.8539816339744828e-01;
acadoVariables.lbValues[32] = 4.9032999999999998e+00;
acadoVariables.lbValues[33] = -7.8539816339744828e-01;
acadoVariables.lbValues[34] = -7.8539816339744828e-01;
acadoVariables.lbValues[35] = 4.9032999999999998e+00;
acadoVariables.lbValues[36] = -7.8539816339744828e-01;
acadoVariables.lbValues[37] = -7.8539816339744828e-01;
acadoVariables.lbValues[38] = 4.9032999999999998e+00;
acadoVariables.lbValues[39] = -7.8539816339744828e-01;
acadoVariables.lbValues[40] = -7.8539816339744828e-01;
acadoVariables.lbValues[41] = 4.9032999999999998e+00;
acadoVariables.lbValues[42] = -7.8539816339744828e-01;
acadoVariables.lbValues[43] = -7.8539816339744828e-01;
acadoVariables.lbValues[44] = 4.9032999999999998e+00;
acadoVariables.lbValues[45] = -7.8539816339744828e-01;
acadoVariables.lbValues[46] = -7.8539816339744828e-01;
acadoVariables.lbValues[47] = 4.9032999999999998e+00;
acadoVariables.lbValues[48] = -7.8539816339744828e-01;
acadoVariables.lbValues[49] = -7.8539816339744828e-01;
acadoVariables.lbValues[50] = 4.9032999999999998e+00;
acadoVariables.lbValues[51] = -7.8539816339744828e-01;
acadoVariables.lbValues[52] = -7.8539816339744828e-01;
acadoVariables.lbValues[53] = 4.9032999999999998e+00;
acadoVariables.lbValues[54] = -7.8539816339744828e-01;
acadoVariables.lbValues[55] = -7.8539816339744828e-01;
acadoVariables.lbValues[56] = 4.9032999999999998e+00;
acadoVariables.lbValues[57] = -7.8539816339744828e-01;
acadoVariables.lbValues[58] = -7.8539816339744828e-01;
acadoVariables.lbValues[59] = 4.9032999999999998e+00;
acadoVariables.ubValues[0] = 7.8539816339744828e-01;
acadoVariables.ubValues[1] = 7.8539816339744828e-01;
acadoVariables.ubValues[2] = 1.4709899999999999e+01;
acadoVariables.ubValues[3] = 7.8539816339744828e-01;
acadoVariables.ubValues[4] = 7.8539816339744828e-01;
acadoVariables.ubValues[5] = 1.4709899999999999e+01;
acadoVariables.ubValues[6] = 7.8539816339744828e-01;
acadoVariables.ubValues[7] = 7.8539816339744828e-01;
acadoVariables.ubValues[8] = 1.4709899999999999e+01;
acadoVariables.ubValues[9] = 7.8539816339744828e-01;
acadoVariables.ubValues[10] = 7.8539816339744828e-01;
acadoVariables.ubValues[11] = 1.4709899999999999e+01;
acadoVariables.ubValues[12] = 7.8539816339744828e-01;
acadoVariables.ubValues[13] = 7.8539816339744828e-01;
acadoVariables.ubValues[14] = 1.4709899999999999e+01;
acadoVariables.ubValues[15] = 7.8539816339744828e-01;
acadoVariables.ubValues[16] = 7.8539816339744828e-01;
acadoVariables.ubValues[17] = 1.4709899999999999e+01;
acadoVariables.ubValues[18] = 7.8539816339744828e-01;
acadoVariables.ubValues[19] = 7.8539816339744828e-01;
acadoVariables.ubValues[20] = 1.4709899999999999e+01;
acadoVariables.ubValues[21] = 7.8539816339744828e-01;
acadoVariables.ubValues[22] = 7.8539816339744828e-01;
acadoVariables.ubValues[23] = 1.4709899999999999e+01;
acadoVariables.ubValues[24] = 7.8539816339744828e-01;
acadoVariables.ubValues[25] = 7.8539816339744828e-01;
acadoVariables.ubValues[26] = 1.4709899999999999e+01;
acadoVariables.ubValues[27] = 7.8539816339744828e-01;
acadoVariables.ubValues[28] = 7.8539816339744828e-01;
acadoVariables.ubValues[29] = 1.4709899999999999e+01;
acadoVariables.ubValues[30] = 7.8539816339744828e-01;
acadoVariables.ubValues[31] = 7.8539816339744828e-01;
acadoVariables.ubValues[32] = 1.4709899999999999e+01;
acadoVariables.ubValues[33] = 7.8539816339744828e-01;
acadoVariables.ubValues[34] = 7.8539816339744828e-01;
acadoVariables.ubValues[35] = 1.4709899999999999e+01;
acadoVariables.ubValues[36] = 7.8539816339744828e-01;
acadoVariables.ubValues[37] = 7.8539816339744828e-01;
acadoVariables.ubValues[38] = 1.4709899999999999e+01;
acadoVariables.ubValues[39] = 7.8539816339744828e-01;
acadoVariables.ubValues[40] = 7.8539816339744828e-01;
acadoVariables.ubValues[41] = 1.4709899999999999e+01;
acadoVariables.ubValues[42] = 7.8539816339744828e-01;
acadoVariables.ubValues[43] = 7.8539816339744828e-01;
acadoVariables.ubValues[44] = 1.4709899999999999e+01;
acadoVariables.ubValues[45] = 7.8539816339744828e-01;
acadoVariables.ubValues[46] = 7.8539816339744828e-01;
acadoVariables.ubValues[47] = 1.4709899999999999e+01;
acadoVariables.ubValues[48] = 7.8539816339744828e-01;
acadoVariables.ubValues[49] = 7.8539816339744828e-01;
acadoVariables.ubValues[50] = 1.4709899999999999e+01;
acadoVariables.ubValues[51] = 7.8539816339744828e-01;
acadoVariables.ubValues[52] = 7.8539816339744828e-01;
acadoVariables.ubValues[53] = 1.4709899999999999e+01;
acadoVariables.ubValues[54] = 7.8539816339744828e-01;
acadoVariables.ubValues[55] = 7.8539816339744828e-01;
acadoVariables.ubValues[56] = 1.4709899999999999e+01;
acadoVariables.ubValues[57] = 7.8539816339744828e-01;
acadoVariables.ubValues[58] = 7.8539816339744828e-01;
acadoVariables.ubValues[59] = 1.4709899999999999e+01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
state[0] = acadoVariables.x[index * 9];
state[1] = acadoVariables.x[index * 9 + 1];
state[2] = acadoVariables.x[index * 9 + 2];
state[3] = acadoVariables.x[index * 9 + 3];
state[4] = acadoVariables.x[index * 9 + 4];
state[5] = acadoVariables.x[index * 9 + 5];
state[6] = acadoVariables.x[index * 9 + 6];
state[7] = acadoVariables.x[index * 9 + 7];
state[8] = acadoVariables.x[index * 9 + 8];
state[117] = acadoVariables.u[index * 3];
state[118] = acadoVariables.u[index * 3 + 1];
state[119] = acadoVariables.u[index * 3 + 2];
state[120] = acadoVariables.od[index * 9];
state[121] = acadoVariables.od[index * 9 + 1];
state[122] = acadoVariables.od[index * 9 + 2];
state[123] = acadoVariables.od[index * 9 + 3];
state[124] = acadoVariables.od[index * 9 + 4];
state[125] = acadoVariables.od[index * 9 + 5];
state[126] = acadoVariables.od[index * 9 + 6];
state[127] = acadoVariables.od[index * 9 + 7];
state[128] = acadoVariables.od[index * 9 + 8];

acado_integrate(state, index == 0);

acadoVariables.x[index * 9 + 9] = state[0];
acadoVariables.x[index * 9 + 10] = state[1];
acadoVariables.x[index * 9 + 11] = state[2];
acadoVariables.x[index * 9 + 12] = state[3];
acadoVariables.x[index * 9 + 13] = state[4];
acadoVariables.x[index * 9 + 14] = state[5];
acadoVariables.x[index * 9 + 15] = state[6];
acadoVariables.x[index * 9 + 16] = state[7];
acadoVariables.x[index * 9 + 17] = state[8];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 9] = acadoVariables.x[index * 9 + 9];
acadoVariables.x[index * 9 + 1] = acadoVariables.x[index * 9 + 10];
acadoVariables.x[index * 9 + 2] = acadoVariables.x[index * 9 + 11];
acadoVariables.x[index * 9 + 3] = acadoVariables.x[index * 9 + 12];
acadoVariables.x[index * 9 + 4] = acadoVariables.x[index * 9 + 13];
acadoVariables.x[index * 9 + 5] = acadoVariables.x[index * 9 + 14];
acadoVariables.x[index * 9 + 6] = acadoVariables.x[index * 9 + 15];
acadoVariables.x[index * 9 + 7] = acadoVariables.x[index * 9 + 16];
acadoVariables.x[index * 9 + 8] = acadoVariables.x[index * 9 + 17];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[180] = xEnd[0];
acadoVariables.x[181] = xEnd[1];
acadoVariables.x[182] = xEnd[2];
acadoVariables.x[183] = xEnd[3];
acadoVariables.x[184] = xEnd[4];
acadoVariables.x[185] = xEnd[5];
acadoVariables.x[186] = xEnd[6];
acadoVariables.x[187] = xEnd[7];
acadoVariables.x[188] = xEnd[8];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[180];
state[1] = acadoVariables.x[181];
state[2] = acadoVariables.x[182];
state[3] = acadoVariables.x[183];
state[4] = acadoVariables.x[184];
state[5] = acadoVariables.x[185];
state[6] = acadoVariables.x[186];
state[7] = acadoVariables.x[187];
state[8] = acadoVariables.x[188];
if (uEnd != 0)
{
state[117] = uEnd[0];
state[118] = uEnd[1];
state[119] = uEnd[2];
}
else
{
state[117] = acadoVariables.u[57];
state[118] = acadoVariables.u[58];
state[119] = acadoVariables.u[59];
}
state[120] = acadoVariables.od[180];
state[121] = acadoVariables.od[181];
state[122] = acadoVariables.od[182];
state[123] = acadoVariables.od[183];
state[124] = acadoVariables.od[184];
state[125] = acadoVariables.od[185];
state[126] = acadoVariables.od[186];
state[127] = acadoVariables.od[187];
state[128] = acadoVariables.od[188];

acado_integrate(state, 1);

acadoVariables.x[180] = state[0];
acadoVariables.x[181] = state[1];
acadoVariables.x[182] = state[2];
acadoVariables.x[183] = state[3];
acadoVariables.x[184] = state[4];
acadoVariables.x[185] = state[5];
acadoVariables.x[186] = state[6];
acadoVariables.x[187] = state[7];
acadoVariables.x[188] = state[8];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[57] = uEnd[0];
acadoVariables.u[58] = uEnd[1];
acadoVariables.u[59] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59];
kkt = fabs( kkt );
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 11 */
real_t tmpDy[ 11 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 9];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 9 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 9 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 9 + 3];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 9 + 4];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 9 + 5];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 9 + 6];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 9 + 7];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 9 + 8];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 11] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 11];
acadoWorkspace.Dy[lRun1 * 11 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 11 + 1];
acadoWorkspace.Dy[lRun1 * 11 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 11 + 2];
acadoWorkspace.Dy[lRun1 * 11 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 11 + 3];
acadoWorkspace.Dy[lRun1 * 11 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 11 + 4];
acadoWorkspace.Dy[lRun1 * 11 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 11 + 5];
acadoWorkspace.Dy[lRun1 * 11 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 11 + 6];
acadoWorkspace.Dy[lRun1 * 11 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 11 + 7];
acadoWorkspace.Dy[lRun1 * 11 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 11 + 8];
acadoWorkspace.Dy[lRun1 * 11 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 11 + 9];
acadoWorkspace.Dy[lRun1 * 11 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 11 + 10];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[180];
acadoWorkspace.objValueIn[1] = acadoVariables.x[181];
acadoWorkspace.objValueIn[2] = acadoVariables.x[182];
acadoWorkspace.objValueIn[3] = acadoVariables.x[183];
acadoWorkspace.objValueIn[4] = acadoVariables.x[184];
acadoWorkspace.objValueIn[5] = acadoVariables.x[185];
acadoWorkspace.objValueIn[6] = acadoVariables.x[186];
acadoWorkspace.objValueIn[7] = acadoVariables.x[187];
acadoWorkspace.objValueIn[8] = acadoVariables.x[188];
acadoWorkspace.objValueIn[9] = acadoVariables.od[180];
acadoWorkspace.objValueIn[10] = acadoVariables.od[181];
acadoWorkspace.objValueIn[11] = acadoVariables.od[182];
acadoWorkspace.objValueIn[12] = acadoVariables.od[183];
acadoWorkspace.objValueIn[13] = acadoVariables.od[184];
acadoWorkspace.objValueIn[14] = acadoVariables.od[185];
acadoWorkspace.objValueIn[15] = acadoVariables.od[186];
acadoWorkspace.objValueIn[16] = acadoVariables.od[187];
acadoWorkspace.objValueIn[17] = acadoVariables.od[188];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 11]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 11 + 1]*acadoVariables.W[12];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 11 + 2]*acadoVariables.W[24];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 11 + 3]*acadoVariables.W[36];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 11 + 4]*acadoVariables.W[48];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 11 + 5]*acadoVariables.W[60];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 11 + 6]*acadoVariables.W[72];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 11 + 7]*acadoVariables.W[84];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 11 + 8]*acadoVariables.W[96];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 11 + 9]*acadoVariables.W[108];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 11 + 10]*acadoVariables.W[120];
objVal += + acadoWorkspace.Dy[lRun1 * 11]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 11 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 11 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 11 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 11 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 11 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 11 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 11 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 11 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 11 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 11 + 10]*tmpDy[10];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[7];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[14];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[21];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[28];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[35];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5];

objVal *= 0.5;
return objVal;
}

