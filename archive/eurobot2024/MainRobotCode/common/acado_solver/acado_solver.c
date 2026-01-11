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
for (lRun1 = 0; lRun1 < 22; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 5 + 4];

acadoWorkspace.state[40] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[41] = acadoVariables.u[lRun1 * 2 + 1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 5] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 5 + 5];
acadoWorkspace.d[lRun1 * 5 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 5 + 6];
acadoWorkspace.d[lRun1 * 5 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 5 + 7];
acadoWorkspace.d[lRun1 * 5 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 5 + 8];
acadoWorkspace.d[lRun1 * 5 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 5 + 9];

acadoWorkspace.evGx[lRun1 * 25] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 25 + 1] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 25 + 2] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 25 + 3] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 25 + 4] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 25 + 5] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 25 + 6] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 25 + 7] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 25 + 8] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 25 + 9] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 25 + 10] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 25 + 11] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 25 + 12] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 25 + 13] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 25 + 14] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 25 + 15] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 25 + 16] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 25 + 17] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 25 + 18] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 25 + 19] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 25 + 20] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 25 + 21] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 25 + 22] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 25 + 23] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 25 + 24] = acadoWorkspace.state[29];

acadoWorkspace.evGu[lRun1 * 10] = acadoWorkspace.state[30];
acadoWorkspace.evGu[lRun1 * 10 + 1] = acadoWorkspace.state[31];
acadoWorkspace.evGu[lRun1 * 10 + 2] = acadoWorkspace.state[32];
acadoWorkspace.evGu[lRun1 * 10 + 3] = acadoWorkspace.state[33];
acadoWorkspace.evGu[lRun1 * 10 + 4] = acadoWorkspace.state[34];
acadoWorkspace.evGu[lRun1 * 10 + 5] = acadoWorkspace.state[35];
acadoWorkspace.evGu[lRun1 * 10 + 6] = acadoWorkspace.state[36];
acadoWorkspace.evGu[lRun1 * 10 + 7] = acadoWorkspace.state[37];
acadoWorkspace.evGu[lRun1 * 10 + 8] = acadoWorkspace.state[38];
acadoWorkspace.evGu[lRun1 * 10 + 9] = acadoWorkspace.state[39];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 22; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 5] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 5 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 5 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 5 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 5 + 4] = acadoWorkspace.objValueOut[4];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[110];
acadoWorkspace.objValueIn[1] = acadoVariables.x[111];
acadoWorkspace.objValueIn[2] = acadoVariables.x[112];
acadoWorkspace.objValueIn[3] = acadoVariables.x[113];
acadoWorkspace.objValueIn[4] = acadoVariables.x[114];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4];
dNew[1] += + Gx1[5]*dOld[0] + Gx1[6]*dOld[1] + Gx1[7]*dOld[2] + Gx1[8]*dOld[3] + Gx1[9]*dOld[4];
dNew[2] += + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4];
dNew[3] += + Gx1[15]*dOld[0] + Gx1[16]*dOld[1] + Gx1[17]*dOld[2] + Gx1[18]*dOld[3] + Gx1[19]*dOld[4];
dNew[4] += + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4];
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
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[15] + Gx1[4]*Gx2[20];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[16] + Gx1[4]*Gx2[21];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[17] + Gx1[4]*Gx2[22];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[23];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[24];
Gx3[5] = + Gx1[5]*Gx2[0] + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[20];
Gx3[6] = + Gx1[5]*Gx2[1] + Gx1[6]*Gx2[6] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[21];
Gx3[7] = + Gx1[5]*Gx2[2] + Gx1[6]*Gx2[7] + Gx1[7]*Gx2[12] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[22];
Gx3[8] = + Gx1[5]*Gx2[3] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[13] + Gx1[8]*Gx2[18] + Gx1[9]*Gx2[23];
Gx3[9] = + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[14] + Gx1[8]*Gx2[19] + Gx1[9]*Gx2[24];
Gx3[10] = + Gx1[10]*Gx2[0] + Gx1[11]*Gx2[5] + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[20];
Gx3[11] = + Gx1[10]*Gx2[1] + Gx1[11]*Gx2[6] + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[21];
Gx3[12] = + Gx1[10]*Gx2[2] + Gx1[11]*Gx2[7] + Gx1[12]*Gx2[12] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[22];
Gx3[13] = + Gx1[10]*Gx2[3] + Gx1[11]*Gx2[8] + Gx1[12]*Gx2[13] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[23];
Gx3[14] = + Gx1[10]*Gx2[4] + Gx1[11]*Gx2[9] + Gx1[12]*Gx2[14] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[24];
Gx3[15] = + Gx1[15]*Gx2[0] + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[15] + Gx1[19]*Gx2[20];
Gx3[16] = + Gx1[15]*Gx2[1] + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[21];
Gx3[17] = + Gx1[15]*Gx2[2] + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[22];
Gx3[18] = + Gx1[15]*Gx2[3] + Gx1[16]*Gx2[8] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[23];
Gx3[19] = + Gx1[15]*Gx2[4] + Gx1[16]*Gx2[9] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[24];
Gx3[20] = + Gx1[20]*Gx2[0] + Gx1[21]*Gx2[5] + Gx1[22]*Gx2[10] + Gx1[23]*Gx2[15] + Gx1[24]*Gx2[20];
Gx3[21] = + Gx1[20]*Gx2[1] + Gx1[21]*Gx2[6] + Gx1[22]*Gx2[11] + Gx1[23]*Gx2[16] + Gx1[24]*Gx2[21];
Gx3[22] = + Gx1[20]*Gx2[2] + Gx1[21]*Gx2[7] + Gx1[22]*Gx2[12] + Gx1[23]*Gx2[17] + Gx1[24]*Gx2[22];
Gx3[23] = + Gx1[20]*Gx2[3] + Gx1[21]*Gx2[8] + Gx1[22]*Gx2[13] + Gx1[23]*Gx2[18] + Gx1[24]*Gx2[23];
Gx3[24] = + Gx1[20]*Gx2[4] + Gx1[21]*Gx2[9] + Gx1[22]*Gx2[14] + Gx1[23]*Gx2[19] + Gx1[24]*Gx2[24];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9];
Gu2[2] = + Gx1[5]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[4] + Gx1[8]*Gu1[6] + Gx1[9]*Gu1[8];
Gu2[3] = + Gx1[5]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[7]*Gu1[5] + Gx1[8]*Gu1[7] + Gx1[9]*Gu1[9];
Gu2[4] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[2] + Gx1[12]*Gu1[4] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[8];
Gu2[5] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[3] + Gx1[12]*Gu1[5] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[9];
Gu2[6] = + Gx1[15]*Gu1[0] + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[6] + Gx1[19]*Gu1[8];
Gu2[7] = + Gx1[15]*Gu1[1] + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[7] + Gx1[19]*Gu1[9];
Gu2[8] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[2] + Gx1[22]*Gu1[4] + Gx1[23]*Gu1[6] + Gx1[24]*Gu1[8];
Gu2[9] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[3] + Gx1[22]*Gu1[5] + Gx1[23]*Gu1[7] + Gx1[24]*Gu1[9];
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
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 88) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8];
acadoWorkspace.H[(iRow * 88) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9];
acadoWorkspace.H[(iRow * 88 + 44) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8];
acadoWorkspace.H[(iRow * 88 + 44) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 88) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 88) + (iCol * 2 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 88 + 44) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 88 + 44) + (iCol * 2 + 1)] = 0.0;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 88) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 88) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 88 + 44) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 88 + 44) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 88) + (iCol * 2)] = acadoWorkspace.H[(iCol * 88) + (iRow * 2)];
acadoWorkspace.H[(iRow * 88) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 88 + 44) + (iRow * 2)];
acadoWorkspace.H[(iRow * 88 + 44) + (iCol * 2)] = acadoWorkspace.H[(iCol * 88) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 88 + 44) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 88 + 44) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)5.0000000000000000e+03*dOld[0];
dNew[1] = + (real_t)5.0000000000000000e+03*dOld[1];
dNew[2] = + (real_t)1.0000000000000000e+01*dOld[2];
dNew[3] = + (real_t)1.0000000000000000e+02*dOld[3];
dNew[4] = +dOld[4];
}

void acado_multQN1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)5.0000000000000000e+04*dOld[0];
dNew[1] = + (real_t)5.0000000000000000e+04*dOld[1];
dNew[2] = + (real_t)1.0000000000000000e+02*dOld[2];
dNew[3] = + (real_t)1.0000000000000000e+03*dOld[3];
dNew[4] = + (real_t)1.0000000000000000e+01*dOld[4];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = 0.0;
;
RDy1[1] = 0.0;
;
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + (real_t)5.0000000000000000e+03*Dy1[0];
QDy1[1] = + (real_t)5.0000000000000000e+03*Dy1[1];
QDy1[2] = + (real_t)1.0000000000000000e+01*Dy1[2];
QDy1[3] = + (real_t)1.0000000000000000e+02*Dy1[3];
QDy1[4] = +Dy1[4];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3] + E1[8]*QDy1[4];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3] + E1[9]*QDy1[4];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[5] + E1[4]*Gx1[10] + E1[6]*Gx1[15] + E1[8]*Gx1[20];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[6] + E1[4]*Gx1[11] + E1[6]*Gx1[16] + E1[8]*Gx1[21];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[7] + E1[4]*Gx1[12] + E1[6]*Gx1[17] + E1[8]*Gx1[22];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[8] + E1[4]*Gx1[13] + E1[6]*Gx1[18] + E1[8]*Gx1[23];
H101[4] += + E1[0]*Gx1[4] + E1[2]*Gx1[9] + E1[4]*Gx1[14] + E1[6]*Gx1[19] + E1[8]*Gx1[24];
H101[5] += + E1[1]*Gx1[0] + E1[3]*Gx1[5] + E1[5]*Gx1[10] + E1[7]*Gx1[15] + E1[9]*Gx1[20];
H101[6] += + E1[1]*Gx1[1] + E1[3]*Gx1[6] + E1[5]*Gx1[11] + E1[7]*Gx1[16] + E1[9]*Gx1[21];
H101[7] += + E1[1]*Gx1[2] + E1[3]*Gx1[7] + E1[5]*Gx1[12] + E1[7]*Gx1[17] + E1[9]*Gx1[22];
H101[8] += + E1[1]*Gx1[3] + E1[3]*Gx1[8] + E1[5]*Gx1[13] + E1[7]*Gx1[18] + E1[9]*Gx1[23];
H101[9] += + E1[1]*Gx1[4] + E1[3]*Gx1[9] + E1[5]*Gx1[14] + E1[7]*Gx1[19] + E1[9]*Gx1[24];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 10; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
dNew[4] += + E1[8]*U1[0] + E1[9]*U1[1];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)5.0000000000000000e+03*Gx1[0];
Gx2[1] = + (real_t)5.0000000000000000e+03*Gx1[1];
Gx2[2] = + (real_t)5.0000000000000000e+03*Gx1[2];
Gx2[3] = + (real_t)5.0000000000000000e+03*Gx1[3];
Gx2[4] = + (real_t)5.0000000000000000e+03*Gx1[4];
Gx2[5] = + (real_t)5.0000000000000000e+03*Gx1[5];
Gx2[6] = + (real_t)5.0000000000000000e+03*Gx1[6];
Gx2[7] = + (real_t)5.0000000000000000e+03*Gx1[7];
Gx2[8] = + (real_t)5.0000000000000000e+03*Gx1[8];
Gx2[9] = + (real_t)5.0000000000000000e+03*Gx1[9];
Gx2[10] = + (real_t)1.0000000000000000e+01*Gx1[10];
Gx2[11] = + (real_t)1.0000000000000000e+01*Gx1[11];
Gx2[12] = + (real_t)1.0000000000000000e+01*Gx1[12];
Gx2[13] = + (real_t)1.0000000000000000e+01*Gx1[13];
Gx2[14] = + (real_t)1.0000000000000000e+01*Gx1[14];
Gx2[15] = + (real_t)1.0000000000000000e+02*Gx1[15];
Gx2[16] = + (real_t)1.0000000000000000e+02*Gx1[16];
Gx2[17] = + (real_t)1.0000000000000000e+02*Gx1[17];
Gx2[18] = + (real_t)1.0000000000000000e+02*Gx1[18];
Gx2[19] = + (real_t)1.0000000000000000e+02*Gx1[19];
Gx2[20] = +Gx1[20];
Gx2[21] = +Gx1[21];
Gx2[22] = +Gx1[22];
Gx2[23] = +Gx1[23];
Gx2[24] = +Gx1[24];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)5.0000000000000000e+04*Gx1[0];
Gx2[1] = + (real_t)5.0000000000000000e+04*Gx1[1];
Gx2[2] = + (real_t)5.0000000000000000e+04*Gx1[2];
Gx2[3] = + (real_t)5.0000000000000000e+04*Gx1[3];
Gx2[4] = + (real_t)5.0000000000000000e+04*Gx1[4];
Gx2[5] = + (real_t)5.0000000000000000e+04*Gx1[5];
Gx2[6] = + (real_t)5.0000000000000000e+04*Gx1[6];
Gx2[7] = + (real_t)5.0000000000000000e+04*Gx1[7];
Gx2[8] = + (real_t)5.0000000000000000e+04*Gx1[8];
Gx2[9] = + (real_t)5.0000000000000000e+04*Gx1[9];
Gx2[10] = + (real_t)1.0000000000000000e+02*Gx1[10];
Gx2[11] = + (real_t)1.0000000000000000e+02*Gx1[11];
Gx2[12] = + (real_t)1.0000000000000000e+02*Gx1[12];
Gx2[13] = + (real_t)1.0000000000000000e+02*Gx1[13];
Gx2[14] = + (real_t)1.0000000000000000e+02*Gx1[14];
Gx2[15] = + (real_t)1.0000000000000000e+03*Gx1[15];
Gx2[16] = + (real_t)1.0000000000000000e+03*Gx1[16];
Gx2[17] = + (real_t)1.0000000000000000e+03*Gx1[17];
Gx2[18] = + (real_t)1.0000000000000000e+03*Gx1[18];
Gx2[19] = + (real_t)1.0000000000000000e+03*Gx1[19];
Gx2[20] = + (real_t)1.0000000000000000e+01*Gx1[20];
Gx2[21] = + (real_t)1.0000000000000000e+01*Gx1[21];
Gx2[22] = + (real_t)1.0000000000000000e+01*Gx1[22];
Gx2[23] = + (real_t)1.0000000000000000e+01*Gx1[23];
Gx2[24] = + (real_t)1.0000000000000000e+01*Gx1[24];
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)5.0000000000000000e+03*Gu1[0];
Gu2[1] = + (real_t)5.0000000000000000e+03*Gu1[1];
Gu2[2] = + (real_t)5.0000000000000000e+03*Gu1[2];
Gu2[3] = + (real_t)5.0000000000000000e+03*Gu1[3];
Gu2[4] = + (real_t)1.0000000000000000e+01*Gu1[4];
Gu2[5] = + (real_t)1.0000000000000000e+01*Gu1[5];
Gu2[6] = + (real_t)1.0000000000000000e+02*Gu1[6];
Gu2[7] = + (real_t)1.0000000000000000e+02*Gu1[7];
Gu2[8] = +Gu1[8];
Gu2[9] = +Gu1[9];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)5.0000000000000000e+04*Gu1[0];
Gu2[1] = + (real_t)5.0000000000000000e+04*Gu1[1];
Gu2[2] = + (real_t)5.0000000000000000e+04*Gu1[2];
Gu2[3] = + (real_t)5.0000000000000000e+04*Gu1[3];
Gu2[4] = + (real_t)1.0000000000000000e+02*Gu1[4];
Gu2[5] = + (real_t)1.0000000000000000e+02*Gu1[5];
Gu2[6] = + (real_t)1.0000000000000000e+03*Gu1[6];
Gu2[7] = + (real_t)1.0000000000000000e+03*Gu1[7];
Gu2[8] = + (real_t)1.0000000000000000e+01*Gu1[8];
Gu2[9] = + (real_t)1.0000000000000000e+01*Gu1[9];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[5] + Hx[2]*Gx[10] + Hx[3]*Gx[15] + Hx[4]*Gx[20];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[6] + Hx[2]*Gx[11] + Hx[3]*Gx[16] + Hx[4]*Gx[21];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[7] + Hx[2]*Gx[12] + Hx[3]*Gx[17] + Hx[4]*Gx[22];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[8] + Hx[2]*Gx[13] + Hx[3]*Gx[18] + Hx[4]*Gx[23];
A01[4] = + Hx[0]*Gx[4] + Hx[1]*Gx[9] + Hx[2]*Gx[14] + Hx[3]*Gx[19] + Hx[4]*Gx[24];
A01[5] = + Hx[5]*Gx[0] + Hx[6]*Gx[5] + Hx[7]*Gx[10] + Hx[8]*Gx[15] + Hx[9]*Gx[20];
A01[6] = + Hx[5]*Gx[1] + Hx[6]*Gx[6] + Hx[7]*Gx[11] + Hx[8]*Gx[16] + Hx[9]*Gx[21];
A01[7] = + Hx[5]*Gx[2] + Hx[6]*Gx[7] + Hx[7]*Gx[12] + Hx[8]*Gx[17] + Hx[9]*Gx[22];
A01[8] = + Hx[5]*Gx[3] + Hx[6]*Gx[8] + Hx[7]*Gx[13] + Hx[8]*Gx[18] + Hx[9]*Gx[23];
A01[9] = + Hx[5]*Gx[4] + Hx[6]*Gx[9] + Hx[7]*Gx[14] + Hx[8]*Gx[19] + Hx[9]*Gx[24];
A01[10] = + Hx[10]*Gx[0] + Hx[11]*Gx[5] + Hx[12]*Gx[10] + Hx[13]*Gx[15] + Hx[14]*Gx[20];
A01[11] = + Hx[10]*Gx[1] + Hx[11]*Gx[6] + Hx[12]*Gx[11] + Hx[13]*Gx[16] + Hx[14]*Gx[21];
A01[12] = + Hx[10]*Gx[2] + Hx[11]*Gx[7] + Hx[12]*Gx[12] + Hx[13]*Gx[17] + Hx[14]*Gx[22];
A01[13] = + Hx[10]*Gx[3] + Hx[11]*Gx[8] + Hx[12]*Gx[13] + Hx[13]*Gx[18] + Hx[14]*Gx[23];
A01[14] = + Hx[10]*Gx[4] + Hx[11]*Gx[9] + Hx[12]*Gx[14] + Hx[13]*Gx[19] + Hx[14]*Gx[24];
A01[15] = + Hx[15]*Gx[0] + Hx[16]*Gx[5] + Hx[17]*Gx[10] + Hx[18]*Gx[15] + Hx[19]*Gx[20];
A01[16] = + Hx[15]*Gx[1] + Hx[16]*Gx[6] + Hx[17]*Gx[11] + Hx[18]*Gx[16] + Hx[19]*Gx[21];
A01[17] = + Hx[15]*Gx[2] + Hx[16]*Gx[7] + Hx[17]*Gx[12] + Hx[18]*Gx[17] + Hx[19]*Gx[22];
A01[18] = + Hx[15]*Gx[3] + Hx[16]*Gx[8] + Hx[17]*Gx[13] + Hx[18]*Gx[18] + Hx[19]*Gx[23];
A01[19] = + Hx[15]*Gx[4] + Hx[16]*Gx[9] + Hx[17]*Gx[14] + Hx[18]*Gx[19] + Hx[19]*Gx[24];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 176) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4] + Hx[3]*E[6] + Hx[4]*E[8];
acadoWorkspace.A[(row * 176) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5] + Hx[3]*E[7] + Hx[4]*E[9];
acadoWorkspace.A[(row * 176 + 44) + (col * 2)] = + Hx[5]*E[0] + Hx[6]*E[2] + Hx[7]*E[4] + Hx[8]*E[6] + Hx[9]*E[8];
acadoWorkspace.A[(row * 176 + 44) + (col * 2 + 1)] = + Hx[5]*E[1] + Hx[6]*E[3] + Hx[7]*E[5] + Hx[8]*E[7] + Hx[9]*E[9];
acadoWorkspace.A[(row * 176 + 88) + (col * 2)] = + Hx[10]*E[0] + Hx[11]*E[2] + Hx[12]*E[4] + Hx[13]*E[6] + Hx[14]*E[8];
acadoWorkspace.A[(row * 176 + 88) + (col * 2 + 1)] = + Hx[10]*E[1] + Hx[11]*E[3] + Hx[12]*E[5] + Hx[13]*E[7] + Hx[14]*E[9];
acadoWorkspace.A[(row * 176 + 132) + (col * 2)] = + Hx[15]*E[0] + Hx[16]*E[2] + Hx[17]*E[4] + Hx[18]*E[6] + Hx[19]*E[8];
acadoWorkspace.A[(row * 176 + 132) + (col * 2 + 1)] = + Hx[15]*E[1] + Hx[16]*E[3] + Hx[17]*E[5] + Hx[18]*E[7] + Hx[19]*E[9];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4];
acadoWorkspace.evHxd[1] = + Hx[5]*tmpd[0] + Hx[6]*tmpd[1] + Hx[7]*tmpd[2] + Hx[8]*tmpd[3] + Hx[9]*tmpd[4];
acadoWorkspace.evHxd[2] = + Hx[10]*tmpd[0] + Hx[11]*tmpd[1] + Hx[12]*tmpd[2] + Hx[13]*tmpd[3] + Hx[14]*tmpd[4];
acadoWorkspace.evHxd[3] = + Hx[15]*tmpd[0] + Hx[16]*tmpd[1] + Hx[17]*tmpd[2] + Hx[18]*tmpd[3] + Hx[19]*tmpd[4];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
lbA[2] -= acadoWorkspace.evHxd[2];
lbA[3] -= acadoWorkspace.evHxd[3];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
ubA[2] -= acadoWorkspace.evHxd[2];
ubA[3] -= acadoWorkspace.evHxd[3];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 5;
/* Vector of auxiliary variables; number of elements: 32. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(1.0000000000000000e+00);
a[4] = (real_t)(1.0050000000000001e-01);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(1.0000000000000000e+00);
a[9] = (real_t)(1.0050000000000001e-01);
a[10] = (real_t)(-1.0000000000000000e+00);
a[11] = (a[9]*a[10]);
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
a[26] = (real_t)(1.0000000000000000e+00);
a[27] = (real_t)(1.0050000000000001e-01);
a[28] = (real_t)(1.0000000000000000e+00);
a[29] = (real_t)(1.0050000000000001e-01);
a[30] = (real_t)(-1.0000000000000000e+00);
a[31] = (a[29]*a[30]);

/* Compute outputs: */
out[0] = (xd[3]+((real_t)(1.0050000000000001e-01)*xd[4]));
out[1] = (xd[3]-((real_t)(1.0050000000000001e-01)*xd[4]));
out[2] = (u[0]+((real_t)(1.0050000000000001e-01)*u[1]));
out[3] = (u[0]-((real_t)(1.0050000000000001e-01)*u[1]));
out[4] = a[0];
out[5] = a[1];
out[6] = a[2];
out[7] = a[3];
out[8] = a[4];
out[9] = a[5];
out[10] = a[6];
out[11] = a[7];
out[12] = a[8];
out[13] = a[11];
out[14] = a[12];
out[15] = a[13];
out[16] = a[14];
out[17] = a[15];
out[18] = a[16];
out[19] = a[17];
out[20] = a[18];
out[21] = a[19];
out[22] = a[20];
out[23] = a[21];
out[24] = a[22];
out[25] = a[23];
out[26] = a[24];
out[27] = a[25];
out[28] = a[26];
out[29] = a[27];
out[30] = a[28];
out[31] = a[31];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 22; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 25 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ lRun1 * 5-5 ]), &(acadoWorkspace.evGx[ lRun1 * 25 ]), &(acadoWorkspace.d[ lRun1 * 5 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 25-25 ]), &(acadoWorkspace.evGx[ lRun1 * 25 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.E[ lRun3 * 10 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 10 ]), &(acadoWorkspace.E[ lRun3 * 10 ]) );
}

for (lRun1 = 0; lRun1 < 21; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multQ1Gu( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QE[ lRun3 * 10 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multQN1Gu( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QE[ lRun3 * 10 ]) );
}

for (lRun1 = 0; lRun1 < 22; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 10 ]) );
for (lRun2 = lRun1; lRun2 < 22; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 10 ]), &(acadoWorkspace.evGx[ lRun2 * 25 ]), &(acadoWorkspace.H10[ lRun1 * 10 ]) );
}
}

for (lRun1 = 0; lRun1 < 22; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1 );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 22; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.QE[ lRun5 * 10 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 22; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 22; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.QE[ lRun5 * 10 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 22; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

acado_multQ1d( acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.d[ 5 ]), &(acadoWorkspace.Qd[ 5 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.Qd[ 10 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.Qd[ 15 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 25 ]), &(acadoWorkspace.Qd[ 25 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.Qd[ 30 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 35 ]), &(acadoWorkspace.Qd[ 35 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.Qd[ 45 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 50 ]), &(acadoWorkspace.Qd[ 50 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 55 ]), &(acadoWorkspace.Qd[ 55 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 65 ]), &(acadoWorkspace.Qd[ 65 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.Qd[ 70 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 75 ]), &(acadoWorkspace.Qd[ 75 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.Qd[ 80 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 85 ]), &(acadoWorkspace.Qd[ 85 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.Qd[ 90 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 95 ]), &(acadoWorkspace.Qd[ 95 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 100 ]), &(acadoWorkspace.Qd[ 100 ]) );
acado_multQN1d( &(acadoWorkspace.d[ 105 ]), &(acadoWorkspace.Qd[ 105 ]) );

for (lRun1 = 0; lRun1 < 22; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 22; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 10 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}
for (lRun1 = 0; lRun1 < 22; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 4] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evH[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[3];

acadoWorkspace.evHx[lRun1 * 20] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 20 + 1] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 20 + 2] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 20 + 3] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 20 + 4] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 20 + 5] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 20 + 6] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 20 + 7] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 20 + 8] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 20 + 9] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 20 + 10] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHx[lRun1 * 20 + 11] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHx[lRun1 * 20 + 12] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHx[lRun1 * 20 + 13] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHx[lRun1 * 20 + 14] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHx[lRun1 * 20 + 15] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHx[lRun1 * 20 + 16] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHx[lRun1 * 20 + 17] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHx[lRun1 * 20 + 18] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHx[lRun1 * 20 + 19] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evHu[lRun1 * 8] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evHu[lRun1 * 8 + 1] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evHu[lRun1 * 8 + 2] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evHu[lRun1 * 8 + 3] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evHu[lRun1 * 8 + 4] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evHu[lRun1 * 8 + 5] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evHu[lRun1 * 8 + 6] = acadoWorkspace.conValueOut[30];
acadoWorkspace.evHu[lRun1 * 8 + 7] = acadoWorkspace.conValueOut[31];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];
acadoWorkspace.A01[6] = acadoWorkspace.evHx[6];
acadoWorkspace.A01[7] = acadoWorkspace.evHx[7];
acadoWorkspace.A01[8] = acadoWorkspace.evHx[8];
acadoWorkspace.A01[9] = acadoWorkspace.evHx[9];
acadoWorkspace.A01[10] = acadoWorkspace.evHx[10];
acadoWorkspace.A01[11] = acadoWorkspace.evHx[11];
acadoWorkspace.A01[12] = acadoWorkspace.evHx[12];
acadoWorkspace.A01[13] = acadoWorkspace.evHx[13];
acadoWorkspace.A01[14] = acadoWorkspace.evHx[14];
acadoWorkspace.A01[15] = acadoWorkspace.evHx[15];
acadoWorkspace.A01[16] = acadoWorkspace.evHx[16];
acadoWorkspace.A01[17] = acadoWorkspace.evHx[17];
acadoWorkspace.A01[18] = acadoWorkspace.evHx[18];
acadoWorkspace.A01[19] = acadoWorkspace.evHx[19];

acado_multHxC( &(acadoWorkspace.evHx[ 20 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 20 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.A01[ 40 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.A01[ 80 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.A01[ 100 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.A01[ 120 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.A01[ 140 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.A01[ 160 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.A01[ 180 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 200 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.A01[ 200 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 220 ]), &(acadoWorkspace.evGx[ 250 ]), &(acadoWorkspace.A01[ 220 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.evGx[ 275 ]), &(acadoWorkspace.A01[ 240 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 260 ]), &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.A01[ 260 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 280 ]), &(acadoWorkspace.evGx[ 325 ]), &(acadoWorkspace.A01[ 280 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 300 ]), &(acadoWorkspace.evGx[ 350 ]), &(acadoWorkspace.A01[ 300 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.evGx[ 375 ]), &(acadoWorkspace.A01[ 320 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 340 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.A01[ 340 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 360 ]), &(acadoWorkspace.evGx[ 425 ]), &(acadoWorkspace.A01[ 360 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 380 ]), &(acadoWorkspace.evGx[ 450 ]), &(acadoWorkspace.A01[ 380 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 400 ]), &(acadoWorkspace.evGx[ 475 ]), &(acadoWorkspace.A01[ 400 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 420 ]), &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.A01[ 420 ]) );

for (lRun2 = 0; lRun2 < 21; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun3);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 20 + 20 ]), &(acadoWorkspace.E[ lRun4 * 10 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[44] = acadoWorkspace.evHu[2];
acadoWorkspace.A[45] = acadoWorkspace.evHu[3];
acadoWorkspace.A[88] = acadoWorkspace.evHu[4];
acadoWorkspace.A[89] = acadoWorkspace.evHu[5];
acadoWorkspace.A[132] = acadoWorkspace.evHu[6];
acadoWorkspace.A[133] = acadoWorkspace.evHu[7];
acadoWorkspace.A[178] = acadoWorkspace.evHu[8];
acadoWorkspace.A[179] = acadoWorkspace.evHu[9];
acadoWorkspace.A[222] = acadoWorkspace.evHu[10];
acadoWorkspace.A[223] = acadoWorkspace.evHu[11];
acadoWorkspace.A[266] = acadoWorkspace.evHu[12];
acadoWorkspace.A[267] = acadoWorkspace.evHu[13];
acadoWorkspace.A[310] = acadoWorkspace.evHu[14];
acadoWorkspace.A[311] = acadoWorkspace.evHu[15];
acadoWorkspace.A[356] = acadoWorkspace.evHu[16];
acadoWorkspace.A[357] = acadoWorkspace.evHu[17];
acadoWorkspace.A[400] = acadoWorkspace.evHu[18];
acadoWorkspace.A[401] = acadoWorkspace.evHu[19];
acadoWorkspace.A[444] = acadoWorkspace.evHu[20];
acadoWorkspace.A[445] = acadoWorkspace.evHu[21];
acadoWorkspace.A[488] = acadoWorkspace.evHu[22];
acadoWorkspace.A[489] = acadoWorkspace.evHu[23];
acadoWorkspace.A[534] = acadoWorkspace.evHu[24];
acadoWorkspace.A[535] = acadoWorkspace.evHu[25];
acadoWorkspace.A[578] = acadoWorkspace.evHu[26];
acadoWorkspace.A[579] = acadoWorkspace.evHu[27];
acadoWorkspace.A[622] = acadoWorkspace.evHu[28];
acadoWorkspace.A[623] = acadoWorkspace.evHu[29];
acadoWorkspace.A[666] = acadoWorkspace.evHu[30];
acadoWorkspace.A[667] = acadoWorkspace.evHu[31];
acadoWorkspace.A[712] = acadoWorkspace.evHu[32];
acadoWorkspace.A[713] = acadoWorkspace.evHu[33];
acadoWorkspace.A[756] = acadoWorkspace.evHu[34];
acadoWorkspace.A[757] = acadoWorkspace.evHu[35];
acadoWorkspace.A[800] = acadoWorkspace.evHu[36];
acadoWorkspace.A[801] = acadoWorkspace.evHu[37];
acadoWorkspace.A[844] = acadoWorkspace.evHu[38];
acadoWorkspace.A[845] = acadoWorkspace.evHu[39];
acadoWorkspace.A[890] = acadoWorkspace.evHu[40];
acadoWorkspace.A[891] = acadoWorkspace.evHu[41];
acadoWorkspace.A[934] = acadoWorkspace.evHu[42];
acadoWorkspace.A[935] = acadoWorkspace.evHu[43];
acadoWorkspace.A[978] = acadoWorkspace.evHu[44];
acadoWorkspace.A[979] = acadoWorkspace.evHu[45];
acadoWorkspace.A[1022] = acadoWorkspace.evHu[46];
acadoWorkspace.A[1023] = acadoWorkspace.evHu[47];
acadoWorkspace.A[1068] = acadoWorkspace.evHu[48];
acadoWorkspace.A[1069] = acadoWorkspace.evHu[49];
acadoWorkspace.A[1112] = acadoWorkspace.evHu[50];
acadoWorkspace.A[1113] = acadoWorkspace.evHu[51];
acadoWorkspace.A[1156] = acadoWorkspace.evHu[52];
acadoWorkspace.A[1157] = acadoWorkspace.evHu[53];
acadoWorkspace.A[1200] = acadoWorkspace.evHu[54];
acadoWorkspace.A[1201] = acadoWorkspace.evHu[55];
acadoWorkspace.A[1246] = acadoWorkspace.evHu[56];
acadoWorkspace.A[1247] = acadoWorkspace.evHu[57];
acadoWorkspace.A[1290] = acadoWorkspace.evHu[58];
acadoWorkspace.A[1291] = acadoWorkspace.evHu[59];
acadoWorkspace.A[1334] = acadoWorkspace.evHu[60];
acadoWorkspace.A[1335] = acadoWorkspace.evHu[61];
acadoWorkspace.A[1378] = acadoWorkspace.evHu[62];
acadoWorkspace.A[1379] = acadoWorkspace.evHu[63];
acadoWorkspace.A[1424] = acadoWorkspace.evHu[64];
acadoWorkspace.A[1425] = acadoWorkspace.evHu[65];
acadoWorkspace.A[1468] = acadoWorkspace.evHu[66];
acadoWorkspace.A[1469] = acadoWorkspace.evHu[67];
acadoWorkspace.A[1512] = acadoWorkspace.evHu[68];
acadoWorkspace.A[1513] = acadoWorkspace.evHu[69];
acadoWorkspace.A[1556] = acadoWorkspace.evHu[70];
acadoWorkspace.A[1557] = acadoWorkspace.evHu[71];
acadoWorkspace.A[1602] = acadoWorkspace.evHu[72];
acadoWorkspace.A[1603] = acadoWorkspace.evHu[73];
acadoWorkspace.A[1646] = acadoWorkspace.evHu[74];
acadoWorkspace.A[1647] = acadoWorkspace.evHu[75];
acadoWorkspace.A[1690] = acadoWorkspace.evHu[76];
acadoWorkspace.A[1691] = acadoWorkspace.evHu[77];
acadoWorkspace.A[1734] = acadoWorkspace.evHu[78];
acadoWorkspace.A[1735] = acadoWorkspace.evHu[79];
acadoWorkspace.A[1780] = acadoWorkspace.evHu[80];
acadoWorkspace.A[1781] = acadoWorkspace.evHu[81];
acadoWorkspace.A[1824] = acadoWorkspace.evHu[82];
acadoWorkspace.A[1825] = acadoWorkspace.evHu[83];
acadoWorkspace.A[1868] = acadoWorkspace.evHu[84];
acadoWorkspace.A[1869] = acadoWorkspace.evHu[85];
acadoWorkspace.A[1912] = acadoWorkspace.evHu[86];
acadoWorkspace.A[1913] = acadoWorkspace.evHu[87];
acadoWorkspace.A[1958] = acadoWorkspace.evHu[88];
acadoWorkspace.A[1959] = acadoWorkspace.evHu[89];
acadoWorkspace.A[2002] = acadoWorkspace.evHu[90];
acadoWorkspace.A[2003] = acadoWorkspace.evHu[91];
acadoWorkspace.A[2046] = acadoWorkspace.evHu[92];
acadoWorkspace.A[2047] = acadoWorkspace.evHu[93];
acadoWorkspace.A[2090] = acadoWorkspace.evHu[94];
acadoWorkspace.A[2091] = acadoWorkspace.evHu[95];
acadoWorkspace.A[2136] = acadoWorkspace.evHu[96];
acadoWorkspace.A[2137] = acadoWorkspace.evHu[97];
acadoWorkspace.A[2180] = acadoWorkspace.evHu[98];
acadoWorkspace.A[2181] = acadoWorkspace.evHu[99];
acadoWorkspace.A[2224] = acadoWorkspace.evHu[100];
acadoWorkspace.A[2225] = acadoWorkspace.evHu[101];
acadoWorkspace.A[2268] = acadoWorkspace.evHu[102];
acadoWorkspace.A[2269] = acadoWorkspace.evHu[103];
acadoWorkspace.A[2314] = acadoWorkspace.evHu[104];
acadoWorkspace.A[2315] = acadoWorkspace.evHu[105];
acadoWorkspace.A[2358] = acadoWorkspace.evHu[106];
acadoWorkspace.A[2359] = acadoWorkspace.evHu[107];
acadoWorkspace.A[2402] = acadoWorkspace.evHu[108];
acadoWorkspace.A[2403] = acadoWorkspace.evHu[109];
acadoWorkspace.A[2446] = acadoWorkspace.evHu[110];
acadoWorkspace.A[2447] = acadoWorkspace.evHu[111];
acadoWorkspace.A[2492] = acadoWorkspace.evHu[112];
acadoWorkspace.A[2493] = acadoWorkspace.evHu[113];
acadoWorkspace.A[2536] = acadoWorkspace.evHu[114];
acadoWorkspace.A[2537] = acadoWorkspace.evHu[115];
acadoWorkspace.A[2580] = acadoWorkspace.evHu[116];
acadoWorkspace.A[2581] = acadoWorkspace.evHu[117];
acadoWorkspace.A[2624] = acadoWorkspace.evHu[118];
acadoWorkspace.A[2625] = acadoWorkspace.evHu[119];
acadoWorkspace.A[2670] = acadoWorkspace.evHu[120];
acadoWorkspace.A[2671] = acadoWorkspace.evHu[121];
acadoWorkspace.A[2714] = acadoWorkspace.evHu[122];
acadoWorkspace.A[2715] = acadoWorkspace.evHu[123];
acadoWorkspace.A[2758] = acadoWorkspace.evHu[124];
acadoWorkspace.A[2759] = acadoWorkspace.evHu[125];
acadoWorkspace.A[2802] = acadoWorkspace.evHu[126];
acadoWorkspace.A[2803] = acadoWorkspace.evHu[127];
acadoWorkspace.A[2848] = acadoWorkspace.evHu[128];
acadoWorkspace.A[2849] = acadoWorkspace.evHu[129];
acadoWorkspace.A[2892] = acadoWorkspace.evHu[130];
acadoWorkspace.A[2893] = acadoWorkspace.evHu[131];
acadoWorkspace.A[2936] = acadoWorkspace.evHu[132];
acadoWorkspace.A[2937] = acadoWorkspace.evHu[133];
acadoWorkspace.A[2980] = acadoWorkspace.evHu[134];
acadoWorkspace.A[2981] = acadoWorkspace.evHu[135];
acadoWorkspace.A[3026] = acadoWorkspace.evHu[136];
acadoWorkspace.A[3027] = acadoWorkspace.evHu[137];
acadoWorkspace.A[3070] = acadoWorkspace.evHu[138];
acadoWorkspace.A[3071] = acadoWorkspace.evHu[139];
acadoWorkspace.A[3114] = acadoWorkspace.evHu[140];
acadoWorkspace.A[3115] = acadoWorkspace.evHu[141];
acadoWorkspace.A[3158] = acadoWorkspace.evHu[142];
acadoWorkspace.A[3159] = acadoWorkspace.evHu[143];
acadoWorkspace.A[3204] = acadoWorkspace.evHu[144];
acadoWorkspace.A[3205] = acadoWorkspace.evHu[145];
acadoWorkspace.A[3248] = acadoWorkspace.evHu[146];
acadoWorkspace.A[3249] = acadoWorkspace.evHu[147];
acadoWorkspace.A[3292] = acadoWorkspace.evHu[148];
acadoWorkspace.A[3293] = acadoWorkspace.evHu[149];
acadoWorkspace.A[3336] = acadoWorkspace.evHu[150];
acadoWorkspace.A[3337] = acadoWorkspace.evHu[151];
acadoWorkspace.A[3382] = acadoWorkspace.evHu[152];
acadoWorkspace.A[3383] = acadoWorkspace.evHu[153];
acadoWorkspace.A[3426] = acadoWorkspace.evHu[154];
acadoWorkspace.A[3427] = acadoWorkspace.evHu[155];
acadoWorkspace.A[3470] = acadoWorkspace.evHu[156];
acadoWorkspace.A[3471] = acadoWorkspace.evHu[157];
acadoWorkspace.A[3514] = acadoWorkspace.evHu[158];
acadoWorkspace.A[3515] = acadoWorkspace.evHu[159];
acadoWorkspace.A[3560] = acadoWorkspace.evHu[160];
acadoWorkspace.A[3561] = acadoWorkspace.evHu[161];
acadoWorkspace.A[3604] = acadoWorkspace.evHu[162];
acadoWorkspace.A[3605] = acadoWorkspace.evHu[163];
acadoWorkspace.A[3648] = acadoWorkspace.evHu[164];
acadoWorkspace.A[3649] = acadoWorkspace.evHu[165];
acadoWorkspace.A[3692] = acadoWorkspace.evHu[166];
acadoWorkspace.A[3693] = acadoWorkspace.evHu[167];
acadoWorkspace.A[3738] = acadoWorkspace.evHu[168];
acadoWorkspace.A[3739] = acadoWorkspace.evHu[169];
acadoWorkspace.A[3782] = acadoWorkspace.evHu[170];
acadoWorkspace.A[3783] = acadoWorkspace.evHu[171];
acadoWorkspace.A[3826] = acadoWorkspace.evHu[172];
acadoWorkspace.A[3827] = acadoWorkspace.evHu[173];
acadoWorkspace.A[3870] = acadoWorkspace.evHu[174];
acadoWorkspace.A[3871] = acadoWorkspace.evHu[175];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = acadoVariables.lbAValues[30] - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = acadoVariables.lbAValues[31] - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = acadoVariables.lbAValues[32] - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = acadoVariables.lbAValues[33] - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = acadoVariables.lbAValues[34] - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = acadoVariables.lbAValues[35] - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = acadoVariables.lbAValues[36] - acadoWorkspace.evH[36];
acadoWorkspace.lbA[37] = acadoVariables.lbAValues[37] - acadoWorkspace.evH[37];
acadoWorkspace.lbA[38] = acadoVariables.lbAValues[38] - acadoWorkspace.evH[38];
acadoWorkspace.lbA[39] = acadoVariables.lbAValues[39] - acadoWorkspace.evH[39];
acadoWorkspace.lbA[40] = acadoVariables.lbAValues[40] - acadoWorkspace.evH[40];
acadoWorkspace.lbA[41] = acadoVariables.lbAValues[41] - acadoWorkspace.evH[41];
acadoWorkspace.lbA[42] = acadoVariables.lbAValues[42] - acadoWorkspace.evH[42];
acadoWorkspace.lbA[43] = acadoVariables.lbAValues[43] - acadoWorkspace.evH[43];
acadoWorkspace.lbA[44] = acadoVariables.lbAValues[44] - acadoWorkspace.evH[44];
acadoWorkspace.lbA[45] = acadoVariables.lbAValues[45] - acadoWorkspace.evH[45];
acadoWorkspace.lbA[46] = acadoVariables.lbAValues[46] - acadoWorkspace.evH[46];
acadoWorkspace.lbA[47] = acadoVariables.lbAValues[47] - acadoWorkspace.evH[47];
acadoWorkspace.lbA[48] = acadoVariables.lbAValues[48] - acadoWorkspace.evH[48];
acadoWorkspace.lbA[49] = acadoVariables.lbAValues[49] - acadoWorkspace.evH[49];
acadoWorkspace.lbA[50] = acadoVariables.lbAValues[50] - acadoWorkspace.evH[50];
acadoWorkspace.lbA[51] = acadoVariables.lbAValues[51] - acadoWorkspace.evH[51];
acadoWorkspace.lbA[52] = acadoVariables.lbAValues[52] - acadoWorkspace.evH[52];
acadoWorkspace.lbA[53] = acadoVariables.lbAValues[53] - acadoWorkspace.evH[53];
acadoWorkspace.lbA[54] = acadoVariables.lbAValues[54] - acadoWorkspace.evH[54];
acadoWorkspace.lbA[55] = acadoVariables.lbAValues[55] - acadoWorkspace.evH[55];
acadoWorkspace.lbA[56] = acadoVariables.lbAValues[56] - acadoWorkspace.evH[56];
acadoWorkspace.lbA[57] = acadoVariables.lbAValues[57] - acadoWorkspace.evH[57];
acadoWorkspace.lbA[58] = acadoVariables.lbAValues[58] - acadoWorkspace.evH[58];
acadoWorkspace.lbA[59] = acadoVariables.lbAValues[59] - acadoWorkspace.evH[59];
acadoWorkspace.lbA[60] = acadoVariables.lbAValues[60] - acadoWorkspace.evH[60];
acadoWorkspace.lbA[61] = acadoVariables.lbAValues[61] - acadoWorkspace.evH[61];
acadoWorkspace.lbA[62] = acadoVariables.lbAValues[62] - acadoWorkspace.evH[62];
acadoWorkspace.lbA[63] = acadoVariables.lbAValues[63] - acadoWorkspace.evH[63];
acadoWorkspace.lbA[64] = acadoVariables.lbAValues[64] - acadoWorkspace.evH[64];
acadoWorkspace.lbA[65] = acadoVariables.lbAValues[65] - acadoWorkspace.evH[65];
acadoWorkspace.lbA[66] = acadoVariables.lbAValues[66] - acadoWorkspace.evH[66];
acadoWorkspace.lbA[67] = acadoVariables.lbAValues[67] - acadoWorkspace.evH[67];
acadoWorkspace.lbA[68] = acadoVariables.lbAValues[68] - acadoWorkspace.evH[68];
acadoWorkspace.lbA[69] = acadoVariables.lbAValues[69] - acadoWorkspace.evH[69];
acadoWorkspace.lbA[70] = acadoVariables.lbAValues[70] - acadoWorkspace.evH[70];
acadoWorkspace.lbA[71] = acadoVariables.lbAValues[71] - acadoWorkspace.evH[71];
acadoWorkspace.lbA[72] = acadoVariables.lbAValues[72] - acadoWorkspace.evH[72];
acadoWorkspace.lbA[73] = acadoVariables.lbAValues[73] - acadoWorkspace.evH[73];
acadoWorkspace.lbA[74] = acadoVariables.lbAValues[74] - acadoWorkspace.evH[74];
acadoWorkspace.lbA[75] = acadoVariables.lbAValues[75] - acadoWorkspace.evH[75];
acadoWorkspace.lbA[76] = acadoVariables.lbAValues[76] - acadoWorkspace.evH[76];
acadoWorkspace.lbA[77] = acadoVariables.lbAValues[77] - acadoWorkspace.evH[77];
acadoWorkspace.lbA[78] = acadoVariables.lbAValues[78] - acadoWorkspace.evH[78];
acadoWorkspace.lbA[79] = acadoVariables.lbAValues[79] - acadoWorkspace.evH[79];
acadoWorkspace.lbA[80] = acadoVariables.lbAValues[80] - acadoWorkspace.evH[80];
acadoWorkspace.lbA[81] = acadoVariables.lbAValues[81] - acadoWorkspace.evH[81];
acadoWorkspace.lbA[82] = acadoVariables.lbAValues[82] - acadoWorkspace.evH[82];
acadoWorkspace.lbA[83] = acadoVariables.lbAValues[83] - acadoWorkspace.evH[83];
acadoWorkspace.lbA[84] = acadoVariables.lbAValues[84] - acadoWorkspace.evH[84];
acadoWorkspace.lbA[85] = acadoVariables.lbAValues[85] - acadoWorkspace.evH[85];
acadoWorkspace.lbA[86] = acadoVariables.lbAValues[86] - acadoWorkspace.evH[86];
acadoWorkspace.lbA[87] = acadoVariables.lbAValues[87] - acadoWorkspace.evH[87];

acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = acadoVariables.ubAValues[30] - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = acadoVariables.ubAValues[31] - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = acadoVariables.ubAValues[32] - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = acadoVariables.ubAValues[33] - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = acadoVariables.ubAValues[34] - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = acadoVariables.ubAValues[35] - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = acadoVariables.ubAValues[36] - acadoWorkspace.evH[36];
acadoWorkspace.ubA[37] = acadoVariables.ubAValues[37] - acadoWorkspace.evH[37];
acadoWorkspace.ubA[38] = acadoVariables.ubAValues[38] - acadoWorkspace.evH[38];
acadoWorkspace.ubA[39] = acadoVariables.ubAValues[39] - acadoWorkspace.evH[39];
acadoWorkspace.ubA[40] = acadoVariables.ubAValues[40] - acadoWorkspace.evH[40];
acadoWorkspace.ubA[41] = acadoVariables.ubAValues[41] - acadoWorkspace.evH[41];
acadoWorkspace.ubA[42] = acadoVariables.ubAValues[42] - acadoWorkspace.evH[42];
acadoWorkspace.ubA[43] = acadoVariables.ubAValues[43] - acadoWorkspace.evH[43];
acadoWorkspace.ubA[44] = acadoVariables.ubAValues[44] - acadoWorkspace.evH[44];
acadoWorkspace.ubA[45] = acadoVariables.ubAValues[45] - acadoWorkspace.evH[45];
acadoWorkspace.ubA[46] = acadoVariables.ubAValues[46] - acadoWorkspace.evH[46];
acadoWorkspace.ubA[47] = acadoVariables.ubAValues[47] - acadoWorkspace.evH[47];
acadoWorkspace.ubA[48] = acadoVariables.ubAValues[48] - acadoWorkspace.evH[48];
acadoWorkspace.ubA[49] = acadoVariables.ubAValues[49] - acadoWorkspace.evH[49];
acadoWorkspace.ubA[50] = acadoVariables.ubAValues[50] - acadoWorkspace.evH[50];
acadoWorkspace.ubA[51] = acadoVariables.ubAValues[51] - acadoWorkspace.evH[51];
acadoWorkspace.ubA[52] = acadoVariables.ubAValues[52] - acadoWorkspace.evH[52];
acadoWorkspace.ubA[53] = acadoVariables.ubAValues[53] - acadoWorkspace.evH[53];
acadoWorkspace.ubA[54] = acadoVariables.ubAValues[54] - acadoWorkspace.evH[54];
acadoWorkspace.ubA[55] = acadoVariables.ubAValues[55] - acadoWorkspace.evH[55];
acadoWorkspace.ubA[56] = acadoVariables.ubAValues[56] - acadoWorkspace.evH[56];
acadoWorkspace.ubA[57] = acadoVariables.ubAValues[57] - acadoWorkspace.evH[57];
acadoWorkspace.ubA[58] = acadoVariables.ubAValues[58] - acadoWorkspace.evH[58];
acadoWorkspace.ubA[59] = acadoVariables.ubAValues[59] - acadoWorkspace.evH[59];
acadoWorkspace.ubA[60] = acadoVariables.ubAValues[60] - acadoWorkspace.evH[60];
acadoWorkspace.ubA[61] = acadoVariables.ubAValues[61] - acadoWorkspace.evH[61];
acadoWorkspace.ubA[62] = acadoVariables.ubAValues[62] - acadoWorkspace.evH[62];
acadoWorkspace.ubA[63] = acadoVariables.ubAValues[63] - acadoWorkspace.evH[63];
acadoWorkspace.ubA[64] = acadoVariables.ubAValues[64] - acadoWorkspace.evH[64];
acadoWorkspace.ubA[65] = acadoVariables.ubAValues[65] - acadoWorkspace.evH[65];
acadoWorkspace.ubA[66] = acadoVariables.ubAValues[66] - acadoWorkspace.evH[66];
acadoWorkspace.ubA[67] = acadoVariables.ubAValues[67] - acadoWorkspace.evH[67];
acadoWorkspace.ubA[68] = acadoVariables.ubAValues[68] - acadoWorkspace.evH[68];
acadoWorkspace.ubA[69] = acadoVariables.ubAValues[69] - acadoWorkspace.evH[69];
acadoWorkspace.ubA[70] = acadoVariables.ubAValues[70] - acadoWorkspace.evH[70];
acadoWorkspace.ubA[71] = acadoVariables.ubAValues[71] - acadoWorkspace.evH[71];
acadoWorkspace.ubA[72] = acadoVariables.ubAValues[72] - acadoWorkspace.evH[72];
acadoWorkspace.ubA[73] = acadoVariables.ubAValues[73] - acadoWorkspace.evH[73];
acadoWorkspace.ubA[74] = acadoVariables.ubAValues[74] - acadoWorkspace.evH[74];
acadoWorkspace.ubA[75] = acadoVariables.ubAValues[75] - acadoWorkspace.evH[75];
acadoWorkspace.ubA[76] = acadoVariables.ubAValues[76] - acadoWorkspace.evH[76];
acadoWorkspace.ubA[77] = acadoVariables.ubAValues[77] - acadoWorkspace.evH[77];
acadoWorkspace.ubA[78] = acadoVariables.ubAValues[78] - acadoWorkspace.evH[78];
acadoWorkspace.ubA[79] = acadoVariables.ubAValues[79] - acadoWorkspace.evH[79];
acadoWorkspace.ubA[80] = acadoVariables.ubAValues[80] - acadoWorkspace.evH[80];
acadoWorkspace.ubA[81] = acadoVariables.ubAValues[81] - acadoWorkspace.evH[81];
acadoWorkspace.ubA[82] = acadoVariables.ubAValues[82] - acadoWorkspace.evH[82];
acadoWorkspace.ubA[83] = acadoVariables.ubAValues[83] - acadoWorkspace.evH[83];
acadoWorkspace.ubA[84] = acadoVariables.ubAValues[84] - acadoWorkspace.evH[84];
acadoWorkspace.ubA[85] = acadoVariables.ubAValues[85] - acadoWorkspace.evH[85];
acadoWorkspace.ubA[86] = acadoVariables.ubAValues[86] - acadoWorkspace.evH[86];
acadoWorkspace.ubA[87] = acadoVariables.ubAValues[87] - acadoWorkspace.evH[87];

acado_macHxd( &(acadoWorkspace.evHx[ 20 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.d[ 5 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.lbA[ 20 ]), &(acadoWorkspace.ubA[ 20 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.d[ 25 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.d[ 35 ]), &(acadoWorkspace.lbA[ 32 ]), &(acadoWorkspace.ubA[ 32 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.lbA[ 36 ]), &(acadoWorkspace.ubA[ 36 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 200 ]), &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.lbA[ 40 ]), &(acadoWorkspace.ubA[ 40 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 220 ]), &(acadoWorkspace.d[ 50 ]), &(acadoWorkspace.lbA[ 44 ]), &(acadoWorkspace.ubA[ 44 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.d[ 55 ]), &(acadoWorkspace.lbA[ 48 ]), &(acadoWorkspace.ubA[ 48 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 260 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.lbA[ 52 ]), &(acadoWorkspace.ubA[ 52 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 280 ]), &(acadoWorkspace.d[ 65 ]), &(acadoWorkspace.lbA[ 56 ]), &(acadoWorkspace.ubA[ 56 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 300 ]), &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.lbA[ 60 ]), &(acadoWorkspace.ubA[ 60 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 320 ]), &(acadoWorkspace.d[ 75 ]), &(acadoWorkspace.lbA[ 64 ]), &(acadoWorkspace.ubA[ 64 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 340 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.lbA[ 68 ]), &(acadoWorkspace.ubA[ 68 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 360 ]), &(acadoWorkspace.d[ 85 ]), &(acadoWorkspace.lbA[ 72 ]), &(acadoWorkspace.ubA[ 72 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 380 ]), &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.lbA[ 76 ]), &(acadoWorkspace.ubA[ 76 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 400 ]), &(acadoWorkspace.d[ 95 ]), &(acadoWorkspace.lbA[ 80 ]), &(acadoWorkspace.ubA[ 80 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 420 ]), &(acadoWorkspace.d[ 100 ]), &(acadoWorkspace.lbA[ 84 ]), &(acadoWorkspace.ubA[ 84 ]) );

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.Dy[70] -= acadoVariables.y[70];
acadoWorkspace.Dy[71] -= acadoVariables.y[71];
acadoWorkspace.Dy[72] -= acadoVariables.y[72];
acadoWorkspace.Dy[73] -= acadoVariables.y[73];
acadoWorkspace.Dy[74] -= acadoVariables.y[74];
acadoWorkspace.Dy[75] -= acadoVariables.y[75];
acadoWorkspace.Dy[76] -= acadoVariables.y[76];
acadoWorkspace.Dy[77] -= acadoVariables.y[77];
acadoWorkspace.Dy[78] -= acadoVariables.y[78];
acadoWorkspace.Dy[79] -= acadoVariables.y[79];
acadoWorkspace.Dy[80] -= acadoVariables.y[80];
acadoWorkspace.Dy[81] -= acadoVariables.y[81];
acadoWorkspace.Dy[82] -= acadoVariables.y[82];
acadoWorkspace.Dy[83] -= acadoVariables.y[83];
acadoWorkspace.Dy[84] -= acadoVariables.y[84];
acadoWorkspace.Dy[85] -= acadoVariables.y[85];
acadoWorkspace.Dy[86] -= acadoVariables.y[86];
acadoWorkspace.Dy[87] -= acadoVariables.y[87];
acadoWorkspace.Dy[88] -= acadoVariables.y[88];
acadoWorkspace.Dy[89] -= acadoVariables.y[89];
acadoWorkspace.Dy[90] -= acadoVariables.y[90];
acadoWorkspace.Dy[91] -= acadoVariables.y[91];
acadoWorkspace.Dy[92] -= acadoVariables.y[92];
acadoWorkspace.Dy[93] -= acadoVariables.y[93];
acadoWorkspace.Dy[94] -= acadoVariables.y[94];
acadoWorkspace.Dy[95] -= acadoVariables.y[95];
acadoWorkspace.Dy[96] -= acadoVariables.y[96];
acadoWorkspace.Dy[97] -= acadoVariables.y[97];
acadoWorkspace.Dy[98] -= acadoVariables.y[98];
acadoWorkspace.Dy[99] -= acadoVariables.y[99];
acadoWorkspace.Dy[100] -= acadoVariables.y[100];
acadoWorkspace.Dy[101] -= acadoVariables.y[101];
acadoWorkspace.Dy[102] -= acadoVariables.y[102];
acadoWorkspace.Dy[103] -= acadoVariables.y[103];
acadoWorkspace.Dy[104] -= acadoVariables.y[104];
acadoWorkspace.Dy[105] -= acadoVariables.y[105];
acadoWorkspace.Dy[106] -= acadoVariables.y[106];
acadoWorkspace.Dy[107] -= acadoVariables.y[107];
acadoWorkspace.Dy[108] -= acadoVariables.y[108];
acadoWorkspace.Dy[109] -= acadoVariables.y[109];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 42 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.QDy[ 5 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.QDy[ 25 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 35 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 50 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 55 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.QDy[ 65 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 70 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.QDy[ 75 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.QDy[ 85 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.QDy[ 95 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 105 ]) );

acadoWorkspace.QDy[110] = + (real_t)5.0000000000000000e+04*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[111] = + (real_t)5.0000000000000000e+04*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[112] = + (real_t)1.0000000000000000e+02*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[113] = + (real_t)1.0000000000000000e+03*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[114] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[4];

acadoWorkspace.QDy[5] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[6] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[78] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[79] += acadoWorkspace.Qd[74];
acadoWorkspace.QDy[80] += acadoWorkspace.Qd[75];
acadoWorkspace.QDy[81] += acadoWorkspace.Qd[76];
acadoWorkspace.QDy[82] += acadoWorkspace.Qd[77];
acadoWorkspace.QDy[83] += acadoWorkspace.Qd[78];
acadoWorkspace.QDy[84] += acadoWorkspace.Qd[79];
acadoWorkspace.QDy[85] += acadoWorkspace.Qd[80];
acadoWorkspace.QDy[86] += acadoWorkspace.Qd[81];
acadoWorkspace.QDy[87] += acadoWorkspace.Qd[82];
acadoWorkspace.QDy[88] += acadoWorkspace.Qd[83];
acadoWorkspace.QDy[89] += acadoWorkspace.Qd[84];
acadoWorkspace.QDy[90] += acadoWorkspace.Qd[85];
acadoWorkspace.QDy[91] += acadoWorkspace.Qd[86];
acadoWorkspace.QDy[92] += acadoWorkspace.Qd[87];
acadoWorkspace.QDy[93] += acadoWorkspace.Qd[88];
acadoWorkspace.QDy[94] += acadoWorkspace.Qd[89];
acadoWorkspace.QDy[95] += acadoWorkspace.Qd[90];
acadoWorkspace.QDy[96] += acadoWorkspace.Qd[91];
acadoWorkspace.QDy[97] += acadoWorkspace.Qd[92];
acadoWorkspace.QDy[98] += acadoWorkspace.Qd[93];
acadoWorkspace.QDy[99] += acadoWorkspace.Qd[94];
acadoWorkspace.QDy[100] += acadoWorkspace.Qd[95];
acadoWorkspace.QDy[101] += acadoWorkspace.Qd[96];
acadoWorkspace.QDy[102] += acadoWorkspace.Qd[97];
acadoWorkspace.QDy[103] += acadoWorkspace.Qd[98];
acadoWorkspace.QDy[104] += acadoWorkspace.Qd[99];
acadoWorkspace.QDy[105] += acadoWorkspace.Qd[100];
acadoWorkspace.QDy[106] += acadoWorkspace.Qd[101];
acadoWorkspace.QDy[107] += acadoWorkspace.Qd[102];
acadoWorkspace.QDy[108] += acadoWorkspace.Qd[103];
acadoWorkspace.QDy[109] += acadoWorkspace.Qd[104];
acadoWorkspace.QDy[110] += acadoWorkspace.Qd[105];
acadoWorkspace.QDy[111] += acadoWorkspace.Qd[106];
acadoWorkspace.QDy[112] += acadoWorkspace.Qd[107];
acadoWorkspace.QDy[113] += acadoWorkspace.Qd[108];
acadoWorkspace.QDy[114] += acadoWorkspace.Qd[109];

for (lRun1 = 0; lRun1 < 22; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 22; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QDy[ lRun2 * 5 + 5 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[1] += + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[2] += + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[3] += + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[4] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[5] += + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[6] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[7] += + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[8] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[9] += + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[10] += + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[11] += + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[12] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[13] += + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[14] += + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[15] += + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[16] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[17] += + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[18] += + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[19] += + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[20] += + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[21] += + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[22] += + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[23] += + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[24] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[25] += + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[26] += + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[27] += + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[28] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[29] += + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[30] += + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[31] += + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[32] += + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[33] += + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[34] += + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[35] += + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[36] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[37] += + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[38] += + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[39] += + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[40] += + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[41] += + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[42] += + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[43] += + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[4];

acadoWorkspace.lb[0] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[43];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+12 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+12 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+12 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)1.0000000000000000e+12 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)1.0000000000000000e+12 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)1.0000000000000000e+12 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+12 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)1.0000000000000000e+12 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)1.0000000000000000e+12 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)1.0000000000000000e+12 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.0000000000000000e+12 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)1.0000000000000000e+12 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)1.0000000000000000e+12 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)1.0000000000000000e+12 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)1.0000000000000000e+12 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)1.0000000000000000e+12 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)1.0000000000000000e+12 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)1.0000000000000000e+12 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)1.0000000000000000e+12 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)1.0000000000000000e+12 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)1.0000000000000000e+12 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)1.0000000000000000e+12 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)1.0000000000000000e+12 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)1.0000000000000000e+12 - acadoVariables.u[43];

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[40] = + acadoWorkspace.A01[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[204]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[41] = + acadoWorkspace.A01[205]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[206]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[207]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[208]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[209]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[42] = + acadoWorkspace.A01[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[214]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[43] = + acadoWorkspace.A01[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[219]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[44] = + acadoWorkspace.A01[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[224]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[45] = + acadoWorkspace.A01[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[228]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[229]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[46] = + acadoWorkspace.A01[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[234]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[47] = + acadoWorkspace.A01[235]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[236]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[237]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[238]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[239]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[48] = + acadoWorkspace.A01[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[244]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[49] = + acadoWorkspace.A01[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[249]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[50] = + acadoWorkspace.A01[250]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[251]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[252]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[253]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[254]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[51] = + acadoWorkspace.A01[255]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[256]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[257]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[258]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[259]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[52] = + acadoWorkspace.A01[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[264]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[53] = + acadoWorkspace.A01[265]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[266]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[267]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[268]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[269]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[54] = + acadoWorkspace.A01[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[274]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[55] = + acadoWorkspace.A01[275]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[276]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[277]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[278]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[279]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[56] = + acadoWorkspace.A01[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[284]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[57] = + acadoWorkspace.A01[285]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[286]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[287]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[288]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[289]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[58] = + acadoWorkspace.A01[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[294]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[59] = + acadoWorkspace.A01[295]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[296]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[297]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[298]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[299]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[60] = + acadoWorkspace.A01[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[304]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[61] = + acadoWorkspace.A01[305]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[306]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[307]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[308]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[309]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[62] = + acadoWorkspace.A01[310]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[311]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[312]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[313]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[314]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[63] = + acadoWorkspace.A01[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[317]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[318]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[319]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[64] = + acadoWorkspace.A01[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[324]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[65] = + acadoWorkspace.A01[325]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[326]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[327]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[328]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[329]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[66] = + acadoWorkspace.A01[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[333]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[334]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[67] = + acadoWorkspace.A01[335]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[336]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[337]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[338]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[339]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[68] = + acadoWorkspace.A01[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[344]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[69] = + acadoWorkspace.A01[345]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[346]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[347]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[348]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[349]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[70] = + acadoWorkspace.A01[350]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[351]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[352]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[353]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[354]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[71] = + acadoWorkspace.A01[355]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[356]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[357]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[358]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[359]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[72] = + acadoWorkspace.A01[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[364]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[73] = + acadoWorkspace.A01[365]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[366]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[367]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[368]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[369]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[74] = + acadoWorkspace.A01[370]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[371]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[372]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[373]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[374]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[75] = + acadoWorkspace.A01[375]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[376]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[377]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[378]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[379]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[76] = + acadoWorkspace.A01[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[383]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[384]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[77] = + acadoWorkspace.A01[385]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[386]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[387]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[388]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[389]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[78] = + acadoWorkspace.A01[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[393]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[394]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[79] = + acadoWorkspace.A01[395]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[396]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[397]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[398]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[399]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[80] = + acadoWorkspace.A01[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[403]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[404]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[81] = + acadoWorkspace.A01[405]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[406]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[407]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[408]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[409]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[82] = + acadoWorkspace.A01[410]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[411]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[412]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[413]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[414]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[83] = + acadoWorkspace.A01[415]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[416]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[417]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[418]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[419]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[84] = + acadoWorkspace.A01[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[424]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[85] = + acadoWorkspace.A01[425]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[426]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[427]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[428]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[429]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[86] = + acadoWorkspace.A01[430]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[431]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[432]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[433]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[434]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[87] = + acadoWorkspace.A01[435]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[436]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[437]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[438]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[439]*acadoWorkspace.Dx0[4];
acadoWorkspace.lbA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.lbA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.lbA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.lbA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.lbA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.lbA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.lbA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.lbA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.lbA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.lbA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.lbA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.lbA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.lbA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.lbA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.lbA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.lbA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.lbA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.lbA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.lbA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.lbA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.lbA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.lbA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.lbA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.lbA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.lbA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.lbA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.lbA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.lbA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.lbA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.lbA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.lbA[49] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.lbA[50] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.lbA[51] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.lbA[52] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.lbA[53] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.lbA[54] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.lbA[55] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.lbA[56] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.lbA[57] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.lbA[58] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.lbA[59] -= acadoWorkspace.pacA01Dx0[59];
acadoWorkspace.lbA[60] -= acadoWorkspace.pacA01Dx0[60];
acadoWorkspace.lbA[61] -= acadoWorkspace.pacA01Dx0[61];
acadoWorkspace.lbA[62] -= acadoWorkspace.pacA01Dx0[62];
acadoWorkspace.lbA[63] -= acadoWorkspace.pacA01Dx0[63];
acadoWorkspace.lbA[64] -= acadoWorkspace.pacA01Dx0[64];
acadoWorkspace.lbA[65] -= acadoWorkspace.pacA01Dx0[65];
acadoWorkspace.lbA[66] -= acadoWorkspace.pacA01Dx0[66];
acadoWorkspace.lbA[67] -= acadoWorkspace.pacA01Dx0[67];
acadoWorkspace.lbA[68] -= acadoWorkspace.pacA01Dx0[68];
acadoWorkspace.lbA[69] -= acadoWorkspace.pacA01Dx0[69];
acadoWorkspace.lbA[70] -= acadoWorkspace.pacA01Dx0[70];
acadoWorkspace.lbA[71] -= acadoWorkspace.pacA01Dx0[71];
acadoWorkspace.lbA[72] -= acadoWorkspace.pacA01Dx0[72];
acadoWorkspace.lbA[73] -= acadoWorkspace.pacA01Dx0[73];
acadoWorkspace.lbA[74] -= acadoWorkspace.pacA01Dx0[74];
acadoWorkspace.lbA[75] -= acadoWorkspace.pacA01Dx0[75];
acadoWorkspace.lbA[76] -= acadoWorkspace.pacA01Dx0[76];
acadoWorkspace.lbA[77] -= acadoWorkspace.pacA01Dx0[77];
acadoWorkspace.lbA[78] -= acadoWorkspace.pacA01Dx0[78];
acadoWorkspace.lbA[79] -= acadoWorkspace.pacA01Dx0[79];
acadoWorkspace.lbA[80] -= acadoWorkspace.pacA01Dx0[80];
acadoWorkspace.lbA[81] -= acadoWorkspace.pacA01Dx0[81];
acadoWorkspace.lbA[82] -= acadoWorkspace.pacA01Dx0[82];
acadoWorkspace.lbA[83] -= acadoWorkspace.pacA01Dx0[83];
acadoWorkspace.lbA[84] -= acadoWorkspace.pacA01Dx0[84];
acadoWorkspace.lbA[85] -= acadoWorkspace.pacA01Dx0[85];
acadoWorkspace.lbA[86] -= acadoWorkspace.pacA01Dx0[86];
acadoWorkspace.lbA[87] -= acadoWorkspace.pacA01Dx0[87];

acadoWorkspace.ubA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.ubA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.ubA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.ubA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.ubA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.ubA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.ubA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.ubA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.ubA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.ubA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.ubA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.ubA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.ubA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.ubA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.ubA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.ubA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.ubA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.ubA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.ubA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.ubA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.ubA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.ubA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.ubA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.ubA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.ubA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.ubA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.ubA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.ubA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.ubA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.ubA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.ubA[49] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.ubA[50] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.ubA[51] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.ubA[52] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.ubA[53] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.ubA[54] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.ubA[55] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.ubA[56] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.ubA[57] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.ubA[58] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.ubA[59] -= acadoWorkspace.pacA01Dx0[59];
acadoWorkspace.ubA[60] -= acadoWorkspace.pacA01Dx0[60];
acadoWorkspace.ubA[61] -= acadoWorkspace.pacA01Dx0[61];
acadoWorkspace.ubA[62] -= acadoWorkspace.pacA01Dx0[62];
acadoWorkspace.ubA[63] -= acadoWorkspace.pacA01Dx0[63];
acadoWorkspace.ubA[64] -= acadoWorkspace.pacA01Dx0[64];
acadoWorkspace.ubA[65] -= acadoWorkspace.pacA01Dx0[65];
acadoWorkspace.ubA[66] -= acadoWorkspace.pacA01Dx0[66];
acadoWorkspace.ubA[67] -= acadoWorkspace.pacA01Dx0[67];
acadoWorkspace.ubA[68] -= acadoWorkspace.pacA01Dx0[68];
acadoWorkspace.ubA[69] -= acadoWorkspace.pacA01Dx0[69];
acadoWorkspace.ubA[70] -= acadoWorkspace.pacA01Dx0[70];
acadoWorkspace.ubA[71] -= acadoWorkspace.pacA01Dx0[71];
acadoWorkspace.ubA[72] -= acadoWorkspace.pacA01Dx0[72];
acadoWorkspace.ubA[73] -= acadoWorkspace.pacA01Dx0[73];
acadoWorkspace.ubA[74] -= acadoWorkspace.pacA01Dx0[74];
acadoWorkspace.ubA[75] -= acadoWorkspace.pacA01Dx0[75];
acadoWorkspace.ubA[76] -= acadoWorkspace.pacA01Dx0[76];
acadoWorkspace.ubA[77] -= acadoWorkspace.pacA01Dx0[77];
acadoWorkspace.ubA[78] -= acadoWorkspace.pacA01Dx0[78];
acadoWorkspace.ubA[79] -= acadoWorkspace.pacA01Dx0[79];
acadoWorkspace.ubA[80] -= acadoWorkspace.pacA01Dx0[80];
acadoWorkspace.ubA[81] -= acadoWorkspace.pacA01Dx0[81];
acadoWorkspace.ubA[82] -= acadoWorkspace.pacA01Dx0[82];
acadoWorkspace.ubA[83] -= acadoWorkspace.pacA01Dx0[83];
acadoWorkspace.ubA[84] -= acadoWorkspace.pacA01Dx0[84];
acadoWorkspace.ubA[85] -= acadoWorkspace.pacA01Dx0[85];
acadoWorkspace.ubA[86] -= acadoWorkspace.pacA01Dx0[86];
acadoWorkspace.ubA[87] -= acadoWorkspace.pacA01Dx0[87];

}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
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

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];

acadoVariables.x[5] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[0];
acadoVariables.x[6] += + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[1];
acadoVariables.x[7] += + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[2];
acadoVariables.x[8] += + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[3];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[4];
acadoVariables.x[10] += + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[5];
acadoVariables.x[11] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[6];
acadoVariables.x[12] += + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[7];
acadoVariables.x[13] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[8];
acadoVariables.x[14] += + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[9];
acadoVariables.x[15] += + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[10];
acadoVariables.x[16] += + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[11];
acadoVariables.x[17] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[12];
acadoVariables.x[18] += + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[13];
acadoVariables.x[19] += + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[14];
acadoVariables.x[20] += + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[15];
acadoVariables.x[21] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[16];
acadoVariables.x[22] += + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[17];
acadoVariables.x[23] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[18];
acadoVariables.x[24] += + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[19];
acadoVariables.x[25] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[20];
acadoVariables.x[26] += + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[21];
acadoVariables.x[27] += + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[22];
acadoVariables.x[28] += + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[23];
acadoVariables.x[29] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[24];
acadoVariables.x[30] += + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[25];
acadoVariables.x[31] += + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[26];
acadoVariables.x[32] += + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[27];
acadoVariables.x[33] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[28];
acadoVariables.x[34] += + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[29];
acadoVariables.x[35] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[30];
acadoVariables.x[36] += + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[31];
acadoVariables.x[37] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[32];
acadoVariables.x[38] += + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[33];
acadoVariables.x[39] += + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[34];
acadoVariables.x[40] += + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[35];
acadoVariables.x[41] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[36];
acadoVariables.x[42] += + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[37];
acadoVariables.x[43] += + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[38];
acadoVariables.x[44] += + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[39];
acadoVariables.x[45] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[40];
acadoVariables.x[46] += + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[41];
acadoVariables.x[47] += + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[42];
acadoVariables.x[48] += + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[43];
acadoVariables.x[49] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[44];
acadoVariables.x[50] += + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[45];
acadoVariables.x[51] += + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[46];
acadoVariables.x[52] += + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[47];
acadoVariables.x[53] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[48];
acadoVariables.x[54] += + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[49];
acadoVariables.x[55] += + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[50];
acadoVariables.x[56] += + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[51];
acadoVariables.x[57] += + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[52];
acadoVariables.x[58] += + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[53];
acadoVariables.x[59] += + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[54];
acadoVariables.x[60] += + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[55];
acadoVariables.x[61] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[56];
acadoVariables.x[62] += + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[57];
acadoVariables.x[63] += + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[58];
acadoVariables.x[64] += + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[59];
acadoVariables.x[65] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[60];
acadoVariables.x[66] += + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[61];
acadoVariables.x[67] += + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[62];
acadoVariables.x[68] += + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[63];
acadoVariables.x[69] += + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[64];
acadoVariables.x[70] += + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[65];
acadoVariables.x[71] += + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[66];
acadoVariables.x[72] += + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[67];
acadoVariables.x[73] += + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[68];
acadoVariables.x[74] += + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[69];
acadoVariables.x[75] += + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[70];
acadoVariables.x[76] += + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[71];
acadoVariables.x[77] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[72];
acadoVariables.x[78] += + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[73];
acadoVariables.x[79] += + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[74];
acadoVariables.x[80] += + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[75];
acadoVariables.x[81] += + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[76];
acadoVariables.x[82] += + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[77];
acadoVariables.x[83] += + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[78];
acadoVariables.x[84] += + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[79];
acadoVariables.x[85] += + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[80];
acadoVariables.x[86] += + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[81];
acadoVariables.x[87] += + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[82];
acadoVariables.x[88] += + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[83];
acadoVariables.x[89] += + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[84];
acadoVariables.x[90] += + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[85];
acadoVariables.x[91] += + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[86];
acadoVariables.x[92] += + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[87];
acadoVariables.x[93] += + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[88];
acadoVariables.x[94] += + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[89];
acadoVariables.x[95] += + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[90];
acadoVariables.x[96] += + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[91];
acadoVariables.x[97] += + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[92];
acadoVariables.x[98] += + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[93];
acadoVariables.x[99] += + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[94];
acadoVariables.x[100] += + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[95];
acadoVariables.x[101] += + acadoWorkspace.evGx[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[484]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[96];
acadoVariables.x[102] += + acadoWorkspace.evGx[485]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[486]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[487]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[488]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[489]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[97];
acadoVariables.x[103] += + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[98];
acadoVariables.x[104] += + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[99];
acadoVariables.x[105] += + acadoWorkspace.evGx[500]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[501]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[502]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[503]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[504]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[100];
acadoVariables.x[106] += + acadoWorkspace.evGx[505]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[506]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[507]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[508]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[509]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[101];
acadoVariables.x[107] += + acadoWorkspace.evGx[510]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[511]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[512]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[513]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[514]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[102];
acadoVariables.x[108] += + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[103];
acadoVariables.x[109] += + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[104];
acadoVariables.x[110] += + acadoWorkspace.evGx[525]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[526]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[527]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[528]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[529]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[105];
acadoVariables.x[111] += + acadoWorkspace.evGx[530]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[531]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[532]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[533]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[534]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[106];
acadoVariables.x[112] += + acadoWorkspace.evGx[535]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[536]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[537]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[538]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[539]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[107];
acadoVariables.x[113] += + acadoWorkspace.evGx[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[544]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[108];
acadoVariables.x[114] += + acadoWorkspace.evGx[545]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[546]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[547]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[548]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[549]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[109];

for (lRun1 = 0; lRun1 < 22; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.x[ lRun2 * 2 ]), &(acadoVariables.x[ lRun1 * 5 + 5 ]) );
}
}
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
acadoVariables.lbAValues[0] = -6.9999999999999996e-01;
acadoVariables.lbAValues[1] = -6.9999999999999996e-01;
acadoVariables.lbAValues[2] = -1.0000000000000000e+00;
acadoVariables.lbAValues[3] = -1.0000000000000000e+00;
acadoVariables.lbAValues[4] = -6.9999999999999996e-01;
acadoVariables.lbAValues[5] = -6.9999999999999996e-01;
acadoVariables.lbAValues[6] = -1.0000000000000000e+00;
acadoVariables.lbAValues[7] = -1.0000000000000000e+00;
acadoVariables.lbAValues[8] = -6.9999999999999996e-01;
acadoVariables.lbAValues[9] = -6.9999999999999996e-01;
acadoVariables.lbAValues[10] = -1.0000000000000000e+00;
acadoVariables.lbAValues[11] = -1.0000000000000000e+00;
acadoVariables.lbAValues[12] = -6.9999999999999996e-01;
acadoVariables.lbAValues[13] = -6.9999999999999996e-01;
acadoVariables.lbAValues[14] = -1.0000000000000000e+00;
acadoVariables.lbAValues[15] = -1.0000000000000000e+00;
acadoVariables.lbAValues[16] = -6.9999999999999996e-01;
acadoVariables.lbAValues[17] = -6.9999999999999996e-01;
acadoVariables.lbAValues[18] = -1.0000000000000000e+00;
acadoVariables.lbAValues[19] = -1.0000000000000000e+00;
acadoVariables.lbAValues[20] = -6.9999999999999996e-01;
acadoVariables.lbAValues[21] = -6.9999999999999996e-01;
acadoVariables.lbAValues[22] = -1.0000000000000000e+00;
acadoVariables.lbAValues[23] = -1.0000000000000000e+00;
acadoVariables.lbAValues[24] = -6.9999999999999996e-01;
acadoVariables.lbAValues[25] = -6.9999999999999996e-01;
acadoVariables.lbAValues[26] = -1.0000000000000000e+00;
acadoVariables.lbAValues[27] = -1.0000000000000000e+00;
acadoVariables.lbAValues[28] = -6.9999999999999996e-01;
acadoVariables.lbAValues[29] = -6.9999999999999996e-01;
acadoVariables.lbAValues[30] = -1.0000000000000000e+00;
acadoVariables.lbAValues[31] = -1.0000000000000000e+00;
acadoVariables.lbAValues[32] = -6.9999999999999996e-01;
acadoVariables.lbAValues[33] = -6.9999999999999996e-01;
acadoVariables.lbAValues[34] = -1.0000000000000000e+00;
acadoVariables.lbAValues[35] = -1.0000000000000000e+00;
acadoVariables.lbAValues[36] = -6.9999999999999996e-01;
acadoVariables.lbAValues[37] = -6.9999999999999996e-01;
acadoVariables.lbAValues[38] = -1.0000000000000000e+00;
acadoVariables.lbAValues[39] = -1.0000000000000000e+00;
acadoVariables.lbAValues[40] = -6.9999999999999996e-01;
acadoVariables.lbAValues[41] = -6.9999999999999996e-01;
acadoVariables.lbAValues[42] = -1.0000000000000000e+00;
acadoVariables.lbAValues[43] = -1.0000000000000000e+00;
acadoVariables.lbAValues[44] = -6.9999999999999996e-01;
acadoVariables.lbAValues[45] = -6.9999999999999996e-01;
acadoVariables.lbAValues[46] = -1.0000000000000000e+00;
acadoVariables.lbAValues[47] = -1.0000000000000000e+00;
acadoVariables.lbAValues[48] = -6.9999999999999996e-01;
acadoVariables.lbAValues[49] = -6.9999999999999996e-01;
acadoVariables.lbAValues[50] = -1.0000000000000000e+00;
acadoVariables.lbAValues[51] = -1.0000000000000000e+00;
acadoVariables.lbAValues[52] = -6.9999999999999996e-01;
acadoVariables.lbAValues[53] = -6.9999999999999996e-01;
acadoVariables.lbAValues[54] = -1.0000000000000000e+00;
acadoVariables.lbAValues[55] = -1.0000000000000000e+00;
acadoVariables.lbAValues[56] = -6.9999999999999996e-01;
acadoVariables.lbAValues[57] = -6.9999999999999996e-01;
acadoVariables.lbAValues[58] = -1.0000000000000000e+00;
acadoVariables.lbAValues[59] = -1.0000000000000000e+00;
acadoVariables.lbAValues[60] = -6.9999999999999996e-01;
acadoVariables.lbAValues[61] = -6.9999999999999996e-01;
acadoVariables.lbAValues[62] = -1.0000000000000000e+00;
acadoVariables.lbAValues[63] = -1.0000000000000000e+00;
acadoVariables.lbAValues[64] = -6.9999999999999996e-01;
acadoVariables.lbAValues[65] = -6.9999999999999996e-01;
acadoVariables.lbAValues[66] = -1.0000000000000000e+00;
acadoVariables.lbAValues[67] = -1.0000000000000000e+00;
acadoVariables.lbAValues[68] = -6.9999999999999996e-01;
acadoVariables.lbAValues[69] = -6.9999999999999996e-01;
acadoVariables.lbAValues[70] = -1.0000000000000000e+00;
acadoVariables.lbAValues[71] = -1.0000000000000000e+00;
acadoVariables.lbAValues[72] = -6.9999999999999996e-01;
acadoVariables.lbAValues[73] = -6.9999999999999996e-01;
acadoVariables.lbAValues[74] = -1.0000000000000000e+00;
acadoVariables.lbAValues[75] = -1.0000000000000000e+00;
acadoVariables.lbAValues[76] = -6.9999999999999996e-01;
acadoVariables.lbAValues[77] = -6.9999999999999996e-01;
acadoVariables.lbAValues[78] = -1.0000000000000000e+00;
acadoVariables.lbAValues[79] = -1.0000000000000000e+00;
acadoVariables.lbAValues[80] = -6.9999999999999996e-01;
acadoVariables.lbAValues[81] = -6.9999999999999996e-01;
acadoVariables.lbAValues[82] = -1.0000000000000000e+00;
acadoVariables.lbAValues[83] = -1.0000000000000000e+00;
acadoVariables.lbAValues[84] = -6.9999999999999996e-01;
acadoVariables.lbAValues[85] = -6.9999999999999996e-01;
acadoVariables.lbAValues[86] = -1.0000000000000000e+00;
acadoVariables.lbAValues[87] = -1.0000000000000000e+00;
acadoVariables.ubAValues[0] = 6.9999999999999996e-01;
acadoVariables.ubAValues[1] = 6.9999999999999996e-01;
acadoVariables.ubAValues[2] = 1.0000000000000000e+00;
acadoVariables.ubAValues[3] = 1.0000000000000000e+00;
acadoVariables.ubAValues[4] = 6.9999999999999996e-01;
acadoVariables.ubAValues[5] = 6.9999999999999996e-01;
acadoVariables.ubAValues[6] = 1.0000000000000000e+00;
acadoVariables.ubAValues[7] = 1.0000000000000000e+00;
acadoVariables.ubAValues[8] = 6.9999999999999996e-01;
acadoVariables.ubAValues[9] = 6.9999999999999996e-01;
acadoVariables.ubAValues[10] = 1.0000000000000000e+00;
acadoVariables.ubAValues[11] = 1.0000000000000000e+00;
acadoVariables.ubAValues[12] = 6.9999999999999996e-01;
acadoVariables.ubAValues[13] = 6.9999999999999996e-01;
acadoVariables.ubAValues[14] = 1.0000000000000000e+00;
acadoVariables.ubAValues[15] = 1.0000000000000000e+00;
acadoVariables.ubAValues[16] = 6.9999999999999996e-01;
acadoVariables.ubAValues[17] = 6.9999999999999996e-01;
acadoVariables.ubAValues[18] = 1.0000000000000000e+00;
acadoVariables.ubAValues[19] = 1.0000000000000000e+00;
acadoVariables.ubAValues[20] = 6.9999999999999996e-01;
acadoVariables.ubAValues[21] = 6.9999999999999996e-01;
acadoVariables.ubAValues[22] = 1.0000000000000000e+00;
acadoVariables.ubAValues[23] = 1.0000000000000000e+00;
acadoVariables.ubAValues[24] = 6.9999999999999996e-01;
acadoVariables.ubAValues[25] = 6.9999999999999996e-01;
acadoVariables.ubAValues[26] = 1.0000000000000000e+00;
acadoVariables.ubAValues[27] = 1.0000000000000000e+00;
acadoVariables.ubAValues[28] = 6.9999999999999996e-01;
acadoVariables.ubAValues[29] = 6.9999999999999996e-01;
acadoVariables.ubAValues[30] = 1.0000000000000000e+00;
acadoVariables.ubAValues[31] = 1.0000000000000000e+00;
acadoVariables.ubAValues[32] = 6.9999999999999996e-01;
acadoVariables.ubAValues[33] = 6.9999999999999996e-01;
acadoVariables.ubAValues[34] = 1.0000000000000000e+00;
acadoVariables.ubAValues[35] = 1.0000000000000000e+00;
acadoVariables.ubAValues[36] = 6.9999999999999996e-01;
acadoVariables.ubAValues[37] = 6.9999999999999996e-01;
acadoVariables.ubAValues[38] = 1.0000000000000000e+00;
acadoVariables.ubAValues[39] = 1.0000000000000000e+00;
acadoVariables.ubAValues[40] = 6.9999999999999996e-01;
acadoVariables.ubAValues[41] = 6.9999999999999996e-01;
acadoVariables.ubAValues[42] = 1.0000000000000000e+00;
acadoVariables.ubAValues[43] = 1.0000000000000000e+00;
acadoVariables.ubAValues[44] = 6.9999999999999996e-01;
acadoVariables.ubAValues[45] = 6.9999999999999996e-01;
acadoVariables.ubAValues[46] = 1.0000000000000000e+00;
acadoVariables.ubAValues[47] = 1.0000000000000000e+00;
acadoVariables.ubAValues[48] = 6.9999999999999996e-01;
acadoVariables.ubAValues[49] = 6.9999999999999996e-01;
acadoVariables.ubAValues[50] = 1.0000000000000000e+00;
acadoVariables.ubAValues[51] = 1.0000000000000000e+00;
acadoVariables.ubAValues[52] = 6.9999999999999996e-01;
acadoVariables.ubAValues[53] = 6.9999999999999996e-01;
acadoVariables.ubAValues[54] = 1.0000000000000000e+00;
acadoVariables.ubAValues[55] = 1.0000000000000000e+00;
acadoVariables.ubAValues[56] = 6.9999999999999996e-01;
acadoVariables.ubAValues[57] = 6.9999999999999996e-01;
acadoVariables.ubAValues[58] = 1.0000000000000000e+00;
acadoVariables.ubAValues[59] = 1.0000000000000000e+00;
acadoVariables.ubAValues[60] = 6.9999999999999996e-01;
acadoVariables.ubAValues[61] = 6.9999999999999996e-01;
acadoVariables.ubAValues[62] = 1.0000000000000000e+00;
acadoVariables.ubAValues[63] = 1.0000000000000000e+00;
acadoVariables.ubAValues[64] = 6.9999999999999996e-01;
acadoVariables.ubAValues[65] = 6.9999999999999996e-01;
acadoVariables.ubAValues[66] = 1.0000000000000000e+00;
acadoVariables.ubAValues[67] = 1.0000000000000000e+00;
acadoVariables.ubAValues[68] = 6.9999999999999996e-01;
acadoVariables.ubAValues[69] = 6.9999999999999996e-01;
acadoVariables.ubAValues[70] = 1.0000000000000000e+00;
acadoVariables.ubAValues[71] = 1.0000000000000000e+00;
acadoVariables.ubAValues[72] = 6.9999999999999996e-01;
acadoVariables.ubAValues[73] = 6.9999999999999996e-01;
acadoVariables.ubAValues[74] = 1.0000000000000000e+00;
acadoVariables.ubAValues[75] = 1.0000000000000000e+00;
acadoVariables.ubAValues[76] = 6.9999999999999996e-01;
acadoVariables.ubAValues[77] = 6.9999999999999996e-01;
acadoVariables.ubAValues[78] = 1.0000000000000000e+00;
acadoVariables.ubAValues[79] = 1.0000000000000000e+00;
acadoVariables.ubAValues[80] = 6.9999999999999996e-01;
acadoVariables.ubAValues[81] = 6.9999999999999996e-01;
acadoVariables.ubAValues[82] = 1.0000000000000000e+00;
acadoVariables.ubAValues[83] = 1.0000000000000000e+00;
acadoVariables.ubAValues[84] = 6.9999999999999996e-01;
acadoVariables.ubAValues[85] = 6.9999999999999996e-01;
acadoVariables.ubAValues[86] = 1.0000000000000000e+00;
acadoVariables.ubAValues[87] = 1.0000000000000000e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 22; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 5];
acadoWorkspace.state[1] = acadoVariables.x[index * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 5 + 4];
acadoWorkspace.state[40] = acadoVariables.u[index * 2];
acadoWorkspace.state[41] = acadoVariables.u[index * 2 + 1];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 5 + 5] = acadoWorkspace.state[0];
acadoVariables.x[index * 5 + 6] = acadoWorkspace.state[1];
acadoVariables.x[index * 5 + 7] = acadoWorkspace.state[2];
acadoVariables.x[index * 5 + 8] = acadoWorkspace.state[3];
acadoVariables.x[index * 5 + 9] = acadoWorkspace.state[4];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 22; ++index)
{
acadoVariables.x[index * 5] = acadoVariables.x[index * 5 + 5];
acadoVariables.x[index * 5 + 1] = acadoVariables.x[index * 5 + 6];
acadoVariables.x[index * 5 + 2] = acadoVariables.x[index * 5 + 7];
acadoVariables.x[index * 5 + 3] = acadoVariables.x[index * 5 + 8];
acadoVariables.x[index * 5 + 4] = acadoVariables.x[index * 5 + 9];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[110] = xEnd[0];
acadoVariables.x[111] = xEnd[1];
acadoVariables.x[112] = xEnd[2];
acadoVariables.x[113] = xEnd[3];
acadoVariables.x[114] = xEnd[4];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[110];
acadoWorkspace.state[1] = acadoVariables.x[111];
acadoWorkspace.state[2] = acadoVariables.x[112];
acadoWorkspace.state[3] = acadoVariables.x[113];
acadoWorkspace.state[4] = acadoVariables.x[114];
if (uEnd != 0)
{
acadoWorkspace.state[40] = uEnd[0];
acadoWorkspace.state[41] = uEnd[1];
}
else
{
acadoWorkspace.state[40] = acadoVariables.u[42];
acadoWorkspace.state[41] = acadoVariables.u[43];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[110] = acadoWorkspace.state[0];
acadoVariables.x[111] = acadoWorkspace.state[1];
acadoVariables.x[112] = acadoWorkspace.state[2];
acadoVariables.x[113] = acadoWorkspace.state[3];
acadoVariables.x[114] = acadoWorkspace.state[4];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 21; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[42] = uEnd[0];
acadoVariables.u[43] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43];
kkt = fabs( kkt );
for (index = 0; index < 44; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 88; ++index)
{
prd = acadoWorkspace.y[index + 44];
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
/** Row vector of size: 5 */
real_t tmpDy[ 5 ];

/** Row vector of size: 5 */
real_t tmpDyN[ 5 ];

for (lRun1 = 0; lRun1 < 22; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 5] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 5];
acadoWorkspace.Dy[lRun1 * 5 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 5 + 1];
acadoWorkspace.Dy[lRun1 * 5 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 5 + 2];
acadoWorkspace.Dy[lRun1 * 5 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 5 + 3];
acadoWorkspace.Dy[lRun1 * 5 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 5 + 4];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[110];
acadoWorkspace.objValueIn[1] = acadoVariables.x[111];
acadoWorkspace.objValueIn[2] = acadoVariables.x[112];
acadoWorkspace.objValueIn[3] = acadoVariables.x[113];
acadoWorkspace.objValueIn[4] = acadoVariables.x[114];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 22; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 5]*(real_t)5.0000000000000000e+03;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 5 + 1]*(real_t)5.0000000000000000e+03;
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 5 + 2]*(real_t)1.0000000000000000e+01;
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 5 + 3]*(real_t)1.0000000000000000e+02;
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 5 + 4];
objVal += + acadoWorkspace.Dy[lRun1 * 5]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 5 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 5 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 5 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 5 + 4]*tmpDy[4];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)5.0000000000000000e+04;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)5.0000000000000000e+04;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)1.0000000000000000e+02;
tmpDyN[3] = + acadoWorkspace.DyN[3]*(real_t)1.0000000000000000e+03;
tmpDyN[4] = + acadoWorkspace.DyN[4]*(real_t)1.0000000000000000e+01;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4];

objVal *= 0.5;
return objVal;
}

