/* Compile ORIGINAL source snapshot for independent numeric regression. */
#include "mex.h"
#include "reference/flux.h"
TestVoltCurr Volt_CurrPara;
TestClarke CLARKE_PCurr;
TestPark PARK_PCurr;
TestSvpwm Svpwm_dq;
TestMotor motor;
float Limit_Sat(float x, float maximum, float minimum)
{ return x > maximum ? maximum : (x < minimum ? minimum : x); }
#include "reference/flux.c"
void mexFunction(int nlhs, mxArray **plhs, int nrhs, const mxArray **prhs)
{
    mwSize n,k; const double *u; double *y; float va,vb;
    if (nrhs<1 || nlhs!=1 || !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ||
        mxGetN(prhs[0])!=10)
        mexErrMsgIdAndTxt("G474:input","Input must be a real N-by-10 double matrix; one output required.");
    n=mxGetM(prhs[0]); u=mxGetPr(prhs[0]);
    plhs[0]=mxCreateDoubleMatrix(n,11,mxREAL); y=mxGetPr(plhs[0]);
    Flux_Observer_Init();
    if (nrhs>1) Foc_observer.Comp_Mode=(uint8_t)mxGetScalar(prhs[1]);
    for(k=0;k<n;++k) {
        Svpwm_dq.Ualpha=(float)u[k]; Svpwm_dq.Ubeta=(float)u[k+n];
        Volt_CurrPara.PhaseU_Curr=(float)u[k+2*n];
        Volt_CurrPara.PhaseV_Curr=(float)u[k+3*n];
        Volt_CurrPara.PhaseW_Curr=(float)u[k+4*n];
        CLARKE_PCurr.Alpha=(float)u[k+5*n]; CLARKE_PCurr.Beta=(float)u[k+6*n];
        PARK_PCurr.Ds=(float)u[k+7*n];
        Volt_CurrPara.BUS_Voltage=(float)u[k+8*n];
        Volt_CurrPara.Svpwm_Km_BackwS=(float)u[k+9*n];
        Observer_Run_HybirdFlux(); Deadtime_Comp_ab(&va,&vb);
        y[k]=Foc_observer.Theta; y[k+n]=Foc_observer.speed_hz;
        y[k+2*n]=Foc_observer.speed_hz*60.0f/(MOTOR_POLES/2.0f);
        y[k+3*n]=FluxR_in_wb[0]; y[k+4*n]=FluxR_in_wb[1];
        y[k+5*n]=Flux_in_wb[0]; y[k+6*n]=Flux_in_wb[1];
        y[k+7*n]=Foc_observer.PLL_Err; y[k+8*n]=Foc_observer.PLL_Ui;
        y[k+9*n]=va; y[k+10*n]=vb;
    }
}
