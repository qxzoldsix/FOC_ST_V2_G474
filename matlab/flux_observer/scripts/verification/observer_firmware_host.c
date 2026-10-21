/* Standalone regression adapter: original firmware C, no MATLAB MEX needed. */
#include <stdio.h>
#include <stdlib.h>
#include "reference/flux.h"
TestVoltCurr Volt_CurrPara;
TestClarke CLARKE_PCurr;
TestPark PARK_PCurr;
TestSvpwm Svpwm_dq;
TestMotor motor;
float Limit_Sat(float x, float maximum, float minimum)
{ return x > maximum ? maximum : (x < minimum ? minimum : x); }
#include "reference/flux.c"
int main(int argc,char **argv)
{
    FILE *in,*out; double u[10],y[11]; float va,vb;
    if(argc!=4) return 2;
    in=fopen(argv[1],"rb"); out=fopen(argv[2],"wb");
    if(!in || !out) return 3;
    Flux_Observer_Init(); Foc_observer.Comp_Mode=(uint8_t)atoi(argv[3]);
    while(fread(u,sizeof(double),10,in)==10) {
        Svpwm_dq.Ualpha=(float)u[0]; Svpwm_dq.Ubeta=(float)u[1];
        Volt_CurrPara.PhaseU_Curr=(float)u[2];
        Volt_CurrPara.PhaseV_Curr=(float)u[3];
        Volt_CurrPara.PhaseW_Curr=(float)u[4];
        CLARKE_PCurr.Alpha=(float)u[5]; CLARKE_PCurr.Beta=(float)u[6];
        PARK_PCurr.Ds=(float)u[7];
        Volt_CurrPara.BUS_Voltage=(float)u[8]; Volt_CurrPara.Svpwm_Km_BackwS=(float)u[9];
        Observer_Run_HybirdFlux(); Deadtime_Comp_ab(&va,&vb);
        y[0]=Foc_observer.Theta; y[1]=Foc_observer.speed_hz;
        y[2]=Foc_observer.speed_hz*60.0f/(MOTOR_POLES/2.0f);
        y[3]=FluxR_in_wb[0]; y[4]=FluxR_in_wb[1];
        y[5]=Flux_in_wb[0]; y[6]=Flux_in_wb[1];
        y[7]=Foc_observer.PLL_Err; y[8]=Foc_observer.PLL_Ui;
        y[9]=va; y[10]=vb;
        if(fwrite(y,sizeof(double),11,out)!=11) return 4;
    }
    fclose(in); fclose(out); return 0;
}
