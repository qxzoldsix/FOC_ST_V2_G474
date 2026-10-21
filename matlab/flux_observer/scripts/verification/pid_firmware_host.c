/* Compile the original PI_Cale.c, not a rewritten approximation. */
#include <stdio.h>
#include <stdlib.h>
#include "reference/headline.h"
float Limit_Sat(float x,float maximum,float minimum)
{ return x>maximum ? maximum : (x<minimum ? minimum : x); }
#include "reference/PI_Cale.c"
int main(int argc,char **argv)
{
    FILE *in,*out; double u[4],y[4]; PI_controller state={0};
    if(argc!=5) return 2;
    in=fopen(argv[1],"rb"); out=fopen(argv[2],"wb");
    if(!in || !out) return 3;
    state.Kp=(float)atof(argv[3]); state.Ki=(float)atof(argv[4]);
    PI_ResetRuntime(&state);
    while(fread(u,sizeof(double),4,in)==4) {
        state.Ref=(float)u[0]; state.Fbk=(float)u[1];
        state.Umax=(float)u[2]; state.Umin=(float)u[3];
        PID_controller(&state);
        y[0]=state.Out; y[1]=state.ui; y[2]=state.v1; y[3]=state.up;
        if(fwrite(y,sizeof(double),4,out)!=4) return 4;
    }
    fclose(in); fclose(out); return 0;
}
