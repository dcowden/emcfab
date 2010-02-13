//an attempt at melt flow compensated
//velocity control

extern struct MeltFlowStruct {

    //position and feedback are absolute
    signed long position;	/* commanded value  4 bytes*/
    signed long feedback;	/* feedback value   4 bytes*/
    signed long prev_position;
    signed long prev_vel;
    float Qmf;
    float tmf;		/* param: proportional gain */
    float tex;		/* param: integral gain */
    float pct_mf;		/* param: derivative gain */
    float mfgain;		/* param: feedforward proportional */
    float qgain;		/* param: feedforward derivative */
    int maxoutput;	/* param: limit for PID output */
    int output;		/* the output value -- outputs are duty cycles, no need for float*/
    unsigned short enable;		/* enable input */
    unsigned short limit_state;	/* 1 if in limit, else 0 */

};

void calc_vel(struct MeltFlowStruct *ps);