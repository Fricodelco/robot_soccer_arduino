#include  <DynamixelWorkbench.h>

namespace Moves{
void initialization();
void setMotors(int m1,int m2,int m3,int m4,int m5,int m6,int m7,int m8,int m9,int m10,int m11,int m12,int m13,int m14,int m15,int m16,int m17,int m18,int m19,int m20,int delayy);
void init(int w1, int w2);
void ready_for_walk(int w1, int w2);
void Init_Speed(int speed1);
void wait(int t);
void F_L_S(int w1, int w2);
void FFT_M(int w1, int w2);
void FRT_M(int w1, int w2);
void FLT_M(int w1, int w2);
void TURN_R(int w1, int w2);
void TURN_L(int w1, int w2);
void M_R(int w1, int w2);
void M_L(int w1, int w2);
void F_Shoot_R(int w1, int w2);
void F_Shoot_L(int w1, int w2);
void GET_UP_BACK(int w1, int w2);
void GET_UP_FRONT(int w1, int w2);
void setMotorsEnc(int m1,int m2,int m3,int m4,int m5,int m6,int m7,int m8,int m9,int m10,int m11,int m12,int m13,int m14,int m15,int m16,int m17,int m18,int m19,int m20,int delayy);

}
