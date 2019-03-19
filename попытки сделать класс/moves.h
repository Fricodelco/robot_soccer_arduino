#include <DynamixelWorkbench.h>
class Bioloid{
public:
  int32_t goal_position[20];
  uint16_t model_number;
  uint8_t dxl_id[20];
  const uint8_t handler_index = 0;
  Bioloid();
  DynamixelWorkbench dxl_wb;
  void initialization();
  void setMotors(int m1,int m2,int m3,int m4,int m5,int m6,int m7,int m8,int m9,int m10,int m11,int m12,int m13,int m14,int m15,int m16,int m17,int m18,int m19,int m20,int delayy);
  void init();
  void ready_for_walk();
  void F_R_S();
  void F_L_S();
  void FLT_L_M();
};
