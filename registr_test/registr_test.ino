#include  <DynamixelWorkbench.h>
#define DEVICE_NAME "3" 

#define BAUDRATE  1000000

uint16_t model_number;
uint8_t id = 19;
bool flag = true;
int32_t goal_position[1] = {0};
const uint8_t handler_index = 0;
DynamixelWorkbench dxl_wb;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  model_number = 0;
  dxl_wb.init(DEVICE_NAME, BAUDRATE);
  dxl_wb.ping(id, &model_number); //arms
  dxl_wb.jointMode(id, 600, 1023); 
  dxl_wb.addSyncWriteHandler(id, "Goal_Position");
}

void loop() {
  const char *log = NULL;
  // put your main code here, to run repeatedly:
const ControlItem *control_item =  dxl_wb.getControlTable(id);
uint8_t the_number_of_control_item = dxl_wb.getTheNumberOfControlItem(id);
 Serial.println("Ebanoe govno");
uint16_t last_register_addr = control_item[the_number_of_control_item-1].address;
uint16_t last_register_addr_length = control_item[the_number_of_control_item-1].data_length;
uint32_t getAllRegisteredData[last_register_addr+last_register_addr_length];
bool result;
  result = dxl_wb.readRegister(id, (uint16_t)0, last_register_addr+last_register_addr_length, getAllRegisteredData,&log);
  if(result == true){
  for (int index = 0; index < the_number_of_control_item; index++){
      Serial.print(index);
      Serial.print("-");
      Serial.println(getAllRegisteredData[index]);
      delay(10);}
  }
  else
      Serial.println("Ebanoe govno");
   flag = !flag;
   if(flag){
      goal_position[0]=0;
     dxl_wb.syncWrite(handler_index, &goal_position[0]);
     delay(1000);
   }
   else{
    goal_position[0]=1000;
     dxl_wb.syncWrite(handler_index, &goal_position[0]);
     delay(5000);
    
   }
}
