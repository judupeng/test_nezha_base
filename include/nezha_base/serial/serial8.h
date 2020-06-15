#include "nezha_base/serial/wiringSerial.h"

#ifdef __cplusplus
extern "C" {
#endif
#define DEV_NAME "/dev/ttyS4"
#define DEV_NAME_LEFT "/dev/ttyUART0"
#define DEV_NAME_RIGHT "/dev/ttyUART1"
#define FILENAME "new.csv"
#define MAX_SIZE 1000
#define SPEED_WHEEL 3240    //   转/分钟
enum action{ServoControl,ActionControl};

enum STM32LIST{STM32_MASTER,STM32_LEFT,STM32_RIGHT,STM32_NONE};
void serial_puts(int fd,char *s,unsigned short length);
int cmd_action(unsigned char *buf,unsigned char action,unsigned short times);
int cmd_single_servo(unsigned char *buf,unsigned char Servo_ID,unsigned short location);
int cmd_servo(unsigned char *buf,unsigned char servonum,unsigned short location);
void cmd_execution_motor_left(unsigned char *buf,unsigned char direction_left,int speed_left,unsigned char direction_right,int speed_right);
void cmd_execution_motor_right(unsigned char *buf,unsigned char direction_left,int speed_left,unsigned char direction_right,int speed_right);
void cmd_execution_action(unsigned char *buf,unsigned char Action_ID,unsigned short times);
int get_str(int index, char *code, int size);
int getSpeedBuf(int leftSpeed,int rightSpeed,unsigned char *buf_left,unsigned char *buf_right);

int main_loop(unsigned char *__direction_left,int *__speed_left,unsigned char *__direction_right,int *__speed_right);
#ifdef __cplusplus
}
#endif
