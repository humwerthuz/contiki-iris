 #include <i2cmaster.h>


 #define ADG715_A 0x48
 #define ADG715_B 0x00

 int main(void)
 {
     i2c_init();                            
     i2c_start_wait(ADG715_A+I2C_WRITE);    
     i2c_write(0xFF);                      
     i2c_stop();

     for(;;);
 }