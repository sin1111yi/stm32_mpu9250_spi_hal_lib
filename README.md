# **stm32_mpu9250_spi_hal_lib**
***v1.1.1***   
2021/12/17   
fix some possible problem about data type

***v1.1.0***   
2021/12/13   
Add function:     
void MPU9250_ReadData(MPU9250* mpu)   
***v1.0.0***   
2021/12/13   
Now it can be used on stm32 through hal lib, please check more details in example/c8t6_mpu9250.   
It is my old GY-91's problem that makes me can not finish this lib.    
Before using GY-91 module, please check weather your module MPU9250's WHO_AM_I register has the value 0x71 or 0x73 through my lib.   
If not, It suppose to be MPU6500, instead of MPU9250, which doesn't have AK8963 inside. 

***v0.9.0***   
2021/12/10  
wait to verify lib. It is able to read Accel and Gyro data now.

---

## **details**
**Callable function**
|  type   |               name               |
| :-----: | :------------------------------: |
| uint8_t | mpu_r_ak963_WhoAmI(MPU9250 *mpu) |
| uint8_t |    mpu_r_WhoAmI(MPU9250 *mpu)    |
| uint8_t |    MPU9250_Init(MPU9250 *mpu)    |
|  void   | MPU9250_ReadAccel(MPU9250 *mpu)  |
|  void   |  MPU9250_ReadGyro(MPU9250 *mpu)  |
|  void   |  MPU9250_ReadMag(MPU9250 *mpu)   |
|  void   |  MPU9250_ReadData(MPU9250 *mpu)  |

 *please check file lib/mpu9250.c for more details*

 ---
 ## **Author**
 ***contact me by email sin1111yi@foxmail.com***

