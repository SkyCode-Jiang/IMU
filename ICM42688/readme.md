说明：

function.h：功能使能

不同MCU更换：writeReg/readReg





AD转换后的数据：ACCInt

经过滤波后的数据：Gyr_filt //启用姿态解算



NOTE：

1.  IIC通信轮询读取正常，中断读取速度太慢，进入IIC总线忙错误
2. SPI中断读取时，DRDY中断频率过快，影响APEX中断功能的使用

2.  使用APEX功能，注意ACC ODR：敲击检测：  ACC 200 500 1000Hz----   倾斜检测 50Hz 不能同时使用
3. 输出数据多，串口数据尽可能快

*******************************************

[全面了解ICM20948传感器：手册、代码与示例-CSDN博客](https://blog.csdn.net/weixin_42598278/article/details/147785394)

[从零开始精通ICM-20948：陀螺仪与加速度计编程秘籍1746781459.html#3.1.1 采集过程详解](file:///F:/Desktop/icm20948/数据手册/从零开始精通ICM-20948：陀螺仪与加速度计编程秘籍1746781459.html#3.1.1 采集过程详解)

[4.1 这个完，全得完！——陀螺仪、加速度计_icm42688-CSDN博客](https://blog.csdn.net/qq_53043199/article/details/135183850)

[六轴传感器（ICM42688）驱动及姿态估计 | Singularity-blog](https://www.singularity-blog.top/2022/07/28/ICM42688P-DrvBySPI/)

[IMU传感器应用秘籍：ICM-42688-P全面深度解析与优化指南 - CSDN文库](https://wenku.csdn.net/column/8998m7ivpw#6.2.2 基于ICM-42688-P的新产品和解决方案开发)