STM32F103工程
PA0,1,2接RGB灯
PA9，10接UART串口

openocd使用：
* 安装:
brew install open-ocd
* 运行:
openocd -f ./script/stlink.cfg -f ./script/stm32f103c8_blue_pill.cfg
* 调试方式:
运行openocd，之后执行bin/arm中的gdb工具：
    * gdb ./intermediate/application.elf < 它要有符号表
    * target remote localhost:3333
