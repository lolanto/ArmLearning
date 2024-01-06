STM32F103工程
PA0,1,2接RGB灯
PA9，10接UART串口

openocd使用：
* 安装:
brew install open-ocd
* 运行:
openocd -f ./script/stlink.cfg ./script/stm32f103c8_blue_pill.cfg