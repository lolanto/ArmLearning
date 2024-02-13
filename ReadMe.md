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
    * target extended-remote localhost:3333 < 使用extended可以有更多指令可执行(e.g. 重启程序)
* gdb调试指令
当成功启动了gdb之后：
1. layout
用于在断点被命中之后，进入可视化的调试模式(TUI)。
    - layout src：显示源码
    - layout asm: 显示汇编
2. start
用于重启程序，并直接在main函数入口位置进行断点
3. run
用于重启程序，但假如你没有设置断点，它会一直运行(即便是出现了崩溃).Ctrl + C强制中断程序
4. 控制执行
    - step 或 s: 逐语句执行，会进入函数内部
    - next 或 n: 逐语句执行，不会进入函数内部
    - finish: 完成当前函数执行并回到调用位置
5. print
用来计算表达式的值(e.g. 观察局部变量具体值)
6. break
用来设置断点
7. info breakpoints
用来显示当前已经设置的断点
8. delete [break point index]
用来删除断点，index可以用info来进行查询
9. i r [register name]
info register，用来在进行汇编调试的时候，显示指定寄存器中的值
