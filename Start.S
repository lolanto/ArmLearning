.code 16            @ 使用thumb指令集
.file "Start.S"
.extern __bss_start__
.extern __bss_end__
.extern main
.text               @ 表示text段开始

.align 2            @ 表示对齐是以2^2进行
.global __My_Start  @ 声明一个全局可见(别的链接对象也可见)的符号__My_Start
.code 16
.thumb_func         @ 声明这个函数将以thumb模式(指令集)进行
.type __My_Start, %function @ 用于告诉链接器等工具__My_Start符号是一个函数
__My_Start:
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__
    sub r1, r2, r1 @ 将bss段的大小记录到r1当中
    mov r2, #0 @ 初始化计数器
__My_Set_Zero_Loop_Begin:
    cmp r1, r2
    bgt __My_Set_Zero_Byte
    b __My_Set_Zero_Loop_End
__My_Set_Zero_Byte:
    ldr r3, =__bss_start__
    add r3, r3, r2 @ 当前要清空的字节地址
    mov r0, #0
    strb r0, [r3]
    add r2, r2, #1
    b __My_Set_Zero_Loop_Begin
__My_Set_Zero_Loop_End:
    bl main
    


