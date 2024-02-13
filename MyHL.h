#ifndef MyHL_H
#define MyHL_H

#include <cstdint>

constexpr uint32_t BIT_RANGE_MASK(uint32_t beg, uint32_t end)
{
    uint32_t a1 = 0;
    if (end == 32)
        a1 = 0xFFFFFFFFu;
    else
        a1 = (1u << end) - 1;
    uint32_t a2 = (1u << beg) - 1;
    return a1 & (~a2);
}

template<uint32_t _beg, uint32_t _len>
struct MaskRanger
{
    static_assert(_beg + _len <= 32 && _len > 0 && _beg >= 0 && _beg < 31);
    constexpr static uint32_t RANGE_MASK = BIT_RANGE_MASK(_beg, _beg + _len);
    constexpr static uint32_t VALID_MASK = (1u << (_len)) - 1;
    constexpr static uint32_t BEG = _beg;
};

template<typename MASK_RANGER, typename HOST_TYPE>
struct BitFieldObject
{
    static_assert(sizeof(HOST_TYPE) == sizeof(uint32_t));
    static uint32_t Get(const HOST_TYPE& input)
    {
        const uint32_t& _input = reinterpret_cast<const uint32_t&>(input);
        return (_input & MASK_RANGER::RANGE_MASK) >> MASK_RANGER::BEG;
    }
    
    static uint32_t Set(HOST_TYPE& input, uint32_t value)
    {
        volatile uint32_t& _input = reinterpret_cast<uint32_t&>(input);
        _input &= ~MASK_RANGER::RANGE_MASK;
        _input |= (value & MASK_RANGER::VALID_MASK) << MASK_RANGER::BEG;
        return _input;
    }
};

class GPIO
{
public:
    enum PORT_GROUP : uint8_t
    {
        PG_A = 0,
        PG_B = 1,
        PG_C = 2,
        PG_D = 3,
        PG_E = 4,
        PG_COUNT
    };
    enum PORT_MODE : uint8_t
    {
        PM_INPUT = 0,
        PM_OUTPUT_10MHz = 1,
        PM_OUTPUT_2MHz = 2,
        PM_OUTPUT_50MHz = 3
    };
    enum PORT_USAGE : uint8_t
    {
        PU_INPUT_ANALOG = 0b000,
        PU_INPUT_FLOATING = 0b001,
        PU_INPUT_PULL_UP_DOWN = 0b011,
        PU_OUTPUT_PUSH_PULL = 0b100, // 默认推挽输出
        PU_OUTPUT_OPEN_DRAIN = 0b101, // 默认开漏输出
        PU_OUTPUT_ALTERNATE_PUSH_PULL = 0b110, // 复用推挽输出
        PU_OUTPUT_ALTERNATE_OPEN_DRAIN = 0b111 // 复用开漏输出
    };
    static constexpr uint32_t PORT_GROUP_BASE_ADDRs[] = 
    {
        0x40010800u, // A
        0X40010C00u, // B
        0x40011000u, // C
        0x40011400u, // D
        0x40011800u  // E
    };
    // Output data register offset (GPIO group base)
    static constexpr uint32_t PORT_ODR_ADDR_OFFSET = 0xCu;
    // Output data recover register address offset 用来重置端口输出
    static constexpr uint32_t PORT_BRR_ADDR_OFFSET = 0x14u;
    // GPIO端口配置寄存器(高位)，8-15 Control Register High
    static constexpr uint32_t PORT_CRH_ADDR_OFFSET = 0x4u;
    // GPIO端口配置寄存器(低位), 0-7 Control Register Low
    static constexpr uint32_t PORT_CRL_ADDR_OFFSET = 0x0u;
public:
    void SetEnable(bool enable);
    void SetPortMode(PORT_MODE mode, PORT_USAGE usage);
    void SetPortOutputValue(bool value);
    bool GetPortValue();
    static GPIO& GetInstance(PORT_GROUP group, uint8_t pin);
private:
    struct ControlRegisterHighSettings
    {
        uint32_t data;
        using PIN_8_MODE = BitFieldObject<MaskRanger<0, 2>, ControlRegisterHighSettings>;
        using PIN_8_CNF = BitFieldObject<MaskRanger<2, 2>, ControlRegisterHighSettings>;

        using PIN_9_MODE = BitFieldObject<MaskRanger<4, 2>, ControlRegisterHighSettings>;
        using PIN_9_CNF = BitFieldObject<MaskRanger<6, 2>, ControlRegisterHighSettings>;

        using PIN_10_MODE = BitFieldObject<MaskRanger<8, 2>, ControlRegisterHighSettings>;
        using PIN_10_CNF = BitFieldObject<MaskRanger<10, 2>, ControlRegisterHighSettings>;

        using PIN_11_MODE = BitFieldObject<MaskRanger<12, 2>, ControlRegisterHighSettings>;
        using PIN_11_CNF = BitFieldObject<MaskRanger<14, 2>, ControlRegisterHighSettings>;

        using PIN_12_MODE = BitFieldObject<MaskRanger<16, 2>, ControlRegisterHighSettings>;
        using PIN_12_CNF = BitFieldObject<MaskRanger<18, 2>, ControlRegisterHighSettings>;

        using PIN_13_MODE = BitFieldObject<MaskRanger<20, 2>, ControlRegisterHighSettings>;
        using PIN_13_CNF = BitFieldObject<MaskRanger<22, 2>, ControlRegisterHighSettings>;

        using PIN_14_MODE = BitFieldObject<MaskRanger<24, 2>, ControlRegisterHighSettings>;
        using PIN_14_CNF = BitFieldObject<MaskRanger<26, 2>, ControlRegisterHighSettings>;

        using PIN_15_MODE = BitFieldObject<MaskRanger<28, 2>, ControlRegisterHighSettings>;
        using PIN_15_CNF = BitFieldObject<MaskRanger<30, 2>, ControlRegisterHighSettings>;
    };
    constexpr static uint32_t(*HPinModeSettingFuncs[8])(ControlRegisterHighSettings&, uint32_t) =
    {
        &ControlRegisterHighSettings::PIN_8_MODE::Set, &ControlRegisterHighSettings::PIN_9_MODE::Set,
        &ControlRegisterHighSettings::PIN_10_MODE::Set, &ControlRegisterHighSettings::PIN_11_MODE::Set,
        &ControlRegisterHighSettings::PIN_12_MODE::Set, &ControlRegisterHighSettings::PIN_13_MODE::Set,
        &ControlRegisterHighSettings::PIN_14_MODE::Set, &ControlRegisterHighSettings::PIN_15_MODE::Set,
    };
    constexpr static uint32_t(*HPinConfigurationSettingFuncs[8])(ControlRegisterHighSettings&, uint32_t) =
    {
        &ControlRegisterHighSettings::PIN_8_CNF::Set, &ControlRegisterHighSettings::PIN_9_CNF::Set,
        &ControlRegisterHighSettings::PIN_10_CNF::Set, &ControlRegisterHighSettings::PIN_11_CNF::Set,
        &ControlRegisterHighSettings::PIN_12_CNF::Set, &ControlRegisterHighSettings::PIN_13_CNF::Set,
        &ControlRegisterHighSettings::PIN_14_CNF::Set, &ControlRegisterHighSettings::PIN_15_CNF::Set,
    };
    static_assert(sizeof(ControlRegisterHighSettings) == sizeof(uint32_t));

    struct ControlRegisterLowSettings
    {
        uint32_t data;
        using PIN_0_MODE = BitFieldObject<MaskRanger<0, 2>, ControlRegisterLowSettings>;
        using PIN_0_CNF = BitFieldObject<MaskRanger<2, 2>, ControlRegisterLowSettings>;

        using PIN_1_MODE = BitFieldObject<MaskRanger<4, 2>, ControlRegisterLowSettings>;
        using PIN_1_CNF = BitFieldObject<MaskRanger<6, 2>, ControlRegisterLowSettings>;

        using PIN_2_MODE = BitFieldObject<MaskRanger<8, 2>, ControlRegisterLowSettings>;
        using PIN_2_CNF = BitFieldObject<MaskRanger<10, 2>, ControlRegisterLowSettings>;

        using PIN_3_MODE = BitFieldObject<MaskRanger<12, 2>, ControlRegisterLowSettings>;
        using PIN_3_CNF = BitFieldObject<MaskRanger<14, 2>, ControlRegisterLowSettings>;

        using PIN_4_MODE = BitFieldObject<MaskRanger<16, 2>, ControlRegisterLowSettings>;
        using PIN_4_CNF = BitFieldObject<MaskRanger<18, 2>, ControlRegisterLowSettings>;

        using PIN_5_MODE = BitFieldObject<MaskRanger<20, 2>, ControlRegisterLowSettings>;
        using PIN_5_CNF = BitFieldObject<MaskRanger<22, 2>, ControlRegisterLowSettings>;

        using PIN_6_MODE = BitFieldObject<MaskRanger<24, 2>, ControlRegisterLowSettings>;
        using PIN_6_CNF = BitFieldObject<MaskRanger<26, 2>, ControlRegisterLowSettings>;

        using PIN_7_MODE = BitFieldObject<MaskRanger<28, 2>, ControlRegisterLowSettings>;
        using PIN_7_CNF = BitFieldObject<MaskRanger<30, 2>, ControlRegisterLowSettings>;
    };
    constexpr static uint32_t(*LPinModeSettingFuncs[8])(ControlRegisterLowSettings&, uint32_t) =
    {
        &ControlRegisterLowSettings::PIN_0_MODE::Set, &ControlRegisterLowSettings::PIN_1_MODE::Set,
        &ControlRegisterLowSettings::PIN_2_MODE::Set, &ControlRegisterLowSettings::PIN_3_MODE::Set,
        &ControlRegisterLowSettings::PIN_4_MODE::Set, &ControlRegisterLowSettings::PIN_5_MODE::Set,
        &ControlRegisterLowSettings::PIN_6_MODE::Set, &ControlRegisterLowSettings::PIN_7_MODE::Set
    };
    constexpr static uint32_t(*LPinConfigurationSettingFuncs[8])(ControlRegisterLowSettings&, uint32_t) =
    {
        &ControlRegisterLowSettings::PIN_0_CNF::Set, &ControlRegisterLowSettings::PIN_1_CNF::Set,
        &ControlRegisterLowSettings::PIN_2_CNF::Set, &ControlRegisterLowSettings::PIN_3_CNF::Set,
        &ControlRegisterLowSettings::PIN_4_CNF::Set, &ControlRegisterLowSettings::PIN_5_CNF::Set,
        &ControlRegisterLowSettings::PIN_6_CNF::Set, &ControlRegisterLowSettings::PIN_7_CNF::Set,
    };
    static_assert(sizeof(ControlRegisterLowSettings) == sizeof(uint32_t));

    struct InputDataRegisterSettings
    {
        uint32_t data;
        using IDR = BitFieldObject<MaskRanger<0, 16>, InputDataRegisterSettings>;
    };
    static_assert(sizeof(InputDataRegisterSettings) == sizeof(uint32_t));

    struct OutputDataRegisterSettings
    {
        uint32_t data;
        using ODR0 = BitFieldObject<MaskRanger<0, 1>, OutputDataRegisterSettings>;
        using ODR1 = BitFieldObject<MaskRanger<1, 1>, OutputDataRegisterSettings>;
        using ODR2 = BitFieldObject<MaskRanger<2, 1>, OutputDataRegisterSettings>;
        using ODR3 = BitFieldObject<MaskRanger<3, 1>, OutputDataRegisterSettings>;
        using ODR4 = BitFieldObject<MaskRanger<4, 1>, OutputDataRegisterSettings>;
        using ODR5 = BitFieldObject<MaskRanger<5, 1>, OutputDataRegisterSettings>;
        using ODR6 = BitFieldObject<MaskRanger<6, 1>, OutputDataRegisterSettings>;
        using ODR7 = BitFieldObject<MaskRanger<7, 1>, OutputDataRegisterSettings>;
        using ODR8 = BitFieldObject<MaskRanger<8, 1>, OutputDataRegisterSettings>;
        using ODR9 = BitFieldObject<MaskRanger<9, 1>, OutputDataRegisterSettings>;
        using ODR10 = BitFieldObject<MaskRanger<10, 1>, OutputDataRegisterSettings>;
        using ODR11 = BitFieldObject<MaskRanger<11, 1>, OutputDataRegisterSettings>;
        using ODR12 = BitFieldObject<MaskRanger<12, 1>, OutputDataRegisterSettings>;
        using ODR13 = BitFieldObject<MaskRanger<13, 1>, OutputDataRegisterSettings>;
        using ODR14 = BitFieldObject<MaskRanger<14, 1>, OutputDataRegisterSettings>;
        using ODR15 = BitFieldObject<MaskRanger<15, 1>, OutputDataRegisterSettings>;
    };
    constexpr static uint32_t(*PinOutputDataRegisterSettingsFuncs[16])(OutputDataRegisterSettings&, uint32_t) =
    {
        &OutputDataRegisterSettings::ODR0::Set,&OutputDataRegisterSettings::ODR1::Set,
        &OutputDataRegisterSettings::ODR2::Set,&OutputDataRegisterSettings::ODR3::Set,
        &OutputDataRegisterSettings::ODR4::Set,&OutputDataRegisterSettings::ODR5::Set,
        &OutputDataRegisterSettings::ODR6::Set,&OutputDataRegisterSettings::ODR7::Set,
        &OutputDataRegisterSettings::ODR8::Set, &OutputDataRegisterSettings::ODR9::Set,
        &OutputDataRegisterSettings::ODR10::Set,&OutputDataRegisterSettings::ODR11::Set,
        &OutputDataRegisterSettings::ODR12::Set,&OutputDataRegisterSettings::ODR13::Set,
        &OutputDataRegisterSettings::ODR14::Set,&OutputDataRegisterSettings::ODR15::Set, 
    };
    static_assert(sizeof(OutputDataRegisterSettings) == sizeof(uint32_t));
private:
    GPIO()
     : mGroup(PORT_GROUP::PG_COUNT) {};
    GPIO(PORT_GROUP group, uint8_t pin)
     : mGroup(group), mPin(pin)
     , mValue(false), mMode(PORT_MODE::PM_INPUT)
     , mUsage(PORT_USAGE::PU_INPUT_FLOATING) {};
    uint32_t getControlRegisterAddr();
    uint32_t getGroupRegisterAddr();
    OutputDataRegisterSettings& getOutputDataRegister();
private:
    bool mValue; // 当前接口上的值，可能是输入的值，也可能是当前正在输出的值。假如当前被复用，则改值未定义
    uint8_t mPin;
    PORT_GROUP mGroup;
    PORT_MODE mMode;
    PORT_USAGE mUsage;
    
};

class Flash
{
public:
    static constexpr uint32_t FLASH_ACR_ADDR = 0x40022000u;
    Flash() = delete;
};

class USART
{
public:
    /**
     * STM32F103支持3组USART串口，引脚设置分别是：
     * USART1: TX:PA9, RX:PA10
     * USART2: TX:PA2, RX:PA3
     * USART3: TX:PB10,RX:PB11
    */
    enum PORT : uint8_t
    {
        U1 = 0,
        U2 = 1,
        U3 = 2,
        PORT_COUNT
    };
    static USART& GetInstance(PORT port);
    static constexpr PORT DEFAULT_USED_PORT = PORT::U3;
    static constexpr uint32_t USARTx_BASE_ADDRs[] = 
    {
        0x40013800u, // USART1
        0x40004400u, // USART2
        0x40004800u, // USART3
    };
    // State register
    static constexpr uint32_t USART_SR_ADDR_OFFSET = 0x0u;
    static uint32_t GetStateRegisterAddr(PORT port = DEFAULT_USED_PORT);
    // Data register
    static constexpr uint32_t USART_DR_ADDR_OFFSET = 0x4u;
    static uint32_t GetDataRegisterAddr(PORT port = DEFAULT_USED_PORT);
    // Baud rate register
    static constexpr uint32_t USART_BRR_ADDR_OFFSET = 0x8u;
    // Control register 1,2,3
    enum ControlRegister : uint8_t
    {
        CR_1 = 0,
        CR_2 = 1,
        CR_3 = 2,
        CR_COUNT
    };
    static constexpr uint32_t USART_CR_BASE_ADDR_OFFSETs[] = { 0x0Cu, 0x10u, 0x14u };
    
    // Guard time and prescaler register
    static constexpr uint32_t USART_GTPR_ADDR_OFFSET = 0x18u;
    static uint32_t GetGuardTimeAndPrescalerRegisterAddr(PORT port = DEFAULT_USED_PORT);
private:
    // State Register Settings
    struct SRSettings
    {
        uint32_t data;
        using PE = BitFieldObject<MaskRanger<0, 1>, SRSettings>; // 是否发生了校验错误
        using FE = BitFieldObject<MaskRanger<1, 1>, SRSettings>; // 是否发生了帧错误
        using NE = BitFieldObject<MaskRanger<2, 1>, SRSettings>; // 是否发生了噪声错误 (需要软件对其重置)
        using ORE = BitFieldObject<MaskRanger<3, 1>, SRSettings>; // 是否发生了过载错误(应该是接收的内容超过了接收寄存器的上限)
        using IDLE = BitFieldObject<MaskRanger<4, 1>, SRSettings>;
        using RXNE = BitFieldObject<MaskRanger<5, 1>, SRSettings>; // 标记当前接收缓冲中的数据是否能够被读取
        using TC = BitFieldObject<MaskRanger<6, 1>, SRSettings>; // 标记当前发送行为是否已经完成
        using TXE = BitFieldObject<MaskRanger<7, 1>, SRSettings>; // 标记当前发送寄存器是否已经全部转移到硬件发送区当中(发送寄存器是否已经为空)
        using LBD = BitFieldObject<MaskRanger<8, 1>, SRSettings>;
        using CTS = BitFieldObject<MaskRanger<9, 1>, SRSettings>;
    };
    static_assert(sizeof(SRSettings) == sizeof(uint32_t));
    struct CR1BitSettings
    {
        static constexpr ControlRegister CR_INDEX = CR_1;
        uint32_t data;
        using SBK = BitFieldObject<MaskRanger<0, 1>, CR1BitSettings>;
        using RWU = BitFieldObject<MaskRanger<1, 1>, CR1BitSettings>;
        using RE = BitFieldObject<MaskRanger<2, 1>, CR1BitSettings>; // 使USART能够用于接收
        using TE = BitFieldObject<MaskRanger<3, 1>, CR1BitSettings>; // 使USART能够用于发送
        using IDLEIE = BitFieldObject<MaskRanger<4, 1>, CR1BitSettings>;
        using RXNEIE = BitFieldObject<MaskRanger<5, 1>, CR1BitSettings>; // 是否在接收缓冲区非空时进行中断
        using TCIE = BitFieldObject<MaskRanger<6, 1>, CR1BitSettings>; // 是否在发送完成时进行中断
        using TXEIE = BitFieldObject<MaskRanger<7, 1>, CR1BitSettings>; // 是否在发送缓冲区为空时进行中断
        using PEIE = BitFieldObject<MaskRanger<8, 1>, CR1BitSettings>;
        using PS = BitFieldObject<MaskRanger<9, 1>, CR1BitSettings>; // 0: 偶校验, 1: 奇校验
        using PCE = BitFieldObject<MaskRanger<10, 1>, CR1BitSettings>; // 用来控制是否对接收和发送数据的数据校验
        using Wake = BitFieldObject<MaskRanger<11, 1>, CR1BitSettings>; // how to wake it up
        using M = BitFieldObject<MaskRanger<12, 1>, CR1BitSettings>; // word length. 0: 8bit; 1: 9bit
        using UE = BitFieldObject<MaskRanger<13, 1>, CR1BitSettings>; // Usart Enable
    };
    static_assert(sizeof(CR1BitSettings) == sizeof(uint32_t));
    struct CR2BitSettings
    {
        static constexpr ControlRegister CR_INDEX = CR_2;
        uint32_t data;
        using ADDR = BitFieldObject<MaskRanger<0, 4>, CR2BitSettings>;
        using LBDL = BitFieldObject<MaskRanger<5, 1>, CR2BitSettings>;
        using LBDIE = BitFieldObject<MaskRanger<6, 1>, CR2BitSettings>;
        using LBCL = BitFieldObject<MaskRanger<8, 1>, CR2BitSettings>;
        using CPHA = BitFieldObject<MaskRanger<9, 1>, CR2BitSettings>;
        using CPOL = BitFieldObject<MaskRanger<10, 1>, CR2BitSettings>;
        using CLKEN = BitFieldObject<MaskRanger<11, 1>, CR2BitSettings>; // 是否输出时钟信号
        using STOP = BitFieldObject<MaskRanger<12, 2>, CR2BitSettings>; // 用来设置停止位的数量 0: 1个停止位; 1: 0.5个停止位; 2: 2个停止位; 3: 1.5个停止位
        using LINEN = BitFieldObject<MaskRanger<14, 1>, CR2BitSettings>;
    };
    static_assert(sizeof(CR2BitSettings) == sizeof(uint32_t));
    struct CR3BitSettings
    {
        static constexpr ControlRegister CR_INDEX = CR_3;
        uint32_t data;
        using EIE = BitFieldObject<MaskRanger<0, 1>, CR3BitSettings>; // 是否在发生错误时产生中断
        using IREN = BitFieldObject<MaskRanger<1, 1>, CR3BitSettings>;
        using IRLP = BitFieldObject<MaskRanger<2, 1>, CR3BitSettings>;
        using HDSEL = BitFieldObject<MaskRanger<3, 1>, CR3BitSettings>; // 是否使用半双工模式
        using NACK = BitFieldObject<MaskRanger<4, 1>, CR3BitSettings>;
        using SCEN = BitFieldObject<MaskRanger<5, 1>, CR3BitSettings>;
        using DMAR = BitFieldObject<MaskRanger<6, 1>, CR3BitSettings>; // 是否使用DMA辅助接收
        using DMAT = BitFieldObject<MaskRanger<7, 1>, CR3BitSettings>; // 是否使用DMA辅助发送
        using RTSE = BitFieldObject<MaskRanger<8, 1>, CR3BitSettings>;
        using CTSE = BitFieldObject<MaskRanger<9, 1>, CR3BitSettings>;
        using CTSIE = BitFieldObject<MaskRanger<10, 1>, CR3BitSettings>;
    };
    static_assert(sizeof(CR3BitSettings) == sizeof(uint32_t));
    struct BaudRateDivSettings
    {
        uint32_t data;
        using Fraction = BitFieldObject<MaskRanger<0, 4>, BaudRateDivSettings>;
        using Mantissa = BitFieldObject<MaskRanger<4, 12>, BaudRateDivSettings>;
    };
    static_assert(sizeof(BaudRateDivSettings) == sizeof(uint32_t));
    struct DataRegisterSettings
    {
        uint32_t data;
        using DR = BitFieldObject<MaskRanger<0, 9>, DataRegisterSettings>;
    };
    static_assert(sizeof(DataRegisterSettings) == sizeof(uint32_t));
public:
    void Enable(uint32_t baudRate);
    void SendSync(const uint8_t* data, const uint32_t len);
private:
    template<typename CRSettings>
    CRSettings getControlRegister()
    {
        uint32_t registerAddr = USART_CR_BASE_ADDR_OFFSETs[CRSettings::CR_INDEX] + USARTx_BASE_ADDRs[mCurrentUsingPort];
        CRSettings settings = (*reinterpret_cast<CRSettings*>(registerAddr));
        return settings;
    }
    template<typename CRSettings>
    void setControlRegister(CRSettings settings)
    {
        uint32_t registerAddr = USART_CR_BASE_ADDR_OFFSETs[CRSettings::CR_INDEX] + USARTx_BASE_ADDRs[mCurrentUsingPort];
        (*reinterpret_cast<CRSettings*>(registerAddr)) = settings;
    }
    void setBaudRateRegister(BaudRateDivSettings settings);
    /**
     * @param fClock: 当前uart串口使用的时钟频率
     * @param desiredBaudRate: 当前期望的波特率
     * @return 返回能够产生对应期望波特率的波特率寄存器设置值
     */
    BaudRateDivSettings calculateBaudRateDiv(uint32_t fClock, uint32_t desiredBaudRate);
    /**
     * 向数据寄存器写入数据(理论上每次只会写入一个字节)
     * @param byte: 目标字节数据
     */
    void writeToDataRegister(const uint8_t& byte);
    // Return true if TXE in state register is true, which means we can write to data register
    bool isDataRegisterReadyToWrite() const;
    bool isLastSendingFinised() const;
private:
    USART(PORT usingPort);
private:
    const PORT mCurrentUsingPort;
};

class RCC
{
public:
    // 需要保证以下函数必须正常执行！！！
    static constexpr uint32_t RCC_BASE_ADDR = 0x40021000u;
    static constexpr uint32_t RCC_APB1ENR_REGISTER_ADDR = RCC_BASE_ADDR + 0x1cu;
    static constexpr uint32_t RCC_APB2ENR_REGISTER_ADDR = RCC_BASE_ADDR + 0x18u;
    static constexpr uint32_t RCC_APB1RSTR_REGISTER_ADDR = RCC_BASE_ADDR + 0x10u;
    static constexpr uint32_t RCC_APB2RSTR_REGISTER_ADDR = RCC_BASE_ADDR + 0x0cu;
    // Controll register
    static constexpr uint32_t RCC_CR_ADDR = RCC_BASE_ADDR + 0x0u;
    // Clock configuration register
    static constexpr uint32_t RCC_CFGR_ADDR = RCC_BASE_ADDR + 0x04u;
    /**
     * 初始化系统时钟树设置，目前默认系统时钟速率是72MHz，外设总线速度也以此为标准
     * @note: APB1是低速外设只有36MHz，APB2是高速外设72MHz
     */
    void InitClock();
    // 辅助函数，用来一口气将所有的GPIO口外设总线全部关闭
    void DisableAllGPIO();
    /**
     * 设置GPIO外设接口的可用性。GPIO是以组为单位挂接到外设总线上的。因此单独启用某一个IO口，都需要先启动所属组的总线时钟
     * @param group: 目标gpio组别
     * @param enable
     */
    void SetGPIOGroup(GPIO::PORT_GROUP group, bool enable);
    /**
     * 重置指定USART串口相关的寄存器(?)
     * @param: 需要重置的USART串口
     * @param: 是否需要进行重置
     * @note: 因为外设的重置开关在RCC的系列寄存器上，所以只能放在这里
     */
    void ResetUSART(USART::PORT port, bool reset);
    /**
     * 使能指定的USART串口。其实就是激活它总线上的时钟(假如是异步，也需要时钟?)
     * @param port: 需要使能的USART串口
     * @param enable: 是否需要使能
     */
    void EnableUSART(USART::PORT port, bool enable);
    static RCC& GetInstance();
private:
    struct ControlRegisterSettings
    {
        uint32_t data;
        using PLLDRY = BitFieldObject<MaskRanger<25, 1>, ControlRegisterSettings>; // PLL锁相环是否已经就绪 DRY: ready
        using PLLON = BitFieldObject<MaskRanger<24, 1>, ControlRegisterSettings>; // PLL锁相环使能
        using HSEBYP = BitFieldObject<MaskRanger<18, 1>, ControlRegisterSettings>; // 外部高速时钟旁路
        using HSEDRY = BitFieldObject<MaskRanger<17, 1>, ControlRegisterSettings>; // 外部高速时钟旁路是否就绪
        using HSEON = BitFieldObject<MaskRanger<16, 1>, ControlRegisterSettings>; // 外部高速时钟使能
        using HSIRDY = BitFieldObject<MaskRanger<1, 1>, ControlRegisterSettings>; // 内部高速时钟是否就绪
        using HSION = BitFieldObject<MaskRanger<0, 1>, ControlRegisterSettings>; // 内部高速时钟使能
    };
    static_assert(sizeof(ControlRegisterSettings) == sizeof(uint32_t));

    struct ConfigureRegisterSettings
    {
        uint32_t data;
        using PLLMUL = BitFieldObject<MaskRanger<18, 4>, ConfigureRegisterSettings>; // PLL锁相环倍频量
        using PLLXTPRE = BitFieldObject<MaskRanger<17, 1>, ConfigureRegisterSettings>; // HSE分频器作为PLL输入
        using PLLSRC = BitFieldObject<MaskRanger<16, 1>, ConfigureRegisterSettings>; // PLL时钟源
        using ADCPRE = BitFieldObject<MaskRanger<14, 2>, ConfigureRegisterSettings>; // ADC预分频器设置
        using PPRE2 = BitFieldObject<MaskRanger<11, 3>, ConfigureRegisterSettings>; // APB2总线预分频器设置
        using PPRE1 = BitFieldObject<MaskRanger<8, 3>, ConfigureRegisterSettings>; // APB1总线预分频器设置
        using HPRE = BitFieldObject<MaskRanger<4, 4>, ConfigureRegisterSettings>; // AHB预分频器设置
        using SWS = BitFieldObject<MaskRanger<2, 2>, ConfigureRegisterSettings>; // 系统时钟切换标志，由硬件来进行设置
        using SW = BitFieldObject<MaskRanger<0, 2>, ConfigureRegisterSettings>; // 系统时钟切换标志，由软件来设置时钟源
    };
    static_assert(sizeof(ConfigureRegisterSettings) == sizeof(uint32_t));

    struct APB2ResetRegisterSettings
    {
        uint32_t data;
        using USART1RST = BitFieldObject<MaskRanger<14, 1>, APB2ResetRegisterSettings>; // USART1串口寄存器复位
    };
    static_assert(sizeof(APB2ResetRegisterSettings) == sizeof(uint32_t));

    struct APB1ResetRegisterSettings
    {
        uint32_t data;
        using USART3RST = BitFieldObject<MaskRanger<18, 1>, APB1ResetRegisterSettings>; // USART3串口寄存器复位
        using USART2RST = BitFieldObject<MaskRanger<17, 1>, APB1ResetRegisterSettings>; // USART2串口寄存器复位

    };
    static_assert(sizeof(APB1ResetRegisterSettings) == sizeof(uint32_t));

    struct AHBPeripheralClockEnableRegisterSettings
    {
        uint32_t data;
    };
    static_assert(sizeof(AHBPeripheralClockEnableRegisterSettings) == sizeof(uint32_t));

    struct APB2PeripheralClockEnableRegisterSettings
    {
        uint32_t data;
        using USART1EN = BitFieldObject<MaskRanger<14, 1>, APB2PeripheralClockEnableRegisterSettings>; // USART1串口使能
        // 系列GPIO A-E 接口使能
        using IOPAEN = BitFieldObject<MaskRanger<2, 1>, APB2PeripheralClockEnableRegisterSettings>;
        using IOPBEN = BitFieldObject<MaskRanger<3, 1>, APB2PeripheralClockEnableRegisterSettings>;
        using IOPCEN = BitFieldObject<MaskRanger<4, 1>, APB2PeripheralClockEnableRegisterSettings>;
        using IOPDEN = BitFieldObject<MaskRanger<5, 1>, APB2PeripheralClockEnableRegisterSettings>;
        using IOPEEN = BitFieldObject<MaskRanger<6, 1>, APB2PeripheralClockEnableRegisterSettings>;
    };
    static_assert(sizeof(APB2PeripheralClockEnableRegisterSettings) == sizeof(uint32_t));

    struct APB1PeripheralClockEnableRegisterSettings
    {
        uint32_t data;
        using USART3EN = BitFieldObject<MaskRanger<18, 1>, APB1PeripheralClockEnableRegisterSettings>; // USART3串口使能
        using USART2EN = BitFieldObject<MaskRanger<17, 1>, APB1PeripheralClockEnableRegisterSettings>; // USART2串口使能
    };
    static_assert(sizeof(APB1PeripheralClockEnableRegisterSettings) == sizeof(uint32_t));
private:
    RCC() = default;
};

class SysTick
{
public:
    static constexpr uint32_t SYST_REGISTER_BASE_ADDR = 0xE000E010u;
    // System tick control and status register
    static constexpr uint32_t SYST_CSR_ADDR = SYST_REGISTER_BASE_ADDR + 0x0u;
    // System tick reload value register
    static constexpr uint32_t SYST_RVR_ADDR = SYST_REGISTER_BASE_ADDR + 0x04u;
    // System tick current value reigster
    static constexpr uint32_t SYST_CVR_ADDR = SYST_REGISTER_BASE_ADDR + 0x08u;
    // System tick calibration value register(Read only)
    static constexpr uint32_t SYST_CALIB_ADDR = SYST_REGISTER_BASE_ADDR + 0x0Cu;
    
    static void Delay(uint32_t delayTimeInMS);

    SysTick() = delete;
};



#endif // MyHL_H
