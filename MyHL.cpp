#include <algorithm>
#include <cstdlib>
#include <math.h>
#include "MyHL.h"
#include "MyLibrary.h"

RCC& RCC::GetInstance()
{
    static RCC _inst;
    return _inst;
}

void RCC::InitClock()
{
    ControlRegisterSettings& CR = (*(reinterpret_cast<ControlRegisterSettings*>(RCC_CR_ADDR)));
    ConfigureRegisterSettings& CFRG = (*(reinterpret_cast<ConfigureRegisterSettings*>(RCC_CFGR_ADDR)));
    APB1PeripheralClockEnableRegisterSettings& APB1Enr = (*(reinterpret_cast<APB1PeripheralClockEnableRegisterSettings*>(RCC_APB1ENR_REGISTER_ADDR)));
    // 使能USART3外设时钟 TODO: 应该开放成接口让外面调用?
    APB1PeripheralClockEnableRegisterSettings::USART3EN::Set(APB1Enr, 1);
    // 启动外部高速时钟，注意：当前硬件外接的时钟周期是8MHz!!!8MHz!!!8MHz!!!8MHz!!!
    ControlRegisterSettings::HSEON::Set(CR, true);
    // 检查外部时钟是否就绪
    while(ControlRegisterSettings::HSEDRY::Get(CR) != 1);
    // 设置低速APB1预分频为2分频(这里连接的时钟频率最高不超过36MHz，而STM32最高是72MHz。因此2分频刚好可以保证不超过最大上限)
    ConfigureRegisterSettings::PPRE1::Set(CFRG, 0x4u);
    // 设置高速APB2预分频 不进行 分频，即依旧保持72MHz
    ConfigureRegisterSettings::PPRE2::Set(CFRG, 0x0u);
    // 将PLLMUL设置为9，即假如是直连8MHz外部时钟，就可以获得一个72MHz的时钟周期(默认是2倍频)
    ConfigureRegisterSettings::PLLMUL::Set(CFRG, 0x7u);
    // 将PLLSRC设置为使用外部时钟(默认是使用2分频后的内部时钟)
    ConfigureRegisterSettings::PLLSRC::Set(CFRG, 1);
    // 调整Flash的时延设置？似乎和Flash的读写正确性相关
    (*reinterpret_cast<uint32_t*>(Flash::FLASH_ACR_ADDR)) |= 0x32u;
    // 启动PLL时钟
    ControlRegisterSettings::PLLON::Set(CR, true);
    // 等待PLL时钟就绪
    while(ControlRegisterSettings::PLLDRY::Get(CR) != 1);
    // 将PLL时钟作为系统时钟(SysClk)
    ConfigureRegisterSettings::SW::Set(CFRG, 2);
    // 等待PLL时钟顺利切换成系统时钟
    while(ConfigureRegisterSettings::SWS::Get(CFRG) != 0x2);
}

void RCC::DisableAllGPIO()
{
    for(uint8_t groupIdx = 0; groupIdx < GPIO::PORT_GROUP::PG_COUNT; ++groupIdx)
    {
        uint32_t groupAddr = GPIO::PORT_GROUP_BASE_ADDRs[groupIdx];
        uint32_t brrAddr = groupAddr + GPIO::PORT_BRR_ADDR_OFFSET;
        uint32_t* brrPtr = reinterpret_cast<uint32_t*>(brrAddr);
        (*brrPtr) &= 0x0u;
        // 相当于将所有的GPIO针脚的输出寄存器(output data register)全部清空
        (*brrPtr) |= 0xFFFFu;
        RCC::SetGPIOGroup(static_cast<GPIO::PORT_GROUP>(groupIdx), false);
    }
}

void RCC::SetGPIOGroup(GPIO::PORT_GROUP group, bool enable)
{
    APB2PeripheralClockEnableRegisterSettings& apb2Enr = (*(reinterpret_cast<APB2PeripheralClockEnableRegisterSettings*>(RCC_APB2ENR_REGISTER_ADDR)));
    switch(group)
    {
    case GPIO::PORT_GROUP::PG_A:
        APB2PeripheralClockEnableRegisterSettings::IOPAEN::Set(apb2Enr, enable);
        break;
    case GPIO::PORT_GROUP::PG_B:
        APB2PeripheralClockEnableRegisterSettings::IOPBEN::Set(apb2Enr, enable);
        break;
    case GPIO::PORT_GROUP::PG_C:
        APB2PeripheralClockEnableRegisterSettings::IOPCEN::Set(apb2Enr, enable);
        break;
    case GPIO::PORT_GROUP::PG_D:
        APB2PeripheralClockEnableRegisterSettings::IOPDEN::Set(apb2Enr, enable);
        break;
    case GPIO::PORT_GROUP::PG_E:
        APB2PeripheralClockEnableRegisterSettings::IOPEEN::Set(apb2Enr, enable);
        break;
    default:
        // Invalid port group!
        std::abort();
    }
}

void RCC::ResetUSART(USART::PORT port, bool reset)
{
    if (port == USART::PORT::U1)
    {
        APB2ResetRegisterSettings& apb2rr = (*(reinterpret_cast<APB2ResetRegisterSettings*>(RCC_APB2RSTR_REGISTER_ADDR)));
        APB2ResetRegisterSettings::USART1RST::Set(apb2rr, reset);
    }
    else
    {
        APB1ResetRegisterSettings& apb1rr = (*(reinterpret_cast<APB1ResetRegisterSettings*>(RCC_APB1RSTR_REGISTER_ADDR)));
        if (port == USART::PORT::U2)
            APB1ResetRegisterSettings::USART2RST::Set(apb1rr, reset);
        else if (port == USART::PORT::U3)
            APB1ResetRegisterSettings::USART3RST::Set(apb1rr, reset);
        else
            std::abort();
    }
}

void RCC::EnableUSART(USART::PORT port, bool enable)
{
    if (port == USART::PORT::U1)
    {
        APB2PeripheralClockEnableRegisterSettings& apb2enr = (*(reinterpret_cast<APB2PeripheralClockEnableRegisterSettings*>(RCC_APB2ENR_REGISTER_ADDR)));
        APB2PeripheralClockEnableRegisterSettings::USART1EN::Set(apb2enr, enable);
    }
    else
    {
        APB1PeripheralClockEnableRegisterSettings& apb1enr = (*(reinterpret_cast<APB1PeripheralClockEnableRegisterSettings*>(RCC_APB1ENR_REGISTER_ADDR)));
        if (port == USART::PORT::U2)
            APB1PeripheralClockEnableRegisterSettings::USART2EN::Set(apb1enr, enable);
        else if (port == USART::PORT::U3)
            APB1PeripheralClockEnableRegisterSettings::USART3EN::Set(apb1enr, enable);
        else
            std::abort();
    }
}

constexpr uint32_t GPIO::PORT_GROUP_BASE_ADDRs[5];

uint32_t GPIO::getGroupRegisterAddr()
{
    uint32_t groupAddr = GPIO::PORT_GROUP_BASE_ADDRs[static_cast<uint32_t>(mGroup)];
    return groupAddr;
}

uint32_t GPIO::getControlRegisterAddr()
{
    uint32_t groupAddr = getGroupRegisterAddr();
    // port idx [0, 7] : GPIOx_CRL(addr offset 0); [8, 15] : GPIOx_CRH(addr offset 0x04)
    uint32_t controlRegisterAddr = groupAddr + (mPin > 7 ? PORT_CRH_ADDR_OFFSET : PORT_CRL_ADDR_OFFSET);
    return controlRegisterAddr;
}

GPIO::OutputDataRegisterSettings& GPIO::getOutputDataRegister()
{
    uint32_t odrAddr = getGroupRegisterAddr() + GPIO::PORT_ODR_ADDR_OFFSET;
    GPIO::OutputDataRegisterSettings& odr = (*(reinterpret_cast<GPIO::OutputDataRegisterSettings*>(odrAddr)));
    return odr;
}

GPIO& GPIO::GetInstance(PORT_GROUP group, uint8_t pin)
{
    static GPIO _insts[PORT_GROUP::PG_COUNT][16] = {};
    GPIO& ret = _insts[group][pin];
    if (ret.mGroup == PORT_GROUP::PG_COUNT)
    {
        ret = GPIO(group, pin);
    }
    return ret;
}

void GPIO::SetEnable(bool enable)
{
    // RCC中的外设总线时钟在时钟初始化的时候已经启动过了
    RCC& rcc = RCC::GetInstance();
    // RCC中对应GPIO组的外设时钟要启动起来
    rcc.SetGPIOGroup(mGroup, true);
}

void GPIO::SetPortMode(GPIO::PORT_MODE targetMode, GPIO::PORT_USAGE portUsage)
{
    if (mMode == targetMode && mUsage == portUsage)
        return;
    mMode = targetMode;
    mUsage = portUsage;
    uint8_t mode = static_cast<uint8_t>(mMode);
    uint8_t cnf = static_cast<uint8_t>(mUsage) & 0b11;
    uint32_t controlRegisterAddr = getControlRegisterAddr();
    if (mPin > 7)
    {
        ControlRegisterHighSettings& crh = (*(reinterpret_cast<ControlRegisterHighSettings*>(controlRegisterAddr)));
        HPinModeSettingFuncs[mPin - 8](crh, mode);
        HPinConfigurationSettingFuncs[mPin - 8](crh, cnf);
    }
    else
    {
        ControlRegisterLowSettings& crl = (*(reinterpret_cast<ControlRegisterLowSettings*>(controlRegisterAddr)));
        LPinModeSettingFuncs[mPin](crl, mode);
        LPinConfigurationSettingFuncs[mPin](crl, cnf);
    }
}

void GPIO::SetPortOutputValue(bool value)
{
    OutputDataRegisterSettings& odr = getOutputDataRegister();
    PinOutputDataRegisterSettingsFuncs[mPin](odr, value);
}

bool GPIO::GetPortValue()
{
    // Not Implement!
    std::abort();
    return false;
}

void SysTick::Delay(uint32_t delayTimeInMS)
{
    if (delayTimeInMS <= 0)
        return;
    // 声明我们需要使用外部时钟源进行计时
    // 其次0bit依旧是0，即sys tick还没有enable
    // 1bit位置是0，即计数器归零之后不会出发SysTick中断
    (*reinterpret_cast<uint32_t*>(SYST_CSR_ADDR)) &= ~(4u);
    // 假设我们的外部时钟在倍频之后就是72MHz，而SysTick是经过了一次8分频，因此是9MHz
    // 不难计算，Tick 9次相当于过去了1us，9k次相当于过去了1ms
    constexpr uint32_t FAC_Us = 9u;
    constexpr uint32_t FAC_Ms = FAC_Us * 1000u;
    constexpr uint32_t MAX_COUNT_VALUE = 0xFFFFFFu;
    // 设置Reload寄存器
    (*reinterpret_cast<uint32_t*>(SYST_RVR_ADDR)) = (std::min(FAC_Ms * delayTimeInMS, MAX_COUNT_VALUE - 1));
    // 清空Current Value寄存器，准备开始重新计时
    (*reinterpret_cast<uint32_t*>(SYST_CVR_ADDR)) = 0x0u;
    // 开始计时
    (*reinterpret_cast<uint32_t*>(SYST_CSR_ADDR)) = 0x1u;
    volatile uint32_t SYST_CSR_Value = (*reinterpret_cast<uint32_t*>(SYST_CSR_ADDR));
    // 假如CSR寄存器中COUNTFLAG的标识位一直是0，说明还没有完成计数，则一直等待
    while((SYST_CSR_Value & 0x1u) && ((SYST_CSR_Value & (1u << 16)) == 0))
    {
        SYST_CSR_Value = (*reinterpret_cast<uint32_t*>(SYST_CSR_ADDR));
    }
    // 停止计数
    (*reinterpret_cast<uint32_t*>(SYST_CSR_ADDR)) = 0x0u;
    // 清空Current Value寄存器
    (*reinterpret_cast<uint32_t*>(SYST_CVR_ADDR)) = 0x0u;
}

USART::USART(USART::PORT port)
 : mCurrentUsingPort(port)
{
}

USART& USART::GetInstance(USART::PORT port)
{
    static USART _insts[USART::PORT::PORT_COUNT] = {
        USART(USART::PORT::U1),
        USART(USART::PORT::U2),
        USART(USART::PORT::U3),
    };
    return _insts[port];
}

uint32_t USART::GetStateRegisterAddr(USART::PORT port)
{
    return USART_SR_ADDR_OFFSET + USARTx_BASE_ADDRs[port];
}

uint32_t USART::GetDataRegisterAddr(USART::PORT port)
{
    return USART_DR_ADDR_OFFSET + USARTx_BASE_ADDRs[port];
}

uint32_t USART::GetGuardTimeAndPrescalerRegisterAddr(USART::PORT port)
{
    return USART_GTPR_ADDR_OFFSET + USARTx_BASE_ADDRs[port];
}

void USART::setBaudRateRegister(BaudRateDivSettings settings)
{
    uint32_t bbrAddr = USART_BRR_ADDR_OFFSET + USARTx_BASE_ADDRs[mCurrentUsingPort];
    (*reinterpret_cast<BaudRateDivSettings*>(bbrAddr)) = settings;
    return;
}

void USART::Enable(uint32_t BaudRate)
{
    // 需要APB1以及APB2的总线使能
    RCC::GetInstance().EnableUSART(mCurrentUsingPort, true);
    // 是否需要对USART串口复位?
    RCC::GetInstance().ResetUSART(mCurrentUsingPort, true);
    // 设置GPIO口的输入输出模式?
    {
        GPIO& gpioPB10 = GPIO::GetInstance(GPIO::PORT_GROUP::PG_B, 10);
        gpioPB10.SetEnable(true);
        gpioPB10.SetPortMode(GPIO::PORT_MODE::PM_OUTPUT_50MHz, GPIO::PORT_USAGE::PU_OUTPUT_ALTERNATE_PUSH_PULL);
        
        GPIO& gpioPB11 = GPIO::GetInstance(GPIO::PORT_GROUP::PG_B, 11);
        gpioPB11.SetEnable(true);
        gpioPB11.SetPortMode(GPIO::PORT_MODE::PM_INPUT, GPIO::PORT_USAGE::PU_INPUT_FLOATING);
    }
    // 是否需要对USART串口复位?
    RCC::GetInstance().ResetUSART(mCurrentUsingPort, false);
    CR1BitSettings cr1 = getControlRegister<CR1BitSettings>();
    CR2BitSettings cr2 = getControlRegister<CR2BitSettings>();
    CR1BitSettings::UE::Set(cr1, 1); // enable usart
    CR1BitSettings::M::Set(cr1, 0); // 8 bit per word
    CR2BitSettings::STOP::Set(cr2, 0); // 1 bit for stop
    CR1BitSettings::TE::Set(cr1, 1); // enable transmit
    setControlRegister(cr1);
    setControlRegister(cr2);
    const uint32_t FClock = (mCurrentUsingPort == USART::PORT::U1 ? 72000000u : 36000000u); // 假设时钟频率是一个常数，另外，因为USART1是在APB1上，可以全速运行，而2，3是在APB2上，最高不超过36MHz
    setBaudRateRegister(calculateBaudRateDiv(FClock, BaudRate));
}

void USART::SendSync(const uint8_t* data, const uint32_t len)
{
    for(uint32_t i = 0; i < len; ++i)
    {
        // 等待数据寄存器处于可写状态
        while(isDataRegisterReadyToWrite() == false);
        writeToDataRegister(*(data + i));
    }
    // 等待最后一次发送也完成了. 是否真的需要这种等待?
    while(isLastSendingFinised() == false);
    return;
}

USART::BaudRateDivSettings USART::calculateBaudRateDiv(uint32_t fClock, uint32_t desiredBaudRate)
{
    constexpr uint32_t oversampleRate = 16; // 似乎是STM32设置的一个定值!?
    float divF = static_cast<float>(fClock) / static_cast<float>(oversampleRate * desiredBaudRate);
    float mantissaF = std::floor(divF);
    uint32_t mantissaI = static_cast<uint32_t>(mantissaF);
    uint32_t fractionI = static_cast<uint32_t>(std::ceil((divF - mantissaF) * oversampleRate));
    BaudRateDivSettings bds;
    BaudRateDivSettings::Mantissa::Set(bds, mantissaI);
    BaudRateDivSettings::Fraction::Set(bds, fractionI);
    return bds;
}

void USART::writeToDataRegister(const uint8_t& data)
{
    DataRegisterSettings& drs = *(reinterpret_cast<DataRegisterSettings*>(USART_DR_ADDR_OFFSET + USARTx_BASE_ADDRs[mCurrentUsingPort]));
    DataRegisterSettings::DR::Set(drs, data);
}

bool USART::isDataRegisterReadyToWrite() const
{
    const SRSettings& srs = *(reinterpret_cast<SRSettings*>(GetStateRegisterAddr(mCurrentUsingPort)));
    return SRSettings::TXE::Get(srs) != 0;
}

bool USART::isLastSendingFinised() const
{
    const SRSettings& srs = *(reinterpret_cast<SRSettings*>(GetStateRegisterAddr(mCurrentUsingPort)));
    return SRSettings::TC::Get(srs) != 0;
}
