//###########################################################################
//
// FILE:    Control_Corriente_Peak
//
// TITLE:  Control de Corriente Peak para Conversor Medio Puente DC-DC
//
//!
//! Este proyecto configura un control de corriente peak convencional
//! para un conversor medio puente dc-dc. Para esto se configura el módulo
//! ePWM1 para activar el módulo ADCD, mientras que el módulo ePWM2 activa
//! la comparación del módulo CMPSS1.

//!
//! Las señales de disparo para los MOSFETs son generadas a través de los
//! módulos ePWM7 y ePWM8.
//!
//! Para implementar el control PI se utiliza el procesador externo CLA y con esto
//! se aplica la referencia a la rampa de compensación incluida en el módulo de
//! comparación.
//!


// --------------------------------------------------------
// Control de Corriente Peak para Conversor Medio Puente DC-DC
// --------------------------------------------------------

//
// Included Files
//

#include "F28x_Project.h"
#include "task_file.h"
#include "F2837xD_Cla_defines.h"

extern uint16_t Cla1ProgRunStart, Cla1ProgLoadStart, Cla1ProgLoadSize;
extern uint16_t CLA1mathTablesRunStart, CLA1mathTablesLoadStart;
extern uint16_t CLA1mathTablesLoadSize;

// Definitions for Selecting Output Pin

#define GPIO_CTRIP_PIN_NUM      14 //OUTPUTXBAR3 is mux'd with GPIO14
#define GPIO_CTRIP_PER_NUM       6 //OUTPUTXBAR3 is peripheral option 6 for
                                   //GPIO14
#define GPIO_CTRIPOUT_PIN_NUM   15 //EPWM8B is mux'd with GPIO15
#define GPIO_CTRIPOUT_PER_NUM    1 //EPWM8B is peripheral option 1 for GPIO15

#define GPIO_CTRIP_PIN_NUM2     11 //OUTPUTXBAR7 is mux'd with GPIO11
#define GPIO_CTRIP_PER_NUM2      3 //OUTPUTXBAR7 is peripheral option 3 for
                                   //GPIO11
#define GPIO_CTRIPOUT_PIN_NUM2  13 //EPWM7B is mux'd with GPIO13
#define GPIO_CTRIPOUT_PER_NUM2   1 //EPWM7B is peripheral option 1 for GPIO13

#ifdef __TI_COMPILER_VERSION__
    #if __TI_COMPILER_VERSION__ >= 15009000
        #define ramFuncSection ".TI.ramfunc"
    #else
        #define ramFuncSection "ramfuncs"
    #endif
#endif

//----------Constantes-Y-Variables--------//

// Constantes de Rampa

float n;
float L_sec;
float Delta_Ramp;
float Delta_X = 5e-9;           //Paso del CLK interno del uC
float Delta_Y;
float AmpRamp;
Uint16 FreqRamp;

// Constantes de Conversión

float V_Max = 130.0;                // Voltaje Máximo a la Salida
float I_Max = 14.0;                 // Corriente Máxima a la Salida
float V_CMax = 700.0;               // Voltaje Máximo en la Entrada
float Base_Max = 125;               // Base del ciclo de trabajo
float V_o = 75.0;                   // Voltaje Nominal
float ADC_Base;
float ADC_I;
float ADC_V;
float ADC_VC;
float DAC_I;

// Paso de muestreo
float h = 6.25e-6;                  // Paso de muestreo t_samp

// Entradas
float Ref = 75.0;                   // Corriente de Referencia
float I_out;                        // Corriente de Primario
float V_out;                        // Voltaje de Salida
float V_Cond;                       // Voltaje de Condensador de Entrada

// Salida
float salida;

// Constantes de PI externo

float ki_ext = 123.1;               // Ki del Lazo Externo
float kp_ext = 0.0146;              // Kp del Lazo Externo
float A_ext;                        // Constante auxiliar
float B_ext;                        // Constante auxiliar
float mult_ext;
float Lim_sup_ext;                  // Limite superior actuación Lazo Interno
float Lim_inf_ext;                  // Limite inferior actuación Lazo Interno


// Constantes de PI interno

float ki_int = 3484.0;              // Ki del Lazo Interno
float kp_int = 0.0146;              // Kp del Lazo Interno
float A_int;                        // Constante auxiliar
float B_int;                        // Constante auxiliar
float mult_int;
float Lim_sup_int;                  // Limite superior actuación Lazo Interno
float Lim_inf_int;                  // Limite inferior actuación Lazo Interno

//----------Variables-de-Prueba---------//

#define BUFFER_SIZE 256
int k = 0;
float u_int_Buffer[BUFFER_SIZE];
float u_ext_Buffer[BUFFER_SIZE];

//------Variables-Maquina-de-Estado-------//

int estado = 2;
#define INICIO 0
#define SOFT_START 1
#define FUNC_NORMAL 2
#define FALLA 3
int boton_ini;
int controlado;
int falla;
int reset;


//------------Variables-de-CLA------------//
#pragma DATA_SECTION(A_int,"CpuToCla1MsgRAM");
#pragma DATA_SECTION(B_int,"CpuToCla1MsgRAM");
#pragma DATA_SECTION(e_ant_int_cpu,"CpuToCla1MsgRAM");
float e_ant_int_cpu;
#pragma DATA_SECTION(u_ant_int_cpu,"CpuToCla1MsgRAM");
float u_ant_int_cpu;
#pragma DATA_SECTION(Lim_sup_int,"CpuToCla1MsgRAM");
#pragma DATA_SECTION(Lim_inf_int,"CpuToCla1MsgRAM");

#pragma DATA_SECTION(A_ext,"CpuToCla1MsgRAM");
#pragma DATA_SECTION(B_ext,"CpuToCla1MsgRAM");
#pragma DATA_SECTION(e_ant_ext_cpu,"CpuToCla1MsgRAM");
float e_ant_ext_cpu;
#pragma DATA_SECTION(u_ant_ext_cpu,"CpuToCla1MsgRAM");
float u_ant_ext_cpu;
#pragma DATA_SECTION(Lim_sup_ext,"CpuToCla1MsgRAM");
#pragma DATA_SECTION(Lim_inf_ext,"CpuToCla1MsgRAM");

#pragma DATA_SECTION(ADC_I,"CpuToCla1MsgRAM");
#pragma DATA_SECTION(ADC_V,"CpuToCla1MsgRAM");
#pragma DATA_SECTION(ADC_VC,"CpuToCla1MsgRAM");
#pragma DATA_SECTION(DAC_I,"CpuToCla1MsgRAM");

#pragma DATA_SECTION(e_ant_int_cla,"Cla1ToCpuMsgRAM");
float e_ant_int_cla;
#pragma DATA_SECTION(u_ant_int_cla,"Cla1ToCpuMsgRAM");
float u_ant_int_cla;
#pragma DATA_SECTION(e_ant_ext_cla,"Cla1ToCpuMsgRAM");
float e_ant_ext_cla;
#pragma DATA_SECTION(u_ant_ext_cla,"Cla1ToCpuMsgRAM");
float u_ant_ext_cla;

//
//  Function Prototypes
//

void CLA_configClaMemory(void);         // Configuración de CLA
void CLA_initCpu1Cla1(void);            // Inicilización de CLA
void ConfigureADC(void);                // Configuración ADC
void ConfigureDAC(void);
void InitEPwm1Example(void);            // Inicialización ePWM1
void InitEPwm2Example(void);            // Inicialización ePWM2
void InitCMPSS(Uint16 MAX,Uint16 FREQ); // Inicialización CMPSS1
void Gpio_setup1(void);
void SetupADCEpwm(Uint16 channel);      // Configuración ADC-ePWM1
void InitEPWM_CMPSS1(void);             // Inicialización ePWM7
void InitEPWM_CMPSS2(void);             // Inicialización ePWM8

interrupt void adcd1_isr(void);

__interrupt void cla1Isr1();
__interrupt void cla1Isr2();
__interrupt void cla1Isr3();
__interrupt void cla1Isr4();
__interrupt void cla1Isr5();
__interrupt void cla1Isr6();
__interrupt void cla1Isr7();
__interrupt void cla1Isr8();

//------------MAIN-------------//

void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
    InitGpio();

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();

//
// Map ISR functions
//

    EALLOW;
    PieVectTable.ADCD1_INT = &adcd1_isr;    //function for ADCD interrupt 1
    EDIS;

//
// Step 4. Initialize CLA Configuration
//

    CLA_configClaMemory();
    CLA_initCpu1Cla1();

//
// Step 5. Calculate Discrete PI Constants
//
    // Parámetros de Conversion
    ADC_I = I_Max/4095;
    ADC_V = V_Max/4095;
    ADC_VC = V_CMax/4095;
    DAC_I = 4095/30;

    // Parámetros de Rampa de Compensación
    L_sec = 7.3e-6;                             // Valor Inductor Secundario
    n = 2.66667;                                // Ratio de Vueltas Tranformador (24:9)
    Delta_Ramp = 2*V_o/(n*L_sec);               // Pendiende de la Rampa de Compensación
    Delta_Y = Delta_Ramp*Delta_X;               // Delta de Voltaje de la Rampa
    Delta_Y = Delta_Y*DAC_I;                    // Acondicionamiento de la señal
    FreqRamp = ((int)Delta_Y << 4) | 0x0;       // Valor Entero de Delta_Y
    AmpRamp = 0xFFF0;                           // Máximo Inicial de la Rampa

    // Parametros de Control Lazo Externo
    mult_ext = h*ki_ext;
    A_ext = 0.5*mult_ext + kp_ext;
    B_ext = kp_ext - 0.5*mult_ext;
    Lim_inf_ext = 0;
    Lim_sup_ext = 150;


    // Parametros de Control Lazo Interno
    mult_int = h*ki_int;
    A_int = 0.5*mult_int + kp_int;
    B_int = kp_int - 0.5*mult_int;
    Lim_inf_int = 0;
    Lim_sup_int = 80;

    // Initialize Variables
    e_ant_int_cpu = 0;
    u_ant_int_cpu = 0;
    e_ant_int_cla = 0;
    u_ant_int_cla = 0;
    e_ant_ext_cpu = 0;
    u_ant_ext_cpu = 0;
    e_ant_ext_cla = 0;
    u_ant_ext_cla = 0;

//
// Step 6. Initialize the Peripherals
//

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    InitEPwm1Example();
    InitEPwm2Example();

    ConfigureADC();
    InitCMPSS(AmpRamp,FreqRamp);
    InitEPWM_CMPSS1();
    InitEPWM_CMPSS2();
    SetupADCEpwm(0);
    ConfigureDAC();

    Gpio_setup1();

    EDIS;
//
// Step 7. User specific code, enable interrupts:
//
// Enable global Interrupts and higher priority real-time debug events:
//
    IER |= M_INT1; //Enable group 1 interrupts
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

//
// Enable ADC INTn in the PIE: Group 1 interrupt 1-3
//
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;

//
// Enable PIE interrupt
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

//
// Step 8. Loop for switching S1 and S2
//
    EALLOW;
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;         // Enable SOCA
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;        // Unfreeze EPWM1, and enter up count mode
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;        // Unfreeze EPWM2, and enter up count mode
    EPwm7Regs.TBCTL.bit.CTRMODE = 0;        // Unfreeze EPWM7, and enter up count mode
    EPwm8Regs.TBCTL.bit.CTRMODE = 0;        // Unfreeze EPWM8, and enter up count mode
    Cmpss1Regs.COMPCTL.bit.COMPDACE = 1;    // Enable Cmpss1
    EDIS;
    //asm("   ESTOP0");
    while(1)
    {
        if (estado == (INICIO)){
            if ((boton_ini == 1)&(falla == 0))
                {estado = SOFT_START;}
            else if (falla == 1)
                {estado = (FALLA);}
            else {estado = estado;}}

        else if (estado == (SOFT_START)){
            if ((controlado == 1)&(falla == 0))
                {estado = FUNC_NORMAL;}
            else if (falla == 1)
                {estado = (FALLA);}
            else {estado = estado;}}

        else if (estado == (FUNC_NORMAL)){
            if ((controlado == 1)&(falla == 0))
                {estado = (FUNC_NORMAL);}
            else if (falla == 1)
                {estado = (FALLA);}
            else {estado = estado;}}

        else if (estado == (FALLA)){
            if ((reset == 1)&(falla == 0))
                {estado = (INICIO);}
            else {estado = (FALLA);}}

        switch (estado)
        {
            case INICIO: //Inicializar algo
                k = 0;
            break;

            case SOFT_START:
                k = 0;
            break;

            case FUNC_NORMAL: //Funcionamiento Normal
                //
                // Trip flags set when CTRIP signal is asserted
                //
                if( EPwm7Regs.TZFLG.bit.CBC  ) //EPwm7Regs.TZFLG.bit.OST
                {
                    EALLOW;

                    //
                    // Wait for comparator CTRIP to de-assert
                    //
                    while( Cmpss1Regs.COMPSTS.bit.COMPHSTS ); //

                    //
                    // Clear trip flags
                    //
                    EPwm7Regs.TZCLR.bit.CBC = 1;   //OST
                    EPwm7Regs.TZCLR.bit.INT = 1;
                    EDIS;
                }
                if( EPwm8Regs.TZFLG.bit.CBC )  //OST
                {
                    EALLOW;

                    //
                    // Wait for comparator CTRIP to de-assert
                    //
                    while( Cmpss2Regs.COMPSTS.bit.COMPHSTS );

                    //
                    // Clear trip flags
                    //
                    EPwm8Regs.TZCLR.bit.CBC = 1;  //OST
                    EPwm8Regs.TZCLR.bit.INT = 1;
                    EDIS;
                }
            break;

            case FALLA: //Inicializar algo
                k = 0;
            break;
        }
    }
}

void CLA_configClaMemory(void)
{
    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;

    EALLOW;
#ifdef _FLASH
    //
    // Copy over code from FLASH to RAM
    //
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
           (uint32_t)&Cla1funcsLoadSize);
#endif //_FLASH

    //
    // Initialize and wait for CLA1ToCPUMsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};

    //
    // Initialize and wait for CPUToCLA1MsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};

    //
    // Select LS5RAM to be the programming space for the CLA
    // First configure the CLA to be the master for LS5 and then
    // set the space to be a program block
    //
    //MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    //MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 1;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 1;

    //
    // Next configure LS0RAM and LS1RAM as data spaces for the CLA
    // First configure the CLA to be the master for LS0(1) and then
    // set the spaces to be code blocks
    //
    MemCfgRegs.LSxMSEL.bit.MSEL_LS3 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS3 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 0;

    EDIS;
}

//
// CLA_initCpu1Cla1 - Initialize CLA1 task vectors and end of task interrupts
//
void CLA_initCpu1Cla1(void)
{
    //
    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    //
    EALLOW;
    Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1);
    Cla1Regs.MVECT2 = (uint16_t)(&Cla1Task2);
    Cla1Regs.MVECT3 = (uint16_t)(&Cla1Task3);
    Cla1Regs.MVECT4 = (uint16_t)(&Cla1Task4);
    Cla1Regs.MVECT5 = (uint16_t)(&Cla1Task5);
    Cla1Regs.MVECT6 = (uint16_t)(&Cla1Task6);
    Cla1Regs.MVECT7 = (uint16_t)(&Cla1Task7);
    Cla1Regs.MVECT8 = (uint16_t)(&Cla1Task8);
    //
    // Enable the IACK instruction to start a task on CLA in software
    // for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
    // subset of tasks) by writing to their respective bits in the
    // MIER register
    //
    Cla1Regs.MCTL.bit.IACKE = 1;
    Cla1Regs.MIER.bit.INT1 = 1;     // Permite que interrupciones o cpu inicien la tarea 1
    Cla1Regs.MIER.bit.INT2 = 1;     // Permite que interrupciones o cpu inicien la tarea 2
    Cla1Regs.MIER.bit.INT6 = 1;     // Permite que interrupciones o cpu inicien la tarea 6
    Cla1Regs.MIER.bit.INT7 = 1;     // Permite que interrupciones o cpu inicien la tarea 7

    //
    // Configure the vectors for the end-of-task interrupt for all
    // 8 tasks
    //
    PieVectTable.CLA1_1_INT = &cla1Isr1;
    PieVectTable.CLA1_2_INT = &cla1Isr2;
    PieVectTable.CLA1_3_INT = &cla1Isr3;
    PieVectTable.CLA1_4_INT = &cla1Isr4;
    PieVectTable.CLA1_5_INT = &cla1Isr5;
    PieVectTable.CLA1_6_INT = &cla1Isr6;
    PieVectTable.CLA1_7_INT = &cla1Isr7;
    PieVectTable.CLA1_8_INT = &cla1Isr8;

//--- Select Task interrupt source                     /******** TRIGGER SOURCE FOR EACH TASK (unlisted numbers are reserved) ********/
    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1 = 16;    // 0=none       8=ADCBINT3  16=ADCDINT1  32=XINT4     42=EPWM7INT   70=TINT2     78=ECAP4INT   95=SD1INT     114=SPIRXINTC
    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 = 0;    // 1=ADCAINT1   9=ADCBINT4  17=ADCDINT2  33=XINT5     43=EPWM8INT   71=MXEVTA    79=ECAP5INT   96=SD2INT
    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK3 = 0;    // 2=ADCAINT2  10=ADCBEVT   18=ADCDINT3  36=EPWM1INT  44=EPWM9INT   72=MREVTA    80=ECAP6INT  107=UPP1INT
    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK4 = 0;    // 3=ADCAINT3  11=ADCCINT1  19=ADCDINT4  37=EPWM2INT  45=EPWM10INT  73=MXEVTB    83=EQEP1INT  109=SPITXINTA
    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK5 = 0;    // 4=ADCAINT4  12=ADCCINT2  20=ADCDEVT   38=EPWM3INT  46=EPWM11INT  74=MREVTB    84=EQEP2INT  110=SPIRXINTA
    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK6 = 0;    // 5=ADCAEVT   13=ADCCINT3  29=XINT1     39=EPWM4INT  47=EPWM12INT  75=ECAP1INT  85=EQEP3INT  111=SPITXINTB
    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK7 = 0;    // 6=ADCBINT1  14=ADCCINT4  30=XINT2     40=EPWM5INT  48=TINT0      76=ECAP2INT  87=HRCAP1INT 112=SPIRXINTB
    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK8 = 0;    // 7=ADCBINT2  15=ADCCEVT   31=XINT3     41=EPWM6INT  69=TINT1      77=ECAP3INT  88=HRCAP2INT 113=SPITXINTC

    DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL1 = 0;     // Write a 1 to lock (cannot be cleared once set)
    DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL2 = 0;     // Write a 1 to lock (cannot be cleared once set)

    //
    // Enable all CLA interrupts at the group and subgroup levels
    //
    PieCtrlRegs.PIEIER11.all = 0xFFFF;
    IER |= (M_INT11 );
    EDIS;
}

void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcdRegs.ADCCTL2.bit.PRESCALE = 2; //set ADCCLK to be SYSCLK/2
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
// InitEPwm1Example - Initialize EPWM1 values
//
void InitEPwm1Example()
{
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0;     // Usar CLK de 200[MHz]
    EPwm1Regs.TBCTL.bit.CTRMODE = 3;                // freeze counter
    EPwm1Regs.TBPRD = 1249;                         // Set frequency to 160[KHz]
    EPwm1Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup ADC Conversion
    //
    EPwm1Regs.ETSEL.bit.SOCAEN = 0;                 // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 2;                // Select SOC on up-count
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;                 // Generate pulse on 1st event

    //
    // Setup Synchronization
    //
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 1;           // Sync output TBCTR = 0 to ePWM2 and ePWM7
    EPwm1Regs.HRPCTL.bit.PWMSYNCSEL = 1;        // Sync output TBCTR = 0
    EPwm1Regs.HRPCTL.bit.PWMSYNCSELX = 0;       // Sync Peripherals TBCTR = 0

    //
    // Set Compare values
    //
    EPwm1Regs.CMPA.bit.CMPA = 50;               // Set compare A value
    EPwm1Regs.CMPB.bit.CMPB = 50;               // Set Compare B value
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;          // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;        // Clear PWM1A on event A,
                                                // up count
    EDIS;

}
//
// InitEPwm2Example - Initialize EPWM2 values
//
void InitEPwm2Example()
{
    //
    // Setup TBCLK
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = 3;                // freeze counter
    EPwm2Regs.TBPRD = 624;                          // Set timer period 320KHz
    EPwm2Regs.TBCTL.bit.PHSEN = 1;                  // Enable phase loading
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;             // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    // Setup Synchronization
    //
    EPwm2Regs.TBCTL.bit.SYNCOSEL = 1;               // Sync output TBCTR = 0 to ePWM3
    EPwm2Regs.HRPCTL.bit.PWMSYNCSEL = 1;            // Sync output TBCTR = 0
    EPwm2Regs.HRPCTL.bit.PWMSYNCSELX = 0;           // Sync Peripherals TBCTR = 0

    //
    // Setup shadow register load on ZERO
    //
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm2Regs.CMPA.bit.CMPA = 50;                   // Set compare A value

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;              // Set PWM1A on Zero
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Clear PWM1A on event A

}

//
// InitCMPSS - Initialize CMPSS1 and configure settings
//
void InitCMPSS(Uint16 MAX,Uint16 FREQ)
{
    EALLOW;
    Cmpss1Regs.COMPCTL.bit.COMPDACE = 0;                // Disable CMPSS1

    //
    //Configurate Ramp
    //
    Cmpss1Regs.COMPCTL.bit.COMPHSOURCE = 0;             // NEG signal comes from DAC
    Cmpss1Regs.COMPDACCTL.bit.RAMPSOURCE = 1;           // Synchronize Cmpss1 with ePWM2
    Cmpss1Regs.COMPDACCTL.bit.SELREF = 0;               // Use VDDA as the reference for DAC
    Cmpss1Regs.COMPDACCTL.bit.SWLOADSEL = 0;            // DACxVALA is updated from DACxVALS on SYSCLK
    Cmpss1Regs.COMPDACCTL.bit.RAMPLOADSEL = 1;          // RAMPSTS is updated from RAMPMAXREFS
    Cmpss1Regs.RAMPMAXREFS = MAX;                       // Max value from Ramp
    Cmpss1Regs.RAMPDECVALS = FREQ;                      // Decrementing value from Ramp
    Cmpss1Regs.COMPCTL.bit.COMPHINV = 0;                // NOT Inverted Comparation Output
    Cmpss1Regs.COMPDACCTL.bit.DACSOURCE = 1;            // DACHVALA is updated from the ramp generator


    //
    // Configure Digital Filter   NOT USED
    //
    Cmpss1Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = 0x000; // Maximum CLKPRESCALE value provides the most time between samples
    Cmpss1Regs.CTRIPHFILCTL.bit.SAMPWIN = 0x00A;        // Maximum SAMPWIN value provides largest number of samples
    Cmpss1Regs.CTRIPHFILCTL.bit.THRESH = 0x06;          // Maximum THRESH value requires static value for entire window
                                                        // THRESH should be GREATER than half of SAMPWIN
    Cmpss1Regs.CTRIPHFILCTL.bit.FILINIT = 1;            // Reset filter logic & start filtering

    //
    // Configure CTRIPOUT path
    // Asynch output feeds CTRIPH and CTRIPOUTH
    //
    Cmpss1Regs.COMPCTL.bit.CTRIPHSEL = 0;               // Select Asynch output for CTRIPH
    Cmpss1Regs.COMPCTL.bit.CTRIPOUTHSEL = 0;            // Select Asynch output for CTRIPOUTH

    //
    // Configure CTRIPOUTH output pin
    // Configure OUTPUTXBAR7 to be CTRIPOUT1H
    //
    OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX0 = 0;

    //
    //Enable OUTPUTXBAR7 Mux for Output
    //
    OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX0 = 1;

    EDIS;
}

//
// InitEPWM - Initialize EPWM8 module settings
//

void InitEPWM_CMPSS1(void)
{
    EALLOW;
    //
    //Configure EPWM to run at SYSCLK
    //
    EPwm7Regs.TBCTL.bit.CTRMODE = 3;             // Freeze counter
    EPwm7Regs.TBPRD = 1249;                      // Set timer period 160KHz
    EPwm7Regs.TBCTL.bit.PHSEN = 1;               // Enable phase loading
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm7Regs.TBCTR = 0x0000;                    // Clear counter
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;     // Clock ratio to SYSCLKOUT
    EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //EPwm7Regs.TBCTL.bit.CLKDIV = 0;
    //EPwm7Regs.TBCTL.bit.HSPCLKDIV = 0;

    //
    // Setup Synchronization
    //

    EPwm7Regs.TBCTL.bit.SYNCOSEL = 1;                   // Sync output TBCTR = 0 to ePWM8
    EPwm7Regs.HRPCTL.bit.PWMSYNCSEL = 1;                // Sync output TBCTR = 0
    EPwm7Regs.HRPCTL.bit.PWMSYNCSELX = 0;               // Sync Peripherals TBCTR = 0
    SyncSocRegs.SYNCSELECT.bit.EPWM7SYNCIN = 0;         // Sync EPWM7 to EPWM1

    //
    //Configure EPWM7B
    //
    EPwm7Regs.AQCTLB.bit.ZRO = AQ_SET;                  // TBCTR = 0 setea Pwm7B
    EPwm7Regs.TZCTL.bit.TZB = 2;                        // Señal de comparacion resetea Pwm7B
    EPwm7Regs.CMPB.bit.CMPB = 618;                      // Set Compare B value
    EPwm7Regs.AQCTLB.bit.CBU = AQ_CLEAR;                // Clear Pwm7B on CMPB 50%
                                                        // up count

    //
    //Configure DCB to be TRIP4
    //
    EPwm7Regs.TZDCSEL.bit.DCBEVT2 = TZ_DCBH_HI;  //Modificado
    EPwm7Regs.DCTRIPSEL.bit.DCBHCOMPSEL = 0xF;
    EPwm7Regs.DCBHTRIPSEL.bit.TRIPINPUT4 = 1;

    //
    //Configure DCB as CBC //OST
    //
    EPwm7Regs.TZSEL.bit.DCBEVT2 = 1;  //Modificado

    //
    //Configure DCB Filter
    //

    EPwm7Regs.DCBCTL.bit.EVT1SRCSEL = 1;                        // DCB Event 1 is filtered
    EPwm7Regs.DCBCTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;         // DCB is not synchronized with EPWMCLK
    EPwm7Regs.DCFCTL.bit.SRCSEL = 2;                            // DCB is chosen to be filtered
    EPwm7Regs.DCFCTL.bit.BLANKE = 1;                            // Blanking Window Enable
    EPwm7Regs.DCFCTL.bit.PULSESEL = 1;                          // Windows is aligned with TBCTR = 0
    EPwm7Regs.DCFOFFSET = 0;                                    // Blanking Window Offset
    EPwm7Regs.DCFWINDOW = 10;                                   // Blanking Window Counter

    //
    //Configure TRIP4 to be CTRIP1H
    //
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0 = 0;

    //
    //Enable TRIP4 Mux for Output
    //
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0 = 1;

    //
    // Clear trip flags
    //

    EPwm7Regs.TZCLR.bit.CBC = 1;   //OST
    EPwm7Regs.TZCLR.bit.INT = 1;

    EDIS;
}

void InitEPWM_CMPSS2(void)
{
    EALLOW;
    //
    //Configure EPWM to run at SYSCLK
    //
    EPwm8Regs.TBCTL.bit.CTRMODE = 3;             // freeze counter
    EPwm8Regs.TBPRD = 1249;                      // Set timer period 160KHz
    EPwm8Regs.TBCTL.bit.PHSEN = 1;               // Enable phase loading
    EPwm8Regs.TBPHS.bit.TBPHS = 624;             // Phase is 180°
    EPwm8Regs.TBCTR = 0x0000;                    // Clear counter
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;     // Clock ratio to SYSCLKOUT
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    //
    //Configure EPWM8B
    //
    EPwm8Regs.AQCTLB.bit.ZRO = AQ_SET;                  // TBCTR = 0 setea Pwm7B
    EPwm8Regs.TZCTL.bit.TZB = 2;                        // Señal de comparacion resetea Pwm7B
    EPwm8Regs.CMPB.bit.CMPB = 618;                      // Set Compare B value
    EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR;                // Clear Pwm7B on CMPB 50%
                                                        // up count

    //
    //Configure DCB to be TRIP4
    //
    EPwm8Regs.TZDCSEL.bit.DCBEVT2 = TZ_DCBH_HI;  //Modificado
    EPwm8Regs.DCTRIPSEL.bit.DCBHCOMPSEL = 0xF;
    EPwm8Regs.DCBHTRIPSEL.bit.TRIPINPUT4 = 1;

    //
    //Configure DCB as CBC //OST
    //
    EPwm8Regs.TZSEL.bit.DCBEVT2 = 1;  //Modificado

    //
    //Configure DCB Filter
    //

    EPwm8Regs.DCBCTL.bit.EVT1SRCSEL = 1;                        // DCB Event 1 is filtered
    EPwm8Regs.DCBCTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;         // DCB is not synchronized with EPWMCLK
    EPwm8Regs.DCFCTL.bit.SRCSEL = 2;                            // DCB is chosen to be filtered
    EPwm8Regs.DCFCTL.bit.BLANKE = 1;                            // Blanking Window Enable
    EPwm8Regs.DCFCTL.bit.PULSESEL = 1;                          // Windows is aligned with TBCTR = 0
    EPwm8Regs.DCFOFFSET = 0;                                    // Blanking Window Offset
    EPwm8Regs.DCFWINDOW = 10;                                   // Blanking Window Counter

    //
    //Configure TRIP4 to be CTRIP1H
    //
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0 = 0;

    //
    //Enable TRIP4 Mux for Output
    //
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0 = 1;

    //
    // Clear trip flags
    //

    EPwm8Regs.TZCLR.bit.CBC = 1;   //OST
    EPwm8Regs.TZCLR.bit.INT = 1;

    EDIS;
}
//
// Gpio_setup1 INIT the  GPIO to see PwmA and PwmB from Epwm1 and Epwm2
//
void Gpio_setup1(void)
{
   //
   // These can be combined into single statements for improved
   // code efficiency.
   //

   //
   // Enable PWM1-2 on GPIO0-GPIO3
   //
   EALLOW;
   GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;   // Enable pullup on GPIO0
   GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pullup on GPIO1
   GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;   // Enable pullup on GPIO2
   GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;   // Enable pullup on GPIO3
   GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;   // Enable pullup on GPIO4
   GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;   // Enable pullup on GPIO5
   GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;  // GPIO0 = PWM1A
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;  // GPIO1 = PWM1B
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;  // GPIO2 = PWM2A
   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;  // GPIO3 = PWM2B
   GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;  // GPIO4 = PWM3A
   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;  // GPIO5 = PWM3B
   EDIS;
   //
   // Configure GPIO11 to output CTRIP2H/EPWM TRIP4
   //
   GPIO_SetupPinMux(GPIO_CTRIP_PIN_NUM2, GPIO_MUX_CPU1, GPIO_CTRIP_PER_NUM2);
   //
   // Configure GPIO13 to output CTRIPOUT2H
   //
   GPIO_SetupPinMux(GPIO_CTRIPOUT_PIN_NUM2, GPIO_MUX_CPU1,
                        GPIO_CTRIPOUT_PER_NUM2);
   //
   // Configure GPIO14 to output CTRIP1H/EPWM TRIP4
   //
   GPIO_SetupPinMux(GPIO_CTRIP_PIN_NUM, GPIO_MUX_CPU1, GPIO_CTRIP_PER_NUM);
   //
   // Configure GPIO15 to output CTRIPOUT1H
   //
   GPIO_SetupPinMux(GPIO_CTRIPOUT_PIN_NUM, GPIO_MUX_CPU1,
                        GPIO_CTRIPOUT_PER_NUM);
   //
   // For this example, only init the pins for the SCI-A port.
   //  GPIO_SetupPinMux() - Sets the GPxMUX1/2 and GPyMUX1/2 register bits
   //  GPIO_SetupPinOptions() - Sets the direction and configuration of the GPIOS
   // These functions are found in the F2837xD_Gpio.c file.
   //
      GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
      GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
      GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
      GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

}

void SetupADCEpwm(Uint16 channel)
{
    Uint16 acqps;

    //
    // Determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcdRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //
    //Select the channels to convert and end of conversion flag
    //
    EALLOW;
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;      //SOC0 will convert pin A0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps;  //sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5;    //trigger on ePWM1 SOCA/C


    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1;      //SOC1 will convert pin A1
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = acqps;  //sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 5;    //trigger on ePWM1 SOCA/C


    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 2;      //SOC2 will convert pin A2
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = acqps;  //sample window is 100 SYSCLK cycles
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 5;    //trigger on ePWM1 SOCA/C

    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 2;  //end of SOC2 will set INT1 flag
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;    //enable INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //make sure INT1 flag is cleared
    EDIS;
}

void ConfigureDAC(void)
{
    EALLOW;

    //
    //write configurations
    //
    DacaRegs.DACCTL.bit.DACREFSEL = 1;
    DacaRegs.DACCTL.bit.LOADMODE = 0;
    DacaRegs.DACCTL.bit.SYNCSEL = 1;

    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(10);

    EDIS;
}

interrupt void adcd1_isr(void)
{
    //asm("   ESTOP0");
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag


//
// Check if overflow has occurred
//

    if(1 == AdcdRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcdRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// cla1Isr1 - CLA1 ISR 1
//
__interrupt void cla1Isr1 () // Fault mode
{

    e_ant_int_cpu = e_ant_int_cla;
    u_ant_int_cpu = u_ant_int_cla;
    e_ant_ext_cpu = e_ant_ext_cla;
    u_ant_ext_cpu = u_ant_ext_cla;

    //asm("   ESTOP0");

    u_int_Buffer[k] = u_ant_int_cpu;
    u_ext_Buffer[k] = u_ant_ext_cpu;
    k++;
    if (k == (BUFFER_SIZE))
    {
//        asm("   ESTOP0");
        k = 0;
    }

    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

       //
       // Check if overflow has occurred
       //

       if(1 == AdcdRegs.ADCINTOVF.bit.ADCINT1)
       {
           AdcdRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
           AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
       }
    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1 | PIEACK_GROUP11);
}

//
// cla1Isr2 - CLA1 ISR 2
//
__interrupt void cla1Isr2 () // Sync_PLL
{
    asm(" ESTOP0");
}

//
// cla1Isr3 - CLA1 ISR 3
//
__interrupt void cla1Isr3 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr4 - CLA1 ISR 4
//
__interrupt void cla1Isr4 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr5 - CLA1 ISR 5
//
__interrupt void cla1Isr5 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr6 - CLA1 ISR 6
//
__interrupt void cla1Isr6 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr7 - CLA1 ISR 7
//
__interrupt void cla1Isr7 () // Normal mode
{
    asm(" ESTOP0");
}

//
// cla1Isr8 - CLA1 ISR 8
//
__interrupt void cla1Isr8 ()
{
    asm(" ESTOP0");
}
