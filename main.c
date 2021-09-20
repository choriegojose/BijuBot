// Archivo Main BijuBot.
/** @file main.c
 *
 * @brief Main module to manage all the operations
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2021 All rights reserved.
 * Propietary: Jose Ignacio Choriego - cho16523@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "motor.h"
#include "encoder.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "uartstdio.h"
#include <math.h>

// Se define la frecuencia principal del PWM
#define PWM_FREQUENCY 55
//
// A boolean that is set when a MPU6050 command has completed.
// Booleano para el Status del MPU6050, cuando ha completado la lectura.
volatile bool g_bMPU6050Done;

//
// I2C master instances
//Instancia o estructura que define, los datos que seran enviados en el bus del I2C

tI2CMInstance g_sI2CMSimpleInst;

//
// Parametro del Reloj
int clockFreq;

// Variables de Control y ajuste del PWM
volatile uint32_t ui32Load;     // La variable para cargar el dato de los ms
//Variables de ajuste
volatile uint32_t ui32PWMClock;
volatile uint8_t ui8Adjust;

//Variables principales para la programcion  del PID, son globles para el programa
float e_k, w_k, e_d, E_k, u_k, e_k_1, E_k_1, encoder1_pos, encoder1go;

//Banderas de control del PID
bool Ek_1f = false;
bool ek_1f = false;

// Delta T del reloj, para controlar al PID
const float deltat = 0.001;

// Constantes del PID (kp,ki,kd)
const float Kp = 1;
const float KI = 1;
const float Kd = 1;

// Variables para control del TIMER0
uint32_t ui32Period;
bool PIDflag = true;

//Variables para el acelerometro, giroscopio y struct del MPU6050
float fAccel[3], fGyro[3];
tMPU6050 sMPU6050;
float y;
float thetagiro,thetagiro_1,HPF,HPF_1,LPF,LPF_1 ;
bool thetagiro_1f = false;
bool HPF_1f = false;
bool LPF_1f = false;
const float lambda = 0.9;


// Variables para la convercion de angulos a constante de ms para el servo.
float outtoservo, inservo;


// Prototipo de función
void PIDblock(void);
void compfiltering(void);

// Empiezan Funciones
void Timer0IntHandlerA(void)
{
    // Borramos la interrupcion del timer
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Leemos el estado actual
    // lo escribimos de vuelta en el estado contrario

   // Se coloca el bloque del PID en el handler de Interrupcion
    PIDflag = true;






}
// The interrupt handler for the I2C module.
// Handler de interrupciones para el modulo de 12C

void I2CMSimpleIntHandler(void)
{
    //
    // Call the I2C master driver interrupt handler
    // El handler necesita la dirección del struct de estatus del I2C
    // para verificar la interrupción o transacción de datos.
    I2CMIntHandler(&g_sI2CMSimpleInst);
}

void TIMER0init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // Habilitamos el periferico del TIMER0

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // Configuramos  el timer 0

    ui32Period = (40000); // El periodo del reloj lo ponemos para que sea 1000 Hz

    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1); //Cargo el valor al reloj

    IntEnable(INT_TIMER0A); // Habilitamos el bloque A del timer0

    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Habilitamos la interrupcion del timer.

    IntMasterEnable();  // Habilitamos las interrupcion

    TimerEnable(TIMER0_BASE, TIMER_A); // Habilito el timer 0

}

//Funcion para iniciar el modulo del PWM
void PWMinit(void)
{
    // Se define el reloj del sistema, con reloj de 40 MHZ
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // configuracion del PWM del motor
    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
 //Inicialización de los motores.

    motor1_configure(100);
    qei_module0_config(100, 12, false);
    qei_module1_config(100, 12, false);
}

//Funcion para reactivar el TIMER0
void TIMER0en(void)
{
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Habilitamos la interrupcion del timer.

    IntMasterEnable();  // Habilitamos las interrupciones generales

    TimerEnable(TIMER0_BASE, TIMER_A); // Habilito el timer 0
}

void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    // Resetea el periferico
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C
    // Se habilitan los perifericos del I2C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Se selecciona la funcion de cada pin
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // (Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.
    // I2C data transfer rate set to 400kbps).

    // Se inicializa el Master Clk a 400kb por segundo y se usa la frecuencia del reloj
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    // Se inicia el I2C colocando los datos, para la struct de la Instancia.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff,
             SysCtlClockGet());

}

void ConfigureUART(void)
{ // Se configura el UART y el Baudrate

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);

}

//
// La funcion de acontinuacion es una llamada cuando los regristos llegan y se completan en la MPU6050

void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // See if an error occurred.
    //
    if (ui8Status != I2CM_STATUS_SUCCESS)
    {
        //
        // Ocurre un error
        //
    }
    //
    // Indicate that the MPU6050 transaction has completed.
    // Se enciende la bandera si el estatus del 12C fue aceptado.
    g_bMPU6050Done = true;
}

void PIDblock(void)
{
    //Constantes principales para la programcion  del PID, son globles para el programa

    if (Ek_1f == false)
    {
        E_k_1 = 0;
        Ek_1f = true;
    }
    if (ek_1f == false)
    {
        e_k_1 = 0;
        ek_1f = true;
    }

    e_k = w_k - encoder1go;
    e_d = e_k - e_k_1;
    E_k = E_k_1 + e_k;
    u_k = Kp * e_k + (KI * E_k) * deltat + (Kd * e_d) / deltat;
    E_k_1 = E_k;
    e_k_1 = e_k;

    u_k = e_k;

}

// Bloque de implementacion de filtro complementario
void compfiltering(void){

    if (thetagiro_1f == false)
     {
        thetagiro_1 = 0;
        thetagiro_1f = true;
     }

    if (LPF_1f == false)
     {
            LPF_1 = 0;
            LPF_1f = true;
     }

    if (HPF_1f == false)
         {
                HPF_1 = 0;
                HPF_1f = true;
         }

    thetagiro = thetagiro_1 + (fGyro[1]*(180/3.1416)*deltat);

   // Se inicia el filtro, pasa bajas
   LPF = (1-lambda)*y + lambda*LPF_1;

    // filtro, pasa altas
   HPF = (lambda*thetagiro) - (lambda*thetagiro_1) + lambda*HPF_1;
   // Resultado
   w_k = LPF + HPF;

   //Variables pasadas
   thetagiro_1 = thetagiro;
   LPF_1 = LPF;
   HPF_1 = HPF;
}



//
// The MPU6050 example.
//

// Rutina principal de Funcionamiento de la MPU6050.
void MPU6050init(void)
{
    //
    // Initialize the MPU6050. This code assumes that the I2C master instance
    // has already been initialized.
    // Secuencía de Inicialización
    g_bMPU6050Done = false;
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback,
                &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    //
    // Configure the MPU6050 for +/- 2 g accelerometer range.
    //
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG,
                           ~MPU6050_ACCEL_CONFIG_AFS_SEL_M,
                           MPU6050_ACCEL_CONFIG_AFS_SEL_2G,
                           MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }
    //
    // Configure the MPU6050 for +/- 250 dps .
    //
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_GYRO_CONFIG,
                           ~MPU6050_GYRO_CONFIG_FS_SEL_M,
                           MPU6050_GYRO_CONFIG_FS_SEL_250,
                           MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }


    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00,
                           0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET,
                           MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2, 0x00, 0x00,
                           MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    //
    // Loop forever reading data from the MPU6050. Typically, this process
    // would be done in the background, but for the purposes of this example,
    // it is shown in an infinite loop.
    //

}

int main()
{

    InitI2C0();
    ConfigureUART();
    MPU6050init();
    PWMinit();
    TIMER0init();

    // Habilito el TIMER0
    TIMER0en();

    while (1)
    {

        // Request another reading from the MPU6050.
        //
        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }
        //
        // Get the new accelerometer and gyroscope readings.
        //
        //Para que la Tiva obtenga los valores del Acelerometro y Giroscopio.
        MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1], &fAccel[2]);
        MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);
        //
        // Do something with the new accelerometer and gyroscope readings.
        //

        y = (atan2(fAccel[0], fAccel[2]) * 180.0) / 3.1416 ;




        // Obtengo el valor de enconder magnetico
        encoder1_pos = get_position_in_degrees(QEI0_BASE, 100, 12);

       // Se realiza la converción del decoder
       encoder1go = atan2(sin(encoder1_pos),cos(encoder1_pos))*(180/3.1416);



        if (PIDflag == true)
            {

        compfiltering();
        PIDblock();

        PIDflag = false;
            }

        // Asigno el valor al bloque del PID
        // Cargo los valores del PID a la variable para controlar el motor, que es similar a un servo.
        outtoservo = u_k;


       motor_velocity_write(PWM0_BASE, PWM_GEN_0,outtoservo,100);


        // Se despliega el valor al UART
        UARTprintf("out_to motor: %d | Ang. IMU: %d\n", (int) outtoservo,(int) w_k );
        // UARTprintf((int) w_k );
        //UARTprintf( (int) encoder1go );

    }
}
