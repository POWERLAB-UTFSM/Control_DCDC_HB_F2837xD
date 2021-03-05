/*
 * task_file.h
 *
 *  Created on: 09-09-2020
 *      Author: Seba
 */

#ifndef TASK_FILE_H_
#define TASK_FILE_H_

#endif /* TASK_FILE_H_ */
//
// Included Files
//

#include "F28x_Project.h"
#include "F2837xD_Cla_defines.h"

#include <stdint.h>

//#define  WORDS_IN_FLASH_BUFFER    0xFF  // Data/Program Buffer used for testing
                                        // the flash API functions



//#ifdef __cplusplus
//extern "C" {
//#endif


// Constantes de Rampa
extern float AmpRamp2;


// Ratio de Vueltas Inverso
#define N 0.375

// Entradas
extern float Ref;
extern float V_out;
extern float I_out;
extern float V_Cond;

// Constantes de Conversion
extern float ADC_I;
extern float ADC_V;
extern float ADC_VC;
extern float DAC_I;

// Parametros de Control Lazo Externo
extern float e_ext;                    // Error actual del Lazo Externo
extern float e_ant_ext_cpu;                // Error anterior del Lazo Externo
extern float e_ant_ext_cla;                // Error anterior del Lazo Externo
extern float u_ext;                    // Actuación actual del lazo Externo
extern float u_ant_ext_cpu;                // Actuación anterior del Lazo Externo
extern float u_ant_ext_cla;                // Actuación anterior del Lazo Externo
extern float mult_ext;                 // Multiplicación auxiliar
extern float A_ext;                    // Constante auxiliar
extern float B_ext;                    // Constante auxiliar
extern float Lim_sup_ext;              // Limite superior actuación Lazo Externo
extern float Lim_inf_ext;              // Limite inferior actuación Lazo Externo
// Parametros de Control Lazo Interno

extern float e_int;                     // Error actual del Lazo Interno
extern float e_ant_int_cpu;                    // Error anterior del Lazo Interno en la CPU
extern float e_ant_int_cla;             // Error anterior del Lazo Interno en la CLA
extern float u_int;                     // Actuación actual del lazo Interno
extern float u_ant_int_cpu;                    // Actuación anterior del Lazo Interno en la CPU
extern float u_ant_int_cla;             // Actuación anterior del Lazo Interno en la CLA
extern float mult_int;                         // Multiplicación auxiliar
extern float A_int;                            // Constante auxiliar
extern float B_int;                            // Constante auxiliar
extern float Lim_sup_int;                      // Limite superior actuación Lazo Interno
extern float Lim_inf_int;                      // Limite inferior actuación Lazo Interno

// Function Prototypes
//
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.
//
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

//#ifdef __cplusplus
//extern "C" }
//#endif //


//
// End of file
//
