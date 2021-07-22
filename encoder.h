/** @file encoder.c
 *
 * @brief Module to control a quadrature encoder.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2018 Barr Group. All rights reserved.
 * Propietary: Christian Sandoval - san16250@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */

#ifndef ENCODER_H
#define ENCODER_H

void qei_module0_config(uint32_t motor_ratio, uint32_t encoder_pulses,
                        bool swap);
void qei_module1_config(uint32_t motor_ratio, uint32_t encoder_pulses,
                        bool swap);
float get_position_in_degrees(uint32_t qei_base, uint32_t motor_ratio,
                        uint32_t encoder_pulses);

#endif /* ENCODER_H */

/*** end of file ***/
