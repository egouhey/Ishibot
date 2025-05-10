#ifndef CONFIG_H
#define CONFIG_H

/**************************************************
 * Motor Declaration
 ****************************************************/

#define   enPin1     15                 // motor 1   
#define   stpPin1    2      
#define   dirPin1    0 

#define   enPin2     4                  // motor 2   
#define   stpPin2    16     
#define   dirPin2    17  

#define   enPin3     19                 // motor 3   
#define   stpPin3    18     
#define   dirPin3    5      

#define  pwmChannel_1 0                 // pwm channel used for motor
#define  pwmChannel_2 2
#define  pwmChannel_3 4

#define resolution_pwm 8                    // resolution of pwm duty cycle

#define granularity 100                 // granularity of motor speed granularity control     

/**************************************************
 * Motor Control
 ****************************************************/

#define   peri_wheel        188.49      // wheel perimeter in millimeter (2*30*pi)
#define   peri_robot        788.5165     // robot diameter in millimeter  (2*?*pi)
#define   step_per_turn     6400        // step per turn

/**************************************************
 * Screen
 ****************************************************/

#define nombreDePixelsEnLargeur 128     // Taille de l'écran OLED, en pixel, au niveau de sa largeur
#define nombreDePixelsEnHauteur 64      // Taille de l'écran OLED, en pixel, au niveau de sa hauteur
#define brocheResetOLED         -1      // Reset de l'OLED partagé avec l'Arduino (d'où la valeur à -1, et non un numéro de pin)
#define adresseI2CecranOLED     0x3C    // Adresse de "mon" écran OLED sur le bus i2c (généralement égal à 0x3C ou 0x3D)

#define Pin_Button_1    32
#define Pin_Button_2    33
#define Pin_Button_3    25

/**************************************************
 * Ultrason
 ****************************************************/

/**************************************************
 * IMU
 ****************************************************/

/**************************************************
 * Actionneur
 ****************************************************/

#endif