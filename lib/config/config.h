#ifndef CONFIG_H
#define CONFIG_H

/**************************************************
 * Motor Declaration
 ****************************************************/

#define   enPin2     15                 // motor 1   
#define   stpPin2    2      
#define   dirPin2    0 

#define   enPin3     4                  // motor 2   
#define   stpPin3    16     
#define   dirPin3    17  

#define   enPin1     19                 // motor 3   
#define   stpPin1    18     
#define   dirPin1    5      

#define   RX_ENCODER    26              // encodeur
#define   TX_ENCODER    27

#define  pwmChannel_1 0                 // pwm channel used for motor
#define  pwmChannel_2 2
#define  pwmChannel_3 4

#define resolution_pwm 8                    // resolution of pwm duty cycle

#define granularity 10                 // granularity of motor speed granularity control     

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
 * Lidar
 ****************************************************/

#define RX_LIDAR 14
#define TX_LIDAR 255

/**************************************************
 * Actionneur
 ****************************************************/

#endif