#ifndef lidar_h
#define lidar_h

#include "Arduino.h"
#include <driver/timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LIDAR_MAX_PTS_SCAN 25  // 1200 

const int length_tab = LIDAR_MAX_PTS_SCAN*47;

const uint8_t LIDAR_PACKET_SIZE = 47;
const uint8_t LIDAR_PTS_PER_PACKETS = 12;
const uint8_t LIDAR_MEASURE_SIZE = 3;

const uint8_t LIDAR_HEADER = 0x54;
const uint8_t LIDAR_VER_SIZE = 0x2c;

struct PacketHandler {
  uint8_t tab[length_tab];
  uint16_t index;
};

/**
 * @class LIDAR
 * @brief Utilisation simplifié du LD06
 */
class LIDAR {
  public:
    /** * @brief Constructeur. Autres fonctions à appeler start_interrupt()
     * @param HardwareSerial &serial, uint8_t RxPin, uint8_t TxPin @return Nothing */
    LIDAR(HardwareSerial &serial, uint8_t RxPin, uint8_t TxPin);
    /** * @brief Contenu de l'interruption sur le pot serie */
    void IRAM_ATTR onUARTReceive();
    /** * @brief Activation de l'interrupton */
    void start_interrupt();
    /** * @brief Désactivation de l'interrupton */
    void stop_interrupt();
  /** * @brief Port série associé au lidar */
    HardwareSerial *lidarSerial;

  private:    
    /** * @brief Initialisation du port serie*/
    void setup_uart();
    /** * @brief analyse du buffer récupéré  */
    void analyse_data();
    /** * @brief Lancement de la tache du lidar  */
    static void task_lidar(void *pvParameters);
    /** * @brief Réception des données */
    bool read_data();
    /** * @brief Gestion des buffers */
    void swapBuffers();

    uint8_t Rxpin;
    uint8_t Txpin;
    PacketHandler bufferA;
    PacketHandler bufferB;
    PacketHandler *currentbuffer = &bufferA;
    PacketHandler *previousbuffer;
    bool change_buffer=true;
    unsigned long baudrate = 230400;
};

void Lidar_start_interrupt(LIDAR *lidar);

#endif