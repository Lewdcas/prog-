#include <stdio.h>
#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"
#include "TMP36_sensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"

using namespace events;

// exemple de < 30 bytes
// Si les données sont plus grande alors ajuster ci-dessous
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Timer en MS pour les retry
 */
#define TX_TIMER                        10000

/**
 * Max events pour chaque try
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Max de retry pour chaque essaie
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

//temps entre chaque envoi :
#define Timer_inter 10
/**
* Queue des données
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Fonction lorawan qui comprends les "event_handler"
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

int main()
{       
    //Initialisation LoRaWAN 
    setup_trace();
    
    lorawan_status_t retcode;
    
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }
    printf("\r\n Mbed LoRaWANStack initialized \r\n");//Confirmation console
    
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);
    
    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK || //Si carte OK alors Uplink 
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Erreur de connection ; code : %d \r\n", retcode); //sinon affiché erreur + son code 
        return -1;
    }
    ev_queue.dispatch_forever();
    return 0;
}
static void send_message() {

    uint16_t valTMP;//Définition d'une variable pour stocker les données du capteur
    valTMP= myTMP36;//Ici on lui fait stocké les données
    
    unsigned char buffer[2];//payload pour envoyer 4 octets
    buffer[0] = ( valTMP>> 8) & 0xFF;//les 2 premiers
    buffer[1] = valTMP & 0xFF;//les 2 suivants et derniers
    wait(Timer_inter);
    int16_t retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, buffer , sizeof(buffer), MSG_CONFIRMED_FLAG);//Fonction envoie
}

/**
 * switch avec différents cas : 
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED: //si connecté alors on envoie et on print dans la console que tout est OK : 
            printf("\r\n Send OK \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);//Sinon on met en queue
            }

            break;
            
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
            
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // on réessaye
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
            
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
            
        case JOIN_FAILURE://si l'OTAA HS (pas de passerelle branchée ou autre...)
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
            
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required  \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
            
        default:
            MBED_ASSERT("Unknown Event");
    }
}

