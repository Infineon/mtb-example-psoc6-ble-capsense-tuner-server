/*******************************************************************************
* File Name: tuner_ble_server.c
*
* Description: This file contains BLE related functions.
*
* Related Document: README.md
*
********************************************************************************
* Copyright (2020), Cypress Semiconductor Corporation. All rights reserved.
********************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

/******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg_capsense.h"
#include "cycfg_ble.h"
#include "cy_retarget_io.h"
#include "tuner_ble_server.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define ENABLE                       (1u)
#define DISABLE                      (0u)
#define CY_ASSERT_FAILED             (0u)
#define DEBUG_BLE_ENABLE             (DISABLE)
#if DEBUG_BLE_ENABLE
#define DEBUG_PRINTF                 (printf)
#else
#define DEBUG_PRINTF(...)
#endif

/* BLE related macros */
#define BLESS_INTR_PRIORITY          (1u)
#define NOTIFICATION_PKT_SIZE        (492u)
#define SUCCESS                      (0U)
#define DEVICE_NAME_LENGTH           (20u)
#define DEFAULT_NOTIFICATION_COUNT   (1u)

/* Length of notification packet to send size of CapSense structure(2 bytes)
 * No of notification packets required to send complete CapSense data(1 byte) */
#define TUNER_BRIDGE_INIT_NTF_SIZE   (3u)
#define CAPSENSE_DS_SIZE_LSB_IDX     (0u)
#define CAPSENSE_DS_SIZE_MSB_IDX     (1u)
#define NOTIFICATION_COUNT_IDX       (2u)

/* Custom Tuner command packet received from GATT Client - 7 bytes */
#define TUNER_COMMAND_SIZE_0_IDX     (0u)
#define TUNER_COMMAND_OFFS_0_IDX     (1u)
#define TUNER_COMMAND_OFFS_1_IDX     (2u)
#define TUNER_COMMAND_DATA_0_IDX     (3u)
#define TUNER_COMMAND_DATA_1_IDX     (4u)
#define TUNER_COMMAND_DATA_2_IDX     (5u)
#define TUNER_COMMAND_DATA_3_IDX     (6u)
#define TUNER_COMMAND_PACKET_SIZE    (7u)
#define MAX_DATA_LENGTH              (4u)
#define MSB_SHIFT                    (8u)


/*******************************************************************************
 * Global variables
 ******************************************************************************/
/* No of notification packets to send one complete CapSense structure */
static uint8_t count = DEFAULT_NOTIFICATION_COUNT;

/* Buffer to hold Tuner receive command packet */
static uint8_t ble_write_buffer[TUNER_COMMAND_PACKET_SIZE] = {0};

/* Size of the CapSense data structure */
static uint16_t capsense_ds_size = 0;

/* To indicate that notification is enabled by GATT client */
static volatile bool ble_notification_enabled = false;

/* To store the connection handle */
static cy_stc_ble_conn_handle_t appConnHandle;

/* To send notification packet to GATT client */
static cy_stc_ble_gatts_handle_value_ntf_t notificationPacket;

static volatile bool ble_disconnected = false;


/*******************************************************************************
 * Function Prototypes
*******************************************************************************/
static void bless_interrupt_handler(void);
static void stack_event_handler(uint32_t event, void* eventParam);


/*******************************************************************************
* Function Name: bless_interrupt_handler
********************************************************************************
* Summary:
* Wrapper function for handling interrupts from BLESS.
*
*******************************************************************************/
static void bless_interrupt_handler(void)
{
    Cy_BLE_BlessIsrHandler();
}


/*******************************************************************************
 * Function Name: ble_capsense_tuner_init()
 *******************************************************************************
 *
 * Summary:
 *   This function initializes the BLE for use in the code example.
 *
 ******************************************************************************/
void ble_capsense_tuner_init(void)
{
    cy_en_ble_api_result_t apiResult = CY_BLE_SUCCESS;

    /* BLESS interrupt configuration structure */
    const cy_stc_sysint_t  blessIsrCfg =
    {
            /* The BLESS interrupt */
            .intrSrc       = bless_interrupt_IRQn,

            /* The interrupt priority number */
            .intrPriority  = BLESS_INTR_PRIORITY
    };

    /* Initialize the BLESS interrupt */
    cy_ble_config.hw->blessIsrConfig = &blessIsrCfg;

    /* Hook interrupt service routines for BLESS */
    Cy_SysInt_Init(cy_ble_config.hw->blessIsrConfig, bless_interrupt_handler);

    /* Register the generic event handler */
    Cy_BLE_RegisterEventCallback(stack_event_handler);

    /* Initialize the BLE host */
    apiResult = Cy_BLE_Init(&cy_ble_config);

    if (apiResult != CY_BLE_SUCCESS)
    {
        /* BLE stack initialization failed, check configuration, notify error
         * and halt CPU in debug mode */
        DEBUG_PRINTF("Cy_BLE_Init API Error: %x \r\n", apiResult);
    }
    else
    {
        DEBUG_PRINTF("Cy_BLE_Init API Success: %x \r\n", apiResult);
    }

    /* Enable BLE */
    apiResult = Cy_BLE_Enable();

    if (apiResult != CY_BLE_SUCCESS)
    {
        /* BLE stack initialization failed, check configuration,
             notify error and halt CPU in debug mode */
        DEBUG_PRINTF("Cy_BLE_Enable API Error: %x \r\n", apiResult);
        CY_ASSERT(CY_ASSERT_FAILED);
    }
}

/*******************************************************************************
* Function Name: stack_event_handler
********************************************************************************
*
* Summary:
*   This is an event callback function to receive events from the BLE Component.
*
* Parameters:
*  uint32_t event     : Event from the BLE stack
*  void* eventParam : Pointer to the value of event specific parameters
*
*******************************************************************************/
static void stack_event_handler(uint32_t event, void* eventParam)
{
    cy_en_ble_api_result_t apiResult = CY_BLE_SUCCESS;

    switch(event)
    {
    /***************************************************************************
     *                       General Events
     **************************************************************************/

    /* This event is received when the BLE component is Started */
    case CY_BLE_EVT_STACK_ON:
    {
        DEBUG_PRINTF("BLE Stack Event: CY_BLE_EVT_STACK_ON \r\n");

        /* Enter into discoverable mode so that remote device can search it */
        apiResult = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,\
                CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        if(apiResult != CY_BLE_SUCCESS)
        {
            DEBUG_PRINTF("Cy_BLE_GAPP_StartAdvertisement API Error: %d \r\n",\
                          apiResult);
        }
        break;
    }

    /* This event is received when there is a timeout */
    case CY_BLE_EVT_TIMEOUT:
    {
        DEBUG_PRINTF("BLE Stack Event - Event timeout");
        break;
    }

    /***************************************************************************
     *                       GAP Events
     **************************************************************************/

    /* This event indicates peripheral device has started/stopped advertising */
    case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
    {
        char device_name[DEVICE_NAME_LENGTH];
        DEBUG_PRINTF("BLE Stack Event :"\
                     "CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:\n\r");
        cy_en_ble_adv_state_t adv_state = CY_BLE_ADV_STATE_STOPPED;

        adv_state = Cy_BLE_GetAdvertisementState();
        if(adv_state == CY_BLE_ADV_STATE_ADVERTISING)
        {
            printf("\n\rDevice is advertising with name: ");
            Cy_BLE_GetLocalName(device_name);
            printf("%s\n\r",device_name);
        }
        else if((adv_state == CY_BLE_ADV_STATE_STOPPED) &&\
                (Cy_BLE_GetConnectionState(appConnHandle) ==\
                 CY_BLE_CONN_STATE_DISCONNECTED))
        {
            printf("Restarting Advertisement \r\n");
            printf("\n\rDevice is advertising with name: ");
            Cy_BLE_GetLocalName(device_name);
            printf("%s\n\r",device_name);

            /* Enter into discoverable mode so that remote device can search it */
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,\
                    CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);

        }
        break;
    }

    case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
    {
        /* BLE link is established */
        DEBUG_PRINTF("BLE Stack Event : CY_BLE_EVT_GAP_DEVICE_CONNECTED \r\n");
        cy_stc_ble_gap_enhance_conn_complete_param_t *conn_param =\
                (cy_stc_ble_gap_enhance_conn_complete_param_t *)eventParam;

        /* Variable to store values to update PHY to 2M */
        cy_stc_ble_set_phy_info_t phy_param;
        phy_param.allPhyMask = CY_BLE_PHY_NO_PREF_MASK_NONE;
        phy_param.bdHandle = appConnHandle.bdHandle;
        phy_param.rxPhyMask = CY_BLE_PHY_MASK_LE_2M;
        phy_param.txPhyMask = CY_BLE_PHY_MASK_LE_2M;

        printf("\n\rConnected to device: ");
        for(uint8_t i = (CY_BLE_BD_ADDR_SIZE); i > 0u; i--)
        {
            printf(" %X", conn_param->peerBdAddr[i - 1u]);
        }

        DEBUG_PRINTF("\r\nBDhandle : 0x%02X\r\n", conn_param->bdHandle);

        /* Reset notification enabled flag */
        ble_notification_enabled = false;

        /* Reset ble_disconnected enabled flag */
        ble_disconnected = false;

        /* Turn ON the user LED when ble connection is established */
        cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1, CYBSP_LED_STATE_ON);

        /* Function call to set PHY to 2M */
        apiResult = Cy_BLE_SetPhy(&phy_param);
        if(apiResult == CY_BLE_SUCCESS)
        {
            DEBUG_PRINTF("Set PHY to 2M successfull");
            DEBUG_PRINTF("Request sent to switch PHY to 2M\r\n");
        }
        else
        {
            DEBUG_PRINTF("Set PHY to 2M API failure, errorcode = 0x%X", apiResult);
        }
        break;
    }

    /* This event is triggered when there is a change to either the maximum
     * Payload length or the maximum transmission time of Data Channel PDUs in
     * either direction */
    case CY_BLE_EVT_DATA_LENGTH_CHANGE:
    {
        DEBUG_PRINTF("CY_BLE_EVT_DATA_LENGTH_CHANGE \r\n");
        break;
    }

    /* This event is generated when disconnected from remote device or failed to
     * establish connection. */
    case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
    {
        if(Cy_BLE_GetConnectionState(appConnHandle) ==\
                                            CY_BLE_CONN_STATE_DISCONNECTED)
        {
            DEBUG_PRINTF("CY_BLE_EVT_GAP_DEVICE_DISCONNECTED %d\r\n",\
                    CY_BLE_CONN_STATE_DISCONNECTED);
            printf("Disconnected. \r\n\n");
        }

        /* Set ble_disconnected flag to true */
        ble_disconnected = true;

        /*Reset ble_notification_enabled flag to false */
        ble_notification_enabled = false;

        /* BLE disconnected - turn off LED */
        cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1, CYBSP_LED_STATE_OFF);

        /* Device disconnected; restart advertisement */
        Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,\
                                       CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        break;
    }

    /* This event indicates that the controller has changed the transmitter
     * PHY or receiver PHY in use */
    case CY_BLE_EVT_PHY_UPDATE_COMPLETE:
    {
        DEBUG_PRINTF("UPDATE PHY parameters\r\n");
        /* To remove unused parameter warning when UART debug is disabled */
#if (DEBUG_BLE_ENABLE == ENABLE)
        cy_stc_ble_events_param_generic_t *param =\
                (cy_stc_ble_events_param_generic_t *)eventParam;
        cy_stc_ble_phy_param_t *phyparam = NULL;
        if(param->status == SUCCESS)
        {
            phyparam = (cy_stc_ble_phy_param_t *)param->eventParams;
            DEBUG_PRINTF("RxPhy Mask : 0x%02X\r\nTxPhy Mask : 0x%02X\r\n",\
                    phyparam->rxPhyMask, phyparam->txPhyMask);
        }
#endif
        break;
    }

    /***************************************************************************
     *                       GATT Events
     **************************************************************************/

    /* This event is generated at the GAP Peripheral end after connection is
     * completed with peer Central device. */
    case CY_BLE_EVT_GATT_CONNECT_IND:
    {
        appConnHandle = *(cy_stc_ble_conn_handle_t *)eventParam;
        DEBUG_PRINTF("CY_BLE_EVT_GATT_CONNECT_IND: %x, %x \r\n",\
                      appConnHandle.attId,\
                      appConnHandle.bdHandle);
        Cy_BLE_GetPhy(appConnHandle.bdHandle);
        break;
    }

    /* This event is generated at the GAP Peripheral end after disconnection. */
    case CY_BLE_EVT_GATT_DISCONNECT_IND:
    {
        DEBUG_PRINTF("CY_BLE_EVT_GATT_DISCONNECT_IND \r\n");

        if(appConnHandle.bdHandle ==\
                (*(cy_stc_ble_conn_handle_t *)eventParam).bdHandle)
        {
            appConnHandle.bdHandle = CY_BLE_INVALID_CONN_HANDLE_VALUE;
            appConnHandle.attId    = CY_BLE_INVALID_CONN_HANDLE_VALUE;
        }

        break;
    }

    /* This event is triggered when GATT MTU Exchange Request received from
     * GATT client device. */
    case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ:
    {
        /* To remove unused parameter warning when UART debug is disabled */
#if (DEBUG_BLE_ENABLE == ENABLE)
        uint16_t negotiated_mtu = 0;
        negotiated_mtu =\
        (((cy_stc_ble_gatt_xchg_mtu_param_t *)eventParam)->mtu < CY_BLE_GATT_MTU) ?\
        ((cy_stc_ble_gatt_xchg_mtu_param_t *)eventParam)->mtu : CY_BLE_GATT_MTU;
        DEBUG_PRINTF("CY_BLE_EVT_GATTS_XCNHG_MTU_REQ negotiated = %d\r\n",\
                      negotiated_mtu);
#endif
        break;
    }

    /* This event is triggered when a  write request is received from a peer
     * Client device */
    case CY_BLE_EVT_GATTS_WRITE_REQ:
    {
        uint16_t size = 0;
        uint8_t tuner_init_buffer[TUNER_BRIDGE_INIT_NTF_SIZE] = {0};

        cy_stc_ble_gatt_write_param_t *write_req_param =\
                (cy_stc_ble_gatt_write_param_t *)eventParam;
        cy_stc_ble_gatts_db_attr_val_info_t attr_param;

        attr_param.connHandle.bdHandle = write_req_param->connHandle.bdHandle;
        attr_param.connHandle.attId = write_req_param->connHandle.attId;
        attr_param.flags = CY_BLE_GATT_DB_PEER_INITIATED;
        attr_param.handleValuePair = write_req_param->handleValPair;
        attr_param.offset = 0;

        if(write_req_param->handleValPair.attrHandle ==\
           CY_BLE_CAPSENSE_TUNER_CAPSENSE_DS_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE)
        {
            /* Write Response to GATT Client in response to Write request */
            Cy_BLE_GATTS_WriteRsp(write_req_param->connHandle);

            Cy_BLE_GATTS_WriteAttributeValuePeer(&appConnHandle, &(write_req_param->handleValPair));

            ble_notification_enabled = attr_param.handleValuePair.value.val[0];
            notificationPacket.connHandle = appConnHandle;
            notificationPacket.handleValPair.attrHandle =\
                    CY_BLE_CAPSENSE_TUNER_CAPSENSE_DS_CHAR_HANDLE;

            /* If notification is enabled, send size of CapSense data structure
             * and the number of notification packets required to send the
             * CapSense data structure to the GATT client to initialize the
             * Tuner bridge */
            if(ble_notification_enabled == true)
            {
                printf("\n\rNotifications enabled... \n\r");
                printf("\n\rSending Tuner bridge initialization parameters"\
                           "to GATT Client... \n\r");
                capsense_ds_size = sizeof(cy_capsense_tuner);
                size = capsense_ds_size;

                /* Calculate how many notification packets are required to
                 * transmit the CapSense data structure */
                while(1)
                {
                    if(size > NOTIFICATION_PKT_SIZE)
                    {
                        size = size - NOTIFICATION_PKT_SIZE;
                        count++;
                    }
                    else
                    {
                        break;
                    }
                }
                tuner_init_buffer[CAPSENSE_DS_SIZE_LSB_IDX] =\
                                           (uint8_t)(capsense_ds_size & 0x00FF);
                tuner_init_buffer[CAPSENSE_DS_SIZE_MSB_IDX] =\
                                           (uint8_t)(capsense_ds_size >> 8);
                tuner_init_buffer[NOTIFICATION_COUNT_IDX] = count;

                printf("Size of CapSense Data Structure: %u\n\r",\
                                                capsense_ds_size);
                printf("Notification packet size: %u \n\r",\
                                                NOTIFICATION_PKT_SIZE);
                printf("No of notifications to send complete data structure: "\
                                                "%u\n\r", count);
                /* Send Bridge initialization parameters */
                notificationPacket.handleValPair.value.len =\
                                                 TUNER_BRIDGE_INIT_NTF_SIZE;
                notificationPacket.handleValPair.value.val = tuner_init_buffer;
                /* Send notification to GATT client to initialize tuner bridge
                 * parameters */
                Cy_BLE_GATTS_Notification(&notificationPacket);
            }
        }
        break;
    }

    /* This event is generated when a write without response is generated from
     * GATT Client */
    case CY_BLE_EVT_GATTS_WRITE_CMD_REQ:
    {
        uint16_t offset_address= 0;
        uint8_t length = 0;

        cy_stc_ble_gatts_write_cmd_req_param_t write_cmd_param =\
                *(cy_stc_ble_gatts_write_cmd_req_param_t *) eventParam;

        /* Check if length of received packet is equal to
         * TUNER_COMMAND_PACKET_SIZE */
        if(write_cmd_param.handleValPair.value.len == TUNER_COMMAND_PACKET_SIZE)
        {
            memcpy(ble_write_buffer,\
                   write_cmd_param.handleValPair.value.val,\
                   write_cmd_param.handleValPair.value.len);

            offset_address =\
            ((uint16_t)ble_write_buffer[TUNER_COMMAND_OFFS_0_IDX] << MSB_SHIFT)\
             | (uint16_t)ble_write_buffer[TUNER_COMMAND_OFFS_1_IDX];

            length = ble_write_buffer[TUNER_COMMAND_SIZE_0_IDX];

            if(length <= MAX_DATA_LENGTH)
            {
                /* Modify CapSense data structure */
                for (uint8_t i = 0 , j = MAX_DATA_LENGTH - 1; i < length; i++, j--)
                {
                    *(((uint8_t *)&cy_capsense_tuner) + offset_address + i) =\
                        ble_write_buffer[TUNER_COMMAND_DATA_0_IDX + j];
                }
            }
        }
        break;
    }

    /***************************************************************************
     *                       Other Events
     **************************************************************************/
    default:
        break;
    }
}


/*******************************************************************************
* Function Name: ble_capsense_process
********************************************************************************
*
* Summary:
*   -  allows the BLE stack to process pending events
*
*******************************************************************************/
void ble_process_events(void)
{
    /* Cy_BLE_ProcessEvents() allows the BLE stack to process pending events */
    Cy_BLE_ProcessEvents();
}


/*******************************************************************************
* Function Name: tuner_send_callback
********************************************************************************
*
* Summary:
*   - Tuner send callback function periodically called by CapSense_RunTuner().
*   - This function sends CapSense data structure to the GATT client as
*     notification packets to be read by the Tuner GUI
*
* Parameters:
*  void * context: The pointer to the CapSense context structure
*
*******************************************************************************/
void tuner_send_callback(void * context)
{
    /* To remove compiler warning  */
    (void)context;
    cy_en_ble_api_result_t api_result = CY_BLE_SUCCESS;
    uint8_t *ptr_capsense = NULL;
    uint8_t notification_count = 0;

    /* Remaining bytes that is less than NOTIFICATION_PKT_SIZE */
    uint16_t remaining_bytes = 0;

    /* Variable to keep track of CapSense structure current index */
    uint8_t index = 0;

    ptr_capsense = (uint8_t *)&cy_capsense_tuner;
    notification_count = count;
    remaining_bytes = capsense_ds_size % NOTIFICATION_PKT_SIZE;

    /* Cy_Ble_ProcessEvents() allows BLE stack to process pending events */
    Cy_BLE_ProcessEvents();

    if(ble_notification_enabled == true)
    {
        while(notification_count > 0u)
        {
            /* Allows BLE stack to process pending events */
            Cy_BLE_ProcessEvents();

            if(ble_disconnected == true)
            {
                /* BLE disconnect event occurred. Exit from while loop */
                break;
            }

            /* Check if BLE stack is free */
            if(Cy_BLE_GATT_GetBusyStatus(appConnHandle.attId) == CY_BLE_STACK_STATE_FREE)
            {
                if(notification_count > 1)
                {
                    notificationPacket.handleValPair.value.len = NOTIFICATION_PKT_SIZE;
                }
                else
                {
                    notificationPacket.handleValPair.value.len = remaining_bytes;
                }

                /* Update the notification packet with CapSense Tuner structure */
                notificationPacket.handleValPair.value.val =\
                        (ptr_capsense + (index * NOTIFICATION_PKT_SIZE));

                /* Send notification to GATT Client */
                api_result = Cy_BLE_GATTS_Notification(&notificationPacket);

                if(api_result == CY_BLE_SUCCESS)
                {
                    notification_count--;
                    index++;
                }
            }
        }
    }
}


/* [] END OF FILE */
