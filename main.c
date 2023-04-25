//
//
//// Standard includes
//#include <stdlib.h>
//#include <string.h>
//
//#include "driverlib.h"
//#include "simplelink.h"
//#include "sl_common.h"
//#include "MQTTClient.h"
//#include "msp.h"
//#include "/Users/amandawalker/Desktop/ECE1188 Cyber Physical/Code Com/tirslk_max_1_00_02/inc/Clock.h"
//#include "/Users/amandawalker/Desktop/ECE1188 Cyber Physical/Code Com/tirslk_max_1_00_02/inc/LaunchPad.h"
//#include "/Users/amandawalker/Desktop/ECE1188 Cyber Physical/Code Com/tirslk_max_1_00_02/inc/MotorSimple.h"
//#include "/Users/amandawalker/Desktop/ECE1188 Cyber Physical/Code Com/tirslk_max_1_00_02/inc/Bump.h"
//#include "/Users/amandawalker/Desktop/ECE1188 Cyber Physical/Code Com/tirslk_max_1_00_02/inc/motorDriver.h"
//#include "/Users/amandawalker/Desktop/ECE1188 Cyber Physical/Code Com/tirslk_max_1_00_02/inc/UART0.h"
//
//
//
//#define SSID_NAME       "Amanda"       /* Access point name to connect to. */
//#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2     /* Security type of the Access piont */
//#define PASSKEY         "amandawalker"   /* Password in case of secure AP */
//#define PASSKEY_LEN     pal_Strlen(PASSKEY)  /* Password length in case of secure AP */
//#define MQTT_BROKER_SERVER  "broker.hivemq.com"
//#define SUBSCRIBE_TOPIC "amw286"
//#define PUBLISH_TOPIC "amw"
//// MQTT message buffer size
//#define BUFF_SIZE 32
//#define APPLICATION_VERSION "1.0.0"
//#define MCLK_FREQUENCY 48000000
//#define PWM_PERIOD 255
//#define SL_STOP_TIMEOUT        0xFF
//#define SMALL_BUF           32
//#define MAX_SEND_BUF_SIZE   512
//#define MAX_SEND_RCV_SIZE   1024
///* Application specific status/error codes */
//typedef enum{
//    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
//    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
//    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
//    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
//    STATUS_CODE_MAX = -0xBB8
//}e_AppStatusCodes;
//
//#define min(X,Y) ((X) < (Y) ? (X) : (Y))
//volatile unsigned int S1buttonDebounce = 0;
//volatile unsigned int S2buttonDebounce = 0;
//volatile int publishID = 1;
//unsigned char macAddressVal[SL_MAC_ADDR_LEN];
//unsigned char macAddressLen = SL_MAC_ADDR_LEN;
//char macStr[18];        // Formatted MAC Address String
//char uniqueID[9];       // Unique ID generated from TLV RAND NUM and MAC Address
//Network n;
//Client hMQTTClient;     // MQTT Client
//_u32  g_Status = 0;
//struct{
//    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
//    _u8 SendBuff[MAX_SEND_BUF_SIZE];
//    _u8 HostName[SMALL_BUF];
//    _u8 CityName[SMALL_BUF];
//    _u32 DestinationIP;
//    _i16 SockID;
//}g_AppData;
//static _i32 establishConnectionWithAP();
//static _i32 configureSimpleLinkToDefaultState();
//static _i32 initializeAppVariables();
//static void displayBanner();
//static void messageArrived(MessageData*);
//static void generateUniqueID();
//void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
//{
//    if(pWlanEvent == NULL)
//        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");
//    switch(pWlanEvent->Event)
//    {
//        case SL_WLAN_CONNECT_EVENT:
//        {
//            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
//        }
//        break;
//
//        case SL_WLAN_DISCONNECT_EVENT:
//        {
//            slWlanConnectAsyncResponse_t*  pEventData = NULL;
//            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
//            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);
//            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;
//            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
//            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
//            {
//                CLI_Write(" Device disconnected from the AP on application's request \n\r");
//            }
//            else
//            {
//                CLI_Write(" Device disconnected from the AP on an ERROR..!! \n\r");
//            }
//        }
//        break;
//        default:
//        {
//            CLI_Write(" [WLAN EVENT] Unexpected event \n\r");
//        }
//        break;
//    }
//}
//
//
//void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
//{
//    if(pNetAppEvent == NULL)
//        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");
//    switch(pNetAppEvent->Event)
//    {
//        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
//        {
//            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);
//
//        }
//        break;
//
//        default:
//        {
//            CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
//        }
//        break;
//    }
//}
//
//void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
//                                  SlHttpServerResponse_t *pHttpResponse)
//{
//    CLI_Write(" [HTTP EVENT] Unexpected event \n\r");
//}
//
//void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
//{
//    CLI_Write(" [GENERAL EVENT] \n\r");
//}
//
//void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
//{
//    if(pSock == NULL)
//        CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");
//
//    switch( pSock->Event )
//    {
//        case SL_SOCKET_TX_FAILED_EVENT:
//        {
//
//            switch( pSock->EventData.status )
//            {
//                case SL_ECLOSE:
//                    CLI_Write(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
//                break;
//
//
//                default:
//                    CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
//                break;
//            }
//        }
//        break;
//
//        default:
//            CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
//        break;
//    }
//}
//
//int main(int argc, char** argv)
//{
//
//    // ***************** Initialize ************************
//
//    Bump_Init();
//    motorPWMInit(15000, 0, 0);
//    Clock_Init48MHz();
//    LaunchPad_Init();
//    UART0_Init();
//    LaunchPad_Init();
//
//
//
//    _i32 retVal = -1;
//    retVal = initializeAppVariables();
//    ASSERT_ON_ERROR(retVal);
//    /* Stop WDT and initialize the system-clock of the MCU */
//    stopWDT();
//    initClk();
//    CLI_Configure();
//    displayBanner();
//    retVal = configureSimpleLinkToDefaultState();
//    if(retVal < 0)
//    {
//        if (DEVICE_NOT_IN_STATION_MODE == retVal)
//            CLI_Write(" Failed to configure the device in its default state \n\r");
//        LOOP_FOREVER();
//    }
//
//    CLI_Write(" Device is configured in default state \n\r");
//    retVal = sl_Start(0, 0, 0);
//    if ((retVal < 0) ||
//        (ROLE_STA != retVal) )
//    {
//        CLI_Write(" Failed to start the device \n\r");
//        LOOP_FOREVER();
//    }
//    CLI_Write(" Device started as STATION \n\r");
//    /* Connecting to WLAN AP */
//    retVal = establishConnectionWithAP();
//    if(retVal < 0)
//    {
//        CLI_Write(" Failed to establish connection w/ an AP \n\r");
//        LOOP_FOREVER();
//    }
//    CLI_Write(" Connection established w/ AP and IP is acquired \n\r");
//    // Obtain MAC Address
//    sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);
//    // Print MAC Addres to be formatted string
//    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//            macAddressVal[0], macAddressVal[1], macAddressVal[2], macAddressVal[3], macAddressVal[4], macAddressVal[5]);
//    // Generate 32bit unique ID from TLV Random Number and MAC Address
//    generateUniqueID();
//    int rc = 0;
//    unsigned char buf[100];
//    unsigned char readbuf[100];
//    NewNetwork(&n);
//    rc = ConnectNetwork(&n, MQTT_BROKER_SERVER, 1883);
//    if (rc != 0) {
//        CLI_Write(" Failed to connect to MQTT broker \n\r");
//        LOOP_FOREVER();
//    }
//    CLI_Write(" Connected to MQTT broker \n\r");
//    MQTTClient(&hMQTTClient, &n, 1000, buf, 100, readbuf, 100);
//    MQTTPacket_connectData cdata = MQTTPacket_connectData_initializer;
//    cdata.MQTTVersion = 3;
//    cdata.clientID.cstring = uniqueID;
//    rc = MQTTConnect(&hMQTTClient, &cdata);
//    if (rc != 0) {
//        CLI_Write(" Failed to start MQTT client \n\r");
//        LOOP_FOREVER();
//    }
//    CLI_Write(" Started MQTT client successfully \n\r");
//    rc = MQTTSubscribe(&hMQTTClient, SUBSCRIBE_TOPIC, QOS0, messageArrived);
//    if (rc != 0) {
//        CLI_Write(" Failed to subscribe to /msp/cc3100/demo topic \n\r");
//        LOOP_FOREVER();
//    }
//    CLI_Write(" Subscribed to /msp/cc3100/demo topic \n\r");
//    rc = MQTTSubscribe(&hMQTTClient, uniqueID, QOS0, messageArrived);
//    if (rc != 0) {
//        CLI_Write(" Failed to subscribe to uniqueID topic \n\r");
//        LOOP_FOREVER();
//    }
//    CLI_Write(" Subscribed to uniqueID topic \n\r");
//
//
//   char status;
//
//
//    while(1){
//            rc = MQTTYield(&hMQTTClient, 10);
//            if (rc != 0) {
//                CLI_Write(" MQTT failed to yield \n\r");
//                LOOP_FOREVER();
//            }
//
//            LaunchPad_LED(0x01);
//
//
//           // PWM_LeftMotor(5000);
//           // PWM_RightMotor(5000);
//
//
//            char startval[8] = "start";
//            if (publishID) {
//                                int rc = 0;
//                                MQTTMessage msg;
//                                msg.dup = 0;
//                                msg.id = 0;
//                                msg.payload = startval;
//                                msg.payloadlen = 8;
//                                msg.qos = QOS0;
//                                msg.retained = 0;
//                                rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC, &msg);
//                                if (rc != 0) {
//                                    CLI_Write(" Failed to publish bumpval to MQTT broker \n\r");
//                                    LOOP_FOREVER();
//                                }
//                                CLI_Write(" Published bumpval successfully \n\r");
//                                publishID = 1;
//                        }
//
//            status = UART0_InChar();
//
//            LaunchPad_LED(0x00);
//
//            while(status == 's'){   // 73
//                    PWM_LeftMotor(0);
//                    PWM_RightMotor(0);
//                    char stopval[8] = "stop";
//                    if (publishID) {
//                                        int rc = 0;
//                                        MQTTMessage msg;
//                                        msg.dup = 0;
//                                        msg.id = 0;
//                                        msg.payload = stopval;
//                                        msg.payloadlen = 8;
//                                        msg.qos = QOS0;
//                                        msg.retained = 0;
//                                        rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC, &msg);
//                                        if (rc != 0) {
//                                            CLI_Write(" Failed to publish bumpval to MQTT broker \n\r");
//                                            LOOP_FOREVER();
//                                        }
//                                        CLI_Write(" Published bumpval successfully \n\r");
//                                        publishID = 1;
//                                }
//
//                                Delay(10);
//                    char status1 = UART0_InChar();
//                    if(status1 == 'g')
//                    {
//                        status = status1;
//                    }
//            } // end while status = stop
//
//
//
//
//
//            while(status == 'g'){  //67
//                PWM_LeftMotor(5000);
//                PWM_RightMotor(5000);
//
//
//                uint8_t val;
//                //int val;    //test for sprintf
//                val = Bump_Read();
//                Delay(10);
//
//                char bumpval[8];
//
//               // sprintf(bumpval, "%i", val);
//                char bump1[8] = "CRASH1!!";
//                char bump2[8] = "CRASH2!!";
//                char bump3[8] = "CRASH3!!";
//                char bump4[8] = "CRASH4!!";
//                char bump5[8] = "CRASH5!!";
//
//                    switch(val){
//                    case 0xEC://236 ec
//                        strcpy(bumpval, bump1);
//                        PWM_LeftMotor(0);
//                        PWM_RightMotor(0);
//                        break;
//                    case 0xE9:// 223 e9
//                        strcpy(bumpval, bump2);
//                        PWM_LeftMotor(0);
//                        PWM_RightMotor(0);
//                        break;
//                    case 0xE5://225 e5
//                        strcpy(bumpval, bump3);
//                        PWM_LeftMotor(0);
//                        PWM_RightMotor(0);
//                        break;
//                    case 0xCD: //205 cd
//                        strcpy(bumpval, bump4);
//                        PWM_LeftMotor(0);
//                        PWM_RightMotor(0);
//                        break;
//                    case 0xAD: //173 ad
//                        strcpy(bumpval, bump5);
//                        PWM_LeftMotor(0);
//                        PWM_RightMotor(0);
//                        break;
//                    }
//
//                //char bumpval1[8] = "TEST";
//
//                if (publishID) {
//                        int rc = 0;
//                        MQTTMessage msg;
//                        msg.dup = 0;
//                        msg.id = 0;
//                        msg.payload = bumpval;
//                        msg.payloadlen = 8;
//                        msg.qos = QOS0;
//                        msg.retained = 0;
//                        rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC, &msg);
//                        if (rc != 0) {
//                            CLI_Write(" Failed to publish bumpval to MQTT broker \n\r");
//                            LOOP_FOREVER();
//                        }
//                        CLI_Write(" Published bumpval successfully \n\r");
//                        publishID = 1;
//                }
//
//                Delay(10);
//
//                char status2 = UART0_InChar();
//                if(status2 == 's'){
//                    status = status2;
//                }
//
//
//           } // end of while status = g
//
//        } // end of while loops
//
//}
//
//static void generateUniqueID() {
//    CRC32_setSeed(TLV->RANDOM_NUM_1, CRC32_MODE);
//    CRC32_set32BitData(TLV->RANDOM_NUM_2);
//    CRC32_set32BitData(TLV->RANDOM_NUM_3);
//    CRC32_set32BitData(TLV->RANDOM_NUM_4);
//    int i;
//    for (i = 0; i < 6; i++)
//    CRC32_set8BitData(macAddressVal[i], CRC32_MODE);
//
//    uint32_t crcResult = CRC32_getResult(CRC32_MODE);
//    sprintf(uniqueID, "%06X", crcResult);
//}
//
//
//static void messageArrived(MessageData* data) {
//    return;
//}
//static _i32 configureSimpleLinkToDefaultState()
//{
//    SlVersionFull   ver = {0};
//    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};
//
//    _u8           val = 1;
//    _u8           configOpt = 0;
//    _u8           configLen = 0;
//    _u8           power = 0;
//    _i32          retVal = -1;
//    _i32          mode = -1;
//    mode = sl_Start(0, 0, 0);
//    ASSERT_ON_ERROR(mode);
//    /* If the device is not in station-mode, try configuring it in station-mode */
//    if (ROLE_STA != mode)
//    {
//        if (ROLE_AP == mode)
//        {
//            /* If the device is in AP mode, we need to wait for this event before doing anything */
//            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
//        }
//        /* Switch to STA role and restart */
//        retVal = sl_WlanSetMode(ROLE_STA);
//        ASSERT_ON_ERROR(retVal);
//        retVal = sl_Stop(SL_STOP_TIMEOUT);
//        ASSERT_ON_ERROR(retVal);
//        retVal = sl_Start(0, 0, 0);
//        ASSERT_ON_ERROR(retVal);
//        /* Check if the device is in station again */
//        if (ROLE_STA != retVal)
//        {
//            /* We don't want to proceed if the device is not coming up in station-mode */
//            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
//        }
//    }
//    /* Get the device's version-information */
//    configOpt = SL_DEVICE_GENERAL_VERSION;
//    configLen = sizeof(ver);
//    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
//    ASSERT_ON_ERROR(retVal);
//    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
//    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
//    ASSERT_ON_ERROR(retVal);
//    /* Remove all profiles */
//    retVal = sl_WlanProfileDel(0xFF);
//    ASSERT_ON_ERROR(retVal);
//    retVal = sl_WlanDisconnect();
//    if(0 == retVal)
//    {
//        /* Wait */
//        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
//    }
//    /* Enable DHCP client*/
//    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
//    ASSERT_ON_ERROR(retVal);
//    /* Disable scan */
//    configOpt = SL_SCAN_POLICY(0);
//    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
//    ASSERT_ON_ERROR(retVal);
//    /* Set Tx power level for station mode
//       Number between 0-15, as dB offset from max power - 0 will set maximum power */
//    power = 0;
//    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
//    ASSERT_ON_ERROR(retVal);
//    /* Set PM policy to normal */
//    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
//    ASSERT_ON_ERROR(retVal);
//    /* Unregister mDNS services */
//    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
//    ASSERT_ON_ERROR(retVal);
//    /* Remove  all 64 filters (8*8) */
//    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
//    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
//                       sizeof(_WlanRxFilterOperationCommandBuff_t));
//    ASSERT_ON_ERROR(retVal);
//    retVal = sl_Stop(SL_STOP_TIMEOUT);
//    ASSERT_ON_ERROR(retVal);
//    retVal = initializeAppVariables();
//    ASSERT_ON_ERROR(retVal);
//    return retVal; /* Success */
//}
//
//static _i32 establishConnectionWithAP()
//{
//    SlSecParams_t secParams = {0};
//    _i32 retVal = 0;
//    secParams.Key = PASSKEY;
//    secParams.KeyLen = PASSKEY_LEN;
//    secParams.Type = SEC_TYPE;
//    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
//    ASSERT_ON_ERROR(retVal);
//    /* Wait */
//    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }
//    return SUCCESS;
//}
//static _i32 initializeAppVariables()
//{
//    g_Status = 0;
//    pal_Memset(&g_AppData, 0, sizeof(g_AppData));
//    return SUCCESS;
//}
//static void displayBanner()
//{
//    CLI_Write("\n\r\n\r");
//    CLI_Write(" MQTT Twitter Controlled RGB LED - Version ");
//    CLI_Write(APPLICATION_VERSION);
//    CLI_Write("\n\r*******************************************************************************\n\r");
//}
//
