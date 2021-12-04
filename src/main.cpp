/**
 ******************************************************************************
 * @file    main.cpp
 * @author  Professor Kleber Lima da Silva <kleber.lima@sp.senai.br
 * @version V0.1.0
 * @date    01-Dez-2021
 * @brief   Code for SAEP 2021.
 ******************************************************************************
 */

/* Includes -----------------------------------------------------------------*/
#include <Arduino.h>
#include <ModbusMaster.h>
#include <WiFi.h>

/* Defines ------------------------------------------------------------------*/
#define ENABLE_MODBUS   1   /* 1 to enable Modbus */
#define ENABLE_PWM      1   /* 1 to enable PWM output */
#define ENABLE_MOTORS   1   /* 1 to enable Stepper Motors */

/* Constants ----------------------------------------------------------------*/
const uint8_t AO_CHANNEL = 0;       /* Analog Output PWM Channel */
const uint8_t CLP_ID = 3;           /* CLP slave Modbus ID */
const uint16_t CLP_REGISTER = 100;  /* CLP register number - %MW100 */
const uint16_t STEP_MIN = 12;       /* Minimum Step delay at 100% potentiometer */
const uint16_t STEP_MAX = 125;      /* Maximum Step delay at 0% potentiometer */
const int STEPS_PER_REV = 48;       /* Number of steps per revolution */
const char* SSID = "ESP32_SAEP_2021";
const char* PASS = "saep2021";

/* Macros -------------------------------------------------------------------*/
#define PWM_OUTPUT(DUTY_CYCLE)  ( ledcWrite(AO_CHANNEL, (uint32_t)(DUTY_CYCLE * 255) / 100) )

/* Pin numbers --------------------------------------------------------------*/
const uint8_t MP1_A = 32;       /* Stepper Motor 1 - Signal A */
const uint8_t MP1_B = 33;       /* Stepper Motor 1 - Signal B */
const uint8_t MP1_C = 25;       /* Stepper Motor 1 - Signal C */
const uint8_t MP1_D = 26;       /* Stepper Motor 1 - Signal D */
const uint8_t MP2_A = 19;       /* Stepper Motor 2 - Signal A */
const uint8_t MP2_B = 21;       /* Stepper Motor 2 - Signal B */
const uint8_t MP2_C = 22;       /* Stepper Motor 2 - Signal C */
const uint8_t MP2_D = 23;       /* Stepper Motor 2 - Signal D */
const uint8_t POT = 36;         /* Potentiometer Pin */
const uint8_t AO_PWM = 18;      /* Analog Output PWM Pin */
const uint8_t RS485_DIR = 4;    /* MAX485 DE Pin */

/* Private variables --------------------------------------------------------*/
ModbusMaster clpSlave;          /* Modbus driver CLP as slave device*/
WiFiServer server(80);          /* ESP32 web server page */
uint16_t stepDelay = STEP_MIN;  /* Step delay (STEP_MIN to STEP_MAX)*/
float motorsSpeed = 0;          /* Stepper Motors speed (rpm) */
uint8_t pwmAO = 0;              /* Analog Output pwm value */
String header;                  /* Header of the web page */
String motorsSpeedString = String(6);   /* Stepper Motors speed to String */
String pwmAOString = String(6);         /* Analog Output pwm to String */

/* Private functions (prototypes) -------------------------------------------*/
void vTaskStepperMotors(void *arg); /* Task - Stepper Motors */
void vTaskModbusPWM(void *arg);     /* Task - Modbus PWM */
void vTaskPrints(void *arg);        /* Task - Prints */

/* Modbus RS485 Callbacks ---------------------------------------------------*/
void RS485_TxCallback()
{
    digitalWrite(RS485_DIR, HIGH);
}
void RS485_RxCallback()
{
    digitalWrite(RS485_DIR, LOW);
}


/* Setup --------------------------------------------------------------------*/
void setup()
{
    /* Pins initializations */
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(MP1_A, OUTPUT);
    pinMode(MP1_B, OUTPUT);
    pinMode(MP1_C, OUTPUT);
    pinMode(MP1_D, OUTPUT);
    pinMode(MP2_A, OUTPUT);
    pinMode(MP2_B, OUTPUT);
    pinMode(MP2_C, OUTPUT);
    pinMode(MP2_D, OUTPUT);
    pinMode(AO_PWM, OUTPUT);
    pinMode(RS485_DIR, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(AO_PWM, LOW);
    digitalWrite(RS485_DIR, LOW);

    /* Serial debug initialization */
    Serial.begin(9600);
    while (!Serial);
    Serial.println("SAEP 2021 - ESP32 | SENAI 9.14");

    /* Connect to Wi-Fi network with SSID and password */
    Serial.print("Setting AP (Access Point)...");
    WiFi.softAP(SSID, PASS);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    server.begin();

    /* Start the Modbus RTU master */
#if ENABLE_MODBUS == 1
    Serial2.begin(9600);
    while (!Serial2);

    /* Modbus slave ID */
    clpSlave.begin(CLP_ID, Serial2);

    /* Interface driver callbacks */
    clpSlave.preTransmission(RS485_TxCallback);
    clpSlave.postTransmission(RS485_RxCallback);
#endif

    /* Configure and start the PWM */
    ledcSetup(AO_CHANNEL, 500, 8);
    ledcAttachPin(AO_PWM, AO_CHANNEL);
    ledcWrite(AO_CHANNEL, 0);

    /* Configure multi tasks */
    xTaskCreatePinnedToCore(vTaskStepperMotors, "vTaskStepperMotors", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskModbusPWM, "vTaskModbusPWM", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vTaskPrints, "vTaskPrints", 2048, NULL, 0, NULL, 0);

    /* End of initializations */
    digitalWrite(LED_BUILTIN, LOW);
}


/* Main Loop ----------------------------------------------------------------*/
void loop()
{
    /* WEB-SERVER */
    WiFiClient client = server.available();

    /* If a new client connects... */
    if (client)
    {
        Serial.println("New Client.");
        String currentLine = "";

        /* Loop while the client's connected */
        while (client.connected())
        {
            if (client.available())
            {
                char c = client.read();
                header += c;
                if (c == '\n')
                {
                    if (currentLine.length() == 0)
                    {
                        /* HTTP header */
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-type:text/html");
                        client.println("Connection: close");
                        client.println();

                        /* Display the HTML web page */
                        client.println("<!DOCTYPE html><html>");
                        client.println("<head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
                        client.println("<link rel=\"icon\" href=\"data:,\">");
                        /* CSS to style page */
                        client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
                        client.println("</style></head>");

                        /* Web Page Body */
                        client.println("<body><h1>Web Server - SAEP 2021</h1>");
                        client.println("<br>");

                        motorsSpeedString = String(motorsSpeed);
                        pwmAOString = String(pwmAO);
                        client.println("<h2>Motores de Passo: " + motorsSpeedString + " rpm</h2>");
                        client.println("<h2>Saída Analógica: " + pwmAOString + " %</h2>");

                        client.println("<br>");
                        client.println("</body></html>");
                        client.println();
                        /* Break out of the while loop */
                        break;
                    }
                    else
                    {   /* If you got a newline, then clear currentLine */
                        currentLine = "";
                    }
                }
                else if (c != '\r')
                {   /* If you got anything else but a carriage return character, */
                    currentLine += c; /* add it to the end of the currentLine */
                }
            }
        }
        /* Clear the header variable and close the connection */
        header = "";
        client.stop();
        Serial.println("Client disconnected.");
        Serial.println("");
    }
} // end of main loop


/* Task - Stepper Motors - Speed Control ------------------------------------*/
void vTaskStepperMotors(void *arg)
{
    uint8_t step = 0;
    uint16_t potADC = 0;

    /* Task LOOP */
    while (1)
    {
        /* Stepper Motors Control */
#if ENABLE_MOTORS == 1
        /* Read the potentiometer value and map speed range */
        potADC = analogRead(POT);
        stepDelay = map(potADC, 0, 4095, STEP_MIN, STEP_MAX);
        motorsSpeed = (60000.0f / STEPS_PER_REV) / stepDelay;

        /* Moves the motors */
        if (potADC < 4095)
        {
            switch (step)
            {
            case 0:
                step++;
                digitalWrite(MP1_A, HIGH);
                digitalWrite(MP1_B, LOW);
                digitalWrite(MP1_C, HIGH);
                digitalWrite(MP1_D, LOW);
                digitalWrite(MP2_A, LOW);
                digitalWrite(MP2_B, LOW);
                digitalWrite(MP2_C, HIGH);
                digitalWrite(MP2_D, HIGH);
                break;
            case 1:
                step++;
                digitalWrite(MP1_A, LOW);
                digitalWrite(MP1_B, HIGH);
                digitalWrite(MP1_C, HIGH);
                digitalWrite(MP1_D, LOW);
                digitalWrite(MP2_A, LOW);
                digitalWrite(MP2_B, HIGH);
                digitalWrite(MP2_C, LOW);
                digitalWrite(MP2_D, HIGH);
                break;
            case 2:
                step++;
                digitalWrite(MP1_A, LOW);
                digitalWrite(MP1_B, HIGH);
                digitalWrite(MP1_C, LOW);
                digitalWrite(MP1_D, HIGH);
                digitalWrite(MP2_A, HIGH);
                digitalWrite(MP2_B, HIGH);
                digitalWrite(MP2_C, LOW);
                digitalWrite(MP2_D, LOW);
                break;
            case 3:
                step = 0;
                digitalWrite(MP1_A, HIGH);
                digitalWrite(MP1_B, LOW);
                digitalWrite(MP1_C, LOW);
                digitalWrite(MP1_D, HIGH);
                digitalWrite(MP2_A, HIGH);
                digitalWrite(MP2_B, LOW);
                digitalWrite(MP2_C, HIGH);
                digitalWrite(MP2_D, LOW);
                break;
            }
        }
        else
        {
            /* Stop motors */
            motorsSpeed = 0;
        }
#endif

        /* Task loop interval (step delay) */
        vTaskDelay(stepDelay);
    }
} // end of vTaskStepperMotors


/* Task - Modbus PWM - Get register and Set Analog Output -------------------*/
void vTaskModbusPWM(void *arg)
{
    uint8_t modbus_result;

    /* Task LOOP */
    while (1)
    {
        /* Read Modbus PWM register */
#if ENABLE_MODBUS == 1
        digitalWrite(LED_BUILTIN, HIGH);
        modbus_result = clpSlave.readHoldingRegisters(CLP_REGISTER, 1);
        if (modbus_result == clpSlave.ku8MBSuccess)
        {
            pwmAO = clpSlave.getResponseBuffer(0);
        }
        else
        {
            Serial.print("Error to read registers: ");
            Serial.println(modbus_result, HEX);
        }
        digitalWrite(LED_BUILTIN, LOW);
#endif

        /* Analog Output - PWM */
#if ENABLE_PWM == 1
        PWM_OUTPUT(pwmAO);
#endif

        /* Task loop interval */
        vTaskDelay(1000);
    }
} // end of vTaskModbusPWM


/* Task - Prints - LOGs -----------------------------------------------------*/
void vTaskPrints(void *arg)
{
    /* Task LOOP */
    while (1)
    {
        /* Prints - Stepper Motors */
        Serial.print("Stepper Motors: ");
        Serial.print(motorsSpeed);
        Serial.println(" rpm");

        /* Prints - Analog Output */
        Serial.print("Analog Output: ");
        //pwmAO = random(100); /* Uncomment for testing purposes */
        Serial.print(pwmAO);
        Serial.println(" %");

        Serial.println("");

        /* Task loop interval */
        vTaskDelay(1000);
    }
} // end of vTaskPrints
