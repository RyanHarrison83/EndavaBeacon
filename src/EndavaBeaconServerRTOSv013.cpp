/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>


#define   VERSION       "V0.1.3"

#define   USE_GRAVITY

// Define the LED display task
void taskLED( void *pvParameters );



#define   RGB_LED   8
#define   PIXEL_COUNT   60
#define   DEVICE_NAME   "Endava Beacon"
#define   REFRESH_RATE   60


#define   I2C_SDA_PIN         6
#define   I2C_SCL_PIN         7
#define   I2C_SPEED           400000

#define I2C_DEV_ADDR          0x19

#define AMBIENT_PIN           0

#define TOUCH1_PIN            2
#define TOUCH2_PIN            3

/*
   ACCELEROMETER REGISTERS
*/
#define REG_CARD_ID      0x0F     ///<The chip id
#define REG_CTRL_REG1    0x20     ///<Control register 1
#define REG_CTRL_REG4    0x23     ///<Control register 2
#define REG_CTRL_REG2    0x21     ///<Control register 3
#define REG_CTRL_REG3    0x22     ///<Control register 4
#define REG_CTRL_REG5    0x24     ///<Control register 5
#define REG_CTRL_REG6    0x25     ///<Control register 6
#define REG_CTRL_REG7    0x3F     ///<Control register 7
#define REG_STATUS_REG   0x27     ///<Status register
#define REG_OUT_X_L      0x28     ///<The low order of the X-axis acceleration register
#define REG_OUT_X_H      0x29     ///<The high point of the X-axis acceleration register
#define REG_OUT_Y_L      0x2A     ///<The low order of the Y-axis acceleration register
#define REG_OUT_Y_H      0x2B     ///<The high point of the Y-axis acceleration register
#define REG_OUT_Z_L      0x2C     ///<The low order of the Z-axis acceleration register
#define REG_OUT_Z_H      0x2D     ///<The high point of the Z-axis acceleration register
#define REG_WAKE_UP_DUR  0x35     ///<Wakeup and sleep duration configuration register
#define REG_FREE_FALL    0x36     ///<Free fall event register
#define REG_STATUS_DUP   0x37     ///<Interrupt event status register
#define REG_WAKE_UP_SRC  0x38     ///<Wakeup source register
#define REG_TAP_SRC      0x39     ///<Tap source register
#define REG_SIXD_SRC     0x3A     ///<6D source register
#define REG_ALL_INT_SRC  0x3B     ///<Reading this register, all related interrupt function flags routed to the INT pads are reset simultaneously

#define REG_TAP_THS_X    0x30     ///<4D configuration enable and TAP threshold configuration .
#define REG_TAP_THS_Y    0x31     ///<Threshold for tap recognition @ FS = ±2 g on Y direction
#define REG_TAP_THS_Z    0x32     ///<Threshold for tap recognition @ FS = ±2 g on Z direction
#define REG_INT_DUR      0x33     ///<Interrupt duration register
#define REG_WAKE_UP_THS  0x34     ///<Wakeup threshold register


#define ONE_G           0x0FFF

/**************
   FUNCTION PROTOTYPES
*/
void readRegister (  uint8_t address, uint8_t *regData, uint8_t size );
void writeRegister ( uint8_t address, uint8_t *regData, uint8_t size );
void initAccelerometer ( void );

void  addPixelAngle( int angle, uint32_t colour );

void gravityInit( void );
void gravityShow ( void );
void gravityIterate ( int angle, float magnitude );

uint32_t  RGBsetting = 0;

BLECharacteristic   *pCharacteristic;
BLEDescriptor        Descriptor(BLEUUID((uint16_t)0x2901));

BLEServer *pServer;

bool  deviceConnected = false;
int ledIndex = 0;
//bool fNewData = false;

static SemaphoreHandle_t LED_sem;     // Waits for parameter to be read

Adafruit_NeoPixel pixels(PIXEL_COUNT, RGB_LED, NEO_GRB + NEO_KHZ800);




// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "495f08db-943e-4a0a-9569-086521cfd6ce"
#define CHARACTERISTIC_UUID "56d39cd5-90ef-4179-8610-943377b77466"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
      pServer->updateConnParams(param->connect.remote_bda, 0x06, 0x06, 0, 100);

      pixels.clear();
      pixels.show();

      Serial.println("Device Connected!");
      pServer->getAdvertising()->stop();
      deviceConnected = true;

    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;

      xSemaphoreGiveFromISR(LED_sem, NULL);  // To unblock the led task

      // Restart advertising
      pServer->getAdvertising()->start();
    }
};

class MyCharacteristicsCallbacks: public BLECharacteristicCallbacks {
    void onWrite( BLECharacteristic* pChar )
    {

      xSemaphoreGiveFromISR( LED_sem, NULL );    // semaphoreGive

    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  pinMode(AMBIENT_PIN, INPUT);

  pinMode(TOUCH1_PIN, INPUT);
  pinMode(TOUCH2_PIN, INPUT);

  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU((PIXEL_COUNT * 4) + 10);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_READ
                    );



  Descriptor.setValue("RGB Setting - 24-bit");
  Descriptor.setAccessPermissions(ESP_GATT_PERM_READ);

  pCharacteristic->addDescriptor(&Descriptor);

  pCharacteristic->setValue(RGBsetting);
  pCharacteristic->setCallbacks(new MyCharacteristicsCallbacks());

  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Server started, characteristic can be written to");

  LED_sem = xSemaphoreCreateBinary();

  initAccelerometer();
  gravityInit();

  // Now set up the LED task.
  xTaskCreate(
    taskLED
    ,  "TaskLED"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
}

void loop() {
  // Empty, a task is going to do the work

}

void taskLED (void *pvParameters)
{
  Serial.print("The LED task is running on core ");
  Serial.println(xPortGetCoreID());

  uint32_t currentColour = pixels.Color(0, 0, 190);
  int i = 0;
  int16_t x, y, z;
  uint8_t bDummy;

  while (1)     // A task shall never exit
  {
    // put your main code here, to run repeatedly:
    uint8_t brightness;



    std::string  RGBValue;


    if ( !deviceConnected )
    {



      readRegister(REG_OUT_X_L, (uint8_t*)&x, 2 );
      readRegister(REG_OUT_Y_L, (uint8_t*)&y, 2 );
      readRegister(REG_OUT_Z_L, (uint8_t*)&z, 2 );

      // Data is 14-bits, stored in the upper bits, so shift it over
      x = x >> 2;
      y = y >> 2;
      z = z >> 2;

      float angle = atan2(y, -x);

      // convert to degrees (0 - 360)
      angle = 180 + (angle * 180 / M_PI);

      //     Serial.print("angle is ");
      //     Serial.println(angle);

      float magnitude = sqrt((x * x) + (y * y));



#ifndef   USE_GRAVITY

      if ( magnitude > ONE_G )
      {
        magnitude = ONE_G;
      }

      brightness = (uint8_t)((magnitude * 255) / ONE_G);

      //     Serial.print("Brightness is ");
      //     Serial.println(brightness);

      // This means the device is barely tilted, so don't turn on the LED at all
      if (brightness < 10)
      {
        brightness = 0;
      }

      //     Serial.print("Ambient Reading is ");
      //     Serial.println(analogRead(AMBIENT_PIN));

      //     Serial.print("Touch 1 = ");
      //     Serial.print(digitalRead(TOUCH1_PIN));
      //     Serial.print(", Touch 2 = ");
      //     Serial.println(digitalRead(TOUCH2_PIN));

      pixels.clear();
      addPixelAngle(angle, currentColour);
      pixels.setBrightness(brightness);
      pixels.show();

#else
      gravityIterate(angle, magnitude);
      gravityShow();

#endif



      vTaskDelay(1000 / REFRESH_RATE);


    }
    else
    {
      if ( xSemaphoreTake( LED_sem, portMAX_DELAY ) ) // semaphoreTake
      {


        //fNewData = false;
        int len =  pCharacteristic->getLength();
        if ( len > (PIXEL_COUNT) )
        {
          len = PIXEL_COUNT;
        }

        Serial.println(pCharacteristic->getLength());

        std::string  RGBValue;

        RGBValue = pCharacteristic->getValue();

        pixels.clear();

        for ( int loop = 0; loop < len; loop++ )
        {
          uint32_t  thisByte = *(uint32_t*)&RGBValue[loop * 4];

          pixels.setPixelColor(loop, thisByte);
        }

        pixels.show();

      }
    }

    taskYIELD();    // Give the idle task a chance to do what it does (ie, kick the watchdog)
  }   // END of while(1)

}

/*******************************
   Accelerometer functions
 *******************************/
void readRegister (  uint8_t address, uint8_t *regData, uint8_t size )
{
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(I2C_DEV_ADDR, size);

  for (int x = 0; x < size; x++)
  {
    regData[x] = Wire.read();
  }
}


void writeRegister ( uint8_t address, uint8_t *regData, uint8_t size )
{
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(&address, 1);

  for ( int x = 0; x < size; x++)
  {
    Wire.write(regData[x]);
  }

  Wire.endTransmission();
}

void initAccelerometer ( void )
{
  uint8_t reg;

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, (uint32_t)I2C_SPEED );


  readRegister(REG_CARD_ID, &reg, 1);

  Serial.print("ID is ");
  Serial.println(reg, HEX);

  if ( reg != 0x44 )      // Accelerometer is not correct chip
  {
    Serial.println("Accelerometer problem, please reboot!");
    while (1);
  }

  // Soft Reset the accelerometer
  readRegister(REG_CTRL_REG2, &reg, 1);
  reg = reg | 0b01000000;
  writeRegister(REG_CTRL_REG2, &reg, 1);

  // Set continuous refresh
  readRegister(REG_CTRL_REG2, &reg, 1);
  reg = reg | (1 << 3);         // enabled
  writeRegister(REG_CTRL_REG2, &reg, 1);

  // Set the data collection rate
  readRegister(REG_CTRL_REG1, &reg, 1);
  reg = reg | 0b01010000;       // 100Hz
  writeRegister(REG_CTRL_REG1, &reg, 1);

  // Set up I2C triggered single conversion
  readRegister(REG_CTRL_REG3, &reg, 1);
  reg = reg | 0b00000010;       // enabled
  writeRegister(REG_CTRL_REG3, &reg, 1);

  // Set the sensing range, and filter settings
  reg = reg | 0b11000100;       // ODR/20, +/-2G, Low-pass filter enabled, low-noise config enabled
  writeRegister(REG_CTRL_REG6, &reg, 1);

  // set the power mode
  reg = 0b01010001;       // 100Hz
  writeRegister(REG_CTRL_REG1, &reg, 1);


}


/************************************
   LED Display Functions
 * **********************************/
#define DEGREES_PER_PIXEL   (360/PIXEL_COUNT)

void  addPixelAngle( int angle, uint32_t colour )
{
  // Find out what pixels to turn on for the given angle
  angle = angle % 360;    // Make sure it is less than 360

  int pixel1, pixel2;

  pixel1 = (angle * PIXEL_COUNT) / 360;
  pixel2 = (pixel1 + 1) % PIXEL_COUNT;      // Keep in bounds

  float brightnessRatio = ((float)angle - (DEGREES_PER_PIXEL * (float)pixel1)) / DEGREES_PER_PIXEL;

  int red, green, blue;

  red = (colour >> 16) & 0xFF;
  green = (colour >> 8) & 0xFF;
  blue = colour & 0xFF;

  uint32_t colour1, colour2;
  colour1 = pixels.Color((float)red * (1 - brightnessRatio), (float)green * (1 - brightnessRatio), (float)blue * (1 - brightnessRatio));
  colour2 = pixels.Color((float)red * (brightnessRatio), (float)green * (brightnessRatio), (float)blue * (brightnessRatio));

  pixels.setPixelColor(pixel1, colour1);
  pixels.setPixelColor(pixel2, colour2);
}


/*
   GRAVITY PIXELS
*/

#define   GRAVITY_PIXEL_COUNT     1

#define   ACCEL_DAMPING_FACTOR    10

typedef struct
{
  int       currentAngle;
  int       currentVelocity;

  uint32_t  color;
} gravityPixel_t;

gravityPixel_t  gPixels[GRAVITY_PIXEL_COUNT];

void gravityInit( void )
{
  for ( int x = 0; x < GRAVITY_PIXEL_COUNT; x++ )
  {
    gPixels[x].currentAngle = x * DEGREES_PER_PIXEL;
    gPixels[x].currentVelocity = 0;       // Start with no velocity

    gPixels[x].color = pixels.Color(150, 0, 0);     // Random colour just for testing
  }
}



void gravityShow ( void )
{
  pixels.clear();

  for ( int x = 0; x < GRAVITY_PIXEL_COUNT; x++ )
  {
    addPixelAngle(gPixels[x].currentAngle, gPixels[x].color);
  }

  pixels.show();
}

void gravityIterate ( int angle, float magnitude )
{
  float newAcceleration, newVelocity;

  Serial.printf ("Angle = %d, Magnitude = %f\n", angle, magnitude);

  // Calculate the new position of the particle based on gravity/physics
  for ( int x = 0; x < GRAVITY_PIXEL_COUNT; x++ )
  {
    if ( angle - gPixels[x].currentAngle >= 180 )
    {
      angle -= 360;
    }
    else if ( angle - gPixels[x].currentAngle <= -180 )
    {
      angle += 360;
    }
    
    newAcceleration = (angle - gPixels[x].currentAngle) * magnitude / (float)ACCEL_DAMPING_FACTOR / ONE_G;
    
    gPixels[x].currentVelocity = gPixels[x].currentVelocity + newAcceleration;
    gPixels[x].currentAngle = gPixels[x].currentAngle + gPixels[x].currentVelocity;

    if ( gPixels[x].currentAngle < 0 )
    {
      gPixels[x].currentAngle += 360;
    }
    

    
    


    Serial.printf("New Accel = %f, New Velocity = %d, New Angle = %d\n", newAcceleration, gPixels[x].currentVelocity, gPixels[x].currentAngle);

  }

}
