#include "Radio.h"
#include "FreqLUT.h"
#include <SimpleKalmanFilter.h>
#define RESTRICT_PITCH
#include <EEPROM.h>

SimpleKalmanFilter KalmanFilter(1, 1, 0.01); 

#define IS_MASTER 1U /// MASTER = 1 , SLAVE = 0

#define TX_OUTPUT_POWER                             13 // dBm
#define RX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_1000_US
#define RX_TIMEOUT_VALUE                            1000 // ms
#define TX_TIMEOUT_VALUE                             10000 // ms
#define BUFFER_SIZE                                 255

#define Label 1    // debug label

typedef unsigned char uchar;

const uint32_t rangingAddress[] = {
  0x10000000,
  0x32100000,
  0x20012301,
  0x20000abc,
  0x32101230
};
#define RANGING_ADDRESS_SIZE 5

/*!
   \brief Ranging raw factors
                                    SF5     SF6     SF7     SF8     SF9     SF10
*/

const uint16_t RNG_CALIB_0400[] = { 10299,  10271,  10244,  10242,  10230,  10246  };
const uint16_t RNG_CALIB_0800[] = { 11486,  11474,  11453,  11426,  11417,  11401  };
const uint16_t RNG_CALIB_1600[] = { 13308,  13493,  13528,  13515,  13430,  13376  };
const double   RNG_FGRAD_0400[] = { -0.148, -0.214, -0.419, -0.853, -1.686, -3.423 };
const double   RNG_FGRAD_0800[] = { -0.041, -0.811, -0.218, -0.429, -0.853, -1.737 };
const double   RNG_FGRAD_1600[] = { 0.103,  -0.041, -0.101, -0.211, -0.424, -0.87  };

const char* IrqRangingCodeName[] = {
  "IRQ_RANGING_SLAVE_ERROR_CODE",
  "IRQ_RANGING_SLAVE_VALID_CODE",
  "IRQ_RANGING_MASTER_ERROR_CODE",
  "IRQ_RANGING_MASTER_VALID_CODE",
  "IRQ_RANGING_REQUEST_VALID_CODE",
  "IRQ_RANGING_SLAVE_RESPONE_CODE"
};

typedef enum
{
  APP_IDLE,
  APP_RX,
  APP_RX_TIMEOUT,
  APP_RX_ERROR,
  APP_TX,
  APP_TX_TIMEOUT,
  APP_RX_SYNC_WORD,
  APP_RX_HEADER,
  APP_RANGING,
  APP_CAD,
  APP_LOWPOWER
} AppStates_t;

void txDoneIRQ( void );
void rxDoneIRQ( void );
void rxSyncWordDoneIRQ( void );
void rxHeaderDoneIRQ( void );
void txTimeoutIRQ( void );
void rxTimeoutIRQ( void );
void rxErrorIRQ( IrqErrorCode_t errCode );
void rangingDoneIRQ( IrqRangingCode_t val );
void cadDoneIRQ( bool cadFlag );
void handleRangingContinueous();
void configPacketType( RadioPacketTypes_t packetType);
void handleRxLoRa();
void handleTxLoRa();
void filterKalman(double rangingResult, int options);
void noFilterKalman(double rangingResult);
void writeEEPROM(int addr, float value);
double readEEPROM(int addr);
RadioCallbacks_t Callbacks = {
  txDoneIRQ,
  rxDoneIRQ,
  rxSyncWordDoneIRQ,
  rxHeaderDoneIRQ,
  txTimeoutIRQ,
  rxTimeoutIRQ,
  rxErrorIRQ,
  rangingDoneIRQ,
  cadDoneIRQ
};

extern const Radio_t Radio;

uint16_t masterIrqMask = IRQ_RANGING_MASTER_RESULT_VALID | IRQ_RANGING_MASTER_TIMEOUT;
uint16_t slaveIrqMask = IRQ_RANGING_SLAVE_RESPONSE_DONE | IRQ_RANGING_SLAVE_REQUEST_DISCARDED;

PacketParams_t packetParams;
PacketStatus_t packetStatus;
ModulationParams_t modulationParams;

AppStates_t AppState = APP_IDLE;
IrqRangingCode_t IrqRangingCode = IRQ_RANGING_MASTER_ERROR_CODE;


uint8_t Buffer[BUFFER_SIZE];
uint8_t BufferSize = BUFFER_SIZE;
uint8_t SendPackage = 111;
int Address = 0;
double SumNoFilter = 0;
double SumFilter = 0;

void setup() {
  Serial.begin(9600);
  if (IS_MASTER)
  {
    Serial.println("SX1280 MASTER");
  }
  else
  {
    Serial.println("SX1280 SLAVE");
  }
  Radio.Init(&Callbacks);
  Radio.SetRegulatorMode( USE_LDO ); // Can also be set in LDO mode but consume more power
  Serial.println( "\n\n\r     SX1280 Ranging Demo Application. \n\n\r");

  modulationParams.PacketType = PACKET_TYPE_RANGING;
  modulationParams.Params.LoRa.SpreadingFactor = LORA_SF10;
  modulationParams.Params.LoRa.Bandwidth = LORA_BW_1600;
  modulationParams.Params.LoRa.CodingRate = LORA_CR_LI_4_5;

  packetParams.PacketType = PACKET_TYPE_RANGING;
  packetParams.Params.LoRa.PreambleLength = 12;
  packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
  packetParams.Params.LoRa.PayloadLength = 1;
  packetParams.Params.LoRa.Crc = LORA_CRC_ON;
  packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

  Radio.SetStandby( STDBY_RC );
  Radio.SetPacketType( modulationParams.PacketType );
  Radio.SetModulationParams( &modulationParams );
  Radio.SetPacketParams( &packetParams );
  Radio.SetRfFrequency( Channels[1] );
  Radio.SetTxParams( TX_OUTPUT_POWER, RADIO_RAMP_20_US );
  Radio.SetBufferBaseAddresses( 0x00, 0x00 );
  Radio.SetRangingCalibration( RNG_CALIB_1600[5] ); // Bandwith 1600, SF10
  Radio.SetInterruptMode();

  if (IS_MASTER)
  {
    Radio.SetRangingRequestAddress(rangingAddress[1]);
    Radio.SetDioIrqParams( masterIrqMask, masterIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    Radio.SetTx((TickTime_t) {
      RADIO_TICK_SIZE_1000_US, 0xFFFF
    });
  }
  else // SLAVE
  {
    Radio.SetRangingIdLength(RANGING_IDCHECK_LENGTH_32_BITS);
    Radio.SetDeviceRangingAddress(rangingAddress[1]);
    Radio.SetDioIrqParams( slaveIrqMask, slaveIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    Radio.SetRx((TickTime_t) {
      RADIO_TICK_SIZE_1000_US, 0xFFFF
    });
  }

  AppState = APP_IDLE;
}

void loop() {
  handleRangingContinueous();
}

void configPacketType( RadioPacketTypes_t packetType) {
  modulationParams.PacketType = packetType;
}

void handleTxLoRa() {
  Radio.SendPayload( &SendPackage, 1, ( TickTime_t ) {
    RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE
  }, 0 );
  delay(1000);
}
void handleRxLoRa() {
  switch (AppState)
  {
    case APP_LOWPOWER:
      break;
    case APP_RX:
      AppState = APP_LOWPOWER;

      Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
      if (BufferSize > 0)
      {
        Serial.print("RX ");
        for (int i = 0; i < BufferSize; i++)
        {
          Serial.println(Buffer[i]);
        }
      }

      Radio.SetRx( ( TickTime_t ) {
        RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE
      }  );
      break;
    case APP_RX_TIMEOUT:
      AppState = APP_LOWPOWER;

      Serial.println("Timeout");
      Radio.SetRx( ( TickTime_t ) {
        RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE
      }  );
      break;
    case APP_RX_ERROR:
      AppState = APP_LOWPOWER;
      break;
    case APP_TX:
      AppState = APP_LOWPOWER;
      break;
    case APP_TX_TIMEOUT:
      AppState = APP_LOWPOWER;
      break;
    default:
      AppState = APP_LOWPOWER;
      break;
  }
}
void handleRangingContinueous() {
  switch (AppState)
  {
    case APP_IDLE:
      break;
    case APP_RX:
      AppState = APP_IDLE;
      // Serial.println("APP_RX");
      break;
    case APP_RX_TIMEOUT:
      AppState = APP_IDLE;
      // Serial.println("APP_RX_TIMEOUT");
      break;
    case APP_RX_ERROR:
      AppState = APP_IDLE;
      // Serial.println("APP_RX_ERROR");
      break;
    case APP_TX:
      AppState = APP_IDLE;
      // Serial.println("APP_TX");
      break;
    case APP_TX_TIMEOUT:
      AppState = APP_IDLE;
      // Serial.println("APP_TX_TIMEOUT");
      break;
    case APP_RX_SYNC_WORD:
      AppState = APP_IDLE;
      // Serial.println("APP_RX_SYNC_WORD");
      break;
    case APP_RX_HEADER:
      AppState = APP_IDLE;
      // Serial.println("APP_RX_HEADER");
      break;
    case APP_RANGING:
      AppState = APP_IDLE;
      //Serial.println(IrqRangingCode);
      if (IS_MASTER)
      {
        switch (IrqRangingCode)
        {
          case IRQ_RANGING_MASTER_VALID_CODE:
            uint8_t reg[3];
            Radio.ReadRegister(REG_LR_RANGINGRESULTBASEADDR, &reg[0], 1);
            Radio.ReadRegister(REG_LR_RANGINGRESULTBASEADDR + 1, &reg[1], 1);
            Radio.ReadRegister(REG_LR_RANGINGRESULTBASEADDR + 2, &reg[2], 1);
            double rangingResult = Radio.GetRangingResult(RANGING_RESULT_RAW);
            filterKalman(rangingResult, 0); // option 0 mean, 1 median
            //noFilterKalman(rangingResult);
            
            break;
          case IRQ_RANGING_MASTER_ERROR_CODE:
            Serial.println("Raging Error");
            break;
          default:
            break;
        }
        Radio.SetTx((TickTime_t) {
          RADIO_TICK_SIZE_1000_US, 0xFFFF
        });
      } else {
        switch (IrqRangingCode)
        {
          case IRQ_RANGING_SLAVE_ERROR_CODE:
            Serial.println("SLAVE ERR");
            break;
          case IRQ_RANGING_REQUEST_VALID_CODE:
            Serial.println("Request");
            break;
          case IRQ_RANGING_SLAVE_RESPONE_CODE:
            Serial.println("Respone");
            break;
          default:
            Serial.println("SLAVE");
            break;
        }
      }
      break;
    case APP_CAD:
      AppState = APP_IDLE;
      Serial.println("APP_CAD");
      break;
    default:
      AppState = APP_IDLE;
      break;
  }
}

void txDoneIRQ( void )
{
  AppState = APP_TX;
}

void rxDoneIRQ( void )
{
  AppState = APP_RX;
}

void rxSyncWordDoneIRQ( void )
{
  AppState = APP_RX_SYNC_WORD;
  Serial.println("sync word");
}

void rxHeaderDoneIRQ( void )
{
  AppState = APP_RX_HEADER;
  Serial.println("rx header");
}

void txTimeoutIRQ( void )
{
  AppState = APP_TX_TIMEOUT;
  Serial.println("tx timeout");
}

void rxTimeoutIRQ( void )
{
  AppState = APP_RX_TIMEOUT;
  Serial.println("rx timeout");
}

void rxErrorIRQ( IrqErrorCode_t errCode )
{
  AppState = APP_RX_ERROR;
}

void rangingDoneIRQ(IrqRangingCode_t val )
{
  AppState = APP_RANGING;
  //  Serial.println("aaa");
  IrqRangingCode = val;
}

void cadDoneIRQ( bool cadFlag )
{
  AppState = APP_CAD;
}
/*=====================User Functions=====================*/
uint32_t FloatToUint(float n)
{
  return (uint32_t)(*(uint32_t*)&n);
}

float UintToFloat(uint32_t n)
{
  return (float)(*(float*)&n);
}

void filterKalman(double rangingResult, int options = 0)
{
  //Serial.println(rangingResult);
  double kalmanFilter = KalmanFilter.updateEstimate(rangingResult);
  if (kalmanFilter < 0 || rangingResult < 0) {
    return;
  }
  else {
    //EEPROM.write(Address, kalmanFilter);
    if(options == 1 ){
      writeEEPROM(Address , kalmanFilter);
    }
    SumFilter += kalmanFilter;
    delay(5);
    if (Address < 400) {
#ifdef Label
      Serial.print('.');
      //Serial.println(rangingResult);
#endif
      Address += 4;
    }
    else {
#ifdef Label
      Serial.println();
#endif
      if (options)
      {
#ifdef Label
        Serial.print("Median of Result : ");
#endif
        Serial.println(readEEPROM(Address / 2));
      }
      else {
        double resultAverage = (double)SumFilter / (100.0);
        SumFilter = 0;
#ifdef Label
        Serial.print("Mean of Result Filter: ");
#endif
        Serial.println(resultAverage);
      }
      Address = 0;
    }
  }
}
void noFilterKalman(double rangingResult)
{
  if (rangingResult < 0)
    return;
  else {
    SumNoFilter += rangingResult;
    if (Address < 400)
    {
#ifdef Label
      Serial.print('.');
      //Serial.println(rangingResult);
#endif
      Address = Address + 4;
    }
    else {
      Serial.println();
#if Label
      Serial.print("Mean of Result NoFilter: ");
#endif
      double reasultNoFilter = SumNoFilter / 100.0;
      SumNoFilter = 0;
      Serial.println(reasultNoFilter);
      Address = 0;
    }
  }
}

void writeEEPROM(int addr, double value)
{
  //Serial.print(sizeof(value));
  //double b;
  byte bytes[4];
  *((double *)bytes) = value;
  EEPROM.write(addr , bytes[0]);
  delay(5);
  EEPROM.write(addr + 1, bytes[1]);
  delay(5);
  EEPROM.write(addr + 2 , bytes[2]);
  delay(5);
  EEPROM.write(addr + 3 , bytes[3]);
  delay(5);
}

double readEEPROM(int addr)
{
  byte bytes[4];
  bytes[0] = EEPROM.read(addr);
  delay(5);
  bytes[1] = EEPROM.read(addr + 1);
  delay(5);
  bytes[2] = EEPROM.read(addr + 2);
  delay(5);
  bytes[3] = EEPROM.read(addr + 3);
  delay(5);
  double f;
  memcpy(&f , bytes, sizeof(f));
  delay(5);
  return f;
}
