#include "pch.h"
#include "hella_oil_level_bmw.h"
#include "digital_input_exti.h"  

#if EFI_HELLA_OIL_BMW

static int cb_num = 0;
static float lastRise = 0;
static float lastTempStart = 0;  // время начала последнего TEMP импульса

static StoredValueSensor levelSensor(SensorType::HellaOilLevel, MS2NT(2000));
static StoredValueSensor tempSensor(SensorType::HellaOilTemperature, MS2NT(2000));
static StoredValueSensor rawLevelSensor(SensorType::HellaOilLevelRawPulse, MS2NT(2000));
static StoredValueSensor rawTempSensor(SensorType::HellaOilTempRawPulse, MS2NT(2000));

static Timer pulseTimer, betweenTimer;


static float lastLevelMm = 0.0f;
static float lastTempC = 0.0f;
static uint32_t lastPulseWidthLevelUs = 0;
static uint32_t lastPulseWidthTempUs = 0;
static bool levelValid = false;
static bool tempValid = false;

#if EFI_PROD_CODE
static Gpio hellaPin = Gpio::Unassigned;


static void hellaOilCallback(efitick_t nowNt, bool value) {
    cb_num++;
    float t_ms = nowNt / 1000.0f;

    if (value) {  // RISE
        lastRise = t_ms;
    } 
    else {  // FALL
        float width_ms = t_ms - lastRise;
        efiPrintf("CB #%d FALL @ %.3f ms, HIGH width=%.3f ms", cb_num, t_ms, width_ms);

        // TEMP = ширина импульса
        if (width_ms >= 10.0f && width_ms <= 130.0f) {
            float temp = interpolateClamped(
                engineConfiguration->hellaOilLevel.minPulseUsTemp, 
                engineConfiguration->hellaOilLevel.minTempC,
                engineConfiguration->hellaOilLevel.maxPulseUsTemp, 
                engineConfiguration->hellaOilLevel.maxTempC,
                width_ms
            );
            efiPrintf("  TEMP: width=%.3f ms → temp=%.3f °C", width_ms, temp);
            
            tempValid = true;
            tempSensor.setValidValue(temp, nowNt);
            rawTempSensor.setValidValue(width_ms, nowNt);
            engineConfiguration->hellaOilLevel.tempC = temp;
            
            // УРОВЕНЬ МАСЛА = интервал между TEMP импульсами
            if (lastTempStart > 0) {
                float levelTime = lastRise - lastTempStart;
                
                float level = interpolateClamped(
                    engineConfiguration->hellaOilLevel.minPulseUsLevel,   // 1000 мс
                    engineConfiguration->hellaOilLevel.minLevelMm,         // 0 мм
                    engineConfiguration->hellaOilLevel.maxPulseUsLevel,   // 3500 мс  
                    engineConfiguration->hellaOilLevel.maxLevelMm,         // 100 мм
                    levelTime
                );
                efiPrintf("  LEVEL: interval=%.3f ms → level=%.3f mm", levelTime, level);
                
                levelValid = true;
                levelSensor.setValidValue(level, nowNt);
                rawLevelSensor.setValidValue(levelTime, nowNt);
                engineConfiguration->hellaOilLevel.levelMm = level;
            }
            
            lastTempStart = lastRise;
        }
        // DIAG и другие импульсы
        else if (width_ms >= 35.0f && width_ms <= 45.0f) {
            efiPrintf("  DIAG: width=%.3f ms", width_ms);
        }
        else if (width_ms >= 155.0f && width_ms <= 165.0f) {
            efiPrintf("  DATA: width=%.3f ms", width_ms);
        }
        else {
            efiPrintf("  Unknown pulse: width=%.3f ms", width_ms);
        }
    }
}


static void hellaExtiCallback(void*, efitick_t nowNt) {
    hellaOilCallback(nowNt, efiReadPin(hellaPin) ^ engineConfiguration->hellaOilLevelInverted);
}
#endif // EFI_PROD_CODE


void initHellaOilLevelSensor(bool isFirstTime) {
    efiPrintf("***** HELLA INIT CALLED! isFirstTime=%d *****", isFirstTime);
    efiPrintf("***** EFI_HELLA_OIL_BMW = %d *****", EFI_HELLA_OIL_BMW);

#if EFI_HELLA_OIL_BMW
    efiPrintf("***** HELLA: Starting real initialization *****");

#if EFI_PROD_CODE
    if (!isBrainPinValid(engineConfiguration->hellaOilLevelPin)) {
        efiPrintf("***** HELLA ERROR: Pin not valid: %s *****", hwPortname(engineConfiguration->hellaOilLevelPin));
        return;
    }

    efiPrintf("***** HELLA: Pin is valid: %s *****", hwPortname(engineConfiguration->hellaOilLevelPin));

    if (efiExtiEnablePin("hellaOil", engineConfiguration->hellaOilLevelPin,
                        PAL_EVENT_MODE_BOTH_EDGES, hellaExtiCallback, nullptr) < 0) {
        efiPrintf("***** HELLA ERROR: Failed to enable EXTI! *****");
        return;
    }

    hellaPin = engineConfiguration->hellaOilLevelPin;
    efiPrintf("***** HELLA: EXTI enabled successfully! *****");

    if (isFirstTime) {
        addConsoleAction("hellainfo", []() {
            efiPrintf("HellaOil Level=%.1fmm[%s] Temp=%.1f°C[%s] RawL=%luμs RawT=%luμs",
                      lastLevelMm, levelValid ? "OK" : "NO",
                      lastTempC, tempValid ? "OK" : "NO",
                      (unsigned long)lastPulseWidthLevelUs, 
                      (unsigned long)lastPulseWidthTempUs);
        });
        efiPrintf("***** HELLA: Console command added! *****");
    }
#else
    efiPrintf("***** HELLA: EFI_PROD_CODE is disabled *****");
#endif // EFI_PROD_CODE

    levelSensor.Register();
    tempSensor.Register();
    rawLevelSensor.Register();
    rawTempSensor.Register();
    efiPrintf("***** HELLA: All sensors registered! *****");

#else
    efiPrintf("***** HELLA: EFI_HELLA_OIL_BMW is disabled *****");
#endif // EFI_HELLA_OIL_BMW
}


void deInitHellaOilLevelSensor() {
    levelSensor.unregister();
    tempSensor.unregister();
    rawLevelSensor.unregister();
    rawTempSensor.unregister();
    
#if EFI_PROD_CODE
    if (isBrainPinValid(hellaPin)) {
        efiExtiDisablePin(hellaPin);
    }
    hellaPin = Gpio::Unassigned;
#endif // EFI_PROD_CODE
}

// Функции доступа к данным
float getHellaOilLevelMm() { return lastLevelMm; }
float getHellaOilTempC() { return lastTempC; }
bool isHellaOilLevelValid() { return levelValid; }
bool isHellaOilTempValid() { return tempValid; }
uint32_t getHellaOilLevelRawUs() { return lastPulseWidthLevelUs; }
uint32_t getHellaOilTempRawUs() { return lastPulseWidthTempUs; }

#else
// Заглушки когда EFI_HELLA_OIL_BMW отключен
void initHellaOilLevelSensor(bool /*isFirstTime*/) {}
void deInitHellaOilLevelSensor() {}
float getHellaOilLevelMm() { return 0.0f; }
float getHellaOilTempC() { return 0.0f; }
bool isHellaOilLevelValid() { return false; }
bool isHellaOilTempValid() { return false; }
uint32_t getHellaOilLevelRawUs() { return 0; }
uint32_t getHellaOilTempRawUs() { return 0; }
#endif // EFI_HELLA_OIL_bMW
