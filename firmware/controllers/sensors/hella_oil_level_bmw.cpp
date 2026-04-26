#include "pch.h"
#include "hella_oil_level_bmw.h"
#include "digital_input_exti.h"

#if EFI_HELLA_OIL_BMW

static int cb_num = 0;
static float lastRise_ms = 0;
static float lastTempStart_ms = 0;  // время начала последнего TEMP импульса

static StoredValueSensor levelSensor(SensorType::HellaOilLevel, MS2NT(2000));
static StoredValueSensor tempSensor(SensorType::HellaOilTemperature, MS2NT(2000));
static StoredValueSensor rawLevelSensor(SensorType::HellaOilLevelRawPulse, MS2NT(2000));
static StoredValueSensor rawTempSensor(SensorType::HellaOilTempRawPulse, MS2NT(2000));

#if EFI_PROD_CODE
static Gpio hellaPin = Gpio::Unassigned;

static void hellaOilCallback(efitick_t nowNt, bool value) {
    cb_num++;
    // nowNt в наносекундах → переводим в миллисекунды
    float t_ms = NT2US(nowNt) / 1000.0f;

    if (value) {  // RISE
        lastRise_ms = t_ms;
    } else {  // FALL
        float width_ms = t_ms - lastRise_ms;
        efiPrintf("CB #%d FALL @ %.3f ms, HIGH width=%.3f ms", cb_num, t_ms, width_ms);

        // Конфиг хранит пороги в us → переводим в ms для сравнения
        float minTempMs = engineConfiguration->hellaOilLevel.minPulseUsTemp / 1000.0f;
        float maxTempMs = engineConfiguration->hellaOilLevel.maxPulseUsTemp / 1000.0f;
        float minLevelMs = engineConfiguration->hellaOilLevel.minPulseUsLevel / 1000.0f;
        float maxLevelMs = engineConfiguration->hellaOilLevel.maxPulseUsLevel / 1000.0f;

        // TEMP = ширина импульса (обычно 10..130 мс)
        if (width_ms >= 10.0f && width_ms <= 130.0f) {
            float temp = interpolateClamped(
                minTempMs, engineConfiguration->hellaOilLevel.minTempC,
                maxTempMs, engineConfiguration->hellaOilLevel.maxTempC,
                width_ms
            );
            efiPrintf("  TEMP: width=%.3f ms -> temp=%.3f C", width_ms, temp);

            tempSensor.setValidValue(temp, nowNt);
            rawTempSensor.setValidValue(width_ms, nowNt);

            // УРОВЕНЬ МАСЛА = интервал между началами TEMP импульсов
            if (lastTempStart_ms > 0) {
                float levelTime_ms = lastRise_ms - lastTempStart_ms;

                float level = interpolateClamped(
                    minLevelMs, engineConfiguration->hellaOilLevel.minLevelMm,
                    maxLevelMs, engineConfiguration->hellaOilLevel.maxLevelMm,
                    levelTime_ms
                );
                efiPrintf("  LEVEL: interval=%.3f ms -> level=%.3f mm", levelTime_ms, level);

                levelSensor.setValidValue(level, nowNt);
                rawLevelSensor.setValidValue(levelTime_ms, nowNt);
            }

            lastTempStart_ms = lastRise_ms;
        }
        // DIAG и другие импульсы
        else if (width_ms >= 35.0f && width_ms <= 45.0f) {
            efiPrintf("  DIAG: width=%.3f ms", width_ms);
        } else if (width_ms >= 155.0f && width_ms <= 165.0f) {
            efiPrintf("  DATA: width=%.3f ms", width_ms);
        } else {
            efiPrintf("  Unknown pulse: width=%.3f ms", width_ms);
        }
    }
}

static void hellaExtiCallback(void*, efitick_t nowNt) {
    hellaOilCallback(nowNt, efiReadPin(hellaPin) ^ engineConfiguration->hellaOilLevelInverted);
}
#endif // EFI_PROD_CODE


void initHellaOilLevelSensor(bool isFirstTime) {
    efiPrintf("HELLA INIT isFirstTime=%d", isFirstTime);

#if EFI_PROD_CODE
    if (!isBrainPinValid(engineConfiguration->hellaOilLevelPin)) {
        efiPrintf("HELLA ERROR: pin not valid: %s", hwPortname(engineConfiguration->hellaOilLevelPin));
        return;
    }

    if (efiExtiEnablePin("hellaOil", engineConfiguration->hellaOilLevelPin,
                         PAL_EVENT_MODE_BOTH_EDGES, hellaExtiCallback, nullptr) < 0) {
        efiPrintf("HELLA ERROR: failed to enable EXTI");
        return;
    }

    hellaPin = engineConfiguration->hellaOilLevelPin;
    efiPrintf("HELLA: EXTI enabled on %s", hwPortname(hellaPin));

    if (isFirstTime) {
        addConsoleAction("hellainfo", []() {
            // Читаем напрямую из Sensor — единый источник данных для TS и CAN
            auto level    = Sensor::get(SensorType::HellaOilLevel);
            auto temp     = Sensor::get(SensorType::HellaOilTemperature);
            auto rawLevel = Sensor::get(SensorType::HellaOilLevelRawPulse);
            auto rawTemp  = Sensor::get(SensorType::HellaOilTempRawPulse);
            efiPrintf("HellaOil Level=%.1fmm[%s] Temp=%.1fC[%s] RawLevel=%.1fms[%s] RawTemp=%.1fms[%s]",
                      level.value_or(0),    level.Valid    ? "OK" : "NO",
                      temp.value_or(0),     temp.Valid     ? "OK" : "NO",
                      rawLevel.value_or(0), rawLevel.Valid ? "OK" : "NO",
                      rawTemp.value_or(0),  rawTemp.Valid  ? "OK" : "NO");
        });
    }
#endif // EFI_PROD_CODE

    levelSensor.Register();
    tempSensor.Register();
    rawLevelSensor.Register();
    rawTempSensor.Register();
    efiPrintf("HELLA: sensors registered");
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

#else // !EFI_HELLA_OIL_BMW

void initHellaOilLevelSensor(bool /*isFirstTime*/) {}
void deInitHellaOilLevelSensor() {}

#endif // EFI_HELLA_OIL_BMW
