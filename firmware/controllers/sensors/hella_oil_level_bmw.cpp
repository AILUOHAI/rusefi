#include "pch.h"
#include "hella_oil_level_bmw.h"
#include "digital_input_exti.h"

#if EFI_HELLA_OIL_BMW

/*
 * Протокол датчика Hella Oil Level BMW (по осциллограмме, масштаб 20ms/div):
 *
 *  Синий  (TEMP) :   ┌─T─┐                      ┌─T─┐
 *                  ──┘   └──────────────────────-┘   └──
 *
 *  Жёлтый (DIAG):   ┌──1──┐  ┌────2────┐
 *                  ─┘     └──┘         └─────────────────
 *
 *  Цифра  Имя     Что это           Норма      При ошибке уровня
 *  ─────────────────────────────────────────────────────────────
 *    1    LOW     пауза перед DIAG  ~20 ms     ~10 ms (вместе с 2: мигание)
 *    2    DIAG    HIGH импульс      ~40 ms     ~20 ms (короче в 2 раза)
 *    3    TEMP    HIGH импульс      10..30 ms  не меняется (кодирует температуру)
 *    4    LEVEL   LOW между TEMP    50..200 ms зависит от уровня масла
 *
 *  Период фрейма: 1 + 2 + (несколько TEMP с паузами LEVEL) ≈ 1 секунда
 *
 *  Детектируем тип по ширине HIGH импульса:
 *    TEMP  : 5..35 ms   → кодирует температуру
 *    DIAG  : 36..60 ms  → диагностический (норма ~40ms, ошибка ~20ms — ловим LOW)
 *    DATA  : >100 ms    → начало/конец фрейма (игнорируем)
 *
 *  DIAG_ERROR детектируем по НИЗКОМУ импульсу (LOW):
 *    Нормально: LOW перед DIAG ≈ 20 ms
 *    Ошибка:    LOW ≈ 10 ms (И следующий DIAG тоже ≈ 20 ms → оба короче в 2 раза)
 */

// ── Пороги (мс) ────────────────────────────────────────────────
static constexpr float TEMP_MIN_MS   =  5.0f;   // минимальная ширина TEMP импульса
static constexpr float TEMP_MAX_MS   = 35.0f;   // максимальная ширина TEMP импульса
static constexpr float DIAG_MIN_MS   = 36.0f;   // минимальная ширина DIAG импульса
static constexpr float DIAG_MAX_MS   = 60.0f;   // максимальная ширина DIAG импульса
static constexpr float DIAG_ERR_MS   = 25.0f;   // ниже этого → DIAG укороченный (ошибка)
static constexpr float LOW_ERR_MS    = 15.0f;   // LOW пауза ниже этого → признак ошибки
static constexpr float LEVEL_MIN_MS  = 20.0f;   // минимальный период между TEMP (не уровень)
static constexpr float LEVEL_MAX_MS  = 500.0f;  // максимальный период между TEMP

static int   cb_num           = 0;
static float lastRiseTime_ms  = 0;   // время последнего RISE
static float lastFallTime_ms  = 0;   // время последнего FALL
static float lastTempRise_ms  = 0;   // время RISE последнего TEMP импульса
static bool  diagError        = false; // флаг ошибки уровня (DIAG укорочен)

static StoredValueSensor levelSensor   (SensorType::HellaOilLevel,           MS2NT(3000));
static StoredValueSensor tempSensor    (SensorType::HellaOilTemperature,      MS2NT(3000));
static StoredValueSensor rawLevelSensor(SensorType::HellaOilLevelRawPulse,    MS2NT(3000));
static StoredValueSensor rawTempSensor (SensorType::HellaOilTempRawPulse,     MS2NT(3000));

#if EFI_PROD_CODE
static Gpio hellaPin = Gpio::Unassigned;

static void hellaOilCallback(efitick_t nowNt, bool isHigh) {
    cb_num++;
    float t_ms = NT2US(nowNt) / 1000.0f;

    if (isHigh) {
        // ── RISE ───────────────────────────────────────────────
        float lowWidth_ms = t_ms - lastFallTime_ms;
        lastRiseTime_ms = t_ms;

        // Детектируем укороченную LOW паузу → признак ошибки уровня
        // (только если уже был хотя бы один FALL, т.е. lowWidth разумный)
        if (lastFallTime_ms > 0 && lowWidth_ms < LOW_ERR_MS && lowWidth_ms > 1.0f) {
            efiPrintf("HELLA: SHORT LOW=%.1fms → error flag", lowWidth_ms);
            diagError = true;
        }

    } else {
        // ── FALL ───────────────────────────────────────────────
        float highWidth_ms = t_ms - lastRiseTime_ms;
        lastFallTime_ms = t_ms;

        efiPrintf("HELLA #%d HIGH=%.1fms", cb_num, highWidth_ms);

        if (highWidth_ms >= TEMP_MIN_MS && highWidth_ms <= TEMP_MAX_MS) {
            // ── TEMP импульс ───────────────────────────────────
            float minTempMs = engineConfiguration->hellaOilLevel.minPulseUsTemp / 1000.0f;
            float maxTempMs = engineConfiguration->hellaOilLevel.maxPulseUsTemp / 1000.0f;

            float temp = interpolateClamped(
                minTempMs, engineConfiguration->hellaOilLevel.minTempC,
                maxTempMs, engineConfiguration->hellaOilLevel.maxTempC,
                highWidth_ms
            );
            tempSensor.setValidValue(temp, nowNt);
            rawTempSensor.setValidValue(highWidth_ms, nowNt);
            efiPrintf("HELLA TEMP: %.1fms → %.1fC", highWidth_ms, temp);

            // ── LEVEL = интервал между RISE соседних TEMP ──────
            if (lastTempRise_ms > 0) {
                float levelTime_ms = lastRiseTime_ms - lastTempRise_ms;

                if (levelTime_ms >= LEVEL_MIN_MS && levelTime_ms <= LEVEL_MAX_MS) {
                    float minLevelMs = engineConfiguration->hellaOilLevel.minPulseUsLevel / 1000.0f;
                    float maxLevelMs = engineConfiguration->hellaOilLevel.maxPulseUsLevel / 1000.0f;

                    float level = interpolateClamped(
                        minLevelMs, engineConfiguration->hellaOilLevel.minLevelMm,
                        maxLevelMs, engineConfiguration->hellaOilLevel.maxLevelMm,
                        levelTime_ms
                    );
                    levelSensor.setValidValue(level, nowNt);
                    rawLevelSensor.setValidValue(levelTime_ms, nowNt);
                    efiPrintf("HELLA LEVEL: interval=%.1fms → %.1fmm", levelTime_ms, level);
                }
            }
            lastTempRise_ms = lastRiseTime_ms;

        } else if (highWidth_ms >= DIAG_MIN_MS && highWidth_ms <= DIAG_MAX_MS) {
            // ── DIAG импульс ───────────────────────────────────
            bool shortDiag = (highWidth_ms < DIAG_ERR_MS);
            if (shortDiag) {
                diagError = true;
            } else {
                // Нормальный DIAG — сбрасываем флаг ошибки
                diagError = false;
            }
            efiPrintf("HELLA DIAG: %.1fms [%s]", highWidth_ms, diagError ? "ERROR" : "OK");

        } else if (highWidth_ms > 100.0f) {
            // ── DATA / начало фрейма — игнорируем ─────────────
            efiPrintf("HELLA DATA/FRAME: %.1fms", highWidth_ms);

        } else {
            efiPrintf("HELLA UNKNOWN: %.1fms", highWidth_ms);
        }
    }
}

static void hellaExtiCallback(void*, efitick_t nowNt) {
    bool pin = efiReadPin(hellaPin);
    hellaOilCallback(nowNt, pin ^ engineConfiguration->hellaOilLevelInverted);
}
#endif // EFI_PROD_CODE


void initHellaOilLevelSensor(bool isFirstTime) {
    efiPrintf("HELLA INIT isFirstTime=%d", isFirstTime);

#if EFI_PROD_CODE
    if (!isBrainPinValid(engineConfiguration->hellaOilLevelPin)) {
        efiPrintf("HELLA ERROR: pin not configured");
        return;
    }

    if (efiExtiEnablePin("hellaOil", engineConfiguration->hellaOilLevelPin,
                         PAL_EVENT_MODE_BOTH_EDGES, hellaExtiCallback, nullptr) < 0) {
        efiPrintf("HELLA ERROR: EXTI failed");
        return;
    }

    hellaPin = engineConfiguration->hellaOilLevelPin;
    efiPrintf("HELLA: EXTI on %s", hwPortname(hellaPin));

    if (isFirstTime) {
        addConsoleAction("hellainfo", []() {
            auto level    = Sensor::get(SensorType::HellaOilLevel);
            auto temp     = Sensor::get(SensorType::HellaOilTemperature);
            auto rawLevel = Sensor::get(SensorType::HellaOilLevelRawPulse);
            auto rawTemp  = Sensor::get(SensorType::HellaOilTempRawPulse);
            efiPrintf("HellaOil Level=%.1fmm[%s] Temp=%.1fC[%s] RawLevel=%.1fms[%s] RawTemp=%.1fms[%s] DiagErr=%d",
                level.value_or(0),    level.Valid    ? "OK" : "NO",
                temp.value_or(0),     temp.Valid     ? "OK" : "NO",
                rawLevel.value_or(0), rawLevel.Valid ? "OK" : "NO",
                rawTemp.value_or(0),  rawTemp.Valid  ? "OK" : "NO",
                (int)diagError);
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
