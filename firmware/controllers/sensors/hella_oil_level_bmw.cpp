#include "pch.h"
#include "hella_oil_level_bmw.h"
#include "digital_input_exti.h"

#if EFI_HELLA_OIL_BMW

/*
 * Протокол датчика Hella Oil Level BMW (по осциллограмме, масштаб 20ms/div):
 *
 *  CH1 (жёлтый):
 *
 *  ──┐        ┌────────────┐                   ┌──────
 *    │        │            │                   │
 *    └────────┘            └───────────────────┘
 *     ← 1 →   ←     2    →  ← TEMP/LEVEL зона →
 *     LOW      HIGH (DIAG)
 *
 *  CH3 (синий, TEMP импульсы):
 *
 *    ┌─T─┐                        ┌─T─┐
 *  ──┘   └────────────────────────┘   └──
 *         ←──────── 4 (LEVEL) ────────→
 *
 *  Цифра  Имя    Норма     Ошибка уровня
 *  ──────────────────────────────────────────────────────────────
 *    1    LOW    ~40 ms    ~20 ms  (вместе с 2 мигают: 20/20/20/20)
 *    2    DIAG   ~40 ms    ~20 ms
 *    3    TEMP   5..35 ms  не меняется (кодирует температуру)
 *    4    LEVEL  50..400ms зависит от уровня масла
 *
 *  ПРАВИЛО ВАЛИДНОСТИ:
 *    Шаг 1: |LOW_width - DIAG_width| <= DIAG_MATCH_TOL_MS
 *            Если не выполнено — кадр невалиден, данные не обновляем.
 *    Шаг 2: Определяем режим по DIAG_width:
 *            DIAG_NORMAL_MS ± DIAG_MATCH_TOL_MS → diagError = false, данные валидны
 *            DIAG_ERROR_MS  ± DIAG_MATCH_TOL_MS → diagError = true,  данные невалидны
 *            Иначе — мусор, игнорируем.
 *    Шаг 3: DIAG < TEMP_MAX_MS → перекрытие диапазонов → невалидно.
 */

// ── Пороги (мс) ────────────────────────────────────────────────────────────
static constexpr float TEMP_MIN_MS        =  5.0f;  // мин. ширина TEMP
static constexpr float TEMP_MAX_MS        = 35.0f;  // макс. ширина TEMP
static constexpr float DIAG_NORMAL_MS     = 40.0f;  // нормальный DIAG
static constexpr float DIAG_ERROR_MS      = 20.0f;  // DIAG при ошибке уровня
static constexpr float DIAG_MATCH_TOL_MS  =  3.0f;  // допуск совпадения LOW vs DIAG
static constexpr float DIAG_MIN_ABS_MS    = 17.0f;  // ниже этого — перекрытие с TEMP → мусор
static constexpr float LEVEL_MIN_MS       = 20.0f;  // мин. валидный интервал между TEMP
static constexpr float LEVEL_MAX_MS       = 500.0f; // макс. валидный интервал между TEMP

static int   cb_num          = 0;
static float lastRiseTime_ms = 0;   // время последнего RISE
static float lastFallTime_ms = 0;   // время последнего FALL
static float lastLowWidth_ms = 0;   // ширина последней LOW паузы
static float lastTempRise_ms = 0;   // время RISE последнего TEMP импульса
static bool  diagError       = false;

static StoredValueSensor levelSensor   (SensorType::HellaOilLevel,        MS2NT(3000));
static StoredValueSensor tempSensor    (SensorType::HellaOilTemperature,   MS2NT(3000));
static StoredValueSensor rawLevelSensor(SensorType::HellaOilLevelRawPulse, MS2NT(3000));
static StoredValueSensor rawTempSensor (SensorType::HellaOilTempRawPulse,  MS2NT(3000));

#if EFI_PROD_CODE
static Gpio hellaPin = Gpio::Unassigned;

static void hellaOilCallback(efitick_t nowNt, bool isHigh) {
    cb_num++;
    float t_ms = NT2US(nowNt) / 1000.0f;

    if (isHigh) {
        // ── RISE: запоминаем ширину LOW паузы ──────────────────
        if (lastFallTime_ms > 0) {
            lastLowWidth_ms = t_ms - lastFallTime_ms;
        }
        lastRiseTime_ms = t_ms;

    } else {
        // ── FALL: анализируем ширину HIGH импульса ──────────────
        float highWidth_ms = t_ms - lastRiseTime_ms;
        lastFallTime_ms = t_ms;

        // ── TEMP импульс ────────────────────────────────────────
        if (highWidth_ms >= TEMP_MIN_MS && highWidth_ms <= TEMP_MAX_MS) {

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

            // LEVEL = интервал между RISE соседних TEMP импульсов
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
                    efiPrintf("HELLA LEVEL: %.1fms → %.1fmm", levelTime_ms, level);
                }
            }
            lastTempRise_ms = lastRiseTime_ms;

        // ── DIAG импульс ────────────────────────────────────────
        } else if (highWidth_ms >= DIAG_MIN_ABS_MS) {

            efiPrintf("HELLA DIAG: LOW=%.1fms HIGH=%.1fms", lastLowWidth_ms, highWidth_ms);

            // Шаг 1: LOW и DIAG должны совпадать по длине
            float diff = highWidth_ms - lastLowWidth_ms;
            if (diff < 0) diff = -diff;
            if (diff > DIAG_MATCH_TOL_MS) {
                efiPrintf("HELLA DIAG: LOW≠HIGH diff=%.1fms → zero all", diff);
                levelSensor.setValidValue(0, nowNt);
                rawLevelSensor.setValidValue(0, nowNt);
                tempSensor.setValidValue(0, nowNt);
                rawTempSensor.setValidValue(0, nowNt);
                lastTempRise_ms = 0;
                return;
            }

            // Шаг 2: определяем режим по ширине DIAG
            float diagCenter = highWidth_ms; // LOW ≈ HIGH, используем HIGH

            if (diagCenter >= (DIAG_NORMAL_MS - DIAG_MATCH_TOL_MS) &&
                diagCenter <= (DIAG_NORMAL_MS + DIAG_MATCH_TOL_MS)) {
                // Норма: оба ~40 мс — данные валидны, расчёты идут штатно
                diagError = false;
                efiPrintf("HELLA DIAG: OK (%.1fms)", diagCenter);

            } else if (diagCenter >= (DIAG_ERROR_MS - DIAG_MATCH_TOL_MS) &&
                       diagCenter <= (DIAG_ERROR_MS + DIAG_MATCH_TOL_MS)) {
                // Ошибка уровня: оба ~20 мс
                // Температуру всё равно передаём (TEMP импульсы не меняются)
                // Уровень = 0 (масло ниже минимума)
                diagError = true;
                efiPrintf("HELLA DIAG: LOW LEVEL ERROR (%.1fms)", diagCenter);
                levelSensor.setValidValue(0, nowNt);
                rawLevelSensor.setValidValue(0, nowNt);
                lastTempRise_ms = 0;

            } else {
                // Непонятная длина — мусор, всё обнуляем
                efiPrintf("HELLA DIAG: UNKNOWN width=%.1fms → zero all", diagCenter);
                levelSensor.setValidValue(0, nowNt);
                rawLevelSensor.setValidValue(0, nowNt);
                tempSensor.setValidValue(0, nowNt);
                rawTempSensor.setValidValue(0, nowNt);
                lastTempRise_ms = 0;
            }

        } else {
            efiPrintf("HELLA: UNKNOWN pulse %.1fms", highWidth_ms);
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
