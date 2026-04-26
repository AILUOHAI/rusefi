/**
 * @file hella_oil_level_bmw.cpp
 * @brief Драйвер датчика уровня и температуры масла Hella (BMW E-серия)
 *
 * Датчик передаёт данные по однопроводному цифровому протоколу.
 * Один полный фрейм содержит:
 *   - DIAG-кадр (диагностика):  LOW пауза + HIGH импульс одинаковой длины
 *   - TEMP/LEVEL зону:          несколько TEMP импульсов разделённых паузами LEVEL
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  ВРЕМЕННАЯ ДИАГРАММА (осциллограмма, масштаб 20 ms/div):
 *
 *  CH1 (жёлтый) — основной сигнал:
 *
 *  ──┐        ┌────────────┐              ┌──────
 *    │        │            │              │
 *    └────────┘            └──────────────┘
 *     ←  1  →  ←    2    →  ← 3 + 4 зона →
 *       LOW      DIAG HIGH
 *
 *  CH3 (синий) — TEMP импульсы внутри зоны:
 *
 *    ┌─ 3 ─┐                   ┌─ 3 ─┐
 *  ──┘     └───────────────────┘     └──
 *           ←────── 4 (LEVEL) ───────→
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  ОПИСАНИЕ ИМПУЛЬСОВ:
 *
 *  №  Имя    Норма      Ошибка уровня   Что кодирует
 *  ─────────────────────────────────────────────────────────────────────
 *  1  LOW    ~40 ms     ~23 ms          Пауза перед DIAG (должна = DIAG)
 *  2  DIAG   ~40 ms     ~23 ms          Диагностический HIGH импульс
 *  3  TEMP    5..20 ms  не меняется     Ширина кодирует температуру масла
 *  4  LEVEL  90..400 ms зависит         Интервал RISE→RISE между TEMP = уровень масла
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  ПРАВИЛА ВАЛИДАЦИИ (выполняются при каждом DIAG FALL):
 *
 *  1. |LOW_width - DIAG_width| <= DIAG_MATCH_TOL_MS (3 ms)
 *       Нет → датчик не работает или помеха → все каналы = 0
 *
 *  2. DIAG_width попадает в одно из двух окон:
 *       37..43 ms → НОРМА     → diagError=false, данные валидны
 *       20..25 ms → ОШИБКА    → diagError=true, level=0, temp передаём
 *       иначе     → МУСОР     → все каналы = 0
 *
 *  При ошибке уровня (масло ниже минимума):
 *    - DIAG и LOW мигают: 20ms/20ms/20ms/20ms...
 *    - TEMP импульсы не изменяются → температуру всё равно передаём
 *    - Уровень = 0 мм (масло ниже минимума датчика)
 */

#include "pch.h"
#include "hella_oil_level_bmw.h"
#include "digital_input_exti.h"

#if EFI_HELLA_OIL_BMW

// ── Пороги детектирования импульсов (мс) ────────────────────────────────────

// Диапазон TEMP импульсов (кодируют температуру масла)
static constexpr float TEMP_MIN_MS       =  5.0f;  // мин. допустимая ширина TEMP
static constexpr float TEMP_MAX_MS       = 20.0f;  // макс. допустимая ширина TEMP

// Ширины DIAG импульса (и LOW паузы перед ним)
static constexpr float DIAG_NORMAL_MS    = 40.0f;  // нормальная работа: оба ~40 мс
static constexpr float DIAG_ERROR_MS     = 24.0f;  // ошибка уровня масла: оба ~20 мс
static constexpr float DIAG_MATCH_TOL_MS =  3.0f;  // допуск: |LOW - DIAG| <= 3 мс
static constexpr float DIAG_MIN_ABS_MS   = 21.0f;  // ниже — перекрытие с TEMP → мусор

// Допустимый диапазон интервала между TEMP импульсами (= уровень масла)
static constexpr float LEVEL_MIN_MS      = 90.0f;  // мин. интервал RISE→RISE
static constexpr float LEVEL_MAX_MS      = 500.0f; // макс. интервал RISE→RISE

// ── Состояние парсера (живёт на всё время работы) ───────────────────────────
static int   cb_num          = 0;     // счётчик EXTI прерываний (для отладки)
static float lastRiseTime_ms = 0;     // абсолютное время последнего RISE (мс)
static float lastFallTime_ms = 0;     // абсолютное время последнего FALL (мс)
static float lastLowWidth_ms = 0;     // ширина LOW паузы перед текущим HIGH (мс)
static float lastTempRise_ms = 0;     // время RISE предыдущего TEMP импульса (мс)
                                      // = 0 если предыдущего TEMP ещё не было
static bool  diagError       = false; // true = датчик сигнализирует об ошибке уровня

// ── Сенсоры rusefi (timeout 3 с — ~3 фрейма) ────────────────────────────────
// Все четыре канала передаются в TunerStudio и доступны для CAN через Sensor::get()
static StoredValueSensor levelSensor   (SensorType::HellaOilLevel,        MS2NT(3000));
static StoredValueSensor tempSensor    (SensorType::HellaOilTemperature,   MS2NT(3000));
static StoredValueSensor rawLevelSensor(SensorType::HellaOilLevelRawPulse, MS2NT(3000));  // сырой интервал, мс
static StoredValueSensor rawTempSensor (SensorType::HellaOilTempRawPulse,  MS2NT(3000));  // сырая ширина TEMP, мс

#if EFI_PROD_CODE
static Gpio hellaPin = Gpio::Unassigned;

/**
 * Основной обработчик фронтов сигнала датчика.
 * Вызывается из EXTI ISR на каждый RISE и FALL.
 *
 * @param nowNt   текущее время в нанотиках (системные тики STM32)
 * @param isHigh  true = RISE (сигнал стал HIGH), false = FALL (сигнал стал LOW)
 */
static void hellaOilCallback(efitick_t nowNt, bool isHigh) {
    cb_num++;
    // Переводим нанотики → миллисекунды для удобства сравнения с протоколом
    float t_ms = NT2US(nowNt) / 1000.0f;

    if (isHigh) {
        // ── RISE ────────────────────────────────────────────────────────────
        // Считаем ширину LOW паузы, которая только что закончилась.
        // lastFallTime_ms == 0 только при самом первом фронте после старта.
        if (lastFallTime_ms > 0) {
            lastLowWidth_ms = t_ms - lastFallTime_ms;
        }
        lastRiseTime_ms = t_ms;

    } else {
        // ── FALL ────────────────────────────────────────────────────────────
        float highWidth_ms = t_ms - lastRiseTime_ms;
        lastFallTime_ms = t_ms;

        efiPrintf("HELLA #%d FALL: HIGH=%.1fms LOW_before=%.1fms", cb_num, highWidth_ms, lastLowWidth_ms);

        // ════════════════════════════════════════════════════════════════════
        //  TEMP импульс: 5..35 мс
        //  Ширина линейно кодирует температуру масла.
        //  Calibration: hellaOilLevel.{min/max}PulseUsTemp (в us в конфиге → /1000 → мс)
        // ════════════════════════════════════════════════════════════════════
        if (highWidth_ms >= TEMP_MIN_MS && highWidth_ms <= TEMP_MAX_MS) {

            // Конфиг хранит пороги в микросекундах → переводим в мс
            float minTempMs = engineConfiguration->hellaOilLevel.minPulseUsTemp / 1000.0f;
            float maxTempMs = engineConfiguration->hellaOilLevel.maxPulseUsTemp / 1000.0f;

            float temp = interpolateClamped(
                minTempMs, engineConfiguration->hellaOilLevel.minTempC,
                maxTempMs, engineConfiguration->hellaOilLevel.maxTempC,
                highWidth_ms
            );
            tempSensor.setValidValue(temp, nowNt);
            rawTempSensor.setValidValue(highWidth_ms, nowNt);  // сырое значение для диагностики
            efiPrintf("HELLA TEMP: %.1fms → %.1fC", highWidth_ms, temp);

            // ── Уровень масла ────────────────────────────────────────────────
            // Уровень = интервал от RISE предыдущего TEMP до RISE текущего TEMP.
            // Первый TEMP после старта/сброса пропускаем (нет предыдущей точки отсчёта).
            if (lastTempRise_ms > 0) {
                float levelTime_ms = lastRiseTime_ms - lastTempRise_ms;

                if (levelTime_ms >= LEVEL_MIN_MS && levelTime_ms <= LEVEL_MAX_MS) {
                    // Конфиг хранит пороги в микросекундах → переводим в мс
                    float minLevelMs = engineConfiguration->hellaOilLevel.minPulseUsLevel / 1000.0f;
                    float maxLevelMs = engineConfiguration->hellaOilLevel.maxPulseUsLevel / 1000.0f;

                    float level = interpolateClamped(
                        minLevelMs, engineConfiguration->hellaOilLevel.minLevelMm,
                        maxLevelMs, engineConfiguration->hellaOilLevel.maxLevelMm,
                        levelTime_ms
                    );
                    levelSensor.setValidValue(level, nowNt);
                    rawLevelSensor.setValidValue(levelTime_ms, nowNt);  // сырое значение для диагностики
                    efiPrintf("HELLA LEVEL: %.1fms → %.1fmm", levelTime_ms, level);
                } else {
                    // Интервал вне допустимого диапазона — шум или начало нового фрейма
                    efiPrintf("HELLA LEVEL: interval %.1fms out of range, skip", levelTime_ms);
                }
            }

            // Запоминаем время RISE этого TEMP для вычисления следующего интервала
            lastTempRise_ms = lastRiseTime_ms;

        // ════════════════════════════════════════════════════════════════════
        //  DIAG импульс: >= 17 мс (всё что не TEMP)
        //  DIAG всегда идёт в паре с LOW паузой той же длины.
        //  По их совместной длине определяем: датчик работает нормально
        //  или сигнализирует об ошибке уровня масла.
        // ════════════════════════════════════════════════════════════════════
        } else if (highWidth_ms >= DIAG_MIN_ABS_MS) {

            // ── Шаг 1: проверяем совпадение LOW и DIAG ──────────────────────
            // По протоколу LOW пауза (цифра 1) всегда равна DIAG (цифра 2).
            // Если они расходятся > 3 мс — сигнал повреждён или датчик не отвечает.
            float diff = highWidth_ms - lastLowWidth_ms;
            if (diff < 0) diff = -diff;

            if (diff > DIAG_MATCH_TOL_MS) {
                // Несовпадение LOW и DIAG — датчик не запустился или помеха на линии
                efiPrintf("HELLA DIAG: LOW(%.1fms) != HIGH(%.1fms) diff=%.1fms → zero all", lastLowWidth_ms, highWidth_ms, diff);
                levelSensor.setValidValue(0, nowNt);
                rawLevelSensor.setValidValue(0, nowNt);
                tempSensor.setValidValue(0, nowNt);
                rawTempSensor.setValidValue(0, nowNt);
                lastTempRise_ms = 0;  // сбрасываем: следующий TEMP будет первым в новом кадре
                return;
            }

            // ── Шаг 2: определяем режим по длине DIAG ───────────────────────
            float diagCenter = highWidth_ms;  // LOW ≈ HIGH, достаточно одного

            if (diagCenter >= (DIAG_NORMAL_MS - DIAG_MATCH_TOL_MS) &&
                diagCenter <= (DIAG_NORMAL_MS + DIAG_MATCH_TOL_MS)) {
                // Нормальная работа: LOW ~40 мс, DIAG ~40 мс
                // Данные от TEMP/LEVEL в этом кадре — валидны
                diagError = false;
                efiPrintf("HELLA DIAG: OK (%.1fms)", diagCenter);

            } else if (diagCenter >= (DIAG_ERROR_MS - DIAG_MATCH_TOL_MS) &&
                       diagCenter <= (DIAG_ERROR_MS + DIAG_MATCH_TOL_MS)) {
                // Ошибка уровня масла: LOW ~20 мс, DIAG ~20 мс, мигание
                // Уровень = 0 (масло ниже минимума датчика)
                // Температуру продолжаем передавать — TEMP импульсы не меняются
                diagError = true;
                efiPrintf("HELLA DIAG: OIL LEVEL ERROR (%.1fms) → level=0", diagCenter);
                levelSensor.setValidValue(0, nowNt);
                rawLevelSensor.setValidValue(0, nowNt);
                lastTempRise_ms = 0;  // сбрасываем: при ошибке LEVEL данные не имеют смысла

            } else {
                // Длина DIAG не попадает ни в одно из ожидаемых окон → мусор
                // Обнуляем всё — нет гарантии что данные этого кадра корректны
                efiPrintf("HELLA DIAG: UNKNOWN width=%.1fms → zero all", diagCenter);
                levelSensor.setValidValue(0, nowNt);
                rawLevelSensor.setValidValue(0, nowNt);
                tempSensor.setValidValue(0, nowNt);
                rawTempSensor.setValidValue(0, nowNt);
                lastTempRise_ms = 0;
            }

        } else {
            // Короткий импульс < DIAG_MIN_ABS_MS и < TEMP_MIN_MS — шум, игнорируем
            efiPrintf("HELLA: noise pulse %.1fms, ignore", highWidth_ms);
        }
    }
}

/**
 * EXTI callback — вызывается аппаратно на каждый фронт GPIO.
 * Читает текущий уровень пина и передаёт в парсер с учётом инверсии.
 */
static void hellaExtiCallback(void*, efitick_t nowNt) {
    bool pin = efiReadPin(hellaPin);
    hellaOilCallback(nowNt, pin ^ engineConfiguration->hellaOilLevelInverted);
}
#endif // EFI_PROD_CODE


/**
 * Инициализация драйвера датчика.
 * Вызывается при старте и при каждом reconfigure().
 *
 * @param isFirstTime  true = первый запуск, регистрируем консольную команду
 */
void initHellaOilLevelSensor(bool isFirstTime) {
    efiPrintf("HELLA INIT isFirstTime=%d", isFirstTime);

#if EFI_PROD_CODE
    if (!isBrainPinValid(engineConfiguration->hellaOilLevelPin)) {
        efiPrintf("HELLA ERROR: pin not configured");
        return;
    }

    // Подписываемся на оба фронта GPIO через EXTI
    if (efiExtiEnablePin("hellaOil", engineConfiguration->hellaOilLevelPin,
                         PAL_EVENT_MODE_BOTH_EDGES, hellaExtiCallback, nullptr) < 0) {
        efiPrintf("HELLA ERROR: EXTI failed");
        return;
    }

    hellaPin = engineConfiguration->hellaOilLevelPin;
    efiPrintf("HELLA: EXTI on %s", hwPortname(hellaPin));

    if (isFirstTime) {
        // Консольная команда для ручной диагностики:
        // > hellainfo
        // Выводит все четыре канала и флаг ошибки датчика
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

    // Регистрируем все четыре сенсора в системе rusefi.
    // После этого значения доступны через Sensor::get() → outputChannels → TunerStudio / CAN
    levelSensor.Register();
    tempSensor.Register();
    rawLevelSensor.Register();
    rawTempSensor.Register();
    efiPrintf("HELLA: sensors registered");
}

/**
 * Деинициализация — вызывается перед reconfigure() или при выключении фичи.
 * Отписывается от EXTI и снимает регистрацию сенсоров.
 */
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
