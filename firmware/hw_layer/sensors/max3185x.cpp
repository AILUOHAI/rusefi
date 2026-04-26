/**
 * @file max3185x.cpp
 * @brief MAX31855 thermocouple driver for rusefi / Proteus F7
 *
 * Root causes fixed:
 *  1. STM32F7 D-cache coherency: spiExchange() uses DMA → CPU reads stale
 *     cache = zeros. Fixed by spiPolledExchange() which reads SPI DR directly.
 *  2. Bus contention: spiAcquireBus/spiReleaseBus mutex wraps every transfer.
 *
 * Design notes:
 *  - Sentinel value 0xDEADBEEF is never a valid MAX31855 packet: bits D17
 *    and D3 are always zero in valid packets; DEADBEEF has both set.
 *  - Cache holds last known-good value so dropouts < STALE_TIMEOUT_MS are
 *    transparent to the rest of the firmware.
 *  - MAX31855 conversion window is ~1 ms per 100 ms cycle. One 2 ms retry
 *    is enough to bridge it without blocking the thread too long.
 */

#include "pch.h"

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "max3185x.h"
#include "hardware.h"

#if EFI_MAX_31855

#include "thread_controller.h"
#include "stored_value_sensor.h"

// Polling period. MAX31855 updates every ~100 ms; 150 ms gives one missed
// conversion before the stale-value timeout (3× = 450 ms) triggers.
#ifndef MAX31855_REFRESH_TIME
#define MAX31855_REFRESH_TIME 150
#endif

// How many consecutive NO_REPLY reads before we invalidate the cached value
// and report an actual error upstream. At 150 ms polling → ~2.25 s grace.
#ifndef MAX31855_CACHE_STALE_COUNT
#define MAX31855_CACHE_STALE_COUNT 15
#endif

// SPI clock prescaler for SPI5 on STM32F7 @ 216 MHz (APB2 = 108 MHz):
//   BR[2:0] = 5 → ÷64 → ~1.69 MHz  (MAX31855 max is 5 MHz, well within spec)
#define MAX31855_SPI_PRESCALER ((5 << SPI_CR1_BR_Pos) & SPI_CR1_BR)

// Sentinel: bits D17 and D3 are always 0 in any valid MAX31855 packet,
// but both are 1 in 0xDEADBEEF → safe to use as an error marker.
static constexpr uint32_t SPI_ERROR_SENTINEL = 0xDEADBEEFu;

class Max31855Read final : public ThreadController<UTILITY_THREAD_STACK_SIZE> {
public:
	Max31855Read()
		: ThreadController("MAX31855", MAX31855_PRIO) {
	}

	enum class State : uint8_t {
		Ok          = 0,
		OpenCircuit = 1,
		ShortToGnd  = 2,
		ShortToVcc  = 3,
		NoReply     = 4,
		NotEnabled  = 5,
	};

	int start(spi_device_e device, egt_cs_array_t cs) {
		driver = getSpiDevice(device);

		for (size_t i = 0; i < EGT_CHANNEL_COUNT; i++) {
			m_cs[i]               = Gpio::Invalid;
			m_lastValidTemp[i]    = 0.0f;
			m_lastValidCj[i]      = 0.0f;
			m_staleCtr[i]         = 0;
			m_hasValidCache[i]    = false;
		}

		if (!driver) {
			efiPrintf("MAX31855: no SPI driver for device %d", (int)device);
			return -1;
		}

		for (size_t i = 0; i < EGT_CHANNEL_COUNT; i++) {
			auto& sensor = egtSensors[i];
			if (Sensor::hasSensor(sensor.type())) {
				continue;
			}
			if (isBrainPinValid(cs[i])) {
				initSpiCs(&spiConfig, cs[i]);
				m_cs[i] = cs[i];
				sensor.Register();
			}
		}

		ThreadController::start();
		return 0;
	}

	void stop() {
		ThreadController::stop();

		if (driver) {
			spiStop(driver);
		}

		for (size_t i = 0; i < EGT_CHANNEL_COUNT; i++) {
			if (!isBrainPinValid(m_cs[i])) {
				continue;
			}
			brain_pin_markUnused(m_cs[i]);
			egtSensors[i].unregister();
		}
	}

	void ThreadTask() override {
		while (!chThdShouldTerminateX()) {
			for (size_t i = 0; i < EGT_CHANNEL_COUNT; i++) {
				float value = 0.0f;
				State ret = readChannel(i, &value, nullptr, /*verbose=*/false);

				if (ret == State::Ok) {
					egtSensors[i].setValidValue(value, getTimeNowNt());
				}
			}

			chThdSleepMilliseconds(MAX31855_REFRESH_TIME);
		}

		chThdExit((msg_t)0x0);
	}

	void showEgtInfo() {
		printSpiState();
		efiPrintf("EGT driver: %s", driver ? "OK" : "NULL");
		efiPrintf("EGT spi: %d", (int)engineConfiguration->max31855spiDevice);

		for (size_t i = 0; i < EGT_CHANNEL_COUNT; i++) {
			if (isBrainPinValid(m_cs[i])) {
				efiPrintf("EGT CS %zu @%s", i + 1, hwPortname(m_cs[i]));
			}
		}
	}

	void egtRead() {
		if (!driver) {
			efiPrintf("MAX31855: no SPI selected");
			return;
		}

		efiPrintf("Reading egt(s)");

		for (size_t i = 0; i < EGT_CHANNEL_COUNT; i++) {
			float temp = 0.0f;
			float refTemp = 0.0f;
			State code = readChannel(i, &temp, &refTemp, /*verbose=*/true);

			efiPrintf("egt%zu: type max31855, code=%d (%s)",
				i + 1, (int)code, stateName(code));

			if (code == State::Ok) {
				efiPrintf(" temperature %.4f reference temperature %.2f", temp, refTemp);
			}
		}
	}

private:
	brain_pin_e  m_cs[EGT_CHANNEL_COUNT];
	SPIDriver*   driver = nullptr;

	// Cache — last successfully decoded reading per channel
	float  m_lastValidTemp[EGT_CHANNEL_COUNT];
	float  m_lastValidCj[EGT_CHANNEL_COUNT];
	uint8_t m_staleCtr[EGT_CHANNEL_COUNT];   // consecutive no-reply count
	bool   m_hasValidCache[EGT_CHANNEL_COUNT];

	SPIConfig spiConfig = {
		.circular = false,
#ifdef _CHIBIOS_RT_CONF_VER_6_1_
		.end_cb  = nullptr,
#else
		.slave    = false,
		.data_cb  = nullptr,
		.error_cb = nullptr,
#endif
		.ssport = nullptr,
		.sspad  = 0,
		.cr1 =
			SPI_CR1_8BIT_MODE        |
			SPI_CR1_SSM              |
			SPI_CR1_SSI              |
			MAX31855_SPI_PRESCALER   |
			SPI_CR1_MSTR,
		.cr2 = SPI_CR2_8BIT_MODE
	};

	// ------------------------------------------------------------------ //
	//  Helpers
	// ------------------------------------------------------------------ //

	static const char* stateName(State s) {
		switch (s) {
		case State::Ok:          return "Ok";
		case State::OpenCircuit: return "Open circuit";
		case State::ShortToGnd:  return "Short to GND";
		case State::ShortToVcc:  return "Short to VCC";
		case State::NoReply:     return "No reply";
		case State::NotEnabled:  return "Not enabled";
		default:                 return "Unknown";
		}
	}

	/**
	 * Read 32 bits from MAX31855 via polled (non-DMA) SPI exchange.
	 *
	 * WHY polled instead of spiExchange (DMA):
	 *   STM32F7 has D-cache enabled. spiExchange writes received bytes to
	 *   physical SRAM via DMA, but the CPU sees the old cache line (zeros).
	 *   The fix is either NO_CACHE buffers or bypassing DMA altogether.
	 *   For 4 bytes, polled transfer costs ~19 µs at 1.69 MHz — negligible
	 *   compared to the 150 ms polling period, and simpler than NO_CACHE.
	 *
	 * Returns SPI_ERROR_SENTINEL on driver/pin error (never a valid packet).
	 */
	uint32_t readRaw32(size_t ch) {
		if (!isBrainPinValid(m_cs[ch]) || !driver) {
			return SPI_ERROR_SENTINEL;
		}

		initSpiCsNoOccupy(&spiConfig, m_cs[ch]);

		spiAcquireBus(driver);              // grab bus mutex
		spiStart(driver, &spiConfig);       // re-apply config (another device may have changed CR1)
		spiSelect(driver);                  // CS low

		uint32_t raw =
			((uint32_t)spiPolledExchange(driver, 0) << 24) |
			((uint32_t)spiPolledExchange(driver, 0) << 16) |
			((uint32_t)spiPolledExchange(driver, 0) <<  8) |
			((uint32_t)spiPolledExchange(driver, 0) <<  0);

		spiUnselect(driver);                // CS high
		spiReleaseBus(driver);              // release mutex

		return raw;
	}

	/**
	 * Read with one retry for the ~1 ms conversion gap.
	 * Returns SPI_ERROR_SENTINEL on hard error, otherwise the raw 32-bit word.
	 */
	uint32_t readRaw32WithRetry(size_t ch) {
		uint32_t raw = readRaw32(ch);
		if (raw == SPI_ERROR_SENTINEL) {
			return raw;
		}

		if (raw == 0x00000000u || raw == 0xFFFFFFFFu) {
			chThdSleepMilliseconds(2);
			raw = readRaw32(ch);
		}

		return raw;
	}

	// ------------------------------------------------------------------ //
	//  MAX31855 packet decoding
	// ------------------------------------------------------------------ //

	static State decodeFault(uint32_t p) {
		if (p == 0x00000000u || p == 0xFFFFFFFFu) {
			return State::NoReply;
		}
		// Fault bit D16 set → one of the fault sub-bits D[2:0] is set
		if (p & BIT(16)) {
			if (p & BIT(0)) return State::OpenCircuit;
			if (p & BIT(1)) return State::ShortToGnd;
			if (p & BIT(2)) return State::ShortToVcc;
		}
		return State::Ok;
	}

	// Bits [31:18]: 14-bit signed, 0.25 °C / LSB
	static float decodeThermocouple(uint32_t p) {
		auto v = static_cast<int16_t>((p >> 18) & 0x3FFF);
		if (v & 0x2000) v |= static_cast<int16_t>(0xC000); // sign-extend
		return v * 0.25f;
	}

	// Bits [15:4]: 12-bit signed, 0.0625 °C / LSB
	static float decodeColdJunction(uint32_t p) {
		auto v = static_cast<int16_t>((p >> 4) & 0x0FFF);
		if (v & 0x0800) v |= static_cast<int16_t>(0xF000); // sign-extend
		return v * 0.0625f;
	}

	// ------------------------------------------------------------------ //
	//  Main read + cache logic
	// ------------------------------------------------------------------ //

	State readChannel(size_t ch, float* temp, float* cjTemp, bool verbose) {
		if (!isBrainPinValid(m_cs[ch]) || !driver) {
			return State::NotEnabled;
		}

		const uint32_t raw = readRaw32WithRetry(ch);

		// Hard SPI error (driver/pin not configured)
		if (raw == SPI_ERROR_SENTINEL) {
			return returnCachedOrError(ch, temp, cjTemp, verbose);
		}

		// Decompose raw into bytes for the verbose log
		if (verbose) {
			efiPrintf("max31855 ch=%zu bytes=%02x %02x %02x %02x raw=0x%08" PRIx32 " cs=%s",
				ch + 1,
				(raw >> 24) & 0xFF, (raw >> 16) & 0xFF,
				(raw >>  8) & 0xFF, (raw >>  0) & 0xFF,
				raw,
				hwPortname(m_cs[ch]));

			efiPrintf("max31855 ch=%zu fault=%d oc=%d scg=%d scv=%d",
				ch + 1,
				(raw & BIT(16)) ? 1 : 0,
				(raw & BIT(0))  ? 1 : 0,
				(raw & BIT(1))  ? 1 : 0,
				(raw & BIT(2))  ? 1 : 0);
		}

		const State code = decodeFault(raw);

		if (code == State::NoReply) {
			return returnCachedOrError(ch, temp, cjTemp, verbose);
		}

		if (code != State::Ok) {
			// Real hardware fault (open/short) — don't cache, report immediately
			m_staleCtr[ch] = 0;
			return code;
		}

		// Valid reading — update cache and reset stale counter
		const float tc = decodeThermocouple(raw);
		const float cj = decodeColdJunction(raw);

		m_lastValidTemp[ch] = tc;
		m_lastValidCj[ch]   = cj;
		m_hasValidCache[ch] = true;
		m_staleCtr[ch]      = 0;

		if (temp)   *temp   = tc;
		if (cjTemp) *cjTemp = cj;

		if (verbose) {
			efiPrintf("max31855 ch=%zu tc=%.2f cj=%.2f", ch + 1, tc, cj);
		}

		return State::Ok;
	}

	/**
	 * Return cached value if fresh enough, or NoReply if stale.
	 * Increments m_staleCtr; once it reaches MAX31855_CACHE_STALE_COUNT
	 * the cache is considered expired and a real error is reported.
	 */
	State returnCachedOrError(size_t ch, float* temp, float* cjTemp, bool verbose) {
		if (!m_hasValidCache[ch]) {
			return State::NoReply;
		}

		m_staleCtr[ch]++;

		if (m_staleCtr[ch] > MAX31855_CACHE_STALE_COUNT) {
			// Cache too old — invalidate and report real error
			m_hasValidCache[ch] = false;
			m_staleCtr[ch]      = 0;
			return State::NoReply;
		}

		if (temp)   *temp   = m_lastValidTemp[ch];
		if (cjTemp) *cjTemp = m_lastValidCj[ch];

		if (verbose) {
			efiPrintf("max31855 ch=%zu using cached tc=%.2f cj=%.2f (stale %d/%d)",
				ch + 1,
				m_lastValidTemp[ch], m_lastValidCj[ch],
				(int)m_staleCtr[ch], (int)MAX31855_CACHE_STALE_COUNT);
		}

		return State::Ok;
	}

	// ------------------------------------------------------------------ //
	//  Sensor array
	// ------------------------------------------------------------------ //

	StoredValueSensor egtSensors[EGT_CHANNEL_COUNT] = {
		{ SensorType::EGT1, MS2NT(MAX31855_REFRESH_TIME * 3) },
		{ SensorType::EGT2, MS2NT(MAX31855_REFRESH_TIME * 3) },
		{ SensorType::EGT3, MS2NT(MAX31855_REFRESH_TIME * 3) },
		{ SensorType::EGT4, MS2NT(MAX31855_REFRESH_TIME * 3) },
		{ SensorType::EGT5, MS2NT(MAX31855_REFRESH_TIME * 3) },
		{ SensorType::EGT6, MS2NT(MAX31855_REFRESH_TIME * 3) },
		{ SensorType::EGT7, MS2NT(MAX31855_REFRESH_TIME * 3) },
		{ SensorType::EGT8, MS2NT(MAX31855_REFRESH_TIME * 3) },
	};
};

// ------------------------------------------------------------------ //
//  Module entry points
// ------------------------------------------------------------------ //

static Max31855Read instance;

void initMax3185x(spi_device_e device, egt_cs_array_t max31855_cs) {
	addConsoleAction("egtinfo", [] { instance.showEgtInfo(); });
	addConsoleAction("egtread", [] { instance.egtRead(); });
	startMax3185x(device, max31855_cs);
}

void stopMax3185x() {
	instance.stop();
}

void startMax3185x(spi_device_e device, egt_cs_array_t max31855_cs) {
	instance.start(device, max31855_cs);
}

#endif // EFI_MAX_31855
