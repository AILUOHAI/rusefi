/**
 * @file max3185x.cpp
 * @brief MAX31855 and MAX31856 Thermocouple-to-Digital Converter driver
 *
 *
 * http://datasheets.maximintegrated.com/en/ds/MAX31855.pdf
 * https://www.analog.com/media/en/technical-documentation/data-sheets/MAX31856.pdf
 *
 *
 * Read-only (MAX31855), RW (MAX31956) communication over 5MHz SPI
 *
 * @date Sep 17, 2014
 * @author Andrey Belomutskiy, (c) 2012-2020
 *
 * @author Andrey Gusakov, 2024
 *
 */

/**
 * @file max3185x.cpp
 * @brief MAX31855 only thermocouple driver with raw SPI debug
 */

/**
 * @file max3185x.cpp
 * @brief MAX31855 only thermocouple driver with raw SPI debug
 */

#include "pch.h"
#include "max3185x.h"
#include "hardware.h"

#if EFI_PROD_CODE
#include "mpu_util.h"
#endif

#if EFI_MAX_31855

#include "thread_controller.h"
#include "stored_value_sensor.h"

#ifndef MAX31855_REFRESH_TIME
#define MAX31855_REFRESH_TIME 100
#endif

class Max31855Read final : public ThreadController<UTILITY_THREAD_STACK_SIZE> {
public:
	Max31855Read()
		: ThreadController("MAX31855", MAX31855_PRIO) {
	}

	enum Max31855State {
		MAX31855_OK = 0,
		MAX31855_OPEN_CIRCUIT = 1,
		MAX31855_SHORT_TO_GND = 2,
		MAX31855_SHORT_TO_VCC = 3,
		MAX31855_NO_REPLY = 4,
		MAX31855_NOT_ENABLED = 5,
	};

	int start(spi_device_e device, egt_cs_array_t cs) {
		driver = getSpiDevice(device);

		for (size_t i = 0; i < EGT_CHANNEL_COUNT; i++) {
			m_cs[i] = Gpio::Invalid;
		}

		if (!driver) {
			efiPrintf("MAX31855: no SPI driver for device %d", device);
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
				float value = 0;
				Max31855State ret = readChannel(i, &value, nullptr, false);
				if (ret == MAX31855_OK) {
					egtSensors[i].setValidValue(value, getTimeNowNt());
				}
			}

			chThdSleepMilliseconds(MAX31855_REFRESH_TIME);
		}

		chThdExit((msg_t)0x0);
	}

	void showEgtInfo() {
#if EFI_PROD_CODE
		printSpiState();
		efiPrintf("EGT driver: %s", driver ? "OK" : "NULL");
		efiPrintf("EGT spi: %d", engineConfiguration->max31855spiDevice);

		for (int i = 0; i < EGT_CHANNEL_COUNT; i++) {
			if (isBrainPinValid(m_cs[i])) {
				efiPrintf("EGT CS %d @%s", i + 1, hwPortname(m_cs[i]));
			}
		}
#endif
	}

	void egtRead() {
		if (driver == nullptr) {
			efiPrintf("MAX31855: no SPI selected");
			return;
		}

		efiPrintf("Reading egt(s)");

		for (size_t i = 0; i < EGT_CHANNEL_COUNT; i++) {
			float temp = 0;
			float refTemp = 0;
			Max31855State code = readChannel(i, &temp, &refTemp, true);

			efiPrintf("egt%d: type max31855, code=%d (%s)",
				(int)i + 1,
				code,
				getErrorName(code));

			if (code == MAX31855_OK) {
				efiPrintf(" temperature %.4f reference temperature %.2f", temp, refTemp);
			}
		}
	}

private:
	static constexpr uint32_t MAX31855_RESERVED_BITS = 0x00020008;

	brain_pin_e m_cs[EGT_CHANNEL_COUNT];
	SPIDriver* driver = nullptr;

	SPIConfig spiConfig = {
		.circular = false,
#ifdef _CHIBIOS_RT_CONF_VER_6_1_
		.end_cb = nullptr,
#else
		.slave = false,
		.data_cb = nullptr,
		.error_cb = nullptr,
#endif
		.ssport = nullptr,
		.sspad = 0,
		.cr1 =
			SPI_CR1_8BIT_MODE |
			SPI_CR1_SSM |
			SPI_CR1_SSI |
			((5 << SPI_CR1_BR_Pos) & SPI_CR1_BR) |
			SPI_CR1_MSTR |
			0,
		.cr2 = SPI_CR2_8BIT_MODE
	};

	const char* getErrorName(Max31855State code) {
		switch (code) {
		case MAX31855_OK: return "Ok";
		case MAX31855_OPEN_CIRCUIT: return "Open";
		case MAX31855_SHORT_TO_GND: return "short gnd";
		case MAX31855_SHORT_TO_VCC: return "short VCC";
		case MAX31855_NO_REPLY: return "no reply";
		case MAX31855_NOT_ENABLED: return "not enabled";
		default: return "invalid";
		}
	}

	int spiTxRx(size_t channel, const uint8_t* tx, uint8_t* rx, size_t n) {
		brain_pin_e cs = m_cs[channel];

		if (!isBrainPinValid(cs) || driver == nullptr) {
			return -1;
		}

		initSpiCsNoOccupy(&spiConfig, cs);

		spiAcquireBus(driver);
		spiStart(driver, &spiConfig);
		spiSelect(driver);
		spiExchange(driver, n, tx, rx);
		spiUnselect(driver);
		spiStop(driver);
		spiReleaseBus(driver);

		return 0;
	}

	int spiRx32(size_t channel, uint32_t* data) {
		uint8_t tx[4] = { 0, 0, 0, 0 };
		uint8_t rx[4] = { 0, 0, 0, 0 };

		int ret = spiTxRx(channel, tx, rx, 4);
		if (ret) {
			return ret;
		}

		if (data) {
			*data =
				((uint32_t)rx[0] << 24) |
				((uint32_t)rx[1] << 16) |
				((uint32_t)rx[2] << 8) |
				((uint32_t)rx[3] << 0);
		}

		return 0;
	}

	Max31855State decodeFault(uint32_t packet) {
		const bool reservedBad = (packet & MAX31855_RESERVED_BITS) != 0;
		const bool allZero = packet == 0x00000000;
		const bool allOne = packet == 0xFFFFFFFF;

		if (reservedBad || allZero || allOne) {
			return MAX31855_NO_REPLY;
		}

		if (packet & BIT(16)) {
			if (packet & BIT(0)) {
				return MAX31855_OPEN_CIRCUIT;
			}
			if (packet & BIT(1)) {
				return MAX31855_SHORT_TO_GND;
			}
			if (packet & BIT(2)) {
				return MAX31855_SHORT_TO_VCC;
			}
		}

		return MAX31855_OK;
	}

	float decodeThermocouple(uint32_t packet) {
		int16_t v = (packet >> 18) & 0x3FFF;
		v = (int16_t)(v << 2);
		v = (int16_t)(v >> 2);
		return v * 0.25f;
	}

	float decodeColdJunction(uint32_t packet) {
		int16_t v = (packet >> 4) & 0x0FFF;
		v = (int16_t)(v << 4);
		v = (int16_t)(v >> 4);
		return v * 0.0625f;
	}

	Max31855State readChannel(size_t channel, float* temp, float* coldJunctionTemp, bool verbose) {
		if (!isBrainPinValid(m_cs[channel]) || driver == nullptr) {
			return MAX31855_NOT_ENABLED;
		}

		uint32_t packet = 0;
		int ret = spiRx32(channel, &packet);

		if (verbose) {
			efiPrintf("max31855 ch=%d raw=0x%08x spiRet=%d cs=%s",
				(int)channel + 1,
				packet,
				ret,
				hwPortname(m_cs[channel]));
		}

		if (ret != 0) {
			return MAX31855_NO_REPLY;
		}

		Max31855State code = decodeFault(packet);

		if (verbose) {
			efiPrintf("max31855 ch=%d fault=%d oc=%d scg=%d scv=%d",
				(int)channel + 1,
				(packet & BIT(16)) ? 1 : 0,
				(packet & BIT(0)) ? 1 : 0,
				(packet & BIT(1)) ? 1 : 0,
				(packet & BIT(2)) ? 1 : 0);
		}

		if (code != MAX31855_OK) {
			return code;
		}

		float tc = decodeThermocouple(packet);
		float cj = decodeColdJunction(packet);

		if (temp) {
			*temp = tc;
		}

		if (coldJunctionTemp) {
			*coldJunctionTemp = cj;
		}

		if (verbose) {
			efiPrintf("max31855 ch=%d tc=%.2f cj=%.2f",
				(int)channel + 1,
				tc,
				cj);
		}

		return MAX31855_OK;
	}

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

static Max31855Read instance;

static void showEgtInfo() {
	instance.showEgtInfo();
}

static void egtRead() {
	instance.egtRead();
}

void initMax3185x(spi_device_e device, egt_cs_array_t max31855_cs) {
	addConsoleAction("egtinfo", (Void)showEgtInfo);
	addConsoleAction("egtread", (Void)egtRead);
	startMax3185x(device, max31855_cs);
}

void stopMax3185x() {
	instance.stop();
}

void startMax3185x(spi_device_e device, egt_cs_array_t max31855_cs) {
	instance.start(device, max31855_cs);
}

#endif /* EFI_MAX_31855 */
