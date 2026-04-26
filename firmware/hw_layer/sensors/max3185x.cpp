/**
 * @file max3185x.cpp
 * @brief MAX31855 only thermocouple driver with raw SPI debug
 */

#include "pch.h"

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "max3185x.h"
#include "hardware.h"

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
	}	void stop() {
		        if (driver) spiStop(driver);
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
		printSpiState();
		efiPrintf("EGT driver: %s", driver ? "OK" : "NULL");
		efiPrintf("EGT spi: %d", (int)engineConfiguration->max31855spiDevice);

		for (int i = 0; i < EGT_CHANNEL_COUNT; i++) {
			if (isBrainPinValid(m_cs[i])) {
				efiPrintf("EGT CS %d @%s", i + 1, hwPortname(m_cs[i]));
			}
		}
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
				(int)code,
				getErrorName(code));

			if (code == MAX31855_OK) {
				efiPrintf(" temperature %.4f reference temperature %.2f", temp, refTemp);
			}
		}
	}

private:
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

		spiStart(driver, &spiConfig);
	
		spiSelect(driver);
		spiExchange(driver, n, tx, rx);
		spiUnselect(driver);
          
		// spiReleaseBus removed - using spiStart per transaction

		return 0;
	}

	int spiRx32(size_t channel, uint32_t* data, uint8_t* rawBytes) {
		uint8_t tx[4] = { 0, 0, 0, 0 };
		uint8_t rx[4] = { 0, 0, 0, 0 };

		int ret = spiTxRx(channel, tx, rx, 4);
		if (ret) {
			return ret;
		}
    
        // MAX31855 outputs 0x00000000 during internal conversion (~1ms every 100ms)
        // Retry once after short delay if all zeros received
        uint32_t firstRead = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
        if (firstRead == 0x00000000 || firstRead == 0xFFFFFFFF) {
            chThdSleepMilliseconds205);
            uint8_t rx2[4] = { 0, 0, 0, 0 };
            spiTxRx(channel, tx, rx2, 4);
            rx[0] = rx2[0]; rx[1] = rx2[1]; rx[2] = rx2[2]; rx[3] = rx2[3];
			// If still zero after first retry, try once more
			uint32_t secondRead = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
			if (secondRead == 0x00000000 || secondRead == 0xFFFFFFFF) {
				chThdSleepMilliseconds(20);
				uint8_t rx3[4] = { 0, 0, 0, 0 };
				spiTxRx(channel, tx, rx3, 4);
				rx[0] = rx3[0]; rx[1] = rx3[1]; rx[2] = rx3[2]; rx[3] = rx3[3];
			}
        }

		if (rawBytes) {
			rawBytes[0] = rx[0];
			rawBytes[1] = rx[1];
			rawBytes[2] = rx[2];
			rawBytes[3] = rx[3];
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
		if (packet == 0x00000000 || packet == 0xFFFFFFFF) {
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
		int16_t v = (int16_t)((packet >> 18) & 0x3FFF);
		if (v & 0x2000) {
			v |= 0xC000;
		}
		return v * 0.25f;
	}

	float decodeColdJunction(uint32_t packet) {
		int16_t v = (int16_t)((packet >> 4) & 0x0FFF);
		if (v & 0x0800) {
			v |= 0xF000;
		}
		return v * 0.0625f;
	}

	Max31855State readChannel(size_t channel, float* temp, float* coldJunctionTemp, bool verbose) {
		if (!isBrainPinValid(m_cs[channel]) || driver == nullptr) {
			return MAX31855_NOT_ENABLED;
		}

		uint32_t packet = 0;
		uint8_t rx[4] = { 0, 0, 0, 0 };

		int ret = spiRx32(channel, &packet, rx);

		if (verbose) {
			efiPrintf("max31855 ch=%d bytes=%02x %02x %02x %02x raw=0x%08" PRIx32 " spiRet=%d cs=%s",
				(int)channel + 1,
				(unsigned int)rx[0],
				(unsigned int)rx[1],
				(unsigned int)rx[2],
				(unsigned int)rx[3],
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

#endif // EFI_MAX_31855
