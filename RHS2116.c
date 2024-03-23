#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "rhs2116.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_spi.h"
#include "app_timer.h"
#include "nrf_timer.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static uint8_t tx_buf[4] = {0};
static uint8_t rx_buf[4] = {0};

// Flag to signal that transfer is complete
static volatile bool spi_xfer_done;

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
	spi_xfer_done = true;
}

void spi_init(void)
{
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}


void rhs2116_init(void) {
	int i;
	rhs2116_checkId();			// make sure the chip is online
	rhs2116_STIM_EN_A(0x0000);// Ensure that stimulation is disabled until we configure all others
	rhs2116_STIM_EN_B(0x0000);	// ^
	rhs2116_DC_AMP_PWR(0xFFFF); // Power up all DC-coupled low-gain amplifiers to avoid excessive power consumption
	rhs2116_clear();

	rhs2116_SUPPS_BIASCURR(32, 40);
	rhs2116_OUTFMT_DSP_AUXDIO(0x00, false, false, false, true, true, false,
	true, false, false);
	rhs2116_IMPCHK_CTRL(false, 0x00, false, false, 0x00);
	rhs2116_IMPCHK_DAC(0x00);				// 0x0080
	rhs2116_RH1_CUTOFF(0x16, 0x00);			// 0x0016
	rhs2116_RH2_CUTOFF(0x17, 0x00);			// 0x0017
	rhs2116_RL_A_CUTOFF(0x28, 0x02, false); // 0x00A8
	rhs2116_RL_B_CUTOFF(0x0A, 0x00, false);
	rhs2116_ACAMP_PWR(0x0000);					// all off
	rhs2116_AMP_FSTSETL(0x0000, true);				
	rhs2116_AMP_LCUTOFF(0xFFFF, true);				
	rhs2116_STIM_CUR_STEP(0x22, 0x07, 0x01);	// 0x00E2
	rhs2116_STIM_BIAS_VOLTS(0x0A, 0x0A);		// 0x00AA
	rhs2116_CHRG_REC_VOLTS(0x80);				// 0x0080
	rhs2116_CHRG_REC_CUR_LIM(0x00, 0x3E, 0x02); // 0x4F00
	rhs2116_STIM_ON(0x0000, true);					
	rhs2116_STIM_POL(0x0000, true);					
	rhs2116_CHRG_RECOVER(0x0000, true);				
	rhs2116_CUR_LMT_CHRG_REC(0x0000, true);			

	for (i = 0; i < 16; i++) {
		rhs2116_NEG_CUR_MAG_X(i, 0x00, 0x80, true); // 0x8000
	}
	for (i = 0; i < 16; i++) {
		rhs2116_POS_CUR_MAG_X(i, 0x00, 0x80, true); // 0x8000
	}
	// Now that all the stimulators are initialized and turned off, enable stimulation
	rhs2116_STIM_EN_A(0xAAAA);
	rhs2116_STIM_EN_B(0x00FF);
	rhs2116_clearComplianceMonitor(); // Dummy command with M flag set to clear the compliance monitor (reg 40)
}

uint16_t do_transfer(void) {

    ret_code_t err_code;
	spi_xfer_done = false;
    err_code = nrf_drv_spi_transfer(&spi, tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf));
    APP_ERROR_CHECK(err_code);

    // Wait for the transfer to complete
	while (!spi_xfer_done) ;

    // dummy cycles
    memset(tx_buf, 0, sizeof(tx_buf));
    memset(rx_buf, 0, sizeof(rx_buf));

    spi_xfer_done = false;
    err_code = nrf_drv_spi_transfer(&spi, tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf));
    APP_ERROR_CHECK(err_code);

	while (!spi_xfer_done) ;

    // Don't have to change tx, tx is still clear, only change rx
    memset(rx_buf, 0, sizeof(rx_buf));

    spi_xfer_done = false;
    err_code = nrf_drv_spi_transfer(&spi, tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf));
    APP_ERROR_CHECK(err_code);

	while (!spi_xfer_done) ;

    uint16_t receivedData = ((uint16_t)rx_buf[2] << 8) | rx_buf[3]; // low16Bits
    return receivedData;
}


bool rhs2116_writeRegister(uint8_t regAddress, uint16_t regValue, bool uFlag, bool mFlag) {
	memset(tx_buf, 0, sizeof(tx_buf));

	// Split regValue into MSB and LSB
	uint8_t dataMSB = (regValue >> 8) & 0xFF; // Extract MSB of the data
	uint8_t dataLSB = regValue & 0xFF;		  // Extract LSB of the data

	// Construct the LSB byte of the command with the specified structure
	uint8_t lsbCommand = 0x80; // Bit 7 set to 1, bits 6-0 set to 0 as base
	if (uFlag) {
		lsbCommand |= 0x20; // Set U flag (bit 5)
	}
	if (mFlag) {
		lsbCommand |= 0x10; // Set M flag (bit 4)
	}

	tx_buf[0] = lsbCommand;      
	tx_buf[1] = regAddress;       
	tx_buf[2] = dataMSB;    
	tx_buf[3] = dataLSB;    

	uint16_t receivedData = do_transfer();

	// Check if the received value matches the sent value
	if (receivedData == regValue) {
		return true; // Data integrity check passed
	}
	return false; // Data integrity check failed
}

uint16_t rhs2116_readRegister(uint8_t regAddress, bool uFlag, bool mFlag) {
	memset(tx_buf, 0, sizeof(tx_buf));

	uint8_t lsbCommand = 0xC0; // Bits 7 and 6 set to 1, bits 5-0 set to 0 as base
	if (uFlag) {
		lsbCommand |= 0x20; // Set U flag (bit 5)
	}
	if (mFlag) {
		lsbCommand |= 0x10; // Set M flag (bit 4)
	}

	tx_buf[0] = lsbCommand;      
	tx_buf[1] = regAddress; 

	uint16_t receivedData = do_transfer();
	return receivedData;
}

void rhs2116_clear(void) {
	// Clear the buffers
	tx_buf[0] = RHS_CLEAR;
	memset(rx_buf, 0, sizeof(rx_buf));
	do_transfer();
}

bool rhs2116_clearComplianceMonitor(void) {
	rhs2116_readRegister(RHS_CHIP_ID, false, true); // Dummy with M flag
	return true;
}

bool rhs2116_checkId(void) {
	uint16_t chipId = rhs2116_readRegister(RHS_CHIP_ID, false, false);
	if (chipId == CHIP_ID) {
		return true;
	} else {
		return false;
	}
}

uint16_t rhs2116_convert(uint8_t channel, bool uFlag, bool mFlag, bool dFlag, bool hFlag) {
	memset(tx_buf, 0, sizeof(tx_buf));

	uint8_t lsbCommand = 0x00; // Bits 7 and 6 set to 1, bits 5-0 set to 0 as base
	if (uFlag) {
		lsbCommand |= 0x20; // Set U flag (bit 5)
	}
	if (mFlag) {
		lsbCommand |= 0x10; // Set M flag (bit 4)
	}
	if (dFlag) {
		lsbCommand |= 0x08;
	}
	if (hFlag) {
		lsbCommand |= 0x04;
	}

	tx_buf[0] = lsbCommand;
	tx_buf[1] = channel;
	uint16_t receivedData = do_transfer();

	// receivedData would be the low 10-bit version if (dFlag), otherwise return high 16-bit
	// rx_buffer is still full from do_transfer()
	
	if (dFlag) {
		receivedData = ((rx_buf[2] & 0x03) << 8) | rx_buf[3]; // low10Bits DC
	}
	else{
		receivedData = ((rx_buf[0]) << 8) | rx_buf[1] ; // high16Bits AC
	}
	return receivedData;
}

void rhs2116_sampling() {
    uint16_t adcValue[16];
    char logStr[350] = {0};
    int offset = 0;
    int n;

    for (uint8_t channel = 0; channel < 16; ++channel) {
        adcValue[channel] = rhs2116_convert(channel, false, false, false, false);
    }

    for (uint8_t channel = 0; channel < 16; ++channel) {
        n = snprintf(logStr + offset, sizeof(logStr) - offset, "%04X ", adcValue[channel]);
        if (n < 0) {
            NRF_LOG_ERROR("Failed to write to log string.");
            return;
        }
        offset += n;
        if (offset >= sizeof(logStr)) break;
    }
		rhs2116_CHRG_RECOVER(0x0000, true);				
		rhs2116_CUR_LMT_CHRG_REC(0x0000, true);		
		rhs2116_STIM_POL(0x0000, true);
		rhs2116_STIM_ON(0x0000, true);	

    NRF_LOG_INFO("%s", NRF_LOG_PUSH(logStr));
    NRF_LOG_FLUSH();
}


/*
 * Configures Register 0: Supply Sensor and ADC Buffer Bias Current
 * MUX bias [5:0]: Configures the bias current of the MUX (function of ADC sampling rate).
 * ADC buffer bias [5:0]: Configures the bias current of the internal reference buffer in the ADC (function of ADC sampling rate).
 */
bool rhs2116_SUPPS_BIASCURR(uint8_t adcBufferBias, uint8_t muxBias) {
	// Ensure the values fit in their respective fields
	adcBufferBias &= 0x3F; // Mask to 6 bits
	muxBias &= 0x3F;	   // Mask to 6 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (adcBufferBias << 6) | (muxBias << 0);
	bool result = rhs2116_writeRegister(RHS_SUPPS_BIASCURR, command, false,
	false);

	return result;
}

/*
 * Configures Register 1: ADC Output Format, DSP Offset Removal, and Auxiliary Digital Outputs
 * DSP cutoff freq [3:0]: Sets the cutoff frequency of the DSP filter for offset removal.
 * DSPen: Enables DSP offset removal when set to 1.
 * absmode: Passes all amplifier ADC conversions through an absolute value function when set to 1.
 * twoscomp: Reports AC high-gain amplifier conversions using two's complement representation when set to 1.
 * weak MISO: Puts MISO line into high impedance mode when CS is high and LVDS_en is low if set to 0.
 * digout1 HiZ: Puts auxout1 into high impedance mode when set to 1.
 * digout1: Drives auxout1 with this bit value when digout1 HiZ is 0.
 * digout2 HiZ: Puts auxout2 into high impedance mode when set to 1.
 * digout2: Drives auxout2 with this bit value when digout2 HiZ is 0.
 * digoutOD: Controls open-drain auxiliary high-voltage digital output pin auxoutOD.
 */
bool rhs2116_OUTFMT_DSP_AUXDIO(uint8_t dspCutoffFreq, bool dspEn, bool absMode,
bool twosComp, bool weakMiso, bool digout1HiZ, bool digout1,
bool digout2HiZ, bool digout2, bool digoutOD) {
	// Ensure the values fit in their respective fields
	dspCutoffFreq &= 0xF; // Mask to 4 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (digoutOD << 12) | (digout2 << 11) | (digout2HiZ << 10)
			| (digout1 << 9) | (digout1HiZ << 8) | (weakMiso << 7)
			| (twosComp << 6) | (absMode << 5) | (dspEn << 4)
			| (dspCutoffFreq << 0);
	bool result = rhs2116_writeRegister(RHS_OUTFMT_DSP_AUXDIO, command, false,
	false);

	return result;
}

/*
 * Configures Register 2: Impedance Check Control
 * Zcheck en: Activates impedance testing mode when set to 1.
 * Zcheck scale [1:0]: Selects the series capacitor for AC current waveform generation (00 = 0.1 pF, 01 = 1.0 pF, 11 = 10 pF).
 * Zcheck load: Adds a capacitor load to the impedance checking network when set to 1 (should be 0 for normal operation).
 * Zcheck DAC power: Activates the on-chip DAC for impedance measurement when set to 1.
 * Zcheck select [5:0]: Selects the electrode for impedance testing.
 */
bool rhs2116_IMPCHK_CTRL(bool zcheckEn, uint8_t zcheckScale,
bool zcheckLoad, bool zcheckDacPower, uint8_t zcheckSelect) {
	// Ensure the values fit in their respective fields
	zcheckSelect &= 0x3F; // Mask to 6 bits
	zcheckScale &= 0x3;	  // Mask to 2 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (zcheckSelect << 8) | (zcheckDacPower << 7)
			| (zcheckLoad << 6) | (zcheckScale << 4) | (zcheckEn << 0);
	bool result = rhs2116_writeRegister(RHS_IMPCHK_CTRL, command, false, false);

	return result;
}

/*
 * Configures Register 3: Impedance Check DAC
 * Zcheck DAC [7:0]: Sets the output voltage of the DAC for impedance checking.
 */
bool rhs2116_IMPCHK_DAC(uint8_t zcheckDac) {
	// Ensure the value fits in its respective field
	zcheckDac &= 0xFF; // Mask to 8 bits

	// Construct the command by placing the value in the correct position
	uint16_t command = zcheckDac;
	bool result = rhs2116_writeRegister(RHS_IMPCHK_DAC, command, false, false);

	return result;
}

/*
 * Configures Register 4: RH1 Cutoff Frequency
 * RH1 sel1 [5:0], RH1 sel2 [4:0]: Sets the upper cutoff frequency of the biopotential amplifiers.
 */
bool rhs2116_RH1_CUTOFF(uint8_t rh1Sel1, uint8_t rh1Sel2) {
	// Ensure the values fit in their respective fields
	rh1Sel1 &= 0x3F; // Mask to 6 bits
	rh1Sel2 &= 0x1F; // Mask to 5 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (rh1Sel2 << 6) | rh1Sel1;
	bool result = rhs2116_writeRegister(RHS_RH1_CUTOFF, command, false, false);

	return result;
}

/*
 * Configures Register 5: RH2 Cutoff Frequency
 * RH2 sel1 [5:0], RH2 sel2 [4:0]: Sets the upper cutoff frequency of the biopotential amplifiers.
 */
bool rhs2116_RH2_CUTOFF(uint8_t rh2Sel1, uint8_t rh2Sel2) {
	// Ensure the values fit in their respective fields
	rh2Sel1 &= 0x3F; // Mask to 6 bits
	rh2Sel2 &= 0x1F; // Mask to 5 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (rh2Sel2 << 6) | rh2Sel1;
	bool result = rhs2116_writeRegister(RHS_RH2_CUTOFF, command, false, false);

	return result;
}

/*
 * Configures Register 6: RL_A Cutoff Frequency
 * RL_A sel1 [6:0], RL_A sel2 [5:0], RL_A sel3: Sets the "A version" of the lower cutoff frequency of the biopotential amplifiers.
 */
bool rhs2116_RL_A_CUTOFF(uint8_t rlASel1, uint8_t rlASel2, bool rlASel3) {
	// Ensure the values fit in their respective fields
	rlASel1 &= 0x7F; // Mask to 7 bits
	rlASel2 &= 0x3F; // Mask to 6 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (rlASel3 << 13) | (rlASel2 << 7) | rlASel1;
	bool result = rhs2116_writeRegister(RHS_ARL_A_CUTOFF, command, false,
	false);

	return result;
}

/*
 * Configures Register 7: RL_B Cutoff Frequency
 * RL_B sel1 [6:0], RL_B sel2 [5:0], RL_B sel3: Sets the "B version" of the lower cutoff frequency of the biopotential amplifiers.
 */
bool rhs2116_RL_B_CUTOFF(uint8_t rlBSel1, uint8_t rlBSel2, bool rlBSel3) {
	// Ensure the values fit in their respective fields
	rlBSel1 &= 0x7F; // Mask to 7 bits
	rlBSel2 &= 0x3F; // Mask to 6 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (rlBSel3 << 13) | (rlBSel2 << 7) | rlBSel1;
	bool result = rhs2116_writeRegister(RHS_ARL_B_CUTOFF, command, false,
	false);

	return result;
}

/*
 * Configures Register 8: Individual AC Amplifier Power
 * AC amp power [15:0]: Powers down individual AC-coupled high-gain amplifiers when set to 0.
 */
bool rhs2116_ACAMP_PWR(uint16_t acAmpPower) {
	// Ensure the value fits in its respective field
	acAmpPower &= 0xFFFF; // Mask to 16 bits

	// The command is the same as the acAmpPower value
	uint16_t command = acAmpPower;
	bool result = rhs2116_writeRegister(RHS_ACAMP_PWR, command, false, false);

	return result;
}

/*
 * Configures Register 10: Amplifier Fast Settle (TRIGGERED REGISTER)
 * amp fast settle [15:0]: Drives AC high-gain amplifier outputs to baseline level when set to 1.
 * Note: Register 10 is a triggered register.
 */
bool rhs2116_AMP_FSTSETL(uint16_t ampFastSettle, bool uFlag) {
	// Ensure the value fits in its respective field
	ampFastSettle &= 0xFFFF; // Mask to 16 bits

	// The command is the same as the ampFastSettle value
	uint16_t command = ampFastSettle;
	bool result = rhs2116_writeRegister(RHS_AMP_FSTSETL, command, uFlag, false);

	return result;
}

/*
 * Configures Register 12: Amplifier Lower Cutoff Frequency Select (TRIGGERED REGISTER)
 * amp fL select [15:0]: Selects between two different lower cutoff frequencies for each AC high-gain amplifier.
 * Note: Register 12 is a triggered register.
 */
bool rhs2116_AMP_LCUTOFF(uint16_t ampFLSelect, bool uFlag) {
	// Ensure the value fits in its respective field
	ampFLSelect &= 0xFFFF; // Mask to 16 bits

	// The command is the same as the ampFLSelect value
	uint16_t command = ampFLSelect;
	bool result = rhs2116_writeRegister(RHS_AMP_LCUTOFF, command, uFlag, false);

	return result;
}

/*
 * Configures Register 32: Stimulation Enable A
 * stim enable A [15:0]: Must be set to 0xAAAA to enable on-chip stimulators.
 */
bool rhs2116_STIM_EN_A(uint16_t stimEnableA) {
	// Ensure the value fits in its respective field
	stimEnableA &= 0xFFFF; // Mask to 16 bits

	// The command is the same as the stimEnableA value
	uint16_t command = stimEnableA;
	bool result = rhs2116_writeRegister(RHS_STIM_EN_A, command, false, false);

	return result;
}

/*
 * Configures Register 33: Stimulation Enable B
 * stim enable B [15:0]: Must be set to 0x00FF to enable on-chip stimulators.
 */
bool rhs2116_STIM_EN_B(uint16_t stimEnableB) {
	// Ensure the value fits in its respective field
	stimEnableB &= 0xFFFF; // Mask to 16 bits

	// The command is the same as the stimEnableB value
	uint16_t command = stimEnableB;
	bool result = rhs2116_writeRegister(RHS_STIM_EN_B, command, false, false);

	return result;
}

/*
 * Configures Register 34: Stimulation Current Step Size
 * step sel1 [6:0], step sel2 [5:0], step sel3 [1:0]: Sets the step size of the current-output DACs in each on-chip stimulator.
 */
bool rhs2116_STIM_CUR_STEP(uint8_t stepSel1, uint8_t stepSel2, uint8_t stepSel3) {
	// Ensure the values fit in their respective fields
	stepSel1 &= 0x7F; // Mask to 7 bits
	stepSel2 &= 0x3F; // Mask to 6 bits
	stepSel3 &= 0x3;  // Mask to 2 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (stepSel3 << 13) | (stepSel2 << 7) | stepSel1;
	bool result = rhs2116_writeRegister(RHS_STIM_CUR_STEP, command, false,
	false);

	return result;
}

/*
 * Configures Register 35: Stimulation Bias Voltages
 * stim Pbias [3:0] and stim Nbias [3:0]: Configures internal bias voltages for the stimulator circuits.
 */
bool rhs2116_STIM_BIAS_VOLTS(uint8_t stimPbias, uint8_t stimNbias) {
	// Ensure the values fit in their respective fields
	stimPbias &= 0xF; // Mask to 4 bits
	stimNbias &= 0xF; // Mask to 4 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (stimPbias << 4) | stimNbias;
	bool result = rhs2116_writeRegister(RHS_STIM_BIAS_VOLTS, command, false,
	false);

	return result;
}

/*
 * Configures Register 36: Current-Limited Charge Recovery Target Voltage
 * charge recovery DAC [7:0]: Sets the output voltage of the DAC for current-limited charge recovery circuits.
 */
bool rhs2116_CHRG_REC_VOLTS(uint8_t chargeRecoveryDac) {
	// Ensure the value fits in its respective field
	chargeRecoveryDac &= 0xFF; // Mask to 8 bits

	// Construct the command by placing the value in the correct position
	uint16_t command = chargeRecoveryDac;
	bool result = rhs2116_writeRegister(RHS_CHRG_REC_VOLTS, command, false,
	false);

	return result;
}

/*
 * Configures Register 37: Charge Recovery Current Limit
 * Imax sel1 [6:0], Imax sel2 [5:0], Imax sel3 [1:0]: Sets the maximum current for the current-limited charge recovery circuit.
 */
bool rhs2116_CHRG_REC_CUR_LIM(uint8_t imaxSel1, uint8_t imaxSel2,
		uint8_t imaxSel3) {
	// Ensure the values fit in their respective fields
	imaxSel1 &= 0x7F; // Mask to 7 bits
	imaxSel2 &= 0x3F; // Mask to 6 bits
	imaxSel3 &= 0x3;  // Mask to 2 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (imaxSel3 << 13) | (imaxSel2 << 7) | imaxSel1;
	bool result = rhs2116_writeRegister(RHS_CHRG_REC_CUR_LIM, command, false,
	false);

	return result;
		}

/*
 * Configures Register 38: Individual DC Amplifier Power
 * DC amp power [15:0]: Powers down individual DC-coupled low-gain amplifiers when set to 0 (not recommended due to a hardware bug).
 */
bool rhs2116_DC_AMP_PWR(uint16_t dcAmpPower) {
	// Ensure the value fits in its respective field
	dcAmpPower &= 0xFFFF; // Mask to 16 bits

	// The command is the same as the dcAmpPower value
	uint16_t command = dcAmpPower;
	bool result = rhs2116_writeRegister(RHS_DC_AMP_PWR, command, false, false);

	return result;
}

/*
 * Reads (only) Register 40: Compliance Monitor
 * compliance monitor [15:0]: This is a read-only variable, but its contents can be cleared to zero by using the M flag.
 */
uint16_t rhs2116_readComplianceMonitor(void) {
	uint16_t value = rhs2116_readRegister(RHS_COMPL_MON, false, false);
	return value;
}

/*
 * Configures Register 42: Stimulator On (TRIGGERED REGISTER)
 * stim on [15:0]: Turns on current sources in corresponding stimulators when set to 1.
 */
bool rhs2116_STIM_ON(uint16_t stimOn, bool uFlag) {
	// Ensure the value fits in its respective field
	stimOn &= 0xFFFF; // Mask to 16 bits

	// The command is the same as the stimOn value
	uint16_t command = stimOn;
	bool result = rhs2116_writeRegister(RHS_STIM_ON, command, uFlag, false);

	return result;
}

/*
 * Configures Register 44: Stimulator Polarity (TRIGGERED REGISTER)
 * stim pol [15:0]: Sets the polarity of current drive in corresponding stimulators.
 * Setting a bit to 0 produces negative current, and setting a bit to 1 produces positive current.
 */
bool rhs2116_STIM_POL(uint16_t stimPol, bool uFlag) {
	// Ensure the value fits in its respective field
	stimPol &= 0xFFFF; // Mask to 16 bits

	// The command is the same as the stimPol value
	uint16_t command = stimPol;
	bool result = rhs2116_writeRegister(RHS_STIM_POL, command, uFlag, false);

	return result;
}

/*
 * Configures Register 46: Charge Recovery Switch (TRIGGERED REGISTER)
 * charge recovery switch [15:0]: Controls on-chip transistor switches for charge recovery.
 * Setting a bit to 1 closes the corresponding switch. Normally, these bits should be set to 0.
 */
bool rhs2116_CHRG_RECOVER(uint16_t chargeRecoverySwitch, bool uFlag) {
	// Ensure the value fits in its respective field
	chargeRecoverySwitch &= 0xFFFF; // Mask to 16 bits

	// The command is the same as the chargeRecoverySwitch value
	uint16_t command = chargeRecoverySwitch;
	bool result = rhs2116_writeRegister(RHS_CHRG_RECOVER, command, uFlag, true);

	return result;
}

/*
 * Configures Register 48: Current-Limited Charge Recovery Enable (TRIGGERED REGISTER)
 * CL charge recovery enable [15:0]: Connects electrodes to a current-limited driver for charge recovery.
 * Setting a bit to 1 connects an electrode to its current-limited driver. Normally, these bits should be set to 0.
 */
bool rhs2116_CUR_LMT_CHRG_REC(uint16_t clChargeRecoveryEnable, bool uFlag) {
	// Ensure the value fits in its respective field
	clChargeRecoveryEnable &= 0xFFFF; // Mask to 16 bits

	// The command is the same as the clChargeRecoveryEnable value
	uint16_t command = clChargeRecoveryEnable;
	bool result = rhs2116_writeRegister(RHS_CUR_LMT_CHRG_REC, command, uFlag,
	true);

	return result;
}

/*
 * Reads (only) Register 50: Fault Monitor
 * fault current detect: This read-only bit is set to one by internal circuitry if the current through 
 * the fault current detector exceeds approximately 20 µA in either direction
 */
uint16_t rhs2116_readFaultMonitor(void) {
	uint16_t value = rhs2116_readRegister(RHS_FAULT_CUR_DET, false, false);
	return value;
}

/*
 * Configures an individual channel's negative stimulation current magnitude and trim (TRIGGERED REGISTERS)
 * channel: The stimulator channel number (0-15).
 * negativeCurrentMagnitude: The magnitude of the negative current for the specified channel.
 * negativeCurrentTrim: The trim value for the negative stimulation current for the specified channel.
 */
bool rhs2116_NEG_CUR_MAG_X(uint8_t channel, uint8_t negativeCurrentMagnitude,
		uint8_t negativeCurrentTrim, bool uFlag) {
	// Ensure the values fit in their respective fields
	negativeCurrentMagnitude &= 0xFF; // Mask to 8 bits
	negativeCurrentTrim &= 0xFF;	  // Mask to 8 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (negativeCurrentTrim << 8) | negativeCurrentMagnitude;
	bool result = rhs2116_writeRegister(channel + RHS_NEG_CUR_MAG_0, command,
	uFlag, false);

	return result;
}

/*
 * Configures an individual channel's positive stimulation current magnitude and trim (TRIGGERED REGISTERS)
 * channel: The stimulator channel number (0-15).
 * positiveCurrentMagnitude: The magnitude of the positive current for the specified channel.
 * positiveCurrentTrim: The trim value for the positive stimulation current for the specified channel.
 */
bool rhs2116_POS_CUR_MAG_X(uint8_t channel, uint8_t positiveCurrentMagnitude,
		uint8_t positiveCurrentTrim, bool uFlag) {
	// Ensure the values fit in their respective fields
	positiveCurrentMagnitude &= 0xFF; // Mask to 8 bits
	positiveCurrentTrim &= 0xFF;	  // Mask to 8 bits

	// Construct the command by shifting and combining the fields
	uint16_t command = (positiveCurrentTrim << 8) | positiveCurrentMagnitude;
	bool result = rhs2116_writeRegister(channel + RHS_NEG_CUR_MAG_0, command,
	uFlag, false);

	return result;
}
