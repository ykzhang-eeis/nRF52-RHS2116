# nRF52-RHS2116

This firmware is to connect nRF52 (Arm Cortex M4) from Nordic with RHS2116 (Intan) - a digital electrophysiology stimulator/amplifier chip to measure biosignals with multiple channels. The chip is targeted for medical instrument application (EEG and ECG) with high precision (16-bit), simultaneous sampling and multichannel signal acquisition. The data rate is configurable to 40 kSPS for all 16 channels with RMS of 2.4 µVrms.

The firmware code was comprised of three main parts:

- Connect the nRF52 SoC with the RHS2116 chip through the Documents/BCI_V0.pdf
- Read data from the RHS2116 chip using SPI communication protocol
- Transfer the data through Bluetooth Low Energy (BLE) to a nRF52 or mobile app

