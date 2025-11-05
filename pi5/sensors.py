#!/usr/bin/python3
# -*- coding:utf-8 -*-

import sys
import os
import time
import json
import datetime

# --- IMPORT BOTH LIBRARIES ---
import lgpio as sbc  # For SHTC3
import smbus         # For LPS22HB, IMU, TCS34087, SGM58031, INA219

# --- IMPORT SENSOR MODULES ---
try:
    import SHTC3
    import LPS22HB
    import IMU        
    import TCS34087
    import SGM58031
    import INA219     # <-- Added UPS monitor module
except ImportError as e:
    print(f"Error: Failed to import sensor modules: {e}")
    sys.exit(1)

# --- Configuration ---
JSON_FILE_PATH = "/dev/shm/sensor_data.json"
UPDATE_INTERVAL = 0.1  # seconds 
ADC_VOLTAGE_MULTIPLIER = 0.000125  # 0.125mV per bit = 0.000125V per bit

# --- I2C Bus and Addresses ---
I2C_BUS_NUMBER = 1
ADDR_SHTC3     = 0x70
ADDR_INA219    = 0x41 # <-- Address for your UPS monitor

print(f"Starting sensor logger. Data will be written to {JSON_FILE_PATH}")
print("Press CTRL+C to stop.")

# --- HELPER FUNCTION FOR LPS22HB ---
def read_lps_data(sensor):
    sensor.LPS22HB_START_ONESHOT()
    time.sleep(0.02) # Give it time to measure

    try:
        u8BufP = [0,0,0]
        u8BufP[0]=sensor._read_byte(LPS22HB.LPS_PRESS_OUT_XL)
        u8BufP[1]=sensor._read_byte(LPS22HB.LPS_PRESS_OUT_L)
        u8BufP[2]=sensor._read_byte(LPS22HB.LPS_PRESS_OUT_H)
        pressure = ((u8BufP[2]<<16) + (u8BufP[1]<<8) + u8BufP[0]) / 4096.0
    except Exception as e:
        print(f"Warning: Failed to read LPS22HB Pressure: {e}")
        pressure = 0.0

    try:
        u8BufT = [0,0]
        u8BufT[0]=sensor._read_byte(LPS22HB.LPS_TEMP_OUT_L)
        u8BufT[1]=sensor._read_byte(LPS22HB.LPS_TEMP_OUT_H)
        temperature = ((u8BufT[1]<<8) + u8BufT[0]) / 100.0
    except Exception as e:
        print(f"Warning: Failed to read LPS22HB Temperature: {e}")
        temperature = 0.0
        
    return pressure, temperature

# --- Main Program ---
def main():
    # --- Initialize ALL sensors + UPS ---
    shtc3 = None
    lps22hb = None
    imu = None
    tcs34087 = None
    sgm58031 = None
    ina219 = None # Initialize ina219 variable

    try:
        # SHTC3 uses lgpio
        shtc3 = SHTC3.SHTC3(sbc, I2C_BUS_NUMBER, ADDR_SHTC3)
        print("SHTC3 (Temp/Humidity) Initialized (lgpio).")
        
        # LPS22HB uses smbus and takes no args
        lps22hb = LPS22HB.LPS22HB()
        print("LPS22HB (Pressure) Initialized (smbus).")

        # IMU uses smbus and takes no args
        imu = IMU.IMU()
        print("IMU (Accel/Gyro/Mag) Initialized (smbus).")

        # TCS34087 uses smbus, takes no args, but needs .TCS34087_init()
        tcs34087 = TCS34087.TCS34087()
        if tcs34087.TCS34087_init() != 0:
            print("TCS34087 (Color Sensor) FAILED to initialize!")
            tcs34087 = None # Disable it
        else:
            print("TCS34087 (Color Sensor) Initialized (smbus).")

        # SGM58031 uses smbus and its class is ADS1015
        sgm58031 = SGM58031.ADS1015()
        print("SGM58031 (ADC) Initialized (smbus).")

        # INA219 uses smbus and needs the correct address
        ina219 = INA219.INA219(addr=ADDR_INA219)
        print(f"INA219 (UPS Monitor @ 0x{ADDR_INA219:02x}) Initialized (smbus).")
        
    except Exception as e:
        print(f"Error initializing devices: {e}")
        sys.exit(1)

    # --- Main Loop ---
    while True:
        try:
            # Create the dictionary structure
            all_data = {}
            all_data['timestamp_utc'] = datetime.datetime.utcnow().isoformat()
            all_data['environment'] = {}
            all_data['motion'] = {}
            all_data['light'] = {}
            all_data['adc_volts'] = {}
            all_data['power'] = {} # <-- Added section for UPS data

            # 1. Read Temperature & Humidity (SHTC3)
            if shtc3:
                try:
                    temp_c = shtc3.SHTC3_Read_TH()
                    humidity = shtc3.SHTC3_Read_RH()
                    all_data['environment']['temperature_shtc_c'] = round(temp_c, 2)
                    all_data['environment']['temperature_f'] = round(temp_c * 9/5 + 32, 2)
                    all_data['environment']['humidity_rh'] = round(humidity, 2)
                except Exception as e:
                    print(f"Error reading SHTC3: {e}")

            # 2. Read Barometric Pressure (LPS22HB)
            if lps22hb:
                try:
                    pressure, lps_temp = read_lps_data(lps22hb)
                    all_data['environment']['pressure_hpa'] = round(pressure, 2)
                    all_data['environment']['temperature_lps_c'] = round(lps_temp, 2)
                except Exception as e:
                    print(f"Error reading LPS22HB: {e}")

            # 3. Read 9-DoF Motion (IMU)
            if imu:
                try:
                    imu.QMI8658_Gyro_Accel_Read()
                    imu.AK09918_MagRead()
                    imu_temp = imu.QMI8658_readTemp()
                    all_data['environment']['temperature_imu_c'] = round(imu_temp, 2)

                    all_data['motion'] = {
                        'accelerometer_raw': { 'x': IMU.Accel[0], 'y': IMU.Accel[1], 'z': IMU.Accel[2] },
                        'gyroscope_raw': { 'x': IMU.Gyro[0], 'y': IMU.Gyro[1], 'z': IMU.Gyro[2] },
                        'magnetometer_raw': { 'x': round(IMU.Mag[0], 2), 'y': round(IMU.Mag[1], 2), 'z': round(IMU.Mag[2], 2) }
                    }
                except Exception as e:
                    print(f"Error reading IMU: {e}")
            
            # 4. Read Color Sensor (TCS34087)
            if tcs34087:
                try:
                    tcs34087.Get_RGBData()
                    lux = 0.0
                    color_temp_k = 0.0
                    try:
                        lux = tcs34087.Get_Lux()
                    except ZeroDivisionError: pass 
                    try:
                        color_temp_k = tcs34087.Get_ColorTemp()
                    except ZeroDivisionError: pass 
                    
                    all_data['light'] = {
                        'red_raw': tcs34087.R, 'green_raw': tcs34087.G, 'blue_raw': tcs34087.B,
                        'clear_raw': tcs34087.C, 'lux': round(lux, 2), 'color_temp_k': round(color_temp_k, 2)
                    }
                except Exception as e:
                    print(f"Error reading TCS34087: {e}")

            # 5. Read ADC (SGM58031)
            if sgm58031:
                try:
                    adc_volts_list = []
                    for i in range(4):
                        raw = sgm58031.ADS1015_SINGLE_READ(i)
                        if raw > 32767: raw = raw - 65536
                        volts = raw * ADC_VOLTAGE_MULTIPLIER
                        adc_volts_list.append(round(volts, 4))
                        
                    all_data['adc_volts'] = {
                        'ain0': adc_volts_list[0], 'ain1': adc_volts_list[1],
                        'ain2': adc_volts_list[2], 'ain3': adc_volts_list[3]
                    }
                except Exception as e:
                    print(f"Error reading SGM58031 (ADC): {e}")
            
            # 6. Read Power Monitor (INA219) <-- Added
            if ina219:
                try:
                    bus_voltage = ina219.getBusVoltage_V()
                    shunt_voltage_mv = ina219.getShuntVoltage_mV()
                    current_ma = ina219.getCurrent_mA()
                    power_w = ina219.getPower_W()
                    
                    # Calculate percentage
                    percent = (bus_voltage - 9.0) / 3.6 * 100.0
                    if percent > 100.0: percent = 100.0
                    if percent < 0.0: percent = 0.0
                    
                    all_data['power'] = {
                        'bus_voltage_V': round(bus_voltage, 3),
                        'shunt_voltage_V': round(shunt_voltage_mv / 1000.0, 6),
                        'current_A': round(current_ma / 1000.0, 6),
                        'power_W': round(power_w, 3),
                        'battery_percent': round(percent, 1)
                    }
                except Exception as e:
                    print(f"Error reading INA219 (UPS): {e}")

            # 7. Write the data to the JSON file in RAM
            temp_file_path = JSON_FILE_PATH + ".tmp"
            with open(temp_file_path, 'w') as f:
                json.dump(all_data, f, indent=4)
            
            os.rename(temp_file_path, JSON_FILE_PATH)
            print(f"Updated {JSON_FILE_PATH} at {all_data['timestamp_utc']}")

            time.sleep(UPDATE_INTERVAL)

        except KeyboardInterrupt:
            print("\nStopping logger.")
            break
        except Exception as e:
            print(f"An error occurred in the main loop: {e}")
            time.sleep(UPDATE_INTERVAL)

if __name__ == '__main__':
    main()