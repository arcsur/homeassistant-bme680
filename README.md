# homeassistant-bme680
BME680 Sensor Platform for Home Assistant

Default sensors:
  - Temperature
  - Humidity
  - Pressure
  - Air Quality (calculated)
  
Optional Sensor:
  - Gas Resistance
  
*Note that the default i2c_adress is 0x77 (Adafruit board) if you have Pimorini board you will probably want 0x76. If unsure run: i2cdetect -y 1*
___

### Example configuration.yaml
    sensor:
      - platform: bme680
      
### Example configuration.yaml with all paramters
    sensor:
      - platform: bme680
        name: BME680 Sensor
        i2c_bus: 1
        i2c_address: 0x77
        monitored_conditions:
          - temperature
          - humidity
          - pressure
          - gas
          - airquality
        oversampling_temperature: 8
        oversampling_humidity: 2
        oversampling_pressure: 4
        filter_size: 3
        gas_heater_temperature: 320
        gas_heater_duration: 150
        aq_burn_in_time: 300
        aq_humidity_baseline: 40
        aq_humidity_bias: 25

### Example groups.yaml
    climate:
      name: Climate
      entities:
        - sensor.bme680_sensor_temperature
        - sensor.bme680_sensor_humidity
        - sensor.bme680_sensor_pressure
        - sensor.bme680_sensor_air_quality

### Example customize.yaml
    sensor.bme680_sensor_temperature:
      icon: mdi:thermometer
      friendly_name: Temperature
    sensor.bme680_sensor_humidity:
      icon: mdi:water
      friendly_name: Humidity
    sensor.bme680_sensor_pressure:
      icon: mdi:gauge
      friendly_name: Pressure
    sensor.bme680_sensor_air_quality:
      icon: mdi:blur
      friendly_name: Air Quality
