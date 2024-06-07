# IDE
- Visual Studio Code with PlatformIO extension for building and uploading to ESP32

# Latest Commit 
### 29 - PGH Deployment Continuous Transmission
- if SD card fails, program will still continue
- payload is also stored in a backup String variable, but gets replaced when a new measurement is made every minute

# Microcontrollers
### ESP32_DevKitC_V4 (WROOM-32E)
- [Datasheet] (https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf)
- Pinout - ![alt text](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/_images/esp32-devkitC-v4-pinout.png)
- Schematic - ![alt text](https://github.com/Maximus112901/coe-199/blob/main/ESP32%20Schematic.png)

### ESP32-C3_DevKitC_02_V1.1
- Datasheet - https://www.espressif.com/sites/default/files/documentation/esp32-c3-wroom-02_datasheet_en.pdf
- Pinout - ![alt text](https://mischianti.org/wp-content/uploads/2023/04/ESP32-C3-DevKitC-02-pinout-low.jpg)
- Schematic - ![alt text](https://github.com/Maximus112901/coe-199/blob/main/ESP32-C3%20Schematic.png)
  
### ESP32 Timestamps
- Guide - https://github.com/fbiego/ESP32Time?utm_source=platformio&utm_medium=piohome

# PCB
### KiCad Files
- Google Drive - https://drive.google.com/drive/folders/152lUshNZ_tiQgE9zgHFScArmlD4AZO-O?fbclid=IwAR0LLwBYDXclYYr9I1wbEfdIuMBKrMFNuLx9QXKqK2TGHA81PWG0cbftSYs_aem_ATW_DaHc2KigtcTPPmrPSTEdQ07d6icW96nvpqb2iTsrmtVP7gO4rqNt-TygdpFyrAxJ00rtscgP_fLmlB0El7N9

# Sensor Functionality
### DFRobot BME680 References
- Wiki - https://wiki.dfrobot.com/BME680_Environmental_Sensor_Module_SKU_SEN0375
- Use different library and code for C3 - https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/issue-when-reading-BME680/td-p/15463

### DFRobot MiCS-4514 References
- Wiki - https://wiki.dfrobot.com/_SKU_SEN0377_Gravity__MEMS_Gas_Sensor_CO__Alcohol__NO2___NH3___I2C___MiCS_4514

### MiCS-VZ89TE References
- Datasheet - https://cdn.sparkfun.com/assets/9/f/1/d/b/MiCS-VZ-89TE_datasheet.pdf
- Program - https://forum.arduino.cc/t/mics-vz-89te-library-error/671920/2

### ULPSM-SO2 968-006 References
- Datasheet - https://www.spec-sensors.com/wp-content/uploads/2016/10/ULPSM-SO2-968-006.pdf

### Sensirion SPS30
- Datasheet - https://sensirion.com/media/documents/8600FF88/616542B5/Sensirion_PM_Sensors_Datasheet_SPS30.pdf
- Program - https://github.com/paulvha/sps30/blob/master/examples/Example13_sps30_BasicReadings_any_I2C/Example13_sps30_BasicReadings_any_I2C.ino
- Mounting Guide - https://www.mouser.com/pdfdocs/PS_AN_SPS30_Mechanical_Design_and_Assembly_Guidelines_v10_D2.pdf

# PainlessMesh
### Documentation
- Gitlab - https://gitlab.com/painlessMesh/painlessMesh
- Functions - https://painlessmesh.gitlab.io/painlessMesh/index.html

### Example
- Program - https://randomnerdtutorials.com/esp-mesh-esp32-esp8266-painlessmesh/
- Sleeping Mesh - https://gitlab.com/painlessMesh/painlessMesh/-/issues/417?fbclid=IwAR3UEwYah37u_YzGcpG8D9W-hjqygD7NTMUpdk4QdzcDEWCgic_TH96RfBc

# Storage
### DFRobot SD Card Module
- Wiki - [https://wiki.dfrobot.com/MicroSD_card_module_for_Arduino__SKU_DFR0229_]
- SDFat - https://github.com/greiman/SdFat
- Example - https://codebender.cc/example/SdFat/ReadWriteSdFat#ReadWriteSdFat.ino

# Battery Module
### INA219
- Datasheet - https://www.ti.com/lit/ds/symlink/ina219.pdf
- Example - https://how2electronics.com/how-to-use-ina219-dc-current-sensor-module-with-arduino/
- Example - https://www.youtube.com/watch?v=lrugreN2K4w&t=266s

### 134N3P Boost Converter (Switching Voltage Regulator)
- Datasheet -
- Example -

# RaspberryPi
### Running a Python Script on Startup
- https://www.raspberrypi-spy.co.uk/2015/10/how-to-autorun-a-python-script-on-boot-using-systemd/

### Running a Bash Script on Startup
- How to - https://www.youtube.com/watch?v=jcE8U1lG514
- Syncing Time w/ ntpdate - https://www.youtube.com/watch?v=Nyr5DI0fuAY&t=265s

### Python Timestamps
- strftime() Formats - https://strftime.org/
- Get Epoch - https://www.geeksforgeeks.org/convert-python-datetime-to-epoch/
- Serial Write - https://stackoverflow.com/questions/22275079/pyserial-write-wont-take-my-string
- Clear Serial Buffer - https://forum.arduino.cc/t/how-to-clear-the-serial-buffer/363891/3

### Simultaneous AP and Client Mode
- Tutorial - https://www.youtube.com/watch?v=gdtkHZtfsbw
- May need to install dhcpcd and iptables
- May not be robust for changing connections

### Hotspot with WiFi Dongle
- https://www.raspberrypi.com/tutorials/host-a-hotel-wifi-hotspot/
- Make sure wlan1 is used for internet and wlan0 is free
- Dongle used for this project - [Lazada Link](https://www.lazada.com.ph/products/usb-wifi-bluetooth-adapter-600mbps-dual-band-245ghz-wireless-network-external-receiver-mini-wifi-dongle-i4372452844-s24537930834.html?c=&channelLpJumpArgs=&clickTrackInfo=query%253Ausb%252Bwifi%252Badapter%253Bnid%253A4372452844%253Bsrc%253ALazadaMainSrp%253Brn%253A7956d6b46ce49505da403758412b5afe%253Bregion%253Aph%253Bsku%253A4372452844_PH%253Bprice%253A399%253Bclient%253Adesktop%253Bsupplier_id%253A500355856049%253Bbiz_source%253Ah5_hp%253Bslot%253A0%253Butlog_bucket_id%253A470687%253Basc_category_id%253A5194%253Bitem_id%253A4372452844%253Bsku_id%253A24537930834%253Bshop_id%253A4125889&fastshipping=0&freeshipping=1&fs_ab=2&fuse_fs=&lang=en&location=Metro%20Manila&price=399&priceCompare=skuId%3A24537930834%3Bsource%3Alazada-search-voucher%3Bsn%3A7956d6b46ce49505da403758412b5afe%3BunionTrace%3A0125419917135240504298855e%3BoriginPrice%3A39900%3BvoucherPrice%3A39900%3BdisplayPrice%3A39900%3BsinglePromotionId%3A-1%3BsingleToolCode%3AmockedSalePrice%3BvoucherPricePlugin%3A1%3BbuyerId%3A0%3Btimestamp%3A1713524050909&ratingscore=4.9965635738831615&request_id=7956d6b46ce49505da403758412b5afe&review=291&sale=4849&search=1&source=search&spm=a2o4l.searchlist.list.0&stock=1)

### Mosquitto Broker
- Allow anonymous connections (External Survey Terminal) - https://www.youtube.com/watch?v=BFyPzC6No8k

# Node-RED
### Dashboard
- Process a CSV - https://www.youtube.com/watch?v=rq3fukD4RJY&t=233s
- Dashboard 2.0 - https://dashboard.flowfuse.com/getting-started.html
- Show/Hide Charts - https://www.youtube.com/watch?v=Kim0hor3j0k

# Grafana

# Hosting Dashboards Online
### playit.gg
- Website - https://playit.gg/
