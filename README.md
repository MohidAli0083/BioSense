Project Documentation
Project: BioSense
DT337C VT25
Fakhra Munawar, Mohid Ali


1.	Project Overview
1.1 Purpose of the system
The BioSense BLE-enabled Multi-Sensor Monitoring System is developed to monitor physiological signals (ECG and motion) using a compact and energy-efficient setup. The system utilizes Bluetooth Low Energy (BLE) to wirelessly transmit real-time sensor data to a mobile device. The sensors are embedded in two separate units:
•	An ECG module (AD8232) placed on the chest to record heart activity.
•	A motion sensor (MPU6050) attached to the wrist or leg to detect body movement.
The system is designed to:
•	Optimize power consumption using the low-power capabilities of the nRF52840 microcontroller.
•	Ensure reliable data communication over BLE.
•	Enable users to monitor sensor values through nRF Connect, a mobile app available on Android and iOS.
•	Support basic medical and fitness observation use cases, especially in remote or low-resource environments.
1.2 Summary of Hardware and Software Integration
Hardware Integration:
•	nRF52840 Development Kit: Central microcontroller with BLE support, responsible for collecting and transmitting sensor data.
•	AD8232 ECG Sensor: Measures electrical signals from the heart; connected to analog and digital input pins on the nRF52840.
•	MPU6050 Accelerometer/Gyroscope: Captures motion data; communicates via I2C interface with the microcontroller.
•	Power Supply (3.3V): Powers both sensor nodes efficiently to support low-power operation.
•	Electrodes and wiring: Properly placed to capture physiological signals effectively.
Software Integration:
•	Developed using Segger Embedded Studio and the nRF5 SDK.
•	BLE profiles and characteristics configured to stream ECG and motion data in real time.
•	Compatible with nRF Connect Mobile App, which receives and displays data on Android (BLE 4.3+) and iOS (BLE 5.0+) devices.
•	Includes simple data handling logic for connection, notification, and sensor data parsing.
2. System Architecture
 ![image](https://github.com/user-attachments/assets/79c962d5-f0d4-4fa8-843b-73c3cd6f7633)
3. Hardware Design
3.1 List of Components
•	NRF52840-DK (Nordic Semiconductor) x 2. [1]
•	MPU-6050 (DFRobot). [2]
•	AD8232 (SparkFun Electronics). [3]
•	Power Profiler Kit II (Nordic Semiconductor). [4]
•	3.7V Lithium Battery
•	USB to Micro-USB Cable x 3
•	3-Pin Electrode Cable
•	Biomedical Electrode
•	Jumper Wires








3.2 Schematic Diagram
Diagram 1: Shows the schematic diagram of nRF52840 and AD8232 Pin Connections
 
![image](https://github.com/user-attachments/assets/a8b974a1-f629-4dd4-9121-dd884498b39f)





3.3 Wiring and Connections
Figure 1: Shows the connection between nRF52840 and AD8232 Sensor
 
![image](https://github.com/user-attachments/assets/42134191-e6ff-4547-9d3d-7c4e8db5b452)





Figure 2: Shows the connection between nRF52840 and MPU6050
 
![image](https://github.com/user-attachments/assets/9143abc6-78ae-4228-9047-25d660bdf3f2)
4. Software Design
4.1 Programming Environment
•	Segger Embedded Studio 8.22a. [5]
•	NRF Connect for Desktop [6]
•	NRF Connect for Mobile
4.2 Firmware Structure
•	BLE Stack Initialization:
o	nrf_sdh_ble.h, ble_advdata.h, ble_advertising.h: Sets up BLE advertising, GAP parameters, and services.
o	Device name: “Accel_sensor”, “ECG Sensor”
o	Advertising interval: 300 units (~187.5ms)
•	GATT, Peer Manager, and Security:
o	GATT Configuration: nrf_ble_gatt.h
o	Security settings: bonding enabled, no MITM
o	Peer manager used for handling bonded devices
•	Timers and Power Management
o	App timer: app_timer.h
o	Power management via nrf_pwr_mgmt.h
•	Custom BLE Service: BLE_ACCEL_SERVICE
o	Created using ble_accel_service.h
o	Sends motion data over BLE notifications when a connection is active
o	CCCD is checked before sending notifications to ensure client subscribed
•	Connection and GATT Events
o	Handles BLE Connection, disconnection and MTU negotiation
•	Peer Manager Events
o	Handles bonded device data and re-connections
•	Advertising and Bond Management
o	Starts advertising on boot unless bond deletion is requested
o	Bonded peer data can be erased for testing or pairing with a new device
•	Logging and Debugging
o	Enabled via nrf_log.h, nrf_log_ctrl.h
o	Logs connection status, MTU updates, and accelerometer output.
4.3 Git Repository URL

5. Mobile App Integration
1.	Using nRF Connect App
2.	Search for Device name: “Accel_sensor”
3.	After connecting, upload “1 or on” to UUID 2902.
4.	Observe UUID: “00001234-1234-5678-1234-56789ABCDEF0”, starts sending value.
5.	Open Log tab to monitor RSSI, and Sensor values.
6.	Export/Save data in different formats (CSV, Text)

Bibliography

[1] 	“nRF52840-DK Product Details,” Nordic semiconductor, [Online]. Available: https://www.nordicsemi.com/Products/Development-hardware/nRF52840-DK.
[2] 	“Digikey, website for order product & Datasheet of product,” DFRobot, [Online]. Available: https://www.digikey.com/en/products/detail/dfrobot/SEN0142/6588492.
[3] 	“Digikey, Analog front end for ECG datasheet and product,” Sparkfun Electronics, [Online]. Available: https://www.digikey.com/en/products/detail/sparkfun-electronics/12650/5824153.
[4] 	“Nordic Semiconductor, Documentation & Certification with key features,” Nordic Semiconductor, [Online]. Available: https://www.nordicsemi.com/Products/Development-hardware/Power-Profiler-Kit-2.
[5] 	“Embedded Segger Studio download website,” Segger, [Online]. Available: https://www.segger.com/downloads/embedded-studio/.
[6] 	“nRF Connect for Desktop, Cross-platform development software for Nordic Products,” Nordic Semiconductor, [Online]. Available: https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop.




