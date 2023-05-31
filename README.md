# Parkinson-Disease
IoT Based Assistive Device for Parkinson's Disease

Parkinson's disease is a brain condition that results in unintentional or uncontrolled movements including trembling, stiffness, and issues with balance and coordination. To allow early detection, differential diagnosis, and objective measurement of symptoms across time, a Parkinson's disease IoT-based device is required. This project focuses on wearable sensors which can offer patients and carers real-time input to assist in the treatment of the condition. Additionally, Parkinson's disease patients can receive individualised and customised treatments and therapies through IoT-based devices, which can enhance their quality of life. 

This IoT-based assistive device for Parkinson's disease will consist of sensors such as accelerometers, gyroscopes, and short-range communication technologies such as Bluetooth and WiFi communication. Additionally, AI-based systems are deployed to develop sophisticated assistive technologies. These sensors will help to track the motion of a patient, detect if they are falling, and also notify the emergency contact with the location information using the GSM module.

**Working:**
1. Firstly, suitable connections were made to connect the GPS module to the gyroscope using MEMS technology. This enabled the device to obtain accurate location and orientation tracking.
2. An Arduino code was written, including the necessary libraries and WIFI username and password, to connect to the WIFI. An event was created and sent, which included a connection to the IFTTT host with a private key. JSON variables were used to read or store the readings produced, and timer variables were created for calculations and storage. 
3. To detect falls, three triggers were initialised at zero, and a sensor object was created to assign the deviation errors of the gyroscope. The WIFI status and MPU chip were checked to ensure a stable connection. 
4. The SPIFFS was initialised, which is a simple file system that manages the HTML and CSS files together, making it easy to implement. The WIFI was set up, and the gyroscope readings were obtained from the sensor, including the acceleration readings. 
5. The readings were then sent to the web page client, which displayed a 3D model tracking of the device. If the gyroscope broke the upper threshold, trigger 1 was activated, which then triggered trigger 2. If the orientation was between 80 to 100 degrees, trigger 3 was activated. If the orientation was between 0 to 10 degrees for some time, this indicated that a fall had been detected. 
6. If the fall was detected, the IFTTT host was invoked using the WIFI, triggering an email notification to the user.
7. All of this setup is connected to a Real-Time Blynk Application which shows the exact status of it as well.

**Technologies Used:**
1. Arduino IDE
2. NodeMCU Esp32
3. Accelerometer Gyroscope Sensor - MPU6050
4. SIM Module (GSM)- SIM800L
5. IFTTT, Twilio
6. Blynk App

**Features Included:**
Here are all the features included in the proposed IoT Based Assistive Device:
1. **Movement tracking:** This feature would use sensors to track the patient's movement and monitor for symptoms such as tremors, stiffness, and slow movement. This information could be used to assess the patient's condition and adjust their treatment as needed.
2. **Fall detection:** People with Parkinson's disease have an increased risk of falls. An assistive device could include sensors to detect falls and alert caregivers or emergency services.
3. **Location Tracking:** Tracks the location of the patient using GPS sensor and notifies it to the emergency contact using GSM module incase of a fall detection
4. **Alerts and Notifications:** If the fall is detected, there are alerts and notifications on E-Mail, message and on the app.
5. **Phone Application:** The Blynk Application shows real-time updates and how the user status is currently.
