# Bluetooth_serial_inspector_esp32
A quick and easy sketch for a esp32. The goal is to allow for an easy way to connect to a serial-capable device with little more than a phone, esp32 and power bank.

The point was to allow to easily monitor LoRa sensors on the field, and in this specific case decentlab's PR36CTD. These are often installed in remote places, and checking them with a cable and a laptop is impractical. 

Connect jumper cables like so:
Esp32 pin 16 -> device RX
Esp32 pin 17 -> device TX

Tape a powerbank to the back of it, install "Bluetooth serial" on your phone, and you are done!

<img width="1536" height="2048" alt="image" src="https://github.com/user-attachments/assets/81941b57-8b40-4a6b-b736-18c92aee2080" />


