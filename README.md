# STM32-Project_CAN-servo

Small project on CAN protocol between MCUs (CAN nodes), controlling servo motor through sensor signal (potentiometer module), specifically:
- Sensor Node: Read value from sensor (angle) through ADC and transmit that value to Actuator Node through CAN, use UART protocol to display message/debug.
- Actuator Node: Receive message from Sensor Node (sensor value) and control servo motor by TIMER PWM, use UART protocol to display message/debug