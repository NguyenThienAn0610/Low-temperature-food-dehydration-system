# Low-temperature-food-dehydration-system
This is a simple project I had to make in the course Microcontrollers-Microprocessors at HCMUT.
<ul>
  <li>The Project includes reading the temperature from the DHT11 sensor. The heat pump and heater are then operated based on the values recorded from the sensor.</li>
  <li>Visualization includes turning on and off corresponding LEDs since I didn't get access to physical heat pump and heater. The temperature and humidity are also shown onto the LCD screen for better view of the situation.
</ul>
<strong>Note</strong>: Since it was infeasible to create a real significant change to the temperature and humidity due to no real heater and heat pump, I decided to simulate the changes by code. In specific, the temperature will increase by 0.2*C/s if heater is on and decrease by 0.1*C/s if heat pump is on.
