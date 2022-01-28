# Tire-warmer-power-board

  This is the software for seeeduino xiao on the original replacement power board of G-FORCE TYRE WARMER.
  http://www.gforce-hobby.jp/products/G0033.html
  
  Xiao senses temparature of 4 tires with ADS1115.
  Xiao controls 4 heater output for each tire with PWM PID.
  User interface is the same as the G-FORCE one, but the interface board is new.
  mcp23017 on it controls 1602 LCD and 4 button inputs through I2C connection with xiao.
  It's like Adafruit LCD keypad board.
  https://learn.adafruit.com/adafruit-16x2-character-lcd-plus-keypad-for-raspberry-pi
  
  If you want to know schematic, PCB layout, LTSpice sim, refer to github site below.
  https://github.com/mayopan/Tire-warmer-power-board.git

  Date:       January 29, 2022
  Copyright:  mayopan = ROUTE 246 GARAGE
  License:    MIT. See license file for more information but you can basically do whatever you want with this code.

  Dependancy: It uses 2 library for ADC and LCD. LCD library is wrapped in the TireWarmer library.
    * https://github.com/wollewald/ADS1115_WE.git
    * lincomatic/LiquidTWI2 @ ^1.2.7
