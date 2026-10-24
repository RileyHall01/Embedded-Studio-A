create_clock -name CLOCK_50 -period 20.000 [get_ports CLOCK_50]
derive_clock_uncertainty
set_false_path -from [get_ports {I2C_SDAT KEY0}]
set_false_path -to   [get_ports {I2C_SCLK I2C_SDAT G_SENSOR_CS_N LED[*] MOVEMENT UART_TXD}]
