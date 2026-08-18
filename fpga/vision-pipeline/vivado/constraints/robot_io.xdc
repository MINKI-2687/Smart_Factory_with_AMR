# Zybo Z7-20 robot I/O allocation.
# JE1: robot UART TX, JE2: robot UART RX, JE3: SR04 TRIG, JE4: SR04 ECHO.
# SW0: automatic run enable, LED0: run-enabled indicator.
# Standard HC-SR04 ECHO is 5 V. Use a divider or level shifter before JE4.

set_property -dict {PACKAGE_PIN V12 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW} [get_ports robot_uart_txd]
set_property -dict {PACKAGE_PIN W16 IOSTANDARD LVCMOS33 PULLUP true} [get_ports robot_uart_rxd]
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW} [get_ports sr04_trig]
set_property -dict {PACKAGE_PIN H15 IOSTANDARD LVCMOS33 PULLDOWN true} [get_ports sr04_echo]
set_property -dict {PACKAGE_PIN G15 IOSTANDARD LVCMOS33} [get_ports run_enable]
set_property -dict {PACKAGE_PIN M14 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW} [get_ports run_led]
