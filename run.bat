@echo off
echo Compiling RTL and Verilog Self-Checking Testbench...
iverilog -o sim.out rtl\spi_master.v rtl\apb_slave.v rtl\apb_spi_top.v tb\tb_apb_spi.v
if %errorlevel% neq 0 exit /b %errorlevel%

echo Running Simulation...
vvp sim.out