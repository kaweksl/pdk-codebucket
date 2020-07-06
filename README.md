# Some projects/code for Padauk microcontrollers

Compiled with SDCC (last used 4.0.2 #11679 (Linux) ) - http://sdcc.sourceforge.net/ <br />
Programmed with easy-pdk-programmer (1.3 - latest development branch) - https://github.com/free-pdk <br />
Using https://github.com/free-pdk/pdk-includes.git <br />
and https://github.com/free-pdk/easy-pdk-includes.git

# Short content description:

**PDK8574** - I2C GPIO expander, similar to PCF8574

**i2c-SFRaccess** - created to ease development and debugging pdk uc's
with it you can set/read internal registers through i2c and test values without wasting OTP ic's

**i2c-fancontroller** - WIP - i2c controller for PC FAN (with 4pins)
currently it can: set fan speed trough pwm duty cycle,
read speed in hertz and RPM.
It contains some tips how to read/write t16 counter, use comparator as additional pin change interrupt

**i2c-slave** - some bare minimum i2c slave implementation

**WARNING:** all that code depends on SDCC peephole optimization.
It's pattern search&replace on asm code. I have wrote some rules to meet timing requirements of i2c communication,
they are in peephole_rules folder

# Compiling
```
git clone https://github.com/kaweksl/pdk-codebucket.git
cd pdk-codebucket
git submodule update --init --recursive
```

Edit Makefile in selected project, adjust sdcc and easypdkprog path and<br />

```
make

make flash-PFS154

```