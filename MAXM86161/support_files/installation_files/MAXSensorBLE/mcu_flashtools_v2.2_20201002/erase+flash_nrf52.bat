openocdbin0.10\openocd -f nrf52_isyst_cmsisdap.cfg -c "init" -c "halt" -c "nrf51 mass_erase" -c "exit"
pause
openocdbin0.10\openocd -f nrf52_isyst_cmsisdap.cfg -c "program os58shHost_nrf52_full_v2.2_20201002.hex verify exit"
pause