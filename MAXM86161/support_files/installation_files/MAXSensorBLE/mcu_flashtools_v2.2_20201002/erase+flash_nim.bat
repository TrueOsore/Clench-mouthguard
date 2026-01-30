openocdbinMaxim0.9\openocd -f max32620_swd_cmsisdap.cfg -c "init" -c "halt" -c "maxim mass_erase 0" -c "exit"
pause
openocdbinMaxim0.9\openocd -f max32620_swd_cmsisdap.cfg -c "program os58_nim_bl_v1.0.hex verify exit"
pause
openocdbinMaxim0.9\openocd -f max32620_swd_cmsisdap.cfg -c "program os58_nim_app_v2.1.elf verify exit"
pause
