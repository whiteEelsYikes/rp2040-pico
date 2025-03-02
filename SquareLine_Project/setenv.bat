rem Sets environment variables for Pico-SDK all-in-one installer to use from normal VScode/commandline sessions
set PICO_INSTALL_PATH=C:\Program Files\Raspberry Pi\Pico SDK v1.5.1
setx PICO_SDK_PATH "%PICO_INSTALL_PATH%\pico-sdk"
setx PATH "%PATH%;%PICO_INSTALL_PATH%\cmake\bin;%PICO_INSTALL_PATH%\ninja;%PICO_INSTALL_PATH%\python;%PICO_INSTALL_PATH%\gcc-arm-none-eabi\bin"

