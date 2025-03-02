# CMake / Visual Studio Code / Eclipse project for SquareLine Studio and LVGL - with 32bit Raspberry Pi Pico as compile-target


This project can be used to cross-build Raspberry Pi Pico C/C++ UI code exported from SquareLine Studio.
(Builds are tested thoroughly on Linux with CMake and VScode. In Eclipse and on Windows some Pico-SDK and C++ stdlib PATH related tweaks might be needed at the moment.)


## Prerequisites

General note: Please avoid folder-names and filenames containing non-ASCII (special/accented/foreign) characters for the installed tools and your exported projects. Some build-tools and terminals/OS-es don't handle those characters well or interpret them differently, which can cause issues during the build-process.


### The official Pico-SDK environment is required to build for Raspberry Pico:

#### On Windows you have the option for an all-in-one installer right from Raspberry Pi Ltd at [Pico Windows Installer downloads](https://github.com/raspberrypi/pico-setup-windows/releases)
- Get the latest installer file, e.g.: [pico-setup-windows-x64-standalone.exe](https://github.com/raspberrypi/pico-setup-windows/releases/download/v1.5.1/pico-setup-windows-x64-standalone.exe)
- It creates special startup-menu entries for VScode and PowerShell which sets the required environment variables at launch and works out-of-the-box.
  (It does more than what is necessary for Pico-development, adds VScode extensions which might clash with the already installed, builds examples.)
- If you want to use Pico-SDK with the normally launched VScode/commandline you should add the included cmake,ninja,python,gcc,pico-sdk folders to the environment variables:
  - setenv.bat is made to make this easier for you, just run that file and it appends/creates the environment variables with the content:
    - sets PICO_SDK_PATH to <AppDir>\pico-sdk
    - adds to PATH: <AppDir>\pico-sdk ; <AppDir>\cmake\bin ; <AppDir>\ninja ; <AppDir>\python ; <AppDir>\gcc-arm-none-eabi\bin
    - (AppDir is 'C:\Program Files\Raspberry Pi\Pico SDK v1.5.1' for the current version if you didn't change it during installation.)
- You can skip the rest of the installations listed below and go on with VScode installation or Usage section

#### Otherwise download Pico-SDK separately from its [GitHub page](https://github.com/raspberrypi/pico-sdk)
- with the ['Download Zip'](https://github.com/raspberrypi/pico-sdk/archive/refs/heads/master.zip) option and extract it into a folder, or by git if you have it: `git clone https://github.com/raspberrypi/pico-sdk.git`
  (You can update communication/etc submodules according to Pico-SDK documents, if you like: `git submodule update --init`. For the Pico-W you'll need cyw43 wireless anyway for a simple built-in LED-blinking. )
- Add the environment variable PICO_SDK_PATH with the value pointing to the pico-sdk folder extracted above (CMake will need it)

### Python3 will be required to use Pico-SDK:

- On Windows download the Python3 installer from [Python download page](https://www.python.org/downloads/windows/), at the moment it is: [python-3.12.0-amd64.exe](https://www.python.org/ftp/python/3.12.0/python-3.12.0-amd64.exe)
  - Don't forget to tick the option 'Add python.exe to PATH', then choose 'Install Now' (basic settings are OK)
- On Linux/OSX you probably already have Python3 but if not, you can install it from the package-manager

### Get and install the essential build-tools (binutils,compiler,linker,make,etc.) on your host-PC if you don't have them yet (needed by Pico-SDK to build some interim x86 tools):

- On Windows [MinGW](https://www.mingw-w64.org/) contains them, the POSIX+SEH+UCRT-build variant at their [GitHub repository](https://github.com/niXman/mingw-builds-binaries/releases) works fine on Windows 10:
  - Download for example: [x86_64-13.2.0-release-posix-seh-ucrt-rt_v11-rev0.7z](https://github.com/niXman/mingw-builds-binaries/releases/download/13.2.0-rt_v11-rev0/x86_64-13.2.0-release-posix-seh-ucrt-rt_v11-rev0.7z)
  - Extract/copy the compressed 7z file's included 'mingw64' folder to C:\  (Can be other folder but we continue with this here.)
  - You need to add the 'C:\mingw64\bin' folder to your PATH environment variable (Unfortunately there's no up-to-date mingw64 installer that sets it for us.)
  (On Windows 10 right-click on This PC, go to Advanced System Settings / Environment Variables, select 'Path' and add "C:\mingw64\bin" to an empty line.)
  - (You can check if the setting works by typing 'mingw32-make' on command line.)
- On Linux type in the Terminal: `sudo apt-get install gcc g++ gdb build-essential`
- On MacOS install latest Xcode

### To compile for Raspberry Pico you'll need the cross-compiler targeting 32bit ARM architecture

Download one 'EABI' ('Embedded Application Binary Interface') variant for your host-OS from ARM's official homepage:
  - Find it at webpage [Current ARM GNU Toolchain downloads](https://developer.arm.com/downloads/-/gnu-rm)
  - For Windows choose the exe installer which sets PATH environment variable for you (be sure to check that option when asked!): [gcc-arm-none-eabi-10.3-2021.10-win32.exe](https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-win32.exe)
  - For Linux: [gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2](https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2)
  (Alternatively you can install by package manager on Linux, for example in LMDE 5 the 10.2 version of gcc-arm could be installed by: `sudo apt-get install gcc-arm-none-eabi` .)
  - For OSX: [gcc-arm-none-eabi-10.3-2021.10-mac.tar.bz2](https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-mac.tar.bz2)
  - After extracting these packages to your preferred place you need to add the contained 'bin' folder to your PATH so you can run arm-none-eabi-gcc and the others from any folder.

### You need to get CMake tool to generate the build-chain (makefiles) for the host PC's platform:

- For Windows go to [CMake download page](https://cmake.org/download/) and select the [CMake MSI-installer binary distribution](https://github.com/Kitware/CMake/releases/download/v3.28.0-rc2/cmake-3.28.0-rc2-windows-x86_64.msi)
  (When asked by the installer, select the option that will take care of setting the PATH variable for the cmake executable.)
- For Linux you just need to get and install CMake from the repository, for example `sudo apt-get install cmake`


### If you want GUI-based build you can get and install [Visual Studio Code](https://code.visualstudio.com/download)

- After running Visual Studio Code, you need to download 2 extensions: 'C/C++ Extension Pack' and 'CMake Tools'

### Or you can choose Eclipse CDT installed on your machine:
- Download it from [Eclipse Download Page](https://www.eclipse.org/downloads/)
- When asked about variants, choose Eclipse for C/C++ Developers from the list (it's usually abbreviated as CDT)
- Wait for the installation to finish, it might take a while (and seem to stall falsely complaining about slow download, but ignore it and wait patiently.)



## Usage - configuring and building the project


You have probably selected the Raspberry Pi Pico board already and created a project with it in SquareLine Studio.
Be sure that you selected the colour-depth of your display-panel in the project-settings correctly. (e.g. '16bit swap' for ILI9341 display)
In SquareLine Studio click 'Create Template Project' and select the folder to save it to.

This board-file was tested with Raspberry Pico W (with built-in wireless chip). (It is 'pico_w' or 'pico' in CMakeLists.txt file that determines whe exact target.)
Rotation for your TFT and touch-screen offset-calibration/rotation can be set in the main.cpp file.


### If command-line build is your preferred way:

#### Navigate to the exported project-folder and you'll see two batch-files (they take care of handling 'build' folder):
- On Windows you can run `build.bat` to clean previous builds and build the project for you in 'build' folder.
  - `build-ninja.bat` is provided too: Ninja is used if you installed the all-in-one Pico-SDK
- On Linux you can run `build.sh`  (You might need to set its execution-privilege by `chmod 755 build.sh` if it changed in the unzipped exported project-template.)
- For Raspberry Pico-W you can use the `build-PicoW.bat` and `build-PicoW.sh` scripts.

#### You can also do the cmake steps manually:
- First go to the 'build' folder and to create the build/make-files type:
  - On Windows: `cmake -G "MinGW Makefiles" ..` or `cmake -G "Ninja" ..`  (depending on the installed build/make-tool)
  - On Linux: `cmake ..` or `cmake -G "Unix Makefiles" ..`  (or if you have and want Ninja-build, `cmake -G "Ninja" ..`)
- When make-files are ready, you can finally build the project by:
  - On Windows: `make` or `ninja`  (depending on the installed build/make-tool)
  - On Linux: `make`  (or `ninja` if you selected this one in the previous step)


### If you want to build in VScode:
- Just open the project in it by 'Open Folder' (in 'File' menu) and select the exported project-template's folder.
  - If asked, allow it to download and install the CMake extension.
  - To continue you might need to press OK that you trust the source (probably asked because of included SDL binaries)
- Open the `main.c` file, and if asked allow it to download and install the C/C++ extension.
- Select the build toolkit on the bottom toolbar. (For example `GCC 10.3 arm-none-eabi-gcc` on Linux/MinGW)
  - (You can choose 'Unspecified' and let the toolchain given by CMake settings be used, that works too.)
- Configure the CMake project with the `CMake: ...` button on the bottom toolbar. (Select preferred release type)
- Build the project with the `Build` button on the bottom toolbar.


### If you want to build in Eclipse-IDE:
- Import the project: File / Import ... / General / Existing Projects into Workspace
- Select the project-template folder exported by Squareline Studio and click 'Finish'.
- Open the `main.c` file.
- Pressing the hammer (build) button will build the project with make.


### To test/run:
- You can copy the generated executable file (with '.uf2' extension in 'build' folder) to the Raspberry Pico through USB
  - (When you plug Raspberry Pico into the PC while pressing its built-in button, it appears as a mass-storage device like a pendrive.)
- Run the executable on the Raspberry Pico by turning it on.
  - (You may also test the built binary beforehand with a QEMU emulated Raspberry Pico on the host machine.)


### Driver support for different TFTs and Touch-screens:

This board-project is built with TFT_eSPI by default. This library originally depends heavily on Arduino-IDE (and its contained Arduino-SDK), but we modified its source to use only the 120kbyte of selected built-in 'driver-libraries/Arduino' source files.
The Pico was connected to a TFT with the popular ILI9341 TFT-controller, and to the popular Touch-screen controller XPT2046, all wired on the same SPI-bus with their distinc chip-select lines. (SPI0 of Raspberry Pico)
The connections we used are seen in the created 'driver-libraries/TFT_eSPI/User_Setups/Setup60b_RP2040_ILI9341.h' file which is included by User_Setup_Select.h file. I followed the default wiring of the existing TFT_eSPI demos, but Raspberry Pico can have SPI0/SPI1 on different pins too (settable .

Here's an abstract how the Pico is wired to the TFT+Touchpad pair:

| Display-pin | Pico-pin | Description or comment                                                                                                                                         |
|-------------|----------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| VCC         | 3.3V     | (Raspberry Pico has an LDO-voltage-regulator output of 3.3V usable as power supply for 3.3V TFT boards)                                                        |
| LED         | 3.3V     | (The LDO can also be used to drive the LED-backlight of the TFT)                                                                                               |
| GND         | GND      | (Any GND pin on the Pico)                                                                                                                                      |
| MISO        | GPIO_0   | (MasterIn-SlaveOut wire (Pico SPI-input): TFT-display (e.g. SDO) data-output and TouchScreen data-output (e.g. T_DO) wired together and connected to Pico)     |
| SCLK        | GPIO_2   | (SPI clock: TFT-clock (e.g. SCK) and Touch-clock (e.g. T_CLK) wired together and connected to Pico)                                                            |
| MOSI        | GPIO_3   | (MasterOut-SlaveIn wire (Pico SPI-output): TFT-display (e.g. SDI) data-input and TouchScreen data-input (e.g. T_DI) wired together and connected to Pico)      |
| TFT_DC      | GPIO_18  | (Data/Control line for the  4-wire SPI protocol used by the TFT)                                                                                               |
| TFT_RST     | GPIO_19  | (TFT input: reset-signal)                                                                                                                                      |
| TFT_CS      | GPIO_20  | (TFT input: Chip-select signal on the SPI bus, faster SPI clock-speeds are usually allowed in the 20..50MHz range)                                             |
| TOUCH_CS    | GPIO_21  | (TouchScreen input: Chep-select signal on the SPI bus - many times slower SPI clockspeed is needed when using the touchscreen, e.g. 2.5MHz in case of XPT2046) |
| (TOUCH_IRQ  | GPIO_22  | - Not really necessary but MCU can benefit from a signal for e.g. wake-up to check if touch-screen has been pressed. )                                         |

More TFT/Touchscreen libraries exist, they're included in the 'driver-libraries' folder but only TFT_eSPI is supported by this board-file at the moment.

This is a comparison-table between these libraries according to what devices they support (wath in monospace-fontset):

LCD:
====

| Model    |lvgl_esp32_drivers|  TFT_eSPI   |    lv_drivers   |   Arduino-GFX       |
|------    |----------------- | ----------- | --------------- | ------------------- |
| (drm         |              |             |        *)       |                     |
| (fbdev       |              |             |        *)       |                     |
| (RGB-Display |              |             |                 |     *)              |
| FT81x        |  *           |             |                 |                     |
| GC9106       |              |             |                 |     *               |
| GC9107       |              |             |                 |     *               |
| GC9A01       |  *           |     *       |        *        |     *               |
| HX8347(C,D)  |              |             |                 |     *               |
| HX8352(C)    |              |             |                 |     *               |
| hx8357       |  *           |             |                 |     *(A)            |
| hx8357b/c/d  |  ?           |     *       |                 |     *(B)            |
| HX8369A      |              |             |                 |     *               |
| il3820       |  *           |             |                 |                     |
| ili9163c     |  *           |     *(-c?)  |                 |                     |
| ili9225      |              |     *       |                 |     *               |
| ILI9331      |              |             |                 |     *               |
| ili9341      |  *           |     *       |       *         |     *               |
| ILI9342      |              |             |                 |     *               |
| ili9481      |  *           |     *       |                 |     *(18bit)        |
| ili9486      |  *           |     *       |                 |     *(normal,18bit) |
| ili9488      |  *           |     *       |                 |     *(normal,18bit) |
| ILI9806      |              |             |                 |     *               |
| JBT6K71      |              |             |                 |     *               |
| jd79653a     |  *           |             |                 |                     |
| NT35310      |              |             |                 |     *               |
| NT35510      |              |             |                 |     *               |
| NT39125      |              |             |                 |     *               |
| NV3041A      |              |             |                 |     *               |
| OTM8009A     |              |             |                 |     *               |
| pcd8544      |  *           |             |                 |                     |
| R61529       |              |             |                 |     *               |
| R61581       |              |     *       |       *         |                     |
| ra8875       |  *           |             |                 |                     |
| RM67162      |              |             |                 |     *               |
| RM68120      |              |     *       |                 |                     |
| RM68140      |              |     *       |                 |                     |
| S6D02A1      |              |     *       |                 |                     |
| SEPS525      |              |             |                 |     *               |
| sh1107       |  *           |             |                 |                     |
| SHARP_MIP    |              |             |       *         |                     |
| SSD1283A     |              |             |                 |     *               |
| ssd1306      |  *           |             |                 |                     |
| SSD1331      |              |             |                 |     *               |
| SSD1351      |              |     *       |                 |     *               |
| SSD1963      |              |     *       |       *         |                     |
| ST7565       |              |             |       *         |                     |
| st7735s      |  *           |     *(-s?)  |                 |     *(-s?)          |
| st7789       |  *           |     *(_2?)  |                 |     *               |
| st7796s      |  *           |     *(-s?)  |                 |     *               |
| UC1610       |              |             |       *         |                     |
| uc8151d      |  *           |             |                 |                     |

TOUCH:
======

| Model        |lvgl_esp32_drivers|  TFT_eSPI   |    lv_drivers  |
| -----        |------------------| ----------- | -------------- |
| adcraw/AD_touch |      *        |             |        *       |
| ft6x6           |      *        |             |                |
| FT6x36?         |               |             |                |
| FT5406EE8       |               |             |        *       |
| FT81x           |      *        |             |                |
| gt911           |      *        |             |                |
| ra8875          |      *        |             |                |
| stmpe610        |      *        |             |                |
| xpt2046         |      *        |      *      |        *       |
| (evdev          |               |             |        *)      |
| (libinput       |               |             |        *)      |
| (xkb            |               |             |        *)      |

