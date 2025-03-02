cp  CMakeLists-PicoW.txt  CMakeLists.txt
rm  -r  build
mkdir  -p  build
cd build
cmake  ..  -DCMAKE_BUILD_TYPE=Release
make  -j
