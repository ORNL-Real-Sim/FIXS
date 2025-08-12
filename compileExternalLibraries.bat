cd .\CommonLib\libevent
if not exist build md build
cd build
cmake -G "Visual Studio 17 2022" -DEVENT__DISABLE_MBEDTLS=ON ..
cmake --build . --config Release 
cmake --build . --config Debug

cd ..\..\..\
cd .\CommonLib\yaml-cpp
if not exist build md build
cd build
cmake -G "Visual Studio 17 2022" ..
cmake --build . --config Release
cmake --build . --config Debug

