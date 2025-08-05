cmake_minimum_required(VERSION 2.6)
project(HNC10)

# default is "Debug"
#set(CMAKE_BUILD_TYPE "Release")

# set this to see the compilation commands
# set(CMAKE_VERBOSE_MAKEFILE 1)

set(CMAKE_CXX_STANDARD 11)

#add_definitions(-DAUTOGOS)

find_package(CppLinuxSerial REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(cJSON REQUIRED)
find_package(OpenSSL REQUIRED)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

link_libraries(${OPENSSL_CRYPTO_LIBRARY} ${OPENSSL_SSL_LIBRARY})

add_executable(HNC10 main.cpp sys_rt.cpp lorahub.cpp thread_pool.cpp uplink.cpp uplink5g.cpp)

target_link_libraries (HNC10 CppLinuxSerial::CppLinuxSerial paho-mqtt3as paho-mqttpp3 cjson wiringPi pthread)

install(TARGETS HNC10 RUNTIME DESTINATION bin)
