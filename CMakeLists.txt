cmake_minimum_required(VERSION 3.15)
project(librtp VERSION 1.0)

set(LIB_SOURCES
      src/rtcp_pkt.c
      src/rtp_jitter.c
      src/rtp_pkt.c
)

add_library(rtp ${LIB_SOURCES})

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

target_include_directories(rtp PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(rtp PRIVATE "RTP_API_EXPORTS")
target_compile_options(rtp PRIVATE "-fvisibility=hidden")
target_compile_options(rtp PRIVATE "-std=gnu99")

if(WIN32)
  target_link_libraries(rtp ws2_32)
endif()

target_link_libraries(rtp futils pomp ulog)