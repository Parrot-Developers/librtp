cmake_minimum_required(VERSION 3.15)
project(rtp VERSION 1.0)


set(LIB_SOURCES
      src/rtcp_pkt.c
      src/rtp_jitter.c
      src/rtp_pkt.c
)

add_library(${PROJECT_NAME} SHARED ${LIB_SOURCES})

if(COMMAND set_up_rpath)
    set_up_rpath()
else()
    message("Set up rpath not defined!")
endif()

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
set_target_properties(${PROJECT_NAME} PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RTP_API_EXPORTS")
target_compile_options(${PROJECT_NAME} PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME} PRIVATE "-std=gnu99")

if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE ws2_32 futils pomp ulog)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE futils pomp ulog)
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )
