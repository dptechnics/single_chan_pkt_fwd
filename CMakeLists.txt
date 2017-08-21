cmake_minimum_required(VERSION 2.6)

PROJECT(single_chan_pkt_fwd C)
INCLUDE(CheckFunctionExists)

SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
ADD_DEFINITIONS(-Os -Wall -Werror -Wmissing-declarations --std=gnu99 -g3)

SET(SOURCES main.c base64.c)

ADD_EXECUTABLE(single_chan_pkt_fwd ${SOURCES})

TARGET_LINK_LIBRARIES(single_chan_pkt_fwd ${LIBS})

INSTALL(TARGETS single_chan_pkt_fwd
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
)
