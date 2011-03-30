include (FindPkgConfig)
pkg_search_module (curg REQUIRED c_urg0.8)
pkg_search_module (playercore REQUIRED playercore)

if (NOT PKG_CONFIG_FOUND)
	message(FATAL_ERROR "Cannot find pkg-config!")
endif ()

include_directories (${curg_INCLUDE_DIRS} ${playercore_INCLUDE_DIRS})
add_library (playerurg2 SHARED playerurg.cpp)
target_link_libraries (playerurg2 ${curg_LIBRARIES} ${playercore_LIBRARIES})

install(TARGETS playerurg2 DESTINATION lib)
install(FILES urglaser.cfg DESTINATION  share/player/config)
