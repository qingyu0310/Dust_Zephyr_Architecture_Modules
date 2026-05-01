if(CONFIG_DEV_MOTOR_DJI)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motor/dji/dji_c6xx.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motor/dji)
endif()

if(CONFIG_DEV_POWERMETER)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/powermeter/powermeter.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/powermeter)
endif()

# if(CONFIG_DEV_MOTOR_DM)
# 	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motor/dm/dji_c6xx.cpp)
# 	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motor/dm)
# endif()
