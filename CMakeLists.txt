# modules/CMakeLists.txt

if(CONFIG_MOD_DEV_IMU)
	target_sources(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/imu/imu.cpp
	)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu)

	if(CONFIG_MOD_DEV_IMU_BMI088)
		target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu/bmi088/bmi088.cpp)
		target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu/bmi088)
	endif()

	if(CONFIG_MOD_DEV_IMU_ICM42688P)
		target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu/icm42688p/icm42688p.cpp)
		target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu/icm42688p)
	endif()

	if(CONFIG_MOD_DEV_IMU_IDENT)
		target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu/ident/ident.cpp)
		target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu/ident)
	endif()
endif()

if(CONFIG_MOD_DEV_MOTOR_DJI)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motors/dji)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motors/dji/dji.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motors/dji)
endif()

if(CONFIG_MOD_DEV_MOTOR_DM)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motors/dm/dm.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motors/dm)
endif()

if(CONFIG_MOD_DEV_POWERMETER)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/powermeter/powermeter.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/powermeter)
endif()

if(CONFIG_MOD_DEV_REMOTE)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes/remote.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes)

	if(CONFIG_MOD_DEV_REMOTE_AUTO OR CONFIG_MOD_DEV_REMOTE_DR16)
		target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes/dr16/dr16.cpp)
		target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes/dr16)
	endif()
	if(CONFIG_MOD_DEV_REMOTE_AUTO OR CONFIG_MOD_DEV_REMOTE_SBUS)
		target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes/sbus/sbus.cpp)
		target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes/sbus)
	endif()
	if(CONFIG_MOD_DEV_REMOTE_AUTO OR CONFIG_MOD_DEV_REMOTE_VT12)
		target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes/vt12/vt12.cpp)
		target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes/vt12)
	endif()
	if(CONFIG_MOD_DEV_REMOTE_AUTO OR CONFIG_MOD_DEV_REMOTE_VT13)
		target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes/vt13/vt13.cpp)
		target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes/vt13)
	endif()
endif()
