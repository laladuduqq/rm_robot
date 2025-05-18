# 创建 modules 静态库
add_library(modules STATIC
        algorithm/controller.c
        algorithm/kalman_filter.c
        algorithm/QuaternionEKF.c
        algorithm/user_lib.c
        algorithm/LQR.c
        RGB/RGB.c 
        systemwatch/systemwatch.c
        offline/offline.c
        BMI088/BMI088.c
        IMU/imu.c 
        IMU/imu_task.c
        REMOTE/sbus.c
        referee/crc_rm.c 
        referee/referee_UI.c 
        referee/referee.c
        MOTOR/motor_task.c
        MOTOR/DJI/dji.c 
        MOTOR/DAMIAO/damiao.c 
        board_com/board_com.c
        message/message_center.c
        DM_IMU/dm_imu.c
)

# 设置包含目录
target_include_directories(modules
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
        ${CMAKE_CURRENT_SOURCE_DIR}/RGB
        ${CMAKE_CURRENT_SOURCE_DIR}/systemwatch
        ${CMAKE_CURRENT_SOURCE_DIR}/offline
        ${CMAKE_CURRENT_SOURCE_DIR}/BMI088
        ${CMAKE_CURRENT_SOURCE_DIR}/IMU
        ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE
        ${CMAKE_CURRENT_SOURCE_DIR}/referee
        ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR 
        ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR/DJI 
        ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR/DAMIAO 
        ${CMAKE_CURRENT_SOURCE_DIR}/board_com
        ${CMAKE_CURRENT_SOURCE_DIR}/message
        ${CMAKE_CURRENT_SOURCE_DIR}/DM_IMU
)

# 链接必要的库
target_link_libraries(modules stm32cubemx FreeRTOS tools bsp application)
target_link_libraries(modules -l:libarm_cortexM4lf_math.a) #链接dsp

# 将 modules 添加到主项目
target_link_libraries(${PROJECT_NAME} modules)