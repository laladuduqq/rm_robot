# 创建 application 静态库
add_library(application STATIC
    robot_init.c
    robot_control/robot_control.c
    robot_control/robot_task.c
    gimbal/gimbalcmd.c 
    shoot/shootcmd.c 
    chassis/chassiscmd.c 
    chassis/chassis_type.c 
)

# 设置包含目录
target_include_directories(application
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/robot_control
        ${CMAKE_CURRENT_SOURCE_DIR}/gimbal
        ${CMAKE_CURRENT_SOURCE_DIR}/shoot
        ${CMAKE_CURRENT_SOURCE_DIR}/chassis
)

# 链接必要的库
target_link_libraries(application stm32cubemx tools modules)

# 将 application 添加到主项目
target_link_libraries(${PROJECT_NAME} application)