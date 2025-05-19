# 创建 tools 静态库
add_library(tools STATIC
    segger/SEGGER_RTT_printf.c 
    segger/SEGGER_RTT.c
    segger/SEGGER_RTT_ASM_ARMv7M.s
    easylogger/elog.c 
    easylogger/elog_port.c 
    easylogger/elog_utils.c
    cm_backtrace/cm_backtrace.c
    cm_backtrace/cmb_fault.S
)

# 设置包含目录
target_include_directories(tools
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/segger
        ${CMAKE_CURRENT_SOURCE_DIR}/easylogger
        ${CMAKE_CURRENT_SOURCE_DIR}/cm_backtrace
        ${CMAKE_CURRENT_SOURCE_DIR}/cm_backtrace/Languages/en-US 
        ${CMAKE_CURRENT_SOURCE_DIR}/cm_backtrace/Languages/zh-CN 
)

# 链接必要的库
target_link_libraries(tools stm32cubemx FreeRTOS)

# 将 segger 添加到主项目
target_link_libraries(${PROJECT_NAME} tools)