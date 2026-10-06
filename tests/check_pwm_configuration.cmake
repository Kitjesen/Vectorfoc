get_filename_component(project_root "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
file(READ "${project_root}/platform/Core/Inc/main.h" platform_header)
file(READ "${project_root}/platform/VectorFOC.ioc" cubemx_project)

# Regeneration must preserve the board as the only timing source, including
# the ADC trigger margin. These aliases are CubeMX's generated macro names.
set(expected_MCPWM_CLOCK_HZ "SYS_CLOCK_HZ")
set(expected_MCPWM_FREQ "HW_PWM_FREQ_HZ")
set(expected_MCPWM_DEADTIME_CLOCKS "HW_PWM_DEADTIME_CLKS")
set(expected_MCPWM_PERIOD_CLOCKS "MCPWM_CLOCK_HZ/(2u*MCPWM_FREQ)")
set(expected_MCPWM_TGRO_TIME "MCPWM_PERIOD_CLOCKS-HW_PWM_ADC_TRIGGER_OFFSET_TICKS")

foreach(alias IN ITEMS MCPWM_CLOCK_HZ MCPWM_FREQ MCPWM_DEADTIME_CLOCKS
                       MCPWM_PERIOD_CLOCKS MCPWM_TGRO_TIME)
    string(REGEX MATCH "#define[ \t]+${alias}[ \t]+([^\r\n]+)"
        header_match "${platform_header}")
    set(header_value "${CMAKE_MATCH_1}")
    string(REGEX MATCH "${alias},([^;\r\n]+)" ioc_match "${cubemx_project}")
    set(ioc_value "${CMAKE_MATCH_1}")
    foreach(source IN ITEMS header ioc)
        string(REGEX REPLACE "[ \t()]" "" normalized_value "${${source}_value}")
        string(REGEX REPLACE "[ \t()]" "" expected_value "${expected_${alias}}")
        if(NOT normalized_value STREQUAL expected_value)
            message(FATAL_ERROR "${source}: ${alias} must derive from board timing")
        endif()
    endforeach()
endforeach()

string(FIND "${platform_header}" "#include \"board_configuration.h\"" board_include)
if(board_include EQUAL -1)
    message(FATAL_ERROR "main.h must retain the board include for generated timer code")
endif()
foreach(pair IN ITEMS "TIM1.PeriodNoDither=MCPWM_PERIOD_CLOCKS"
                      "TIM1.PulseNoDither_4=MCPWM_TGRO_TIME")
    string(FIND "${cubemx_project}" "${pair}" pair_position)
    if(pair_position EQUAL -1)
        message(FATAL_ERROR "CubeMX must retain ${pair}")
    endif()
endforeach()
message(STATUS "CubeMX and generated timer aliases follow board timing")
