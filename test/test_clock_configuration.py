#!/usr/bin/env python3
"""Regression checks for the STM32G431 VectorFOC clock-tree contract."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HSE_HZ = 8_000_000
PLL_N = 42
PLL_R = 2
SYSCLK_HZ = 168_000_000


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


class ClockConfigurationTest(unittest.TestCase):
    def test_hse_value_is_consistent_across_cmsis_hal_and_builds(self) -> None:
        system = read("Lib/Core/Src/system_stm32g4xx.c")
        hal_conf = read("Lib/Core/Inc/stm32g4xx_hal_conf.h")

        self.assertRegex(system, r"#define HSE_VALUE 8000000U")
        self.assertRegex(hal_conf, r"#define HSE_VALUE \(8000000UL\)")
        for cmake_file in ("CMakeLists.txt", "CMakeLists_Bootloader.txt"):
            self.assertIn("HSE_VALUE=8000000U", read(cmake_file))

    def test_vector_pll_is_the_documented_168mhz_configuration(self) -> None:
        self.assertEqual((HSE_HZ * PLL_N) // PLL_R, SYSCLK_HZ)

        for source_file in ("Lib/Core/Src/main.c", "Src/BOOT/boot_main.c"):
            source = read(source_file)
            self.assertIn("HSE 8MHz -> PLL -> 168MHz SYSCLK", source)
            self.assertRegex(source, r"PLLM\s*=\s*RCC_PLLM_DIV1")
            self.assertRegex(source, r"PLLN\s*=\s*42")
            self.assertRegex(source, r"PLLR\s*=\s*RCC_PLLR_DIV2")

    def test_vector_pwm_timing_comes_from_board_contract(self) -> None:
        board = read("Src/config/boards/board_vectorfoc.h")
        timer = read("Lib/Core/Src/tim.c")
        ioc = read("VectorFOC.ioc")

        self.assertRegex(board, r"#define HW_PWM_PERIOD_TICKS\s+4200U")
        self.assertRegex(
            board, r"#define HW_PWM_ADC_TRIGGER_OFFSET_TICKS\s+100U"
        )
        self.assertIn(
            "HW_PWM_PERIOD_TICKS - HW_PWM_ADC_TRIGGER_OFFSET_TICKS", timer
        )
        self.assertIn("HW_PWM_DEADTIME_CLKS", timer)
        self.assertNotIn("sConfigOC.Pulse = 4190", timer)
        self.assertIn("MCPWM_PERIOD_CLOCKS-100", ioc)

    def test_vector_pwm_phases_align_with_current_channels(self) -> None:
        board = read("Src/config/boards/board_vectorfoc.h")
        expected = (
            r"#define HW_PWM_CH_PHASE_A\s+HW_PWM_CH_U",
            r"#define HW_PWM_CH_PHASE_B\s+HW_PWM_CH_V",
            r"#define HW_PWM_CH_PHASE_C\s+HW_PWM_CH_W",
            r"#define HW_ADC_IA_JDR\s+HW_ADC_JDR_IA",
            r"#define HW_ADC_IB_JDR\s+HW_ADC_JDR_IB",
            r"#define HW_ADC_IC_JDR\s+HW_ADC_JDR_IC",
        )
        for contract in expected:
            self.assertRegex(board, contract)
    def test_bootloader_stops_when_clock_setup_fails(self) -> None:
        source = read("Src/BOOT/boot_main.c")
        for call in (
            "HAL_PWREx_ControlVoltageScaling",
            "HAL_RCC_OscConfig",
            "HAL_RCC_ClockConfig",
        ):
            self.assertRegex(
                source,
                rf"if \({call}\([^;]+?\) != HAL_OK\) \{{\s*Error_Handler\(\);",
            )


if __name__ == "__main__":
    unittest.main()
