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
