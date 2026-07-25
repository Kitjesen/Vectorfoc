"""Executable dependency rules for the replaceable hardware/transport seams."""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


class LayerArchitectureTests(unittest.TestCase):
    def assert_forbidden(self, relative_path: str, patterns: tuple[str, ...]) -> None:
        source = read(relative_path)
        for pattern in patterns:
            self.assertIsNone(
                re.search(pattern, source),
                f"{relative_path} violates layer rule: {pattern}",
            )

    def test_stm32_hal_wrappers_do_not_reach_into_motor_state(self) -> None:
        for relative_path in (
            "Src/HAL/stm32g4/hal_adc.c",
            "Src/HAL/stm32g4/hal_pwm.c",
            "Src/HAL/stm32g4/hal_encoder.c",
        ):
            with self.subTest(path=relative_path):
                self.assert_forbidden(
                    relative_path,
                    (r'#include\s+"motor\.h"', r"\bmotor_data\b"),
                )

    def test_bsp_can_has_no_protocol_or_motor_dependency(self) -> None:
        patterns = (
            r'#include\s+"manager\.h"',
            r'#include\s+"motor\.h"',
            r'#include\s+"protocol_types\.h"',
            r"\bProtocol_",
            r"\bg_can_baudrate\b",
            r"\bCAN_Frame\b",
        )
        for relative_path in ("Src/HAL/bsp/bsp_can.c", "Src/HAL/bsp/bsp_can.h"):
            with self.subTest(path=relative_path):
                self.assert_forbidden(relative_path, patterns)

    def test_bsp_can_public_interface_is_mcu_type_free(self) -> None:
        self.assert_forbidden(
            "Src/HAL/bsp/bsp_can.h",
            (r"\bFDCAN_HandleTypeDef\b", r"\bFDCANInstance\b"),
        )

    def test_can_transport_adapter_lives_above_the_bsp(self) -> None:
        self.assertFalse((ROOT / "Src/HAL/bsp/can_transport.c").exists())
        self.assertFalse((ROOT / "Src/HAL/bsp/can_transport.h").exists())
        self.assertTrue((ROOT / "Src/COMM/transport/can_transport.c").is_file())
        self.assertTrue((ROOT / "Src/COMM/transport/can_transport.h").is_file())
        self.assert_forbidden(
            "Src/COMM/transport/can_transport.c",
            (r'#include\s+"manager\.h"', r"\bProtocol_QueueRxFrame\b"),
        )

    def test_protocol_manager_does_not_fall_back_to_a_concrete_bsp(self) -> None:
        self.assert_forbidden(
            "Src/COMM/manager/manager.c",
            (r'#include\s+"bsp_can\.h"', r"\bBSP_CAN_"),
        )

    def test_generic_hal_abstraction_does_not_own_protocol_can_frames(self) -> None:
        patterns = (
            r'#include\s+"protocol_types\.h"',
            r"\bCAN_Frame\b",
            r"\bHAL_CAN_",
        )
        for relative_path in (
            "Src/HAL/interface/hal_abstraction.h",
            "Src/HAL/stm32g4/hal_abstraction.c",
        ):
            with self.subTest(path=relative_path):
                self.assert_forbidden(relative_path, patterns)
    def test_algorithm_layer_has_no_led_output_dependency(self) -> None:
        for path in (ROOT / "Src/ALGO").rglob("*.[ch]"):
            source = path.read_text(encoding="utf-8")
            relative_path = path.relative_to(ROOT)
            self.assertNotRegex(
                source,
                r'#include\s+"led\.h"|\bRGB_DisplayColorById\b',
                f"{relative_path} must not drive the UI LED",
            )


if __name__ == "__main__":
    unittest.main()