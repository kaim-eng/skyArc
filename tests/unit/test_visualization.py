# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import unittest
from pathlib import Path

import _bootstrap  # noqa: F401

from skyarc.configuration import load_yaml, resolve_tube_layout
from skyarc.names import BODY_CART, BODY_ROCKET
from skyarc.state import BodyState, SimulationState
from skyarc.visualization.cameras import camera_views, tracked_view
from skyarc.visualization.rocket_asset import resolve_rocket_visual_asset


PROJECT = Path(__file__).resolve().parents[2]


class CameraDefinitionTests(unittest.TestCase):
    def test_all_seven_design_views_are_named(self) -> None:
        layout = resolve_tube_layout(load_yaml(PROJECT / "configs" / "curved_2kms.yaml").config)
        views = camera_views(layout)
        self.assertEqual(
            set(views),
            {
                "full_system_side",
                "tube_cutaway",
                "cart_rocket_chase",
                "cart_forward",
                "exit_separation",
                "rocket_chase",
                "overhead_diagnostic",
            },
        )
        self.assertTrue(views["full_system_side"].schematic)
        self.assertFalse(views["tube_cutaway"].schematic)

    def test_tracking_views_translate_without_changing_relative_framing(self) -> None:
        layout = resolve_tube_layout(load_yaml(PROJECT / "configs" / "curved_2kms.yaml").config)
        view = camera_views(layout)["rocket_chase"]
        state = SimulationState(
            time_s=1.0,
            step_index=1,
            dt_s=0.001,
            bodies={
                BODY_CART: BodyState(name=BODY_CART),
                BODY_ROCKET: BodyState(name=BODY_ROCKET, position=(100.0, 20.0, 30.0)),
            },
        ).frozen()
        resolved = tracked_view(view, state)
        self.assertEqual(
            tuple(resolved.position_m[index] - view.position_m[index] for index in range(3)),
            (100.0, 20.0, 30.0),
        )
        self.assertEqual(
            tuple(resolved.look_at_m[index] - view.look_at_m[index] for index in range(3)),
            (100.0, 20.0, 30.0),
        )


class RocketVisualAssetTests(unittest.TestCase):
    def test_jupiter_c_asset_is_visual_only_and_fits_the_conservative_envelope(self) -> None:
        asset = resolve_rocket_visual_asset(target_length_m=4.0, target_diameter_m=1.0)
        self.assertTrue(asset.usd_path.is_file())
        self.assertTrue(asset.manifest_path.is_file())
        self.assertAlmostEqual(asset.native_length_m, 2.8835745126008978)
        self.assertAlmostEqual(asset.native_diameter_m, 0.8513954281806946)
        self.assertAlmostEqual(asset.native_length_m * asset.axial_scale, 4.0)
        self.assertAlmostEqual(asset.native_diameter_m * asset.radial_scale, 1.0)
        self.assertEqual(asset.redistribution_status, "cleared")

    def test_invalid_target_envelope_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "length"):
            resolve_rocket_visual_asset(target_length_m=0.0, target_diameter_m=1.0)


if __name__ == "__main__":
    unittest.main()
