from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path

from tests import experiment_angular_time_constant as angular_experiment
from tests import experiment_kl_kr_turn_in_place as turn_experiment
from tests import experiment_time_constant as linear_experiment


class ExperimentScriptsTestCase(unittest.TestCase):
    """Проверки, что экспериментальные скрипты пишут нужные данные."""

    def test_turn_experiment_builds_expected_12_trial_plan(self) -> None:
        plan = turn_experiment._build_trial_plan()

        self.assertEqual(12, len(plan))
        self.assertEqual((20, 0), (plan[0].right_track_percent, plan[0].left_track_percent))
        self.assertEqual((-60, 0), (plan[5].right_track_percent, plan[5].left_track_percent))
        self.assertEqual((0, 20), (plan[6].right_track_percent, plan[6].left_track_percent))
        self.assertEqual((0, -60), (plan[11].right_track_percent, plan[11].left_track_percent))

    def test_turn_experiment_writes_required_csv_and_json_fields(self) -> None:
        row = turn_experiment.TurnTrialResult(
            experiment_no=1,
            right_track_percent=20,
            left_track_percent=0,
            angle_window_start_unix_s=1.0,
            angle_window_end_unix_s=2.0,
            angle_window_start_iso="2026-04-03T10:00:00.000Z",
            angle_window_end_iso="2026-04-03T10:00:05.000Z",
            angle_window_start_rel_s=5.0,
            angle_window_end_rel_s=10.0,
            yaw_start_deg=12.0,
            yaw_end_deg=42.0,
            angle_delta_deg=30.0,
        )

        with tempfile.TemporaryDirectory() as tmp_dir_name:
            tmp_dir = Path(tmp_dir_name)
            turn_experiment._write_results(tmp_dir, [row])

            csv_path = tmp_dir / turn_experiment.CSV_FILE_NAME
            json_path = tmp_dir / turn_experiment.JSON_FILE_NAME
            self.assertTrue(csv_path.exists())
            self.assertTrue(json_path.exists())

            with csv_path.open("r", encoding="utf-8", newline="") as file_obj:
                reader = csv.DictReader(file_obj)
                rows = list(reader)
            self.assertEqual(1, len(rows))
            self.assertEqual("1", rows[0]["experiment_no"])
            self.assertEqual("20", rows[0]["right_track_percent"])
            self.assertEqual("0", rows[0]["left_track_percent"])
            self.assertIn("angle_delta_deg", rows[0])

            payload = json.loads(json_path.read_text(encoding="utf-8"))
            self.assertEqual("kR_kL_turn_response", payload["experiment"])
            self.assertEqual(1, len(payload["rows"]))

    def test_linear_run_csv_uses_required_columns(self) -> None:
        rows = [
            {"time_s": 0.0, "acceleration_m_s2": 0.1},
            {"time_s": 0.1, "acceleration_m_s2": 0.2},
        ]

        with tempfile.TemporaryDirectory() as tmp_dir_name:
            tmp_dir = Path(tmp_dir_name)
            file_name = linear_experiment._write_run_csv(
                results_dir=tmp_dir,
                speed_percent=20,
                rows=rows,
            )

            csv_path = tmp_dir / file_name
            with csv_path.open("r", encoding="utf-8", newline="") as file_obj:
                reader = csv.DictReader(file_obj)
                self.assertEqual(["time_s", "acceleration_m_s2"], reader.fieldnames)
                data_rows = list(reader)
            self.assertEqual("0.1", data_rows[0]["acceleration_m_s2"])

    def test_angular_run_csv_uses_required_columns(self) -> None:
        rows = [
            {"time_s": 0.0, "angular_velocity_deg_s": 1.5},
            {"time_s": 0.1, "angular_velocity_deg_s": 2.5},
        ]

        with tempfile.TemporaryDirectory() as tmp_dir_name:
            tmp_dir = Path(tmp_dir_name)
            file_name = angular_experiment._write_run_csv(
                results_dir=tmp_dir,
                speed_percent=20,
                rows=rows,
            )

            csv_path = tmp_dir / file_name
            with csv_path.open("r", encoding="utf-8", newline="") as file_obj:
                reader = csv.DictReader(file_obj)
                self.assertEqual(["time_s", "angular_velocity_deg_s"], reader.fieldnames)
                data_rows = list(reader)
            self.assertEqual("1.5", data_rows[0]["angular_velocity_deg_s"])


if __name__ == "__main__":
    unittest.main()
