import importlib.util
from pathlib import Path
import tempfile
import textwrap
import unittest


def load_path_generator_module():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "path_generator.py"
    spec = importlib.util.spec_from_file_location("path_generator", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class PathGeneratorMapLoadingTest(unittest.TestCase):
    def test_load_map_meta_honors_nav2_negate_and_thresholds(self):
        module = load_path_generator_module()

        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = Path(tmp_dir)
            image_path = tmp_path / "map.pgm"
            yaml_path = tmp_path / "map.yaml"

            image_path.write_text(
                textwrap.dedent(
                    """\
                    P2
                    3 1
                    255
                    0 128 255
                    """
                ),
                encoding="ascii",
            )
            yaml_path.write_text(
                textwrap.dedent(
                    """\
                    image: map.pgm
                    mode: trinary
                    resolution: 0.5
                    origin: [0.0, 0.0, 0.0]
                    negate: 1
                    occupied_thresh: 0.65
                    free_thresh: 0.196
                    """
                ),
                encoding="utf-8",
            )

            app = object.__new__(module.PathGeneratorApp)
            meta = app._load_map_meta(yaml_path)

            self.assertEqual(meta["width_px"], 3)
            self.assertEqual(meta["height_px"], 1)
            self.assertEqual(meta["image_array"].tolist(), [[255, 127, 0]])


if __name__ == "__main__":
    unittest.main()
