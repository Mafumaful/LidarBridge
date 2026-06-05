#!/usr/bin/env python3

import argparse
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
from matplotlib import transforms
import numpy as np
from PIL import Image
import yaml


def resolve_path(path_str: str, base_dir: Path) -> Path:
    path = Path(path_str)
    if path.is_absolute():
        return path
    return (base_dir / path).resolve()


def quaternion_from_yaw(yaw_rad: float) -> Dict[str, float]:
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw_rad * 0.5),
        "w": math.cos(yaw_rad * 0.5),
    }


def wrap_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


@dataclass
class PathGeneratorConfig:
    map_yaml_file: Path
    output_yaml_file: Path
    output_csv_file: Path
    sample_interval_m: float
    window_title: str
    show_grid: bool
    show_control_point_labels: bool
    load_existing_path: bool

    @classmethod
    def from_yaml(cls, config_file: Path) -> "PathGeneratorConfig":
        with config_file.open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream) or {}

        base_dir = config_file.parent
        figure = raw.get("figure", {})

        sample_interval_m = float(raw.get("sample_interval_m", 0.1))
        if sample_interval_m <= 0.0:
            raise RuntimeError("sample_interval_m must be greater than 0")

        return cls(
            map_yaml_file=resolve_path(raw["map_yaml_file"], base_dir),
            output_yaml_file=resolve_path(raw["output_yaml_file"], base_dir),
            output_csv_file=resolve_path(raw["output_csv_file"], base_dir),
            sample_interval_m=sample_interval_m,
            window_title=str(figure.get("title", "Path Generator")),
            show_grid=bool(figure.get("show_grid", True)),
            show_control_point_labels=bool(figure.get("show_control_point_labels", True)),
            load_existing_path=bool(raw.get("load_existing_path", True)),
        )


class NaturalCubicSpline1D:
    def __init__(self, x: np.ndarray, y: np.ndarray) -> None:
        if len(x) != len(y):
            raise RuntimeError("x and y must have the same size")
        if len(x) < 2:
            raise RuntimeError("at least two points are required")

        self.x = x.astype(float)
        self.a = y.astype(float).copy()
        self.n = len(x)
        self.h = np.diff(self.x)

        if np.any(self.h <= 0.0):
            raise RuntimeError("spline parameter must be strictly increasing")

        self.b = np.zeros(self.n - 1, dtype=float)
        self.c = np.zeros(self.n, dtype=float)
        self.d = np.zeros(self.n - 1, dtype=float)

        if self.n == 2:
            slope = (self.a[1] - self.a[0]) / self.h[0]
            self.b[0] = slope
            return

        alpha = np.zeros(self.n, dtype=float)
        for i in range(1, self.n - 1):
            alpha[i] = (
                3.0 / self.h[i] * (self.a[i + 1] - self.a[i]) -
                3.0 / self.h[i - 1] * (self.a[i] - self.a[i - 1])
            )

        l = np.ones(self.n, dtype=float)
        mu = np.zeros(self.n, dtype=float)
        z = np.zeros(self.n, dtype=float)

        for i in range(1, self.n - 1):
            l[i] = 2.0 * (self.x[i + 1] - self.x[i - 1]) - self.h[i - 1] * mu[i - 1]
            mu[i] = self.h[i] / l[i]
            z[i] = (alpha[i] - self.h[i - 1] * z[i - 1]) / l[i]

        for j in range(self.n - 2, -1, -1):
            self.c[j] = z[j] - mu[j] * self.c[j + 1]
            self.b[j] = (
                (self.a[j + 1] - self.a[j]) / self.h[j] -
                self.h[j] * (self.c[j + 1] + 2.0 * self.c[j]) / 3.0
            )
            self.d[j] = (self.c[j + 1] - self.c[j]) / (3.0 * self.h[j])

    def _segment_index(self, xq: float) -> int:
        if xq <= self.x[0]:
            return 0
        if xq >= self.x[-1]:
            return self.n - 2
        return int(np.searchsorted(self.x, xq, side="right") - 1)

    def evaluate(self, xq: float) -> float:
        i = self._segment_index(xq)
        dx = xq - self.x[i]
        return self.a[i] + self.b[i] * dx + self.c[i] * dx * dx + self.d[i] * dx * dx * dx

    def first_derivative(self, xq: float) -> float:
        i = self._segment_index(xq)
        dx = xq - self.x[i]
        return self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx * dx

    def second_derivative(self, xq: float) -> float:
        i = self._segment_index(xq)
        dx = xq - self.x[i]
        return 2.0 * self.c[i] + 6.0 * self.d[i] * dx


class ParametricCubicSpline2D:
    def __init__(self, control_points: List[np.ndarray]) -> None:
        if len(control_points) < 2:
            raise RuntimeError("at least two control points are required")

        points = np.asarray(control_points, dtype=float)
        segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if np.any(segment_lengths <= 1e-9):
            raise RuntimeError("adjacent control points must not overlap")

        self.points = points
        self.s = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        self.length = float(self.s[-1])
        self.sx = NaturalCubicSpline1D(self.s, points[:, 0])
        self.sy = NaturalCubicSpline1D(self.s, points[:, 1])

    def sample(self, sample_interval_m: float) -> List[Dict[str, float]]:
        sample_count = max(int(math.ceil(self.length / sample_interval_m)) + 1, 2)
        samples = np.linspace(0.0, self.length, sample_count)
        path = []

        for index, s_value in enumerate(samples):
            x_m = self.sx.evaluate(s_value)
            y_m = self.sy.evaluate(s_value)
            dx = self.sx.first_derivative(s_value)
            dy = self.sy.first_derivative(s_value)
            ddx = self.sx.second_derivative(s_value)
            ddy = self.sy.second_derivative(s_value)
            yaw_rad = math.atan2(dy, dx)
            denom = math.pow(dx * dx + dy * dy, 1.5)
            curvature = 0.0 if denom <= 1e-9 else (dx * ddy - dy * ddx) / denom

            path.append(
                {
                    "index": index,
                    "s_m": float(s_value),
                    "x_m": float(x_m),
                    "y_m": float(y_m),
                    "yaw_rad": float(wrap_angle(yaw_rad)),
                    "curvature": float(curvature),
                    "orientation": quaternion_from_yaw(yaw_rad),
                }
            )

        return path


class PathGeneratorApp:
    def __init__(self, config: PathGeneratorConfig) -> None:
        self.config = config
        self.control_points: List[np.ndarray] = []
        self.sampled_path: List[Dict[str, float]] = []
        self.map_meta = self._load_map_meta(config.map_yaml_file)
        self.control_points_artist = None
        self.control_label_artists: List[Any] = []
        self.spline_path_artist = None
        self.status_text_artist = None

        if config.load_existing_path and config.output_yaml_file.exists():
            self._load_existing_path(config.output_yaml_file)

        self.figure, self.ax = plt.subplots(figsize=(11, 8))
        self.figure.canvas.manager.set_window_title(config.window_title)
        self.figure.suptitle(config.window_title, fontsize=14)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(config.show_grid, linestyle="--", linewidth=0.5, alpha=0.4)

        self._draw_map()
        self._connect_events()
        self._redraw()

    def _load_map_meta(self, map_yaml_file: Path) -> Dict[str, Any]:
        with map_yaml_file.open("r", encoding="utf-8") as stream:
            map_config = yaml.safe_load(stream)

        image_path = resolve_path(str(map_config["image"]), map_yaml_file.parent)
        image = Image.open(image_path).convert("L")
        image_array = np.flipud(np.asarray(image))

        return {
            "image_array": image_array,
            "width_px": image_array.shape[1],
            "height_px": image_array.shape[0],
            "resolution_m": float(map_config["resolution"]),
            "origin_x_m": float(map_config["origin"][0]),
            "origin_y_m": float(map_config["origin"][1]),
            "origin_yaw_rad": float(map_config["origin"][2]) if len(map_config["origin"]) > 2 else 0.0,
            "image_path": image_path,
        }

    def _load_existing_path(self, path_yaml_file: Path) -> None:
        with path_yaml_file.open("r", encoding="utf-8") as stream:
            saved = yaml.safe_load(stream) or {}

        control_points = saved.get("control_points", [])
        self.control_points = [
            np.array([float(point["x_m"]), float(point["y_m"])], dtype=float)
            for point in control_points
        ]
        self._update_spline()

    def _draw_map(self) -> None:
        resolution_m = self.map_meta["resolution_m"]
        origin_x_m = self.map_meta["origin_x_m"]
        origin_y_m = self.map_meta["origin_y_m"]
        origin_yaw_rad = self.map_meta["origin_yaw_rad"]
        width_px = self.map_meta["width_px"]
        height_px = self.map_meta["height_px"]

        map_transform = (
            transforms.Affine2D()
            .scale(resolution_m, resolution_m)
            .rotate(origin_yaw_rad)
            .translate(origin_x_m, origin_y_m)
            + self.ax.transData
        )

        self.ax.imshow(
            self.map_meta["image_array"],
            cmap="gray",
            origin="lower",
            extent=(0.0, width_px, 0.0, height_px),
            interpolation="nearest",
            transform=map_transform,
            alpha=0.95,
        )

        corners = np.array(
            [
                [0.0, 0.0],
                [width_px * resolution_m, 0.0],
                [width_px * resolution_m, height_px * resolution_m],
                [0.0, height_px * resolution_m],
            ],
            dtype=float,
        )

        rotation = np.array(
            [
                [math.cos(origin_yaw_rad), -math.sin(origin_yaw_rad)],
                [math.sin(origin_yaw_rad), math.cos(origin_yaw_rad)],
            ],
            dtype=float,
        )
        world_corners = (corners @ rotation.T) + np.array([origin_x_m, origin_y_m], dtype=float)
        min_corner = np.min(world_corners, axis=0)
        max_corner = np.max(world_corners, axis=0)

        margin_m = 1.0
        self.ax.set_xlim(min_corner[0] - margin_m, max_corner[0] + margin_m)
        self.ax.set_ylim(min_corner[1] - margin_m, max_corner[1] + margin_m)

    def _connect_events(self) -> None:
        self.figure.canvas.mpl_connect("button_press_event", self._on_mouse_click)
        self.figure.canvas.mpl_connect("key_press_event", self._on_key_press)

    def _on_mouse_click(self, event: Any) -> None:
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return

        if event.button == 1:
            self.control_points.append(np.array([event.xdata, event.ydata], dtype=float))
            self._update_spline()
            self._redraw()
        elif event.button == 3:
            self.undo_last_point()

    def _on_key_press(self, event: Any) -> None:
        if event.key == "u":
            self.undo_last_point()
        elif event.key == "c":
            self.control_points = []
            self.sampled_path = []
            self._redraw()
        elif event.key == "s":
            self.save_path()
        elif event.key == "q":
            plt.close(self.figure)

    def undo_last_point(self) -> None:
        if not self.control_points:
            return
        self.control_points.pop()
        self._update_spline()
        self._redraw()

    def _update_spline(self) -> None:
        if len(self.control_points) < 2:
            self.sampled_path = []
            return

        try:
            spline = ParametricCubicSpline2D(self.control_points)
            self.sampled_path = spline.sample(self.config.sample_interval_m)
        except RuntimeError as exc:
            print(f"[path_generator] spline update skipped: {exc}")
            self.sampled_path = []

    def _redraw(self) -> None:
        if self.control_points_artist is not None:
            self.control_points_artist.remove()
            self.control_points_artist = None

        for artist in self.control_label_artists:
            artist.remove()
        self.control_label_artists = []

        if self.spline_path_artist is not None:
            self.spline_path_artist.remove()
            self.spline_path_artist = None

        if self.status_text_artist is not None:
            self.status_text_artist.remove()
            self.status_text_artist = None

        if self.control_points:
            points = np.asarray(self.control_points)
            (self.control_points_artist,) = self.ax.plot(
                points[:, 0],
                points[:, 1],
                "o--",
                color="#ff7f0e",
                linewidth=1.5,
                markersize=6,
                label="control points",
            )

            if self.config.show_control_point_labels:
                for index, point in enumerate(points):
                    label = self.ax.text(
                        point[0],
                        point[1],
                        f"  P{index}",
                        color="#8c2d04",
                        fontsize=9,
                    )
                    self.control_label_artists.append(label)

        if self.sampled_path:
            xs = [point["x_m"] for point in self.sampled_path]
            ys = [point["y_m"] for point in self.sampled_path]
            (self.spline_path_artist,) = self.ax.plot(
                xs,
                ys,
                "-",
                color="#00b300",
                linewidth=2.0,
                label="cubic spline",
            )

        status = self._status_text()
        self.status_text_artist = self.ax.text(
            0.01,
            0.01,
            status,
            transform=self.ax.transAxes,
            fontsize=9,
            color="#111111",
            bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#999999"},
        )

        self.figure.canvas.draw_idle()

    def _status_text(self) -> str:
        if self.sampled_path:
            path_length_m = self.sampled_path[-1]["s_m"]
        else:
            path_length_m = 0.0

        return (
            "Left click: add point | Right click / U: undo | C: clear | S: save | Q: quit\n"
            f"Control points: {len(self.control_points)} | Path length: {path_length_m:.2f} m"
        )

    def save_path(self) -> None:
        if len(self.control_points) < 2 or not self.sampled_path:
            print("[path_generator] at least two valid control points are required before saving")
            return

        self.config.output_yaml_file.parent.mkdir(parents=True, exist_ok=True)
        self.config.output_csv_file.parent.mkdir(parents=True, exist_ok=True)

        control_points = [
            {
                "index": index,
                "x_m": float(point[0]),
                "y_m": float(point[1]),
            }
            for index, point in enumerate(self.control_points)
        ]

        path_yaml = {
            "format_version": 1,
            "frame_id": "map",
            "map_yaml_file": str(self.config.map_yaml_file),
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "sample_interval_m": self.config.sample_interval_m,
            "control_points": control_points,
            "spline_points": self.sampled_path,
        }

        with self.config.output_yaml_file.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(path_yaml, stream, sort_keys=False, allow_unicode=False)

        with self.config.output_csv_file.open("w", encoding="utf-8") as stream:
            stream.write("index,s_m,x_m,y_m,yaw_rad,curvature\n")
            for point in self.sampled_path:
                stream.write(
                    f"{point['index']},{point['s_m']:.6f},{point['x_m']:.6f},"
                    f"{point['y_m']:.6f},{point['yaw_rad']:.6f},{point['curvature']:.6f}\n"
                )

        print(f"[path_generator] saved yaml: {self.config.output_yaml_file}")
        print(f"[path_generator] saved csv:  {self.config.output_csv_file}")

    def run(self) -> None:
        plt.show()


def parse_args() -> argparse.Namespace:
    package_share = Path(get_package_share_directory("simulation"))
    default_config = package_share / "config" / "path_generator_params.yaml"

    parser = argparse.ArgumentParser(description="Interactive path generator for Nav2 maps")
    parser.add_argument(
        "--config",
        default=str(default_config),
        help="Path generator config yaml",
    )

    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    config_file = Path(args.config).resolve()
    config = PathGeneratorConfig.from_yaml(config_file)
    app = PathGeneratorApp(config)
    app.run()


if __name__ == "__main__":
    main()
