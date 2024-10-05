from typing import Optional
import cv2

import numpy as np


def set_h(x, self):
    self.h = x


def set_s(x, self):
    self.s = x


def set_v(x, self):
    self.v = x


def set_value(x, self):
    self.value = x


class HsvCalibrator:
    h: int
    s: int
    v: int
    name: str

    def __init__(
        self, name: str, show: bool, initial_values: tuple[int, int, int], prefix=""
    ) -> None:
        self.name = name
        self.h = initial_values[0]
        self.s = initial_values[1]
        self.v = initial_values[2]
        if show:
            cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
            cv2.createTrackbar(
                f"{prefix} H", name, self.h, 179, lambda x: set_h(x, self)
            )
            cv2.createTrackbar(
                f"{prefix} S", name, self.s, 255, lambda x: set_s(x, self)
            )
            cv2.createTrackbar(
                f"{prefix} V", name, self.v, 255, lambda x: set_v(x, self)
            )

    def value(self):
        return np.array([self.h, self.s, self.v])


class HsvRangeCalibrator:
    lower: HsvCalibrator
    upper: HsvCalibrator

    def __init__(
        self,
        name: str,
        show: bool,
        initial_lower: tuple[int, int, int],
        initial_upper: tuple[int, int, int],
    ) -> None:
        self.lower = HsvCalibrator(name, show, initial_lower, "lower")
        self.upper = HsvCalibrator(name, show, initial_upper, "upper")


class ParameterCalibrator:
    value: int

    def __init__(self, name, show, initial_value: int, prefix="", max_val=255) -> None:
        self.value = initial_value
        if show:
            cv2.namedWindow(name)
            cv2.createTrackbar(
                f"{prefix} val", name, self.value, max_val, lambda x: set_value(x, self)
            )


def draw_line(img: cv2.typing.MatLike, params: np.ndarray):
    vx = int(params.item(0))
    vy = int(params.item(1))
    x = int(params.item(2))
    y = int(params.item(3))
    mult = max(img.shape[0], img.shape[1])
    start = (x - mult * vx, y - mult * vy)
    end = (x + mult * vx, y + mult * vy)
    _, start, end = cv2.clipLine((0, 0, img.shape[0], img.shape[1]), start, end)
    cv2.line(img, start, end, (0, 255, 0), 2)


class LineDetector:
    def __init__(self, int_matrix, calibrate=False) -> None:
        self.int_matrix = int_matrix
        self.range = HsvRangeCalibrator("line", calibrate, (0, 0, 240), (179, 56, 255))
        self.open_kernel_size = ParameterCalibrator("contour", calibrate, 3, "open")
        self.close_kernel_size = ParameterCalibrator("contour", calibrate, 29, "close")
        self.contour_threshold = ParameterCalibrator(
            "contour", calibrate, 29, "threshold", max_val=5000
        )

    def detect(
        self,
        frame: cv2.typing.MatLike,
        debug_frame: Optional[cv2.typing.MatLike] = None,
    ):
        # thresholding
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            frame_hsv, self.range.lower.value(), self.range.upper.value()
        )

        # cleaning up noise
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.open_kernel_size.value, self.open_kernel_size.value)
        )
        morph = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.close_kernel_size.value, self.close_kernel_size.value)
        )
        morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, kernel)
        if debug_frame is not None:
            cv2.imshow("linemorph", morph)

        # find contours
        return morph
        contours = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        for c in contours:
            print(c.shape)
            area = cv2.contourArea(c)
            if area > self.contour_threshold.value:
                line = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
                if debug_frame is not None:
                    cv2.drawContours(debug_frame, [c], -1, (0, 0, 255), 2)
                    draw_line(debug_frame, line)
