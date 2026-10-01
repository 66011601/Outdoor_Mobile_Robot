"""
dcvg_ocr.py  (REAL HARDWARE — DCVG meter display OCR)

Reads the two numbers off the DCVG meter's LCD through a USB webcam pointed at
it: the ON reading (top row) and the OFF reading (bottom row).

This is a package-native port of ~/Desktop/DCVG_Robot/ocr/screen_ocr.py. The
algorithm is unchanged — same preprocessing, same digit-anchor trick, same sign
detection — but everything it hard-coded is now configurable, and everything it
recomputed per frame is now computed once. Three concrete differences:

  - **Templates ship with the package.** They install to
    share/dcvg_bringup/ocr/ and are found through the ament index, so nothing
    depends on the process working directory. The original loaded them with the
    relative paths 'ocr/append_7.jpg' / 'ocr/append_8.jpg', which forced any
    caller to chdir into the DCVG_Robot tree first.
  - **Geometry is data, not literals.** Every pixel region, threshold and cutoff
    below is a field of `OcrGeometry` with the original's value as its default,
    so re-aiming or re-mounting the camera is a parameter change rather than a
    source edit. This matters more than it sounds: all of it is keyed to exactly
    where the meter sits in a 320x240 frame.
  - **No ROS, no camera.** `read()` takes a frame and returns two ints. The
    capture loop, the parameters and the logging live in the caller
    (`teensy_feedback_server`'s `_OcrWorker`), which keeps this testable offline
    against a saved PNG.

How it reads the display
────────────────────────
easyocr will not reliably segment the LCD's two rows on their own, so the
original works around it by pasting a pre-rendered '7' into the top row and an
'8' into the bottom row, just right of the digits. Those anchors give the
detector something with a strong shape to latch onto, and they double as row
labels: a detection whose text ends in '7' is the ON reading, one ending in '8'
is the OFF reading, and the anchor digit is then stripped back off with
`text[:-1]`. Pixels holding the unit label are overwritten with a copy of the
row below so easyocr never sees them as glyphs.

The sign is NOT read as a glyph — a minus sign is too thin to survive the
downscale. Instead a small patch where the sign would be is thresholded, and its
mean brightness decides: bright means positive, dark means a minus is present.

Consequences worth knowing
──────────────────────────
  - A camera not aimed at the meter reads as (0, 0), NOT as an error. Both
    values start at zero and are only overwritten when a matching detection is
    found, so "no meter in view" and "meter reading zero" are indistinguishable
    in the return value. Callers that need to tell them apart should watch
    `Reading.confident`.
  - Every geometry default assumes a 320x240 frame with the meter in a specific
    position. Change the camera, the lens or the bracket and they all need
    re-deriving. `annotate()` exists to make that quick: it draws every region
    on a frame so you can see what the numbers actually select.
"""

from dataclasses import dataclass, field, replace
import os

import cv2
import numpy as np

# Frame size the geometry defaults were derived against. Not enforced — a
# different size is allowed and simply means the defaults no longer apply.
REFERENCE_FRAME_SIZE = (320, 240)   # (width, height)

# Only digits: the display shows nothing else, and letting easyocr consider
# letters turns '0' into 'O' and '1' into 'I' often enough to matter.
DIGIT_ALLOWLIST = '0123456789'


def _region(y0: int, y1: int, x0: int, x1: int):
    """A (y0, y1, x0, x1) slice window, in the order the original indexed."""
    return (y0, y1, x0, x1)


@dataclass
class OcrGeometry:
    """Where things are in the frame, and the cutoffs used to read them.

    Every default is the literal the original screen_ocr.py used. Regions are
    (y0, y1, x0, x1) — row range first, matching numpy's indexing order.
    """

    # Where the '7' and '8' anchor digits get pasted. Must match the template
    # images' own dimensions (40x25); validated at load.
    top_paste: tuple = field(default_factory=lambda: _region(105, 145, 230, 255))
    bot_paste: tuple = field(default_factory=lambda: _region(140, 180, 230, 255))

    # The unit label ('mV') sits here on the top row and would be read as
    # glyphs. It gets painted over with a copy of `unit_blank_src` — a patch of
    # the row below, which is blank at this x range. Both regions must be the
    # same shape.
    unit_blank_dst: tuple = field(default_factory=lambda: _region(105, 145, 250, 300))
    unit_blank_src: tuple = field(default_factory=lambda: _region(140, 180, 250, 300))

    # Small patches where a minus sign would be, one per row.
    sign_top: tuple = field(default_factory=lambda: _region(120, 130, 125, 145))
    sign_bot: tuple = field(default_factory=lambda: _region(155, 165, 125, 145))
    # Binarisation cutoff applied to those patches, then the mean of the result
    # is compared against sign_level: above = positive, below = negative.
    # sign_level is deliberately near 255 — the patch is almost entirely white
    # unless a sign stroke is actually in it.
    sign_threshold: int = 140
    sign_level: float = 250.0

    # A detection is only considered if its top-left x falls in this band; that
    # is the digit column, and it excludes the anchors' neighbours and any edge
    # artefacts.
    digit_x_min: int = 150
    digit_x_max: int = 250
    # Detections above this y are the ON row, below it the OFF row.
    row_split_y: int = 120

    # Contrast-limited histogram equalisation, to even out LCD glare.
    clahe_clip: float = 2.0
    clahe_grid: int = 8

    def scaled(self, width: int, height: int) -> 'OcrGeometry':
        """This geometry mapped onto a differently-sized frame.

        A convenience for trying the same mount at a different capture
        resolution. It is a linear rescale and nothing more — it will not
        rescue a re-aimed camera, which needs the regions re-derived.
        """
        sx = width / REFERENCE_FRAME_SIZE[0]
        sy = height / REFERENCE_FRAME_SIZE[1]

        def s(r):
            return _region(round(r[0] * sy), round(r[1] * sy),
                           round(r[2] * sx), round(r[3] * sx))

        return replace(
            self,
            top_paste=s(self.top_paste), bot_paste=s(self.bot_paste),
            unit_blank_dst=s(self.unit_blank_dst), unit_blank_src=s(self.unit_blank_src),
            sign_top=s(self.sign_top), sign_bot=s(self.sign_bot),
            digit_x_min=round(self.digit_x_min * sx),
            digit_x_max=round(self.digit_x_max * sx),
            row_split_y=round(self.row_split_y * sy))


@dataclass
class Reading:
    """One pass over one frame."""

    on: int = 0
    off: int = 0
    # False when a value was not actually detected and is reporting its initial
    # zero. This is the distinction the original could not express: it returned
    # (0, 0) both for "meter reads zero" and "no meter in view".
    on_found: bool = False
    off_found: bool = False

    @property
    def confident(self) -> bool:
        return self.on_found and self.off_found

    def as_tuple(self):
        """(on, off) — the shape the original dcvg_ocr() returned."""
        return self.on, self.off


def default_template_dir() -> str:
    """Where the installed digit templates live.

    Resolved through the ament index so it works from any working directory.
    Falls back to the source tree when running from a non-installed checkout.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('dcvg_bringup')
        candidate = os.path.join(share, 'ocr')
        if os.path.isdir(candidate):
            return candidate
    except Exception:       # noqa: BLE001 - not installed / no ament; try source
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ocr')


class DcvgOcr:
    """Reads the meter display. Construct once, call `read()` per frame."""

    def __init__(self, reader, geometry: OcrGeometry = None, template_dir: str = None):
        """
        `reader` is an easyocr.Reader. It is injected rather than built here so
        the caller controls when the ~5 s model load happens (and can share one
        reader, or substitute a fake in tests).
        """
        self.geometry = geometry or OcrGeometry()
        self._reader = reader
        self._template_dir = template_dir or default_template_dir()

        # Loaded and preprocessed ONCE. The original redid both imread()s and
        # their normalisation on every frame; it is only ~0.17 ms, so this is
        # about the templates being a fixed asset rather than about speed.
        self._top = self._load_template('append_7.jpg', self.geometry.top_paste)
        self._bot = self._load_template('append_8.jpg', self.geometry.bot_paste)

        # Reused across frames; building it per frame allocates its lookup
        # tables every time for no benefit.
        self._clahe = cv2.createCLAHE(
            clipLimit=self.geometry.clahe_clip,
            tileGridSize=(self.geometry.clahe_grid, self.geometry.clahe_grid))

        g = self.geometry
        if _shape(g.unit_blank_dst) != _shape(g.unit_blank_src):
            raise ValueError(
                f'unit_blank_dst {_shape(g.unit_blank_dst)} and unit_blank_src '
                f'{_shape(g.unit_blank_src)} must be the same shape')

    # ── Setup ───────────────────────────────────────────────────────────────
    def _load_template(self, name: str, paste: tuple) -> np.ndarray:
        path = os.path.join(self._template_dir, name)
        img = cv2.imread(path)
        if img is None:
            raise FileNotFoundError(
                f'digit template {path} is missing or unreadable — it should '
                f'have been installed to share/dcvg_bringup/ocr/ by setup.py')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.convertScaleAbs(img, alpha=1, beta=0)
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        want = _shape(paste)
        if img.shape != want:
            # The original would have raised a bare numpy broadcast error deep
            # inside the per-frame path. Say which file and which region.
            raise ValueError(
                f'{path} is {img.shape}, but its paste region is {want} — '
                f'adjust the geometry or re-render the template')
        return img

    # ── Per frame ───────────────────────────────────────────────────────────
    def preprocess(self, frame: np.ndarray) -> np.ndarray:
        """Grey, equalise, normalise, paste anchors, blank the unit label."""
        g = self.geometry
        if frame is None or getattr(frame, 'size', 0) == 0:
            # cv2.cvtColor on an empty Mat raises a bare "!_src.empty()"
            # assertion from deep inside color.cpp, which says nothing about
            # where the frame came from. A capture that returned ok=False, or a
            # camera taken over by another process, lands here.
            raise ValueError(
                'empty frame — the capture returned nothing. Another process '
                'may hold the camera (V4L2 gives exclusive access), or it was '
                'unplugged.')
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        grey = self._clahe.apply(grey)
        grey = cv2.convertScaleAbs(grey, alpha=1, beta=0)
        grey = cv2.normalize(grey, None, 0, 255, cv2.NORM_MINMAX)

        _put(grey, g.top_paste, self._top)
        _put(grey, g.bot_paste, self._bot)
        # Order matters: this overwrites the unit label AFTER the anchors are
        # in, exactly as the original did.
        _put(grey, g.unit_blank_dst, _get(grey, g.unit_blank_src))
        return grey

    def _is_positive(self, grey: np.ndarray, region: tuple) -> bool:
        patch = _get(grey, region)
        _, patch = cv2.threshold(patch, self.geometry.sign_threshold, 255, cv2.THRESH_BINARY)
        return cv2.mean(patch)[0] > self.geometry.sign_level

    def read(self, frame: np.ndarray) -> Reading:
        """Read one frame.

        Unreadable *display* content is not an error: a frame with no legible
        digits returns zeros with on_found/off_found False. An unusable *frame*
        (None or empty) does raise — that is a capture fault, not a reading.
        """
        g = self.geometry
        grey = self.preprocess(frame)
        positive_top = self._is_positive(grey, g.sign_top)
        positive_bot = self._is_positive(grey, g.sign_bot)

        out = Reading()
        for bbox, text, _prob in self._reader.readtext(grey, allowlist=DIGIT_ALLOWLIST):
            x, y = int(bbox[0][0]), int(bbox[0][1])
            if not (g.digit_x_min < x < g.digit_x_max):
                continue
            # text[-1] is the pasted anchor: '7' marks the ON row, '8' the OFF
            # row. text[:-1] is the reading itself. A detection that caught only
            # the anchor leaves text[:-1] empty, which int() rejects below.
            try:
                if y < g.row_split_y and text[-1] == '7':
                    out.on = int(text[:-1]) if positive_top else -int(text[:-1])
                    out.on_found = True
                elif y > g.row_split_y and text[-1] == '8':
                    out.off = int(text[:-1]) if positive_bot else -int(text[:-1])
                    out.off_found = True
            except (IndexError, ValueError):
                # Anchor-only or malformed detection. Dropping it silently is
                # deliberate: at 10 Hz a per-frame complaint would bury the log,
                # and the caller already sees the miss as a stale reading.
                continue
        return out

    # ── Tuning aid ──────────────────────────────────────────────────────────
    def annotate(self, frame: np.ndarray) -> np.ndarray:
        """A BGR copy of the preprocessed frame with every region drawn on it.

        For re-deriving the geometry after the camera moves: write this to a PNG
        and you can see what the current numbers actually select, instead of
        inferring it from two numbers that came out wrong.
        """
        g = self.geometry
        canvas = cv2.cvtColor(self.preprocess(frame), cv2.COLOR_GRAY2BGR)
        for region, colour, label in (
                (g.top_paste, (0, 255, 0), 'top anchor'),
                (g.bot_paste, (0, 255, 0), 'bot anchor'),
                (g.unit_blank_dst, (255, 0, 0), 'unit blank'),
                (g.sign_top, (0, 0, 255), 'sign+'),
                (g.sign_bot, (0, 0, 255), 'sign-')):
            y0, y1, x0, x1 = region
            cv2.rectangle(canvas, (x0, y0), (x1, y1), colour, 1)
            cv2.putText(canvas, label, (x0, max(10, y0 - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, colour, 1)
        h = canvas.shape[0]
        cv2.line(canvas, (g.digit_x_min, 0), (g.digit_x_min, h), (0, 255, 255), 1)
        cv2.line(canvas, (g.digit_x_max, 0), (g.digit_x_max, h), (0, 255, 255), 1)
        cv2.line(canvas, (0, g.row_split_y), (canvas.shape[1], g.row_split_y),
                 (255, 0, 255), 1)
        return canvas


def _shape(region: tuple):
    y0, y1, x0, x1 = region
    return (y1 - y0, x1 - x0)


def _get(img: np.ndarray, region: tuple) -> np.ndarray:
    y0, y1, x0, x1 = region
    return img[y0:y1, x0:x1]


def _put(img: np.ndarray, region: tuple, patch: np.ndarray) -> None:
    y0, y1, x0, x1 = region
    img[y0:y1, x0:x1] = patch
