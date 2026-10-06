import os

import cv2
from ament_index_python.packages import get_package_share_directory

TOP_PASTE = (105, 145, 230, 255)
BOT_PASTE = (140, 180, 230, 255)
UNIT_BLANK_DST = (105, 145, 250, 300)
UNIT_BLANK_SRC = (140, 180, 250, 300)
SIGN_TOP = (120, 130, 125, 145)
SIGN_BOT = (155, 165, 125, 145)
SIGN_THRESHOLD = 140
SIGN_LEVEL = 250.0
DIGIT_X_MIN = 150
DIGIT_X_MAX = 250
ROW_SPLIT_Y = 120


def get(img, region):
    y0, y1, x0, x1 = region
    return img[y0:y1, x0:x1]


def put(img, region, patch):
    y0, y1, x0, x1 = region
    img[y0:y1, x0:x1] = patch


def normalize(grey):
    grey = cv2.convertScaleAbs(grey, alpha=1, beta=0)
    return cv2.normalize(grey, None, 0, 255, cv2.NORM_MINMAX)


class DcvgOcr:

    def __init__(self, reader):
        self.reader = reader
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        template_dir = os.path.join(get_package_share_directory('dcvg_bringup'), 'ocr')
        self.top = self.load_template(os.path.join(template_dir, 'append_7.jpg'))
        self.bot = self.load_template(os.path.join(template_dir, 'append_8.jpg'))

    @staticmethod
    def load_template(path):
        img = cv2.imread(path)
        if img is None:
            raise FileNotFoundError(f'digit template {path} is missing or unreadable')
        return normalize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))

    def is_positive(self, grey, region):
        _, patch = cv2.threshold(get(grey, region), SIGN_THRESHOLD, 255, cv2.THRESH_BINARY)
        return cv2.mean(patch)[0] > SIGN_LEVEL

    def read(self, frame):
        if frame is None or frame.size == 0:
            raise ValueError('empty frame from the meter camera')
        grey = normalize(self.clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))
        put(grey, TOP_PASTE, self.top)
        put(grey, BOT_PASTE, self.bot)
        put(grey, UNIT_BLANK_DST, get(grey, UNIT_BLANK_SRC))
        positive_top = self.is_positive(grey, SIGN_TOP)
        positive_bot = self.is_positive(grey, SIGN_BOT)

        on = off = None
        for bbox, text, _ in self.reader.readtext(grey, allowlist='0123456789'):
            x, y = int(bbox[0][0]), int(bbox[0][1])
            if not DIGIT_X_MIN < x < DIGIT_X_MAX:
                continue
            try:
                value = int(text[:-1])
            except ValueError:
                continue
            if y < ROW_SPLIT_Y and text[-1] == '7':
                on = value if positive_top else -value
            elif y > ROW_SPLIT_Y and text[-1] == '8':
                off = value if positive_bot else -value
        return on, off
