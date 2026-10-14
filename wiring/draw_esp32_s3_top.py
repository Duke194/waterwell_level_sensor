"""Front view of the ESP32-S3-DevKitC-1 with the four harness pins marked."""

from PIL import Image, ImageDraw, ImageFont

OUT = "esp32-s3-top.png"

RED = (196, 32, 32)
YELLOW = (214, 168, 0)
GREEN = (0, 140, 55)
BLACK = (25, 25, 25)
BOARD = (28, 28, 30)
PAD = (212, 175, 55)
GRAY = (120, 120, 124)
INK = (40, 40, 40)
WHITE = (255, 255, 255)

J1 = [
    "3V3", "3V3", "RST", "4", "5", "6", "7", "15", "16", "17", "18",
    "8", "3", "46", "9", "10", "11", "12", "13", "14", "5V", "GND",
]
J3 = [
    "GND", "TX", "RX", "1", "2", "42", "41", "40", "39", "38", "37",
    "36", "35", "0", "45", "48", "47", "21", "20", "19", "GND", "GND",
]
# Header index -> (cable color, callout). Pin 1 is index 0, at the antenna end.
MARKS = {
    0: (RED, "3V3  →  5V"),
    3: (YELLOW, "GPIO4  →  RX"),
    4: (GREEN, "GPIO5  →  TX"),
    21: (BLACK, "GND  →  GND"),
}


def main() -> None:
    font = ImageFont.truetype(r"C:\Windows\Fonts\arial.ttf", 20)
    font_b = ImageFont.truetype(r"C:\Windows\Fonts\arialbd.ttf", 22)
    font_s = ImageFont.truetype(r"C:\Windows\Fonts\arial.ttf", 16)
    font_title = ImageFont.truetype(r"C:\Windows\Fonts\arialbd.ttf", 32)

    width, height = 1100, 1420
    image = Image.new("RGB", (width, height), WHITE)
    draw = ImageDraw.Draw(image)

    draw.text((40, 24), "ESP32-S3-DevKitC-1", fill=INK, font=font_title)
    draw.text((40, 66), "Front view. Antenna at the top, USB sockets at the bottom. J1 is the left header.", fill=INK, font=font_s)

    board = (390, 110, 710, 1185)
    draw.rounded_rectangle(board, radius=18, fill=BOARD)

    draw.rounded_rectangle((470, 145, 630, 310), radius=6, fill=(88, 88, 92))
    draw.text((488, 210), "ESP32-S3", fill=WHITE, font=font_b)
    draw.polygon([(530, 145), (550, 112), (570, 145)], fill=(170, 170, 174))
    draw.text((500, 118), "Antenna", fill=WHITE, font=font_s)

    draw.rounded_rectangle((455, 1145, 545, 1245), radius=6, fill=(55, 55, 58), outline=(150, 150, 154))
    draw.rounded_rectangle((555, 1145, 645, 1245), radius=6, fill=(55, 55, 58), outline=(150, 150, 154))
    draw.text((478, 1180), "UART", fill=WHITE, font=font_s)
    draw.text((582, 1180), "USB", fill=WHITE, font=font_s)

    count = len(J1)
    top = 150
    bottom = 1120
    step = (bottom - top) / (count - 1)
    left_x = board[0]
    right_x = board[2]

    draw.text((left_x - 8, 84), "J1", fill=INK, font=font_b)
    draw.text((right_x - 8, 84), "J3", fill=INK, font=font_b)

    for index, (left_name, right_name) in enumerate(zip(J1, J3)):
        y = top + index * step
        mark = MARKS.get(index)
        draw.ellipse((left_x - 6, y - 6, left_x + 6, y + 6), fill=PAD)
        draw.ellipse((right_x - 6, y - 6, right_x + 6, y + 6), fill=PAD)
        draw.text((left_x + 16, y - 10), left_name, fill=(210, 210, 214), font=font_s)
        draw.text((right_x + 16, y - 10), right_name, fill=(176, 176, 180), font=font_s)
        if mark is None:
            continue
        color, callout = mark
        radius = 11
        draw.ellipse((left_x - radius - 2, y - radius - 2, left_x + radius + 2, y + radius + 2), fill=WHITE)
        draw.ellipse((left_x - radius, y - radius, left_x + radius, y + radius), fill=color)
        draw.line((210, y, left_x - radius, y), fill=color, width=4)
        draw.text((36, y - 12), callout, fill=color, font=font_b)

    legend_y = 1280
    draw.text((40, legend_y), "Marked pins are the harness wires. Other 3V3 and GND pins stay unused.", fill=INK, font=font_s)
    items = [
        (RED, "Red: J1 pin 1, 3V3, to SR04M-2 5V"),
        (YELLOW, "Yellow: J1 pin 4, GPIO4, to SR04M-2 RX"),
        (GREEN, "Green: J1 pin 5, GPIO5, to SR04M-2 TX"),
        (BLACK, "Black: J1 pin 22, GND, to SR04M-2 GND"),
    ]
    for offset, (color, text) in enumerate(items):
        y = legend_y + 36 + offset * 28
        draw.rectangle((40, y, 64, y + 16), fill=color, outline=(180, 180, 180))
        draw.text((76, y - 3), text, fill=INK, font=font_s)

    image.save(OUT, "PNG")


if __name__ == "__main__":
    main()
