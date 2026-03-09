#!/usr/bin/env python3
"""Gasi wszystkie LEDy NeoPixel sterowane po SPI na Jetsonie."""

import sys
import time
from pathlib import Path

import board
import neopixel_spi as neopixel

NUM_PIXELS = 7
BRIGHTNESS = 1.0
PIXEL_ORDER = neopixel.GRB
SPI_NODE = Path("/dev/spidev0.0")


def make_pixels():
    """Zwraca obiekt NeoPixel albo None gdy SPI nie jest dostępne."""
    if not SPI_NODE.exists():
        print(
            f"Brak {SPI_NODE}. W Jetson-IO włącz SPI na złączu (MOSI pin 19).",
            file=sys.stderr,
        )
        return None

    try:
        spi = board.SPI()  # domyślne SCK + MOSI (pin 19)
    except (OSError, RuntimeError) as exc:
        print(
            f"SPI niedostępne: {exc}. Sprawdź uprawnienia i konfigurację SPI.",
            file=sys.stderr,
        )
        return None

    return neopixel.NeoPixel_SPI(
        spi,
        NUM_PIXELS,
        brightness=BRIGHTNESS,
        pixel_order=PIXEL_ORDER,
        auto_write=False,
    )


def main():
    pixels = make_pixels()
    if pixels is None:
        return 1

    pixels.fill((0, 0, 0))
    pixels.show()
    time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
