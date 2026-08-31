#!/usr/bin/env python3
"""
Live telemetry dashboard for the STM32 Nucleo-F446RE.

Reads telemetry lines off the board's USB virtual COM port (USART2, 115200 baud)
and streams parsed values to a browser dashboard over a websocket.

Usage:
    python3 server.py [--port /dev/tty.usbmodemXXXX] [--baud 115200] [--http-port 8000]

If --port is omitted, the script scans /dev/tty.usbmodem* and picks the first
match, so it survives replugging the board (which can change the device number).
"""

import argparse
import glob
import json
import re
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

import serial
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

BAUD_DEFAULT = 115200
HTTP_PORT_DEFAULT = 8000
RECONNECT_DELAY_S = 1.0

LINE_RE = re.compile(
    r"APPS1:(?P<APPS1>-?\d+(?:\.\d+)?)\s+"
    r"APPS2:(?P<APPS2>-?\d+(?:\.\d+)?)\s+"
    r"BRAKE:(?P<BRAKE>[01])\s+"
    r"FAULT:(?P<FAULT>[01])\s+"
    r"PCUT:(?P<PCUT>[01])\s+"
    r"WSPD:(?P<WSPD>-?\d+(?:\.\d+)?)\s+"
    r"GYRO:(?P<GYRO>-?\d+(?:\.\d+)?)\s+"
    r"SLIP:(?P<SLIP>[01])"
)


def find_port() -> str | None:
    candidates = sorted(glob.glob("/dev/tty.usbmodem*"))
    return candidates[0] if candidates else None


def parse_line(line: str) -> dict | None:
    m = LINE_RE.search(line)
    if not m:
        return None
    d = m.groupdict()
    return {
        "APPS1": float(d["APPS1"]),
        "APPS2": float(d["APPS2"]),
        "BRAKE": int(d["BRAKE"]),
        "FAULT": int(d["FAULT"]),
        "PCUT": int(d["PCUT"]),
        "WSPD": float(d["WSPD"]),
        "GYRO": float(d["GYRO"]),
        "SLIP": int(d["SLIP"]),
        "ts": time.time(),
    }


class Hub:
    """Tracks connected websocket clients and the latest connection status."""

    def __init__(self):
        self.clients: set[WebSocket] = set()
        self.lock = threading.Lock()
        self.loop = None  # set once the asyncio loop is running

    async def register(self, ws: WebSocket):
        await ws.accept()
        with self.lock:
            self.clients.add(ws)

    def unregister(self, ws: WebSocket):
        with self.lock:
            self.clients.discard(ws)

    def broadcast(self, message: dict):
        """Thread-safe: called from the serial-reader background thread."""
        if self.loop is None:
            return
        payload = json.dumps(message)
        with self.lock:
            targets = list(self.clients)
        for ws in targets:
            self.loop.call_soon_threadsafe(
                lambda ws=ws: self.loop.create_task(self._safe_send(ws, payload))
            )

    async def _safe_send(self, ws: WebSocket, payload: str):
        try:
            await ws.send_text(payload)
        except Exception:
            self.unregister(ws)


hub = Hub()


@asynccontextmanager
async def lifespan(app: FastAPI):
    import asyncio

    hub.loop = asyncio.get_event_loop()
    yield


app = FastAPI(lifespan=lifespan)

STATIC_DIR = Path(__file__).parent


@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "index.html")


@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await hub.register(websocket)
    try:
        while True:
            # No messages expected from the client; just keep the connection open.
            await websocket.receive_text()
    except WebSocketDisconnect:
        hub.unregister(websocket)


def serial_reader(port_arg: str | None, baud: int, stop_event: threading.Event):
    """Background thread: opens the serial port (auto-detecting/reconnecting as
    needed) and broadcasts each parsed telemetry line to connected clients."""
    while not stop_event.is_set():
        port = port_arg or find_port()
        if not port:
            hub.broadcast({"status": "waiting_for_device"})
            time.sleep(RECONNECT_DELAY_S)
            continue

        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                hub.broadcast({"status": "connected", "port": port})
                print(f"[serial] connected on {port} @ {baud} baud")
                while not stop_event.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue
                    if "Board booted" in line or "UART alive" in line:
                        continue
                    parsed = parse_line(line)
                    if parsed:
                        hub.broadcast({"status": "connected", "data": parsed})
        except (serial.SerialException, OSError) as e:
            print(f"[serial] lost connection on {port} ({e}); retrying...")
            hub.broadcast({"status": "waiting_for_device"})
            time.sleep(RECONNECT_DELAY_S)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial device path (default: auto-detect /dev/tty.usbmodem*)")
    parser.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    parser.add_argument("--http-port", type=int, default=HTTP_PORT_DEFAULT)
    args = parser.parse_args()

    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=serial_reader, args=(args.port, args.baud, stop_event), daemon=True
    )
    reader_thread.start()

    try:
        uvicorn.run(app, host="127.0.0.1", port=args.http_port, log_level="info")
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
