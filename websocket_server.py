"""Small asynchronous WebSocket echo server."""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
from collections.abc import Sequence

from websockets.asyncio.server import ServerConnection, serve
from websockets.exceptions import ConnectionClosed


LOGGER = logging.getLogger("websocket_server")


async def echo(websocket: ServerConnection) -> None:
    """Echo every text or binary message back to one connected client."""
    remote = websocket.remote_address
    LOGGER.info("Client connected: %s", remote)

    try:
        async for message in websocket:
            await websocket.send(message)
    except ConnectionClosed as exc:
        LOGGER.info("Client connection closed abnormally: %s (%s)", remote, exc)
    finally:
        LOGGER.info("Client disconnected: %s", remote)


async def run_server(host: str, port: int) -> None:
    """Run the server until it receives SIGINT or SIGTERM."""
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    async with serve(echo, host, port):
        LOGGER.info("Listening on ws://%s:%d", host, port)
        await stop.wait()
        LOGGER.info("Shutting down")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a WebSocket echo server.")
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="interface to bind to (default: %(default)s)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8765,
        help="port to listen on (default: %(default)s)",
    )
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    return args


def main(argv: Sequence[str] | None = None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    asyncio.run(run_server(args.host, args.port))
    LOGGER.info("Server stopped")


if __name__ == "__main__":
    main()
