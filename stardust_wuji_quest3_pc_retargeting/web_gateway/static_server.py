from __future__ import annotations

import ssl
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from functools import partial


class NoStoreStaticRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Pragma", "no-cache")
        super().end_headers()


def make_static_server(
    host: str,
    port: int,
    static_root: str | Path,
    cert_file: str | Path | None = None,
    key_file: str | Path | None = None,
) -> ThreadingHTTPServer:
    root = Path(static_root).resolve()
    handler = partial(NoStoreStaticRequestHandler, directory=str(root))
    server = ThreadingHTTPServer((host, int(port)), handler)
    if cert_file and key_file and Path(cert_file).exists() and Path(key_file).exists():
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(str(cert_file), str(key_file))
        server.socket = context.wrap_socket(server.socket, server_side=True)
    return server
