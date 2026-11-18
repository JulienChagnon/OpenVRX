# SPDX-License-Identifier: GPL-3.0-or-later
"""Find the bench ESP32's network console when its mDNS name does not resolve.

    from esp32_net import resolve
    url = resolve("socket://openvrx-bench.local:2323")   # returns a working socket:// URL or the input

mDNS on the bench host drops out now and then (the name answers, then times out minutes later), while
the ESP32 stays on Wi-Fi. Order: the URL as given; the last address that worked (cached in
~/.cache/openvrx/esp32-url); every IPv4 neighbour in the ARP table that answers "get" with "OK get"
on the console port. A found address is cached for next time. Only TCP connects are used: opening the
ESP32's USB port would reset it.
"""
import os
import re
import socket
import subprocess
from urllib.parse import urlparse

CACHE = os.path.expanduser("~/.cache/openvrx/esp32-url")


def _is_console(host, port, timeout=1.0):
    """True when host:port answers 'get' like att_ctl ('OK get db=...')."""
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.settimeout(timeout)
            s.sendall(b"get\n")
            buf = b""
            while b"\n" not in buf or b"OK get" not in buf:
                chunk = s.recv(1024)
                if not chunk:
                    break
                buf += chunk
                if len(buf) > 4096:
                    break
            return b"OK get" in buf
    except OSError:
        return False


def _cache(url):
    try:
        os.makedirs(os.path.dirname(CACHE), exist_ok=True)
        with open(CACHE, "w") as f:
            f.write(url + "\n")
    except OSError:
        pass


def _neighbours():
    p = subprocess.run(["ip", "-4", "neigh"], capture_output=True, text=True)
    out = []
    for line in p.stdout.splitlines():
        if "FAILED" in line or "INCOMPLETE" in line:
            continue
        m = re.match(r"(\d+\.\d+\.\d+\.\d+) ", line)
        if m:
            out.append(m.group(1))
    return out


def resolve(url, verbose=True):
    u = urlparse(url)
    if u.scheme != "socket" or not u.hostname:
        return url
    port = u.port or 2323
    if _is_console(u.hostname, port):
        try:  # remember the address behind the name for the next time mDNS drops out
            _cache(f"socket://{socket.gethostbyname(u.hostname)}:{port}")
        except OSError:
            pass
        return url
    tried = {u.hostname}
    cands = []
    try:
        with open(CACHE) as f:
            cached = urlparse(f.read().strip())
        if cached.hostname:
            cands.append(cached.hostname)
    except OSError:
        pass
    cands += _neighbours()
    for host in cands:
        if host in tried:
            continue
        tried.add(host)
        if _is_console(host, port, timeout=0.5):
            found = f"socket://{host}:{port}"
            _cache(found)
            if verbose:
                print(f"ESP32: {u.hostname} not reachable, using {found}")
            return found
    return url
