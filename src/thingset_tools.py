#
# Copyright (c) 2021-present LAAS-CNRS
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# @author Luiz Villa <luiz.villa@laas.fr>
#

"""
thingset_tools.py

Host-side helper for talking to a ThingSet-over-serial-shell device such as
the one built by this example (see THINGSET_SERIAL_EXAMPLE.md). Wraps the
Zephyr-shell `select thingset` dance and the ThingSet Text Mode protocol
(https://thingset.io/spec/latest/text-mode/) behind a small class.

Quick start:

    from thingset_tools import ThingSetTools

    ts = ThingSetTools()                       # port is optional: auto-
                                                # detected by USB VID/PID,
                                                # then a handshake probe
                                                # (pass a port string to
                                                # skip auto-detection)
    ts.discover()                              # walks the tree, writes
                                                # thingset_objects.json

    ts.read("Measurements/rV1Low_V")           # method-style access
    ts.write("Config", {"wBlinkPeriod_s": 0.2})

    ts.objects.Measurements.rV1Low_V           # same read, as an attribute
    ts.objects.Config.wBlinkPeriod_s = 0.2     # same write, as an attribute

`ts.objects` is populated by discover() and supports tab-completion in an
interactive session (IPython/Jupyter): `ts.objects.<TAB>` lists groups,
`ts.objects.Measurements.<TAB>` lists that group's items, etc.

Auto-detection (see find_ports()) first tries ports matching OwnTech's USB
vendor ID, falling back to every serial port on the system if none match.
Since a board's console and ThingSet-shell ports share the same VID/PID,
each candidate is actually opened and sent the `select thingset` handshake
- only the shell port responds - so this is more than a simple VID/PID
filter.
"""

import json
import re
import time

import serial

# ThingSet response status codes (thingset.io/spec, "Access Functions").
STATUS_CODES = {
    0x81: "Created",
    0x82: "Deleted",
    0x84: "Changed",
    0x85: "Content",
    0xA0: "Bad Request",
    0xA1: "Unauthorized",
    0xA3: "Forbidden (read-only value)",
    0xA4: "Not Found",
    0xA5: "Method Not Allowed",
    0xA8: "Request Entity Incomplete",
    0xA9: "Conflict",
    0xAD: "Request Entity Too Large",
    0xAF: "Unsupported Content-Format",
    0xC0: "Internal Server Error",
    0xC1: "Not Implemented",
    0xC4: "Gateway Timeout",
    0xC5: "Not a Gateway",
}

# ThingSet leaf-name prefix convention -> human-readable access type.
_LEAF_TYPES = {
    "r": "read-only",
    "w": "writable",
    "s": "writable-setting",
    "x": "executable",
}
_WRITABLE_TYPES = ("writable", "writable-setting")

_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
_PROMPT_RE = re.compile(r"[A-Za-z0-9_-]+:~\$\s*")
_RESP_RE = re.compile(r"^:([0-9A-Fa-f]{2})(?:\s+(.*))?$", re.MULTILINE)


class ThingSetError(Exception):
    """Raised when a ThingSet request returns an error status code."""

    def __init__(self, code, message):
        self.code = code
        super().__init__(
            f"0x{code:02X} {message}" if code is not None else message
        )


# OwnTech boards' USB vendor ID. Both the console and the ThingSet-shell
# CDC-ACM ports of the same board share this VID (and usually the same
# PID too), so it narrows the search but doesn't single out the shell
# port by itself - see the handshake-probing loop in __init__ below.
# Adapted from old/old4/find_devices.py.
OWNTECH_USB_VID = 0x2FE3


class ThingSetTools:
    """Talks ThingSet Text Mode to a device over its dedicated shell UART."""

    def __init__(self, port=None, baudrate=115200, timeout=1.0,
                 vid=OWNTECH_USB_VID, pid=None):
        """Connect to a ThingSet-over-shell device.

        If `port` is omitted, candidate ports are found by USB `vid`/`pid`
        (see `find_ports()`), falling back to every serial port on the
        system if none match. Each candidate is tried in turn - opened,
        and handed the `select thingset` handshake - since the console
        and shell ports of the same board share the same VID/PID and only
        the shell one will actually respond. The first one that works is
        used; RuntimeError is raised if none do.
        """
        self._tree = None
        self.objects = None

        if port is not None:
            self.port = port
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(0.3)
            self.ser.reset_input_buffer()
            self._transact("")
            self._transact("select thingset")
            self.get("")
            return

        candidates = self.find_ports(vid=vid, pid=pid)
        if not candidates:
            from serial.tools import list_ports

            candidates = [info.device for info in list_ports.comports()]
        for device in candidates:
            try:
                self.__init__(device, baudrate=baudrate, timeout=timeout)
                return
            except Exception:
                continue
        raise RuntimeError(
            f"no ThingSet-over-shell device found (tried {len(candidates)} "
            "candidate port(s))"
        )

    @staticmethod
    def find_ports(vid=OWNTECH_USB_VID, pid=None):
        """List serial ports matching a USB vendor ID (and optionally a
        specific product ID). Defaults to OwnTech's VID. Adapted from
        old/old4/find_devices.py."""
        from serial.tools import list_ports

        return [
            info.device
            for info in list_ports.comports()
            if info.vid == vid and (pid is None or info.pid == pid)
        ]

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def __repr__(self):
        return f"<ThingSetTools {self.port}>"

    # ---- low-level transport -------------------------------------------

    def _transact(self, cmd, timeout=3.0):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\r\n").encode())

        deadline = time.time() + timeout
        raw = b""
        plain = ""
        while time.time() < deadline:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                raw += chunk
                plain = _ANSI_RE.sub("", raw.decode(errors="replace"))
                if _PROMPT_RE.search(plain):
                    break
            else:
                time.sleep(0.02)
        else:
            raise TimeoutError(f"no response to {cmd!r} on {self.port}")

        body = _PROMPT_RE.sub("", plain)
        lines = [line.strip("\r") for line in body.splitlines()]
        lines = [line for line in lines if line.strip() and line.strip() != cmd.strip()]
        return "\n".join(lines).strip()

    def _parse_response(self, body):
        match = _RESP_RE.search(body)
        if not match:
            raise ThingSetError(None, f"unexpected response: {body!r}")
        code = int(match.group(1), 16)
        payload = match.group(2)
        if code >= 0xA0:
            raise ThingSetError(code, STATUS_CODES.get(code, "Unknown error"))
        if not payload:
            return None
        try:
            return json.loads(payload)
        except json.JSONDecodeError:
            return payload

    # ---- ThingSet requests ----------------------------------------------

    def get(self, path=""):
        """GET a path. Returns the parsed JSON value (or a group dump)."""
        return self._parse_response(self._transact(f"?{path}"))

    def fetch_children(self, path=""):
        """FETCH-with-null a group path; returns the list of child names."""
        result = self._parse_response(self._transact(f"?{path} null"))
        return result if isinstance(result, list) else []

    def read(self, path):
        """Read a single item's value."""
        return self.get(path)

    def write(self, path, values):
        """UPDATE `path` with `values` (a dict of {item_name: value}).

        Handles the Zephyr shell's unescaped-double-quote stripping
        automatically, so callers just pass a normal dict.
        """
        payload = json.dumps(values, separators=(",", ":")).replace('"', '\\"')
        self._parse_response(self._transact(f"={path} {payload}"))
        return True

    # ---- discovery --------------------------------------------------------

    @staticmethod
    def _classify_leaf(name):
        if not name:
            return "unknown"
        return _LEAF_TYPES.get(name[0], "informational")

    def _discover(self, path):
        tree = {}
        for name in self.fetch_children(path):
            child_path = f"{path}/{name}" if path else name
            # A group's plain GET returns a JSON object (its children);
            # a leaf returns a scalar/array. Reserved groups such as
            # "_Reporting" don't follow the r/w/s/x/Capitalized naming
            # convention, so this is the only reliable group/leaf test.
            value = self.get(child_path)
            if isinstance(value, dict):
                tree[name] = {
                    "type": "group",
                    "path": child_path,
                    "children": self._discover(child_path),
                }
            else:
                tree[name] = {
                    "type": self._classify_leaf(name),
                    "path": child_path,
                }
        return tree

    def discover(self, json_path="thingset_objects.json"):
        """Recursively walk the device's ThingSet object tree, save it to
        `json_path`, populate `self.objects` for attribute-style access,
        and return the tree as a dict."""
        tree = self._discover("")
        self._tree = tree
        self.objects = _ThingSetNode(self, "")
        self.objects._populate(tree)
        if json_path:
            with open(json_path, "w") as f:
                json.dump(tree, f, indent=2)
        return tree

    # ---- bulk read / write -------------------------------------------------

    def read_all(self, tree=None):
        """Read every readable item found by discover(), as a nested dict
        mirroring the object tree. Calls discover() first if needed."""
        if tree is None:
            tree = self._tree if self._tree is not None else self.discover()
        values = {}
        for name, meta in tree.items():
            if meta["type"] == "group":
                values[name] = self.read_all(meta["children"])
            elif meta["type"] == "executable":
                continue
            else:
                try:
                    values[name] = self.read(meta["path"])
                except ThingSetError as exc:
                    values[name] = f"<error: {exc}>"
        return values

    def write_values(self, values):
        """Write `values` to the device's writable ("w"/"s") items.

        `values` may be a nested dict mirroring the discovered tree
        (e.g. {"Config": {"wBlinkPeriod_s": 0.2}}) or flat path keys
        (e.g. {"Config/wBlinkPeriod_s": 0.2}). Requires discover() to
        have run first, so items can be checked against their declared
        access type before anything is sent.

        Returns a dict of {path: "ok" or "error: ..."}.
        """
        if self._tree is None:
            self.discover()

        flat = self._flatten(values)
        results = {}
        by_group = {}
        for path, value in flat.items():
            meta = self._lookup(path)
            if meta is None:
                results[path] = "error: unknown ThingSet path"
                continue
            if meta["type"] not in _WRITABLE_TYPES:
                results[path] = f"error: {path} is {meta['type']}, not writable"
                continue
            parent, _, leaf = path.rpartition("/")
            by_group.setdefault(parent, {})[leaf] = value

        for parent, leaves in by_group.items():
            try:
                self.write(parent, leaves)
                for leaf in leaves:
                    results[f"{parent}/{leaf}" if parent else leaf] = "ok"
            except ThingSetError as exc:
                for leaf in leaves:
                    results[f"{parent}/{leaf}" if parent else leaf] = f"error: {exc}"
        return results

    @staticmethod
    def _flatten(values, prefix=""):
        flat = {}
        for key, value in values.items():
            path = f"{prefix}/{key}" if prefix else key
            if isinstance(value, dict):
                flat.update(ThingSetTools._flatten(value, path))
            else:
                flat[path] = value
        return flat

    def _lookup(self, path):
        tree = self._tree
        node = None
        for part in path.split("/"):
            if tree is None:
                return None
            node = tree.get(part)
            if node is None:
                return None
            tree = node.get("children")
        return node


class _ThingSetNode:
    """Dynamic attribute view over a discovered ThingSet subtree.

    Reading an attribute performs a GET; assigning to one performs an
    UPDATE. `dir()` lists real child names, which is what interactive
    shells (IPython/Jupyter) use for tab-completion.
    """

    def __init__(self, client, path):
        object.__setattr__(self, "_client", client)
        object.__setattr__(self, "_path", path)
        object.__setattr__(self, "_children", {})

    def _populate(self, tree):
        for name, meta in tree.items():
            if meta["type"] == "group":
                node = _ThingSetNode(self._client, meta["path"])
                node._populate(meta["children"])
                self._children[name] = node
            else:
                self._children[name] = meta["type"]

    def __getattr__(self, name):
        try:
            child = self._children[name]
        except KeyError:
            raise AttributeError(
                f"no ThingSet object {name!r} under '{self._path or '/'}'"
            ) from None
        if isinstance(child, _ThingSetNode):
            return child
        path = f"{self._path}/{name}" if self._path else name
        return self._client.read(path)

    def __setattr__(self, name, value):
        child = self._children.get(name)
        if child is None:
            raise AttributeError(
                f"no ThingSet object {name!r} under '{self._path or '/'}'"
            )
        if isinstance(child, _ThingSetNode) or child not in _WRITABLE_TYPES:
            raise AttributeError(f"{name!r} is not a writable ThingSet item")
        path = f"{self._path}/{name}" if self._path else name
        parent, _, leaf = path.rpartition("/")
        self._client.write(parent, {leaf: value})

    def __dir__(self):
        return sorted(self._children)

    def __repr__(self):
        return f"<ThingSetNode {self._path or '/'} ({len(self._children)} items)>"


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port, e.g. /dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--json", default="thingset_objects.json", help="discovery output file"
    )
    args = parser.parse_args()

    ts = ThingSetTools(args.port, baudrate=args.baud)
    print(f"Connected to {ts.port}")

    tree = ts.discover(json_path=args.json)
    print(f"Discovered {len(tree)} top-level objects, saved to {args.json}")

    print(json.dumps(ts.read_all(), indent=2))
