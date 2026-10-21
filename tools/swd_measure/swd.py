"""SWD access to the measurement block `meas` (old17 / old18) through an
OpenOCD server (TCL RPC port), while the firmware runs."""
import os, socket, subprocess, time, re

OCD = os.path.expanduser("~/.platformio/packages/tool-openocd")
NM = os.path.expanduser("~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm")

FIELDS = ["magic", "request", "key", "done", "key_arg", "cpu_hz", "window_cycles"] + \
    [f"valleys{i}" for i in range(6)] + [f"peaks{i}" for i in range(6)] + \
    [f"cnt_max{i}" for i in range(6)] + ["capt_samples", "capt_cycles", "capt_addr",
    "mcr", "cr2"] + [f"{n}{i}" for n in ["per", "cmp1", "cmp3", "rst", "timcr", "timcr2",
    "outr", "dtr", "set1", "rst1"] for i in range(6)]


class Swd:
    def __init__(self, elf):
        out = subprocess.run([NM, elf], capture_output=True, text=True).stdout
        self.base = int(re.search(r"^([0-9a-f]+) [BbDd] meas$", out, re.M).group(1), 16)
        self.proc = subprocess.Popen(
            [OCD + "/bin/openocd", "-s", OCD + "/openocd/scripts",
             "-f", "interface/stlink.cfg", "-f", "target/stm32g4x.cfg",
             "-c", "gdb_port disabled", "-c", "telnet_port disabled",
             "-c", "tcl_port 6666", "-c", "init"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.s = None
        for _ in range(100):
            try:
                self.s = socket.create_connection(("127.0.0.1", 6666), timeout=30)
                break
            except OSError:
                time.sleep(0.2)
        if self.s is None:
            raise RuntimeError("openocd did not start")

    def cmd(self, c):
        self.s.sendall(c.encode() + b"\x1a")
        buf = b""
        while not buf.endswith(b"\x1a"):
            chunk = self.s.recv(65536)
            if not chunk:
                raise RuntimeError("openocd closed the connection")
            buf += chunk
        return buf[:-1].decode(errors="replace")

    def mdw(self, addr, n=1):
        out = self.cmd(f"capture {{mdw 0x{addr:08x} {n}}}")
        words = []
        for line in out.splitlines():
            m = re.match(r"0x[0-9a-f]+:\s+(.*)", line.strip())
            if m:
                words += [int(w, 16) for w in m.group(1).split()]
        return words[:n]

    def mww(self, addr, v):
        self.cmd(f"mww 0x{addr:08x} 0x{v:08x}")

    def field_addr(self, name):
        return self.base + 4 * FIELDS.index(name)

    def read(self):
        return dict(zip(FIELDS, self.mdw(self.base, len(FIELDS))))

    def _wait(self, done0, timeout=15.0):
        t = time.time()
        while time.time() - t < timeout:
            if self.mdw(self.field_addr("done"))[0] != done0:
                return
            time.sleep(0.05)
        raise TimeoutError("firmware did not answer")

    def request(self, code):
        d = self.mdw(self.field_addr("done"))[0]
        self.mww(self.field_addr("request"), code)
        self._wait(d)
        return self.read()

    def key(self, k):
        d = self.mdw(self.field_addr("done"))[0]
        self.mww(self.field_addr("key"), ord(k))
        self._wait(d)

    def dump(self, addr, size, path):
        self.cmd(f"dump_image {path} 0x{addr:08x} {size}")
        return open(path, "rb").read()

    def close(self):
        try:
            self.cmd("shutdown")
        except Exception:
            pass
        try:
            self.proc.wait(timeout=10)
        except Exception:
            self.proc.kill()
