"""Hardware test of old17 (Spin app + measurement) through SWD."""
import json, sys
sys.path.insert(0, ".")
from swd import Swd, FIELDS
from analyse import frequencies, analyse_capture, TIMERS

ELF = sys.argv[1]
OUT = sys.argv[2]
s = Swd(ELF)
log = {}


def regs():
    r = s.request(3)
    out = {"MCR": hex(r["mcr"]), "CR2": hex(r["cr2"])}
    for i, n in enumerate(TIMERS):
        if r["mcr"] & (1 << (17 + i)):
            out[n] = {"PER": r[f"per{i}"], "CMP1": r[f"cmp1{i}"], "CMP3": r[f"cmp3{i}"],
                      "CKPSC": r[f"timcr{i}"] & 7, "UDM": (r[f"timcr2{i}"] >> 4) & 1,
                      "RST": hex(r[f"rst{i}"]), "SET1": hex(r[f"set1{i}"]),
                      "RST1": hex(r[f"rst1{i}"]), "DTEN": (r[f"outr{i}"] >> 8) & 1,
                      "DTR": hex(r[f"dtr{i}"]),
                      "SWP": (r["cr2"] >> (16 + i)) & 1}
    return out


def freq():
    f = frequencies(s.request(1))
    return {k: {"f_hz": round(v["f_valleys"]), "f_peaks_hz": round(v["f_peaks"]),
                "cnt_max": v["cnt_max"]} for k, v in f.items()}


def capture(name):
    r = s.request(2)
    buf = s.dump(r["capt_addr"], r["capt_samples"], f"/tmp/{name}.bin")
    return analyse_capture(buf, r["capt_cycles"], r["cpu_hz"])


try:
    log["T1_registers"] = regs()
    log["T2_frequencies"] = freq()
    log["T3_capture"] = capture("t3")

    s.key("u")
    log["T4a_common_210k"] = freq()
    s.key("j"); s.key("j")
    log["T4b_common_190k"] = freq()
    log["T4b_capture"] = capture("t4b")
    s.key("u")
    log["T4c_common_back_200k"] = freq()

    s.key("d")
    log["T5a_pwmd_60k"] = freq()
    log["T5a_capture"] = capture("t5a")
    s.key("c")
    log["T5b_pwmd_back_50k"] = freq()
    s.key("c")
    log["T5c_pwmd_40k_refused"] = freq()
    log["T5c_registers_D"] = regs()["D"]
    s.key("r")
    log["T5d_after_realign_capture"] = capture("t5d")

    s.key("o")
    log["T6a_outputs_off_capture"] = capture("t6a")
    s.key("o")
    log["T6b_outputs_on_capture"] = capture("t6b")
    log["T6b_frequencies"] = freq()
finally:
    s.close()

json.dump(log, open(OUT, "w"), indent=1, default=str)
print(json.dumps(log, indent=1, default=str))
