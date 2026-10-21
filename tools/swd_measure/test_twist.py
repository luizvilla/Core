"""Hardware test of old18 (Ownverter per-leg frequency app on a Twist)."""
import json, sys
sys.path.insert(0, ".")
from swd import Swd
from analyse import frequencies, analyse_capture, TIMERS

ELF, OUT = sys.argv[1], sys.argv[2]
SCR = "/tmp"
s = Swd(ELF)
log = {}
LEGS = {"A": "LEG1", "C": "LEG2", "E": "LEG3"}

def regs():
    r = s.request(3)
    out = {"MCR": hex(r["mcr"]), "CR2": hex(r["cr2"])}
    for i, n in enumerate(TIMERS):
        if r["mcr"] & (1 << (17 + i)):
            out[n] = {"PER": r[f"per{i}"], "CMP1": r[f"cmp1{i}"], "CMP3": r[f"cmp3{i}"],
                      "CKPSC": r[f"timcr{i}"] & 7, "UDM": (r[f"timcr2{i}"] >> 4) & 1,
                      "RST": hex(r[f"rst{i}"]), "SET1": hex(r[f"set1{i}"]),
                      "RST1": hex(r[f"rst1{i}"]), "DTEN": (r[f"outr{i}"] >> 8) & 1,
                      "DTR": hex(r[f"dtr{i}"]), "SWP": (r["cr2"] >> (16 + i)) & 1}
    return out

UDM = {}
def freq():
    f = frequencies(s.request(1), UDM)
    return {f"{k} ({LEGS.get(k, '-')})": {"f_hz": round(v["f_valleys"]), "cnt_max": v["cnt_max"]}
            for k, v in f.items()}

def capture(name):
    r = s.request(2)
    buf = s.dump(r["capt_addr"], r["capt_samples"], f"{SCR}/{name}.bin")
    c = analyse_capture(buf, r["capt_cycles"], r["cpu_hz"])
    return {(f"{k} ({LEGS[k]})" if k in LEGS else k): v for k, v in c.items() if k != "D"}

try:
    log["T1_registers"] = regs()
    UDM.update({n: v["UDM"] for n, v in log["T1_registers"].items() if isinstance(v, dict)})
    log["T2_frequencies"] = freq()
    log["T3_capture"] = capture("tw3")
    s.key("u"); log["T4a_common_210k"] = freq()
    s.key("j"); s.key("j"); log["T4b_common_190k"] = freq()
    s.key("u"); log["T4c_common_back_200k"] = freq()
    s.key("d"); log["T5a_leg2_110k"] = freq()
    log["T5a_capture"] = capture("tw5a")
    s.key("c"); log["T5b_leg2_back_100k"] = freq()
    s.key("v"); log["T5c_leg3_40k_refused"] = freq()
    s.key("f"); log["T5d_leg3_60k"] = freq()
    s.key("v"); log["T5e_leg3_back_50k"] = freq()
    rst_before = regs()["E"]["RST"]
    s.key("s")
    log["T6_phase_shift_leg3"] = {"RST_E_before": rst_before, "RST_E_after": regs()["E"]["RST"],
                                  "frequencies": freq()}
    s.key("o"); log["T7a_outputs_off_capture"] = capture("tw7a")
    s.key("o"); log["T7b_outputs_on_capture"] = capture("tw7b")
finally:
    s.close()
json.dump(log, open(OUT, "w"), indent=1, default=str)
