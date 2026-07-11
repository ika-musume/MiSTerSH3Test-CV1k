#!/bin/bash
# Build the stress program, run it on the real HS3 core in Verilator,
# capture the golden signatures, rebuild with goldens baked in, and confirm
# every stream then passes (RESULT bitmap == 0).
#
#   ./run_cpu.sh              # full flow (capture goldens if missing)
#   ./run_cpu.sh --recapture  # force re-capture (after changing the program)
set -e
cd "$(dirname "$0")"

HS3="../HS3"
RTL="../rtl"

SRCS="$HS3/cpu_core/cpu_bus_if.sv $HS3/cpu_core/cache_pkg.sv $HS3/cpu_core/int_pipe_pkg.sv \
      $HS3/peri/peri_bus_if.sv \
      $HS3/cpu_core/cache_mem.sv $HS3/cpu_core/cache.sv $HS3/cpu_core/agu.sv \
      $HS3/cpu_core/int_pipe_mem.sv $HS3/cpu_core/int_pipe.sv $HS3/cpu_core/ctrl_reg.sv \
      $HS3/cpu_core/exc_handler.sv $HS3/cpu_core/cpu_core.sv \
      $HS3/peri/ibus_arb.sv $HS3/peri/ibus_splitter.sv $HS3/peri/ibus_bridge.sv \
      $HS3/peri/bsc.sv $HS3/peri/cpg_wdt.sv $HS3/peri/dmac_channel.sv $HS3/peri/dmac.sv \
      $HS3/peri/intc.sv $HS3/peri/ioport.sv $HS3/peri/rtc.sv $HS3/peri/tmu.sv \
      $HS3/HS3.sv"
SW="../sw"

if [ "${1:-}" = "--recapture" ]; then
    rm -f "$SW/goldens.json"
fi

echo "== assemble program =="
python3 "$SW/make_prog.py"

echo "== verilate =="
verilator --binary --timing -j 0 -O2 \
   -Wno-fatal \
   $SRCS \
   "$RTL/sh3test_cpu.sv" tb_cpu.sv \
   --top-module tb_cpu -o tb_cpu --Mdir obj_cpu

run_and_check() {
    ./obj_cpu/tb_cpu "$@" | tee run_cpu.log
    grep -q "^DONE" run_cpu.log || { echo "FAIL: no DONE"; exit 1; }
}

if [ ! -f "$SW/goldens.json" ]; then
    echo "== capture goldens (run 1, zero goldens) =="
    run_and_check +iters=2
    python3 - "$SW/goldens.json" <<'EOF'
import json, re, sys
sigs = {}
for line in open("run_cpu.log"):
    m = re.match(r"SIG\[(\d)\]=([0-9A-Fa-f]{8})", line)
    if m:
        sigs[int(m.group(1))] = int(m.group(2), 16)   # last write wins
assert sorted(sigs) == [0, 1, 2, 3, 4], f"missing SIGs: {sigs}"
json.dump({"sig": [sigs[i] for i in range(5)]}, open(sys.argv[1], "w"))
print("goldens:", " ".join(f"{sigs[i]:08X}" for i in range(5)))
EOF
    echo "== rebuild with goldens =="
    python3 "$SW/make_prog.py"
fi

echo "== verify run (goldens baked in) =="
run_and_check +iters=4
if grep -q "RESULT map=00" run_cpu.log \
   && ! grep -q "bad_results=[1-9]" run_cpu.log \
   && grep -q "seqerrs=0 tbseq=0" run_cpu.log; then
    echo "PASS: all streams match goldens, sequence clean, kicks flowing"
else
    echo "FAIL: bad RESULT or sequence violation with baked goldens"
    exit 1
fi
