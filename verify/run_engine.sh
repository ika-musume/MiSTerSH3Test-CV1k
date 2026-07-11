#!/bin/bash
# Full-engine simulation (SIM_SPEEDUP): clean 10-step schedule run, then the
# 102.8 MHz halt-rule (alarm) path using a zero-golden program image.
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
      $HS3/HS3.sv \
      $RTL/to_bcd.sv $RTL/test_overlay.sv $RTL/test_videoout.sv \
      $RTL/sh3test_cpu.sv $RTL/sh3test_mgr.sv $RTL/sh3test_cv1k.sv"
SW="../sw"

echo "== assemble program (goldens baked) + bad-golden variant =="
python3 "$SW/make_prog.py"
python3 "$SW/make_prog.py" --zero-goldens --out "$RTL/sh3test_prog_badgold.hex"

echo "== build engine tb (clean) =="
verilator --binary --timing -j 0 -O2 -Wno-fatal \
   -I"$RTL" $SRCS tb_engine.sv \
   --top-module tb_engine -o tb_engine --Mdir obj_engine

echo "== run: clean full schedule =="
./obj_engine/tb_engine | tee run_engine.log
grep -q "TB_PASS (clean full run)" run_engine.log || { echo "FAIL"; exit 1; }

echo "== build engine tb (bad goldens) =="
verilator --binary --timing -j 0 -O2 -Wno-fatal \
   -GTB_HEX='"../rtl/sh3test_prog_badgold.hex"' \
   -I"$RTL" $SRCS tb_engine.sv \
   --top-module tb_engine -o tb_engine_bad --Mdir obj_engine_bad

echo "== run: alarm path =="
./obj_engine_bad/tb_engine_bad +alarm | tee run_alarm.log
grep -q "TB_PASS (alarm path)" run_alarm.log || { echo "FAIL"; exit 1; }

echo "== build + run: watchdog unit test =="
verilator --binary --timing -j 0 -O2 -Wno-fatal \
   "$RTL/sh3test_mgr.sv" tb_wd.sv \
   --top-module tb_wd -o tb_wd --Mdir obj_wd
./obj_wd/tb_wd | tee run_wd.log
grep -q "TB_PASS (watchdog)" run_wd.log || { echo "FAIL"; exit 1; }

echo "ALL ENGINE TESTS PASS"
