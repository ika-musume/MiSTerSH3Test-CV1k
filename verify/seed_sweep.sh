#!/usr/bin/env bash
#
# seed_sweep.sh — compile SH3Test with N fitter seeds in parallel (one headless
# raetro/quartus:17.0 container each, in its own copy of the project) and keep
# only the build with the best CPU-clock setup slack.
#
#   verify/seed_sweep.sh                 # 6 seeds, full compile
#   verify/seed_sweep.sh --map-only      # 1 seed, analysis+synthesis only
#                                        #   (smoke test: elaboration, RAM init)
#   SEEDS="1 2 3 4 5 6" verify/seed_sweep.sh
#
# The winning build's output_files/ + reports are copied back into the project;
# the losing copies are deleted.
#

set -u -o pipefail

PROJ_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT="SH3Test"
IMAGE="raetro/quartus:17.0"

# CPU clock: the row to read out of the Setup Summary table in the STA report.
CPU_CLK='emu|pll|pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk'

MAP_ONLY=0
[ "${1:-}" = "--map-only" ] && MAP_ONLY=1

SEEDS="${SEEDS:-1 2 3 4 5 6}"
[ "$MAP_ONLY" = 1 ] && SEEDS="1"

# 6 concurrent Quartus processes; keep the sum of their worker threads under
# the core count so the fitters do not thrash each other.
NCPU="$(nproc)"
NSEEDS="$(wc -w <<<"$SEEDS")"
PER_JOB=$(( NCPU / (NSEEDS + 1) ))
[ "$PER_JOB" -lt 1 ] && PER_JOB=1
[ "$PER_JOB" -gt 8 ] && PER_JOB=8

SWEEP="${SWEEP_DIR:-$PROJ_ROOT/build_sweep}"
rm -rf "$SWEEP" && mkdir -p "$SWEEP"

echo "== sweep dir : $SWEEP"
echo "== seeds     : $SEEDS  (${PER_JOB} threads each, ${NCPU} cores)"

# The program image must have exactly one ASCII line per memory word.
HEX="$PROJ_ROOT/rtl/sh3test_prog.hex"
WORDS=$(grep -c '' "$HEX")
if [ "$WORDS" -ne 4096 ]; then
    echo "FATAL: $HEX has $WORDS lines, expected 4096 (mem[0:4095])" >&2
    exit 1
fi
echo "== prog hex  : $WORDS lines, ok"

##########################################################################
## launch one container per seed
##########################################################################

for s in $SEEDS; do
    dst="$SWEEP/seed$s"
    mkdir -p "$dst"
    tar -C "$PROJ_ROOT" \
        --exclude=./db --exclude=./incremental_db --exclude=./output_files \
        --exclude=./build_sweep --exclude=./verify/obj_dir --exclude=./.git \
        -cf - . | tar -C "$dst" -xf -

    sed -i "s/^set_global_assignment -name SEED .*/set_global_assignment -name SEED $s/" "$dst/$PROJECT.qsf"
    grep -q '^set_global_assignment -name SEED ' "$dst/$PROJECT.qsf" || \
        echo "set_global_assignment -name SEED $s" >> "$dst/$PROJECT.qsf"
    sed -i "s/^set_global_assignment -name NUM_PARALLEL_PROCESSORS .*/set_global_assignment -name NUM_PARALLEL_PROCESSORS $PER_JOB/" "$dst/$PROJECT.qsf"

    if [ "$MAP_ONLY" = 1 ]; then
        cmd="quartus_map $PROJECT --analysis_and_elaboration"
    else
        cmd="quartus_sh --flow compile $PROJECT && quartus_sta -t verify/sta_paths.tcl"
    fi

    # --entrypoint bash + `-c` (not `-lc`: a login shell re-sources /etc/profile
    # and drops the image's Quartus PATH).
    ( docker run --rm -u "$(id -u):$(id -g)" -e HOME=/host \
        -v "$dst":/host -w /host --entrypoint bash "$IMAGE" \
        -c "$cmd" >"$dst/build.log" 2>&1
      echo "$?" > "$dst/build.status" ) &
    echo "-- seed $s launched (pid $!)"
done

wait
echo "== all containers finished"

##########################################################################
## collect: worst CPU-clock setup slack per seed
##########################################################################

if [ "$MAP_ONLY" = 1 ]; then
    s=1
    st=$(cat "$SWEEP/seed$s/build.status")
    echo "== map-only status=$st"
    grep -iE "readmemh|Error |Critical Warning.*(RAM|memory|initial)" \
        "$SWEEP/seed$s/build.log" | head -40
    exit "$st"
fi

best_seed=""; best_slack=""
for s in $SEEDS; do
    dir="$SWEEP/seed$s"
    st=$(cat "$dir/build.status" 2>/dev/null || echo 99)

    # The program BRAM must land in M10K, not 131072 flops (Quartus silently
    # falls back to registers if the byte-enable write style regresses).
    grep -q 'Inferred altsyncram.*u_cpu|mem_rtl_0' "$dir/build.log" || \
        echo "seed $s: WARNING — program memory was NOT inferred as RAM"
    rpt="$dir/output_files/$PROJECT.sta.rpt"
    slack=""
    if [ "$st" = 0 ] && [ -f "$rpt" ]; then
        slack=$(awk -F';' -v clk="$CPU_CLK" '
            /^; Setup Summary/ { insum=1 }
            insum && index($2, clk) { gsub(/ /,"",$3); print $3; exit }
        ' "$rpt")
    fi
    if [ -z "$slack" ]; then
        echo "seed $s: FAILED (status=$st)"
        continue
    fi
    echo "seed $s: setup slack ${slack} ns"
    if [ -z "$best_slack" ] || awk "BEGIN{exit !($slack > $best_slack)}"; then
        best_slack="$slack"; best_seed="$s"
    fi
done

if [ -z "$best_seed" ]; then
    echo "FATAL: no seed produced a timing report; see $SWEEP/seed*/build.log" >&2
    exit 1
fi

echo "== best: seed $best_seed  slack ${best_slack} ns"

##########################################################################
## keep the winner, drop the rest
##########################################################################

win="$SWEEP/seed$best_seed"
rm -rf "$PROJ_ROOT/output_files"
cp -a "$win/output_files" "$PROJ_ROOT/output_files"
for f in sta_intra_pll.txt sta_to_pll2.txt build.log; do
    [ -f "$win/$f" ] && cp -a "$win/$f" "$PROJ_ROOT/$f"
done
sed -i "s/^set_global_assignment -name SEED .*/set_global_assignment -name SEED $best_seed/" "$PROJ_ROOT/$PROJECT.qsf"

{
    echo "seed sweep: OPTIMIZATION_MODE = AGGRESSIVE PERFORMANCE"
    echo "clock      : $CPU_CLK (constrained 9.727 ns = 102.8 MHz)"
    for s in $SEEDS; do
        rpt="$SWEEP/seed$s/output_files/$PROJECT.sta.rpt"
        sl=$(awk -F';' -v clk="$CPU_CLK" '/^; Setup Summary/{i=1} i && index($2,clk){gsub(/ /,"",$3);print $3;exit}' "$rpt" 2>/dev/null)
        printf "seed %-2s : %s\n" "$s" "${sl:-FAILED}"
    done
    echo "winner     : seed $best_seed (${best_slack} ns)"
} > "$PROJ_ROOT/seed_sweep.txt"

for s in $SEEDS; do
    [ "$s" = "$best_seed" ] || rm -rf "$SWEEP/seed$s"
done

echo "== kept $win ; summary in seed_sweep.txt"
