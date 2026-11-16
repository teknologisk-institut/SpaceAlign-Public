#!/usr/bin/env bash
# SpaceAlign system orchestrator.
#
# Flow:
#   1. Opens Terminal 1 with Isaac Sim. You configure the sim manually,
#      then press Enter here to continue.
#   2. In a loop (until Ctrl+C here, or Isaac Sim exits/errors):
#        - Terminal 2: ros2 launch launch_configs robot_system.launch.py (from $HOME)
#        - 7 s delay
#        - Terminal 3: ros2 run spacealign_main lin_move_norminal       (from $HOME)
#        - When lin_move_norminal finishes, Terminal 2 is closed too.
#        - $HOME/test_linNorm.xml is moved into the newest folder in
#          $HOME/Test_SPA_test/lin_Nominal
#   3. Ctrl+C in THIS terminal closes all spawned terminals and exits.
#   4. Isaac Sim (Terminal 1) is monitored; if it exits or errors, all
#      terminals are stopped and the script exits.

set -u

ISAAC_SIM="$HOME/isaacsim/isaac-sim.sh"
ROS_WS="$HOME/SpaceAlign/SpaceAlign-ros_ws"
RESULT_XML="$HOME/test_linNorm.xml"
RESULT_DIR="$HOME/Test_SPA_test/lin_Nominal"
T2_T3_DELAY=10

RUNDIR=$(mktemp -d /tmp/spacealign_run.XXXXXX)

# ---------------------------------------------------------------- helpers ---

# Generate a wrapper script that the gnome-terminal runs. It records the PID
# of the (setsid'ed) process group and its exit code, and forwards SIGINT.
make_wrapper() {
    local name=$1 do_ros=$2 cmd=$3
    local ros_setup=""
    if [[ $do_ros == ros ]]; then
        ros_setup="export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
source \"$ROS_WS/install/local_setup.bash\""
    fi
    cat > "$RUNDIR/$name.sh" <<EOF
#!/usr/bin/env bash
cd "\$HOME"
$ros_setup
setsid $cmd &
pid=\$!
echo \$pid > "$RUNDIR/$name.pid"
trap 'kill -INT -- -\$pid 2>/dev/null' INT TERM
wait \$pid
code=\$?
echo \$code > "$RUNDIR/$name.exit"
# keep the window open on real errors (130 = stopped via Ctrl+C, that is fine)
if [[ \$code -ne 0 && \$code -ne 130 ]]; then
    echo "[$name] exited with code \$code - press Enter to close this window"
    read -r
fi
EOF
    chmod +x "$RUNDIR/$name.sh"
}

spawn_terminal() {  # <name> <title>
    local name=$1 title=$2
    rm -f "$RUNDIR/$name.pid" "$RUNDIR/$name.exit"
    gnome-terminal --title="$title" -- bash "$RUNDIR/$name.sh"
    # wait for the PID file to appear
    for _ in $(seq 1 60); do
        [[ -f "$RUNDIR/$name.pid" ]] && return 0
        sleep 0.5
    done
    echo "ERROR: terminal '$name' did not start." >&2
    return 1
}

term_done()    { [[ -f "$RUNDIR/$1.exit" ]]; }
term_pid()     { cat "$RUNDIR/$1.pid" 2>/dev/null; }
term_running() {
    local pid; pid=$(term_pid "$1") || return 1
    [[ -n $pid ]] && ! term_done "$1" && kill -0 "$pid" 2>/dev/null
}

stop_terminal() {  # graceful INT, then TERM after 10 s
    local name=$1 pid
    pid=$(term_pid "$name")
    [[ -n ${pid:-} ]] || return 0
    kill -INT -- "-$pid" 2>/dev/null
    for _ in $(seq 1 20); do
        term_running "$name" || return 0
        sleep 0.5
    done
    kill -TERM -- "-$pid" 2>/dev/null
}

cleanup() {
    trap - INT TERM EXIT
    echo
    echo ">>> Shutting down all terminals..."
    stop_terminal t3
    stop_terminal t2
    stop_terminal t1
    rm -rf "$RUNDIR"
    echo ">>> Done."
    exit "${1:-0}"
}
trap 'cleanup 130' INT TERM

isaac_check() {  # exits the whole script if Isaac Sim is gone
    if ! term_running t1; then
        local code
        code=$(cat "$RUNDIR/t1.exit" 2>/dev/null || echo "?")
        if [[ $code == "0" ]]; then
            echo ">>> Isaac Sim was closed. Stopping everything."
        else
            echo ">>> Isaac Sim exited with ERROR (code $code). Stopping everything." >&2
        fi
        cleanup 1
    fi
}

move_result_xml() {
    if [[ ! -f $RESULT_XML ]]; then
        echo ">>> WARNING: $RESULT_XML not found, nothing to move." >&2
        return
    fi
    local dest
    dest=$(find "$RESULT_DIR" -mindepth 1 -maxdepth 1 -type d -printf '%T@\t%p\n' 2>/dev/null \
           | sort -nr | head -1 | cut -f2-)
    if [[ -z $dest ]]; then
        echo ">>> WARNING: no folder found in $RESULT_DIR, leaving $RESULT_XML in place." >&2
        return
    fi
    mv "$RESULT_XML" "$dest/"
    echo ">>> Moved test_linNorm.xml -> $dest/"
}

# ------------------------------------------------------------------- main ---

[[ -x $ISAAC_SIM ]] || { echo "Error: $ISAAC_SIM not found/executable" >&2; exit 1; }

make_wrapper t1 plain "\"$ISAAC_SIM\""
make_wrapper t2 ros   "ros2 launch launch_configs robot_system.launch.py"
make_wrapper t3 ros   "ros2 run spacealign_main lin_move_norminal"

echo ">>> Starting Isaac Sim (Terminal 1)..."
spawn_terminal t1 "Isaac Sim" || cleanup 1

echo
echo ">>> Configure Isaac Sim now."
read -rp ">>> Press Enter when the sim is ready to start the test loop (Ctrl+C aborts everything)... "

cycle=1
while true; do
    isaac_check

    echo
    echo "=============== Test cycle $cycle ==============="
    echo ">>> Starting robot_system launch (Terminal 2)..."
    spawn_terminal t2 "robot_system launch" || cleanup 1

    echo ">>> Waiting ${T2_T3_DELAY}s before starting lin_move_norminal..."
    sleep "$T2_T3_DELAY"
    isaac_check

    echo ">>> Starting lin_move_norminal (Terminal 3)..."
    spawn_terminal t3 "lin_move_norminal" || cleanup 1

    # Wait for lin_move_norminal to finish while watching Isaac Sim
    while term_running t3; do
        isaac_check
        sleep 1
    done
    t3_code=$(cat "$RUNDIR/t3.exit" 2>/dev/null || echo "?")
    echo ">>> lin_move_norminal finished (exit code $t3_code)."
    [[ $t3_code != "0" ]] && echo ">>> WARNING: lin_move_norminal did not exit cleanly." >&2

    echo ">>> Closing robot_system launch (Terminal 2)..."
    stop_terminal t2

    move_result_xml

    isaac_check
    cycle=$((cycle + 1))
    echo ">>> Restarting in 2s (Ctrl+C to stop)..."
    sleep 2
done
