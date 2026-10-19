#!/usr/bin/env bash
# serial_log.sh - open the Nucleo serial log (ST-LINK VCP) and save it to a new file
#
#   pc/serial_log.sh            -> ~/nucleo_MMDD_HHMM.txt
#   pc/serial_log.sh udp_test   -> ~/nucleo_udp_test_MMDD_HHMM.txt
#
# Ends any old screen session first (a detached one keeps /dev/ttyACM0 busy and the
# new screen exits with "[screen is terminating]"). Quit with Ctrl-A k y.
set -euo pipefail

PORT=${PORT:-/dev/ttyACM0}
NAME=${1:+${1}_}
FILE="$HOME/nucleo_${NAME}$(date +%m%d_%H%M).txt"

# (screen -ls / grep return non-zero when there is no session: must not stop the script)
{ screen -ls 2>/dev/null | grep -o '[0-9]*\.pts[^[:space:]]*' | xargs -r -I{} screen -S {} -X quit; } || true

if [ ! -e "$PORT" ]; then
    echo "$PORT not found - is the Nucleo plugged in?" >&2
    exit 1
fi

echo "Logging $PORT to $FILE  (quit: Ctrl-A k y)"
sleep 1
exec screen -L -Logfile "$FILE" "$PORT" 115200
