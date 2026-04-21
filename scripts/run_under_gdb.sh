#!/usr/bin/env bash
# Roslaunch: bash this_script /path/to/btsa_node __name:=... __log:=...
set -euo pipefail
for g in /usr/bin/gdb /bin/gdb; do
  if [ -x "$g" ]; then
    exec "$g" -q -ex run --args "$@"
  fi
done
if command -v gdb >/dev/null 2>&1; then
  exec gdb -q -ex run --args "$@"
fi
echo "run_under_gdb.sh: 未找到 gdb。请安装: sudo apt install gdb" >&2
exit 127
