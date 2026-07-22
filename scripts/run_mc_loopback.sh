#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <controller> [controller args...]"
  echo "Example from mc-build: sudo ../scripts/run_mc_loopback.sh ./user/MIT_Controller/mit_ctrl"
  echo "If no controller args are given, this script runs: <controller> m r f"
  exit 1
fi

run_as_root() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    sudo "$@"
  fi
}

controller="$1"
shift

if [ "$#" -eq 0 ]; then
  set -- m r f
fi

# When WiFi/Ethernet is down, LCM still needs a multicast route.  Route
# multicast over loopback so local LCM publishers/subscribers can initialize.
if command -v ip >/dev/null 2>&1; then
  run_as_root ip link set dev lo up
  run_as_root ip link set dev lo multicast on
  run_as_root ip route replace 224.0.0.0/4 dev lo
else
  run_as_root ifconfig lo multicast
  if ! route -n | awk '$1 == "224.0.0.0" && $8 == "lo" { found = 1 } END { exit !found }'; then
    run_as_root route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
  fi
fi

# Keep the library setup style used by run_mc.sh, then start the controller.
run_as_root env LD_LIBRARY_PATH=. ldconfig
run_as_root env LD_LIBRARY_PATH=. "$controller" "$@"
