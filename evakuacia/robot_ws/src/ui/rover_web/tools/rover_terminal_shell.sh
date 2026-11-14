#!/usr/bin/env bash
set -eo pipefail

workspace="${1:-${ROVER_WORKSPACE:-$HOME/sverk_rover}}"
workspace="$(cd "$(dirname "$workspace")" 2>/dev/null && pwd)/$(basename "$workspace")"

if [ ! -d "$workspace" ]; then
  workspace="$HOME"
fi

tmp_rc="$(mktemp /tmp/rover-terminal-rc.XXXXXX)"
cleanup() {
  rm -f "$tmp_rc"
}
trap cleanup EXIT

shell_bin="/bin/bash"
shell_name="bash"
shell_args=(--noprofile --rcfile "$tmp_rc" -i)

cat >"$tmp_rc" <<EOF
export TERM="\${TERM:-xterm-256color}"
export COLORTERM="\${COLORTERM:-truecolor}"
export ROVER_WORKSPACE="$workspace"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

if [ -f "$workspace/install/setup.bash" ]; then
  source "$workspace/install/setup.bash"
fi

cd "$workspace" 2>/dev/null || cd "\$HOME"

clear
echo "Rover terminal ready"
echo "Workspace: \$PWD"
echo "Shell: $shell_name"
echo

PS1='[rover] \u@\h:\w\$ '
EOF

exec "$shell_bin" "${shell_args[@]}"
