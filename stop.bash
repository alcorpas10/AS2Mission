#!/bin/bash
tmux_exception="$1"

if [[ $tmux_exception == "" ]]; then
  tmux kill-server
else
  tmux ls | awk '{print $1}' | grep -v "^$tmux_exception:" | xargs -I % sh -c 'tmux kill-session -t %'
fi
