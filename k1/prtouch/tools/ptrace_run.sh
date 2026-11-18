#!/bin/sh
# usage: ptrace_run.sh <name> <gcode...>  -- run gcode via Moonraker, wait for idle, save trace
L=/usr/data/printer_data/logs; N=$1; shift
: > $L/prtouch_trace.log
python3 /usr/data/ptrace_post.py "$*" > /tmp/ptr.out 2>&1 &
CP=$!
sleep 3
while kill -0 $CP 2>/dev/null || curl "http://127.0.0.1:7125/printer/objects/query?idle_timeout=state" 2>/dev/null | grep -q '"Printing"'; do sleep 3; done
sleep 2
cp $L/prtouch_trace.log $L/trace_$N.log; : > $L/prtouch_trace.log
echo "== $N: $(wc -l < $L/trace_$N.log) lines; result: $(cat /tmp/ptr.out)"
grep " EXC " $L/trace_$N.log | cut -c1-300 | head -5
