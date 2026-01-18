#!/usr/bin/env bash
#!/usr/bin/env bash

# Simple reconnect script for a previously paired Xbox controller.
# Controller must be ON (not in pairing mode).

bluetoothctl <<EOF
power on
agent on
default-agent
connect 9C:AA:1B:98:3E:3F
EOF