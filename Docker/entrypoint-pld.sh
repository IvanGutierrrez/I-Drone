#!/bin/bash
set -e

# Copy SSH keys and fix permissions
if [ -d "/tmp/.ssh" ]; then
    mkdir -p /root/.ssh
    cp -r /tmp/.ssh/* /root/.ssh/
    chmod 700 /root/.ssh
    chmod 600 /root/.ssh/id_* 2>/dev/null || true
    chmod 644 /root/.ssh/*.pub 2>/dev/null || true
    chown -R root:root /root/.ssh
fi

# Execute the command
exec "$@"
