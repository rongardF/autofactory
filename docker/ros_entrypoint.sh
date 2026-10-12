#!/bin/bash
set -e

# Validate production settings even when the image is run outside Compose.
if [ "${APP_PROFILE:-}" = "prod" ]; then
    if [ -z "${WORKSTATION_ID:-}" ]; then
        echo "ERROR: WORKSTATION_ID must be set for prod profile" >&2
        exit 1
    fi
fi

. /usr/local/share/autofactory/setup.bash

exec "$@"
