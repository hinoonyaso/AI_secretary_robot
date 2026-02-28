#!/bin/bash
# Tail logs for specific service or all

SERVICE="${1:-}"

cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [ -n "$SERVICE" ]; then
    docker compose logs -f --tail=100 "$SERVICE"
else
    docker compose logs -f --tail=100
fi
