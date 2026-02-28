#!/bin/bash
# JetRover System Shutdown Script

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

echo "Stopping JetRover system..."

docker compose down

echo "System stopped."
echo ""
echo "Database is preserved in: db/rover.db"
echo "Restart with: ./scripts/start_system.sh"
