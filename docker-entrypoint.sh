#!/bin/bash
set -e

# Initialize database if not exists
DB_FILE=/opt/rover/db/rover.db

if [ ! -f "$DB_FILE" ]; then
    echo "Initializing database..."
    sqlite3 "$DB_FILE" < /opt/rover/db/init_schema.sql
    echo "Database initialized: $DB_FILE"
else
    echo "Database already exists: $DB_FILE"
fi

# Execute CMD
exec "$@"
