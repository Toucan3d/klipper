# Fleet KPI PostgreSQL Server On Raspberry Pi

This guide prepares a Raspberry Pi as the central database for Klipper fleet KPI
uploads. It assumes Raspberry Pi OS Bookworm or newer, a static LAN IP address,
and printers on the same trusted network.

## 1. Install Packages

```sh
sudo apt update
sudo apt install -y postgresql postgresql-contrib ufw
sudo systemctl enable --now postgresql
```

Record the Pi address. The examples below use `192.168.10.20`; replace it with
the real LAN address.

## 2. Create Database And Role

Generate a database password and store it in your password manager.

```sh
openssl rand -base64 32
```

Create the role and database. Replace `CHANGE_ME_STRONG_PASSWORD`.

```sh
sudo -u postgres psql
```

```sql
CREATE ROLE fleet_kpi_app
    LOGIN
    PASSWORD 'CHANGE_ME_STRONG_PASSWORD';

CREATE DATABASE fleet_kpi
    OWNER fleet_kpi_app
    TEMPLATE template0
    ENCODING 'UTF8';

\q
```

Install the schema from the Klipper checkout:

```sh
psql "postgresql://fleet_kpi_app:CHANGE_ME_STRONG_PASSWORD@localhost/fleet_kpi" \
    -f docs/fleet_kpi_schema.sql
```

## 3. Restrict Network Access

Edit PostgreSQL listen addresses:

```sh
sudo nano /etc/postgresql/*/main/postgresql.conf
```

Set:

```conf
listen_addresses = 'localhost,192.168.10.20'
```

Allow only the printer LAN in `pg_hba.conf`. Replace `192.168.10.0/24` with the
actual subnet.

```sh
sudo nano /etc/postgresql/*/main/pg_hba.conf
```

Append:

```conf
host    fleet_kpi    fleet_kpi_app    192.168.10.0/24    scram-sha-256
```

Restart PostgreSQL:

```sh
sudo systemctl restart postgresql
```

Configure the firewall for LAN-only database access:

```sh
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow from 192.168.10.0/24 to any port 5432 proto tcp
sudo ufw enable
sudo ufw status verbose
```

## 4. Backend Upload Service

The printer-side agent uploads to an HTTP service, not directly to PostgreSQL.
This repository includes a minimal service at
`scripts/fleet_metrics_backend.py`. Install its PostgreSQL driver:

```sh
sudo apt install -y python3-psycopg2
```

Create the backend environment file:

```sh
sudo install -d -m 0750 -o root -g root /etc/fleet-kpi
sudo nano /etc/fleet-kpi/backend.env
```

```sh
DATABASE_URL=postgresql://fleet_kpi_app:CHANGE_ME_STRONG_PASSWORD@localhost/fleet_kpi
FLEET_KPI_TOKEN=CHANGE_ME_UPLOAD_TOKEN
LISTEN_HOST=0.0.0.0
LISTEN_PORT=8080
```

Create `/etc/systemd/system/fleet-kpi-backend.service`:

```ini
[Unit]
Description=Fleet KPI Upload Backend
After=network-online.target postgresql.service
Wants=network-online.target

[Service]
Type=simple
EnvironmentFile=/etc/fleet-kpi/backend.env
WorkingDirectory=/home/pi/klipper
ExecStart=/usr/bin/python3 /home/pi/klipper/scripts/fleet_metrics_backend.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable it:

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now fleet-kpi-backend
sudo journalctl -u fleet-kpi-backend -f
```

The service exposes these endpoints:

```text
POST /api/v1/heartbeat
Authorization: Bearer <token>
Content-Type: application/json
```

The handler validates the token and runs:

```sql
SELECT ingest_fleet_heartbeat($1::jsonb);
```

```text
POST /api/v1/events/batch
Authorization: Bearer <token>
Content-Type: application/json
```

The request body is:

```json
{
  "printer_id": "printer-001",
  "events": []
}
```

The handler runs `ingest_fleet_event()` once for each entry in `events`.

Firewall rule for the upload API:

```sh
sudo ufw allow from 192.168.10.0/24 to any port 8080 proto tcp
```

## 5. Printer Agent Configuration

On each printer host, create `/etc/klipper/fleet_metrics_agent.conf`:

```ini
[fleet]
printer_id = printer-001
backend_url = http://192.168.10.20:8080
auth_token = CHANGE_ME_UPLOAD_TOKEN
klipper_socket = /tmp/klippy_uds
queue_db = /var/lib/klipper-fleet-agent/queue.sqlite3
heartbeat_interval_seconds = 60
```

The `printer_id` must be stable and unique for the fleet.

Example systemd unit for the printer host:

```ini
[Unit]
Description=Klipper Fleet KPI Agent
After=network-online.target klipper.service
Wants=network-online.target

[Service]
Type=simple
User=pi
Group=pi
ExecStart=/usr/bin/python3 /home/pi/klipper/scripts/fleet_metrics_agent.py \
  --config /etc/klipper/fleet_metrics_agent.conf
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Install it:

```sh
sudo install -d -m 0755 /var/lib/klipper-fleet-agent
sudo nano /etc/systemd/system/klipper-fleet-agent.service
sudo systemctl daemon-reload
sudo systemctl enable --now klipper-fleet-agent
sudo journalctl -u klipper-fleet-agent -f
```

## 6. Verify Connectivity

From a printer host:

```sh
curl -i \
  -H "Authorization: Bearer CHANGE_ME_UPLOAD_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"printer_id":"connectivity-test","observed_at":"2026-01-01T00:00:00Z"}' \
  http://192.168.10.20:8080/api/v1/heartbeat
```

From the Pi:

```sh
psql "postgresql://fleet_kpi_app:CHANGE_ME_STRONG_PASSWORD@localhost/fleet_kpi" \
  -c "SELECT printer_id, last_seen FROM printers ORDER BY last_seen DESC LIMIT 5;"
```

## 7. Grafana Queries

Install Grafana separately and add PostgreSQL database `fleet_kpi` as a data
source. Useful first panels:

```sql
SELECT * FROM fleet_printer_latest ORDER BY last_seen DESC;
```

```sql
SELECT * FROM fleet_print_kpis ORDER BY total_print_hours DESC;
```

```sql
SELECT * FROM fleet_version_distribution ORDER BY printer_count DESC;
```

## 8. Backups And Retention

Create a backup directory:

```sh
sudo install -d -m 0750 -o postgres -g postgres /var/backups/fleet-kpi
```

Add `/etc/cron.daily/fleet-kpi-backup`:

```sh
#!/bin/sh
set -eu
out="/var/backups/fleet-kpi/fleet_kpi_$(date +%Y%m%d).dump"
sudo -u postgres pg_dump -Fc fleet_kpi > "$out"
find /var/backups/fleet-kpi -type f -name 'fleet_kpi_*.dump' -mtime +30 -delete
```

Enable it:

```sh
sudo chmod 0755 /etc/cron.daily/fleet-kpi-backup
sudo /etc/cron.daily/fleet-kpi-backup
```

Restore check:

```sh
createdb fleet_kpi_restore_test
pg_restore -d fleet_kpi_restore_test /var/backups/fleet-kpi/fleet_kpi_YYYYMMDD.dump
dropdb fleet_kpi_restore_test
```

## 9. Token Rotation

1. Generate a new token with `openssl rand -base64 32`.
2. Configure the backend to accept both old and new tokens temporarily.
3. Update `/etc/klipper/fleet_metrics_agent.conf` on each printer.
4. Restart agents with `sudo systemctl restart klipper-fleet-agent`.
5. Remove the old token from the backend after all printers have checked in.

## 10. Operational Notes

- Keep PostgreSQL and the upload API LAN-only unless a VPN is in place.
- Use a high-endurance SD card or SSD for the Pi because heartbeat data writes
  regularly.
- Watch disk usage with `df -h` and database size with:

```sql
SELECT pg_size_pretty(pg_database_size('fleet_kpi'));
```

- If upload service downtime occurs, printer agents keep outbound events in
  local SQLite and retry later.
