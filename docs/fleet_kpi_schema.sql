CREATE TABLE IF NOT EXISTS printers (
    printer_id TEXT PRIMARY KEY,
    hostname TEXT,
    first_seen TIMESTAMPTZ NOT NULL DEFAULT now(),
    last_seen TIMESTAMPTZ NOT NULL DEFAULT now(),
    software_version TEXT,
    mcu_versions JSONB NOT NULL DEFAULT '{}'::jsonb,
    config_file TEXT,
    cpu_info TEXT
);

CREATE TABLE IF NOT EXISTS printer_heartbeats (
    id BIGSERIAL PRIMARY KEY,
    printer_id TEXT NOT NULL REFERENCES printers(printer_id) ON DELETE CASCADE,
    observed_at TIMESTAMPTZ NOT NULL,
    received_at TIMESTAMPTZ NOT NULL DEFAULT now(),
    state TEXT,
    state_message TEXT,
    software_version TEXT,
    hostname TEXT,
    system_stats JSONB NOT NULL DEFAULT '{}'::jsonb,
    mcu JSONB NOT NULL DEFAULT '{}'::jsonb,
    current_print JSONB NOT NULL DEFAULT '{}'::jsonb,
    queue_depth INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS print_jobs (
    job_id UUID PRIMARY KEY,
    printer_id TEXT NOT NULL REFERENCES printers(printer_id) ON DELETE CASCADE,
    filename TEXT NOT NULL DEFAULT '',
    started_at TIMESTAMPTZ,
    finished_at TIMESTAMPTZ,
    final_state TEXT,
    total_duration DOUBLE PRECISION NOT NULL DEFAULT 0,
    print_duration DOUBLE PRECISION NOT NULL DEFAULT 0,
    filament_used DOUBLE PRECISION NOT NULL DEFAULT 0,
    message TEXT NOT NULL DEFAULT '',
    total_layer INTEGER,
    current_layer INTEGER,
    updated_at TIMESTAMPTZ NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS printer_events (
    event_id UUID PRIMARY KEY,
    printer_id TEXT NOT NULL REFERENCES printers(printer_id) ON DELETE CASCADE,
    job_id UUID,
    kind TEXT NOT NULL,
    observed_at TIMESTAMPTZ NOT NULL,
    received_at TIMESTAMPTZ NOT NULL DEFAULT now(),
    payload JSONB NOT NULL
);

CREATE INDEX IF NOT EXISTS printer_heartbeats_printer_time_idx
    ON printer_heartbeats(printer_id, observed_at DESC);

CREATE INDEX IF NOT EXISTS printer_events_printer_time_idx
    ON printer_events(printer_id, observed_at DESC);

CREATE INDEX IF NOT EXISTS print_jobs_printer_started_idx
    ON print_jobs(printer_id, started_at DESC);

CREATE OR REPLACE VIEW fleet_printer_latest AS
SELECT DISTINCT ON (p.printer_id)
    p.printer_id,
    COALESCE(h.hostname, p.hostname) AS hostname,
    h.observed_at AS last_seen,
    h.state,
    h.state_message,
    COALESCE(h.software_version, p.software_version) AS software_version,
    h.mcu,
    h.current_print,
    h.queue_depth
FROM printers p
LEFT JOIN printer_heartbeats h ON h.printer_id = p.printer_id
ORDER BY p.printer_id, h.observed_at DESC NULLS LAST;

CREATE OR REPLACE VIEW fleet_print_kpis AS
SELECT
    p.printer_id,
    p.hostname,
    COALESCE(SUM(j.print_duration), 0) / 3600.0 AS total_print_hours,
    COUNT(j.job_id) FILTER (WHERE j.final_state = 'complete') AS completed_prints,
    COUNT(j.job_id) FILTER (WHERE j.final_state = 'error') AS errored_prints,
    COUNT(j.job_id) FILTER (WHERE j.final_state = 'cancelled') AS cancelled_prints,
    CASE
        WHEN COUNT(j.job_id) = 0 THEN 0
        ELSE COUNT(j.job_id) FILTER (WHERE j.final_state = 'error')::DOUBLE PRECISION
             / COUNT(j.job_id)
    END AS error_rate
FROM printers p
LEFT JOIN print_jobs j ON j.printer_id = p.printer_id
GROUP BY p.printer_id, p.hostname;

CREATE OR REPLACE VIEW fleet_version_distribution AS
SELECT
    software_version,
    COUNT(*) AS printer_count
FROM fleet_printer_latest
GROUP BY software_version;

CREATE OR REPLACE FUNCTION ingest_fleet_heartbeat(payload JSONB)
RETURNS VOID LANGUAGE plpgsql AS $$
DECLARE
    pid TEXT := payload->>'printer_id';
    observed TIMESTAMPTZ := COALESCE((payload->>'observed_at')::timestamptz, now());
BEGIN
    INSERT INTO printers (
        printer_id, hostname, last_seen, software_version, mcu_versions,
        config_file, cpu_info
    )
    VALUES (
        pid,
        payload->>'hostname',
        observed,
        payload->>'software_version',
        COALESCE(payload->'mcu', '{}'::jsonb),
        payload->>'config_file',
        payload->>'cpu_info'
    )
    ON CONFLICT (printer_id) DO UPDATE SET
        hostname = EXCLUDED.hostname,
        last_seen = EXCLUDED.last_seen,
        software_version = EXCLUDED.software_version,
        mcu_versions = EXCLUDED.mcu_versions,
        config_file = EXCLUDED.config_file,
        cpu_info = EXCLUDED.cpu_info;

    INSERT INTO printer_heartbeats (
        printer_id, observed_at, state, state_message, software_version,
        hostname, system_stats, mcu, current_print, queue_depth
    )
    VALUES (
        pid,
        observed,
        payload->>'state',
        payload->>'state_message',
        payload->>'software_version',
        payload->>'hostname',
        COALESCE(payload->'system_stats', '{}'::jsonb),
        COALESCE(payload->'mcu', '{}'::jsonb),
        COALESCE(payload->'current_print', '{}'::jsonb),
        COALESCE((payload->>'queue_depth')::integer, 0)
    );
END;
$$;

CREATE OR REPLACE FUNCTION ingest_fleet_event(payload JSONB)
RETURNS VOID LANGUAGE plpgsql AS $$
DECLARE
    event_kind TEXT := payload->>'kind';
    pid TEXT := payload->>'printer_id';
    jid UUID := NULLIF(payload->>'job_id', '')::uuid;
    observed TIMESTAMPTZ := COALESCE((payload->>'observed_at')::timestamptz, now());
BEGIN
    INSERT INTO printer_events (
        event_id, printer_id, job_id, kind, observed_at, payload
    )
    VALUES (
        (payload->>'event_id')::uuid,
        pid,
        jid,
        event_kind,
        observed,
        payload
    )
    ON CONFLICT (event_id) DO NOTHING;

    IF event_kind = 'print_started' THEN
        INSERT INTO print_jobs (
            job_id, printer_id, filename, started_at, final_state,
            total_duration, print_duration, filament_used, message,
            total_layer, current_layer
        )
        VALUES (
            jid,
            pid,
            COALESCE(payload->>'filename', ''),
            observed,
            payload->>'state',
            COALESCE((payload->>'total_duration')::double precision, 0),
            COALESCE((payload->>'print_duration')::double precision, 0),
            COALESCE((payload->>'filament_used')::double precision, 0),
            COALESCE(payload->>'message', ''),
            NULLIF(payload->>'total_layer', '')::integer,
            NULLIF(payload->>'current_layer', '')::integer
        )
        ON CONFLICT (job_id) DO UPDATE SET
            updated_at = now();
    ELSIF event_kind = 'print_finished' THEN
        INSERT INTO print_jobs (
            job_id, printer_id, filename, finished_at, final_state,
            total_duration, print_duration, filament_used, message,
            total_layer, current_layer
        )
        VALUES (
            jid,
            pid,
            COALESCE(payload->>'filename', ''),
            observed,
            payload->>'state',
            COALESCE((payload->>'total_duration')::double precision, 0),
            COALESCE((payload->>'print_duration')::double precision, 0),
            COALESCE((payload->>'filament_used')::double precision, 0),
            COALESCE(payload->>'message', ''),
            NULLIF(payload->>'total_layer', '')::integer,
            NULLIF(payload->>'current_layer', '')::integer
        )
        ON CONFLICT (job_id) DO UPDATE SET
            finished_at = EXCLUDED.finished_at,
            final_state = EXCLUDED.final_state,
            total_duration = EXCLUDED.total_duration,
            print_duration = EXCLUDED.print_duration,
            filament_used = EXCLUDED.filament_used,
            message = EXCLUDED.message,
            total_layer = EXCLUDED.total_layer,
            current_layer = EXCLUDED.current_layer,
            updated_at = now();
    END IF;
END;
$$;
