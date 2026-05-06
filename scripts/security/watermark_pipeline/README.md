# Watermark Pipeline

Watermark processing scripts for `/cmd_vel_raw` pipelines.

Scripts:

- `cmd_vel_watermark_realtime.py`: applies live tshark-driven delay + jitter and publishes to `/inky/cmd_vel`.
- `cmd_vel_passthrough_with_delay_topic.py`: bypass mode that still publishes `/inky/watermark_delay` telemetry.

Typical two-terminal flow:

1. Motion source: `python3 scripts/security/motion_patterns/babypark_cmd_vel_raw.py`
2. Watermark stage: `python3 scripts/security/watermark_pipeline/cmd_vel_watermark_realtime.py`
