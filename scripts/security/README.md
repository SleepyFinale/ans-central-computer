# Security Scripts

Security experiment scripts and related test data.

Layout:

- `motion_patterns/`: robot motion generators used to drive watermark tests.
- `watermark_pipeline/`: watermark and passthrough processors for `/cmd_vel_raw`.
- `test_data/battery_csv_logs/`: preserved CSV datasets from prior battery/watermark runs.

Notes:

- The CSV files are intentionally retained and should not be deleted.
- Most scripts default to `/inky/...` topics; update topic names for other robots.
