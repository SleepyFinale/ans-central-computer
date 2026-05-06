# Motion Patterns

Motion pattern generators used by the security watermark experiments.

Scripts:

- `precise_u_pattern.py`: repeating U-pattern motion on `/inky/cmd_vel`.
- `babypark_cmd_vel_direct.py`: baby-park trajectory directly on `/inky/cmd_vel`.
- `babypark_cmd_vel_raw.py`: baby-park trajectory on `/inky/cmd_vel_raw` (for watermark pipeline input).

Usage:

1. Update topic names if your robot namespace is not `/inky`.
2. Run one motion script in a dedicated terminal, for example:
   `python3 scripts/security/motion_patterns/babypark_cmd_vel_raw.py`
