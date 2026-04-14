#!/usr/bin/env python3
"""
Central explorer structured event logger (copy-ready utility).

Intended target: ans-central-computer/scripts/multi_robot_explorer.py
Usage:
  from central_explorer_event_logger import ExplorerEventLogger
  logger = ExplorerEventLogger()
  logger.log("goal_selected", robot="pinky", goal={"x": 1.2, "y": 0.4})
"""

from __future__ import annotations

import json
import os
import socket
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional
from uuid import uuid4


def _iso_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class ExplorerEventLogger:
    """
    JSONL event logger for central multi-robot explorer decisions.
    """

    def __init__(
        self,
        output_dir: Optional[str] = None,
        session_id: Optional[str] = None,
        filename: Optional[str] = None,
    ) -> None:
        out_dir = output_dir or os.environ.get(
            "CENTRAL_DEBUG_LOG_DIR", "~/.ros/nav2_debug_central"
        )
        self.output_dir = Path(os.path.expanduser(out_dir))
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.session_id = (
            session_id
            or os.environ.get("DEBUG_SESSION_ID")
            or datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
        )
        host = socket.gethostname()
        default_name = f"session-{self.session_id}-{host}-{uuid4().hex[:8]}.jsonl"
        self.path = self.output_dir / (filename or default_name)
        self._fh = self.path.open("a", encoding="utf-8")

        self.log(
            "session_start",
            logger_path=str(self.path),
            pid=os.getpid(),
            host=host,
        )

    def log(
        self,
        event: str,
        *,
        robot: Optional[str] = None,
        t_ros_ns: Optional[int] = None,
        **payload: Any,
    ) -> None:
        line: Dict[str, Any] = {
            "session_id": self.session_id,
            "event": event,
            "t_wall": _iso_now(),
            "t_ros_ns": t_ros_ns,
        }
        if robot is not None:
            line["robot"] = robot
        if payload:
            line.update(payload)
        self._fh.write(json.dumps(line, separators=(",", ":")) + "\n")
        self._fh.flush()

    def log_frontiers_detected(
        self,
        *,
        robot: Optional[str],
        count: int,
        map_topic: str,
        map_meta: Dict[str, Any],
        merge_state: Optional[str],
        top_candidates: Optional[list] = None,
        t_ros_ns: Optional[int] = None,
    ) -> None:
        self.log(
            "frontiers_detected",
            robot=robot,
            t_ros_ns=t_ros_ns,
            count=count,
            map_topic=map_topic,
            map=map_meta,
            merge_state=merge_state,
            top_candidates=top_candidates or [],
        )

    def log_goal_selected(
        self,
        *,
        robot: str,
        goal_x: float,
        goal_y: float,
        frame: str,
        map_topic: str,
        map_meta: Dict[str, Any],
        merge_state: Optional[str],
        score: Dict[str, Any],
        reason: str,
        robot_pose_used: Optional[Dict[str, float]] = None,
        t_ros_ns: Optional[int] = None,
    ) -> None:
        self.log(
            "goal_selected",
            robot=robot,
            t_ros_ns=t_ros_ns,
            goal={"x": goal_x, "y": goal_y, "frame": frame},
            map_topic=map_topic,
            map=map_meta,
            merge_state=merge_state,
            scoring=score,
            reason=reason,
            robot_pose_used=robot_pose_used,
        )

    def log_goal_sent(
        self,
        *,
        robot: str,
        goal_x: float,
        goal_y: float,
        frame: str,
        action_name: str,
        t_ros_ns: Optional[int] = None,
    ) -> None:
        self.log(
            "goal_sent",
            robot=robot,
            t_ros_ns=t_ros_ns,
            goal={"x": goal_x, "y": goal_y, "frame": frame},
            action=action_name,
        )

    def log_goal_result(
        self,
        *,
        robot: str,
        status_code: int,
        status_text: str,
        result: Optional[Dict[str, Any]] = None,
        t_ros_ns: Optional[int] = None,
    ) -> None:
        self.log(
            "goal_result",
            robot=robot,
            t_ros_ns=t_ros_ns,
            status_code=status_code,
            status_text=status_text,
            result=result or {},
        )

    def log_goal_cancelled(
        self,
        *,
        robot: str,
        reason: str,
        old_goal: Optional[Dict[str, float]] = None,
        new_goal: Optional[Dict[str, float]] = None,
        t_ros_ns: Optional[int] = None,
    ) -> None:
        self.log(
            "goal_cancelled",
            robot=robot,
            t_ros_ns=t_ros_ns,
            reason=reason,
            old_goal=old_goal,
            new_goal=new_goal,
        )

    def log_retarget_decision(
        self,
        *,
        robot: str,
        reason: str,
        old_goal: Dict[str, float],
        new_goal: Dict[str, float],
        utility_delta: float,
        t_ros_ns: Optional[int] = None,
    ) -> None:
        self.log(
            "retarget_decision",
            robot=robot,
            t_ros_ns=t_ros_ns,
            reason=reason,
            old_goal=old_goal,
            new_goal=new_goal,
            utility_delta=utility_delta,
        )

    def log_blacklist_event(
        self,
        *,
        robot: str,
        action: str,  # "add" | "clear"
        point: Dict[str, float],
        reason: str,
        t_ros_ns: Optional[int] = None,
    ) -> None:
        self.log(
            f"blacklist_{action}",
            robot=robot,
            t_ros_ns=t_ros_ns,
            point=point,
            reason=reason,
        )

    def log_state_transition(
        self,
        *,
        old_state: str,
        new_state: str,
        reason: str,
        robot: Optional[str] = None,
        t_ros_ns: Optional[int] = None,
    ) -> None:
        self.log(
            "state_transition",
            robot=robot,
            t_ros_ns=t_ros_ns,
            old_state=old_state,
            new_state=new_state,
            reason=reason,
        )

    def close(self) -> None:
        try:
            self.log("session_end")
            self._fh.close()
        except Exception:
            pass


if __name__ == "__main__":
    logger = ExplorerEventLogger()
    logger.log("example", note="central logger utility smoke test")
    logger.close()
    print(logger.path)
