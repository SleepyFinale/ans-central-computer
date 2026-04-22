#!/usr/bin/env python3
"""
Generate per-robot ROS 2 domain_bridge configs from fleet contracts.

This utility compiles:
  - config/fleet_domain_map.yaml
  - config/fleet_bridge_contract.yaml
into one domain_bridge YAML file per robot namespace.
"""

from __future__ import annotations

import argparse
import importlib
from pathlib import Path
from typing import Any

try:
    yaml = importlib.import_module("yaml")
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing dependency: PyYAML. Install with `sudo apt install python3-yaml` "
        "or `pip install pyyaml`."
    ) from exc


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise SystemExit(f"Missing required file: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise SystemExit(f"Expected mapping at YAML root: {path}")
    return data


def _resolve_topic_specs(
    contract: dict[str, Any], key: str
) -> list[dict[str, Any]]:
    root = contract.get("fleet_bridge_contract", {})
    items = root.get(key, [])
    if not isinstance(items, list):
        raise SystemExit(f"Expected list for fleet_bridge_contract.{key}")
    out: list[dict[str, Any]] = []
    for item in items:
        if not isinstance(item, dict):
            continue
        topic = item.get("topic")
        msg_type = item.get("msg_type")
        if not isinstance(topic, str):
            continue
        if not isinstance(msg_type, str):
            # Action pseudo-types in the contract are documented as phrases.
            # Bridge generator handles NavigateToPose action separately.
            continue
        out.append({"topic": topic, "msg_type": msg_type})
    return out


def _build_bridge_doc(
    robot: str,
    robot_domain: int,
    central_domain: int,
    robot_to_central: list[dict[str, Any]],
    central_to_robot: list[dict[str, Any]],
) -> dict[str, Any]:
    topics: dict[str, Any] = {}

    # Explicit topic contract.
    for spec in robot_to_central + central_to_robot:
        raw_topic = spec["topic"].replace("{robot}", robot)
        if (
            "/_action/" in raw_topic
            or raw_topic.endswith("/navigate_to_pose")
            or raw_topic.endswith("/compute_path_to_pose")
        ):
            continue
        topics[raw_topic] = {
            "type": spec["msg_type"],
            "from_domain": robot_domain if spec in robot_to_central else central_domain,
            "to_domain": central_domain if spec in robot_to_central else robot_domain,
        }

    # Required for central local-map subscriptions in local-first mode
    # (multi_robot_explorer local_map_topic_pattern="/{robot}/map").
    topics[f"/{robot}/map"] = {
        "type": "nav_msgs/msg/OccupancyGrid",
        "from_domain": robot_domain,
        "to_domain": central_domain,
    }

    # Required for TF relay in central startup when using namespaced TF trees.
    topics[f"/{robot}/tf"] = {
        "type": "tf2_msgs/msg/TFMessage",
        "from_domain": robot_domain,
        "to_domain": central_domain,
    }
    topics[f"/{robot}/tf_static"] = {
        "type": "tf2_msgs/msg/TFMessage",
        "from_domain": robot_domain,
        "to_domain": central_domain,
    }

    # Hidden action topics for Nav2 actions used from the central explorer.
    # Services for these actions are relayed by fleet_navigate_to_pose_service_relay.py.
    for action_stem, feedback_msg, result_msg in (
        (
            "navigate_to_pose",
            "nav2_msgs/action/NavigateToPose_FeedbackMessage",
            "nav2_msgs/action/NavigateToPose_Result",
        ),
        (
            "compute_path_to_pose",
            "nav2_msgs/action/ComputePathToPose_FeedbackMessage",
            "nav2_msgs/action/ComputePathToPose_Result",
        ),
    ):
        action_prefix = f"/{robot}/{action_stem}"
        for suffix, msg_type in (
            ("/_action/status", "action_msgs/msg/GoalStatusArray"),
            ("/_action/feedback", feedback_msg),
            ("/_action/result", result_msg),
        ):
            topics[f"{action_prefix}{suffix}"] = {
                "type": msg_type,
                "from_domain": robot_domain,
                "to_domain": central_domain,
            }

    # Action *services* (send_goal / get_result / cancel_goal) are not
    # configurable through domain_bridge YAML on Humble — the executable only
    # loads topic bridges. Service forwarding is handled by
    # scripts/fleet_navigate_to_pose_service_relay.py alongside start_central.sh.

    return {
        "name": f"{robot}_bridge",
        "topics": topics,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate per-robot domain_bridge YAML configs."
    )
    parser.add_argument(
        "--domain-map",
        type=Path,
        default=Path("config/fleet_domain_map.yaml"),
        help="Path to fleet_domain_map.yaml",
    )
    parser.add_argument(
        "--bridge-contract",
        type=Path,
        default=Path("config/fleet_bridge_contract.yaml"),
        help="Path to fleet_bridge_contract.yaml",
    )
    parser.add_argument(
        "--robots",
        nargs="+",
        required=True,
        help="Robot namespaces to generate configs for",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("config/generated_domain_bridge"),
        help="Directory for generated YAML files",
    )
    parser.add_argument(
        "--central-domain",
        type=int,
        default=None,
        help=(
            "Central ROS domain ID for bridge to_domain/from_domain endpoints. "
            "When omitted, uses fleet_domain_map.central_domain_id."
        ),
    )
    args = parser.parse_args()

    domain_map = _load_yaml(args.domain_map).get("fleet_domain_map", {})
    robot_domain_ids = domain_map.get("robot_domain_ids", {})
    if args.central_domain is not None:
        central_domain = args.central_domain
    else:
        central_domain = domain_map.get("central_domain_id")
    if not isinstance(central_domain, int):
        raise SystemExit(
            "Central domain must be an integer: pass --central-domain or set "
            "fleet_domain_map.central_domain_id in the domain map."
        )
    if not isinstance(robot_domain_ids, dict):
        raise SystemExit("fleet_domain_map.robot_domain_ids must be a mapping")

    contract = _load_yaml(args.bridge_contract)
    robot_to_central = _resolve_topic_specs(contract, "robot_to_central")
    central_to_robot = _resolve_topic_specs(contract, "central_to_robot")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for robot in args.robots:
        robot_domain = robot_domain_ids.get(robot)
        if not isinstance(robot_domain, int):
            raise SystemExit(
                f"Robot '{robot}' missing integer domain in {args.domain_map}"
            )
        doc = _build_bridge_doc(
            robot=robot,
            robot_domain=robot_domain,
            central_domain=central_domain,
            robot_to_central=robot_to_central,
            central_to_robot=central_to_robot,
        )
        out_path = args.output_dir / f"{robot}.domain_bridge.yaml"
        with out_path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, sort_keys=False)
        print(f"Generated {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
