#!/usr/bin/env python3
"""IndyDCP3 실물 로봇 상태 모니터.

ROS/MoveIt 명령을 보내는 동안 별도 터미널에 켜두고, 실물 로봇이 멈추거나
violation/collision/서보 꺼짐 상태로 빠지는 이유를 바로 확인한다.

실행 방법:
  # 패키지 빌드 후 권장 실행
  ros2 run indy7_task dcp3_monitor --robot-ip 166.104.214.96

  # 상태가 바뀔 때만 출력하고 로그도 남기기
  ros2 run indy7_task dcp3_monitor --robot-ip 166.104.214.96 --changes-only --log-file /tmp/indy_dcp3_monitor.log

  # 빌드 전 소스에서 바로 테스트
  python3 indy7_task/indy7_task/dcp3_monitor.py --robot-ip 166.104.214.96

  # 한 번만 찍어서 현재 상태 확인
  ros2 run indy7_task dcp3_monitor --robot-ip 166.104.214.96 --once
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    import grpc
    from google.protobuf import json_format
    from neuromeka import IndyDCP3
    from neuromeka.indydcp3 import common_msgs
except ImportError as exc:  # pragma: no cover - environment guard
    print(
        "neuromeka DCP3 의존성을 import하지 못했습니다: "
        f"{exc}\nneuromeka Python 패키지 설치 상태를 먼저 확인하세요.",
        file=sys.stderr,
    )
    raise SystemExit(2)


OP_STATE_NAMES = {
    0: "SYSTEM_OFF",
    1: "SYSTEM_ON",
    2: "VIOLATE",
    3: "RECOVER_HARD",
    4: "RECOVER_SOFT",
    5: "IDLE",
    6: "MOVING",
    7: "TEACHING",
    8: "COLLISION",
    9: "STOP_AND_OFF",
    10: "COMPLIANCE",
    11: "BRAKE_CONTROL",
    12: "SYSTEM_RESET",
    13: "SYSTEM_SWITCH",
    15: "VIOLATE_HARD",
    16: "MANUAL_RECOVER",
    17: "TELE_OP",
}

PROGRAM_STATE_NAMES = {
    0: "IDLE",
    1: "RUNNING",
    2: "PAUSING",
    3: "STOPPING",
}

STOP_CATEGORY_NAMES = {
    -1: "NONE",
    0: "CAT0_IMMEDIATE_BRAKE",
    1: "CAT1_SMOOTH_BRAKE",
    2: "CAT2_SMOOTH_ONLY",
}

TRAJ_STATE_NAMES = {
    0: "NONE",
    1: "INIT",
    2: "CALC",
    3: "STAND_BY",
    4: "ACC",
    5: "CRUISE",
    6: "DEC",
    7: "CANCELLING",
    8: "FINISHED",
    9: "ERROR",
}

DISPLAY_LEVEL_NAMES = {
    "OK": "정상",
    "WARN": "주의",
    "ALERT": "위험",
}

BAD_OP_STATES = {0, 2, 3, 4, 8, 9, 12, 15, 16}
COMMAND_BLOCKING_OP_STATES = {1, 7, 10, 11, 13, 17}
OK_OP_STATES = {5, 6}
BOOT_CRITICAL_FIELDS = (
    "main_pw_relay_on",
    "safety_pw_relay_on",
    "robot_pw_supply_on",
    "ethercat_connected",
    "control_on",
)
ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


@dataclass(frozen=True)
class RpcCall:
    name: str
    stub: str
    rpc: str


BASE_CALLS = (
    RpcCall("boot", "boot", "GetBootStatus"),
    RpcCall("robot", "rtde", "GetControlData"),
)

STATUS_CALLS = (
    RpcCall("violation", "rtde", "GetViolationData"),
    RpcCall("violation_queue", "rtde", "GetViolationMessageQueue"),
    RpcCall("servo", "rtde", "GetServoData"),
    RpcCall("motion", "rtde", "GetMotionData"),
    RpcCall("program", "rtde", "GetProgramData"),
    RpcCall("stop", "rtde", "GetStopState"),
    RpcCall("safety", "device", "GetSafetyControlData"),
    RpcCall("auto_mode", "device", "CheckAutoMode"),
    RpcCall("reduced_mode", "device", "CheckReducedMode"),
    RpcCall("safety_function", "device", "GetSafetyFunctionState"),
)

STATIC_CALLS = (
    RpcCall("control_info", "control", "GetControlInfo"),
    RpcCall("device_info", "device", "GetDeviceInfo"),
)

CONTROL_STATE_CALL = RpcCall("control_state", "rtde", "GetControlState")


def proto_to_dict(response: Any) -> Dict[str, Any]:
    """protobuf 응답을 Python dict로 바꾼다."""
    kwargs = {
        "preserving_proto_field_name": True,
        "use_integers_for_enums": True,
    }
    try:
        return json_format.MessageToDict(
            response,
            including_default_value_fields=True,
            **kwargs,
        )
    except TypeError:
        return json_format.MessageToDict(
            response,
            always_print_fields_with_no_presence=True,
            **kwargs,
        )


def state_label(value: Any, mapping: Dict[int, str]) -> str:
    """숫자 enum 값을 사람이 읽는 상태 문자열로 바꾼다."""
    value_int = as_int(value)
    if value_int is None:
        return "알수없음"
    name = mapping.get(value_int, "알수없음")
    return f"{name}({value_int})"


def as_int(value: Any) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def bool_label(value: Any) -> str:
    if value is None:
        return "-"
    return str(as_bool(value))


def short_text(value: Any, limit: int = 120) -> str:
    text = str(value).strip()
    if len(text) <= limit:
        return text
    return text[: limit - 3] + "..."


def short_list(values: Iterable[Any], digits: int = 1, limit: int = 6) -> str:
    result = []
    for index, value in enumerate(values):
        if index >= limit:
            result.append("...")
            break
        try:
            result.append(f"{float(value):.{digits}f}")
        except (TypeError, ValueError):
            result.append(str(value))
    return "[" + ", ".join(result) + "]"


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def format_rpc_error(exc: Exception) -> str:
    if isinstance(exc, grpc.RpcError):
        code = exc.code()
        code_name = code.name if code is not None else "RPC_ERROR"
        details = exc.details() or ""
        return f"{code_name}: {details}".strip()
    return f"{type(exc).__name__}: {exc}"


def call_rpc(indy: IndyDCP3, call: RpcCall, timeout: float) -> Dict[str, Any]:
    """DCP3 RPC를 timeout과 함께 호출해 모니터가 멈추지 않게 한다."""
    stub = getattr(indy, call.stub)
    rpc = getattr(stub, call.rpc)
    response = rpc(common_msgs.Empty(), timeout=timeout)
    return proto_to_dict(response)


def collect_snapshot(
    indy: IndyDCP3,
    timeout: float,
    include_control_state: bool = False,
) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, str]]:
    """한 주기의 로봇 상태를 모은다. 실패한 RPC는 errors에 따로 담는다."""
    data: Dict[str, Dict[str, Any]] = {}
    errors: Dict[str, str] = {}

    for call in BASE_CALLS:
        try:
            data[call.name] = call_rpc(indy, call, timeout)
        except Exception as exc:  # noqa: BLE001 - 진단은 계속 진행
            errors[call.name] = format_rpc_error(exc)

    if len(errors) == len(BASE_CALLS):
        return data, errors

    calls: List[RpcCall] = list(STATUS_CALLS)
    if include_control_state:
        calls.append(CONTROL_STATE_CALL)

    for call in calls:
        try:
            data[call.name] = call_rpc(indy, call, timeout)
        except Exception as exc:  # noqa: BLE001 - 진단은 계속 진행
            errors[call.name] = format_rpc_error(exc)

    return data, errors


def collect_static(indy: IndyDCP3, timeout: float) -> Dict[str, Dict[str, Any]]:
    """시작 시 한 번만 필요한 로봇/컨트롤러 정보를 읽는다."""
    data: Dict[str, Dict[str, Any]] = {}
    for call in STATIC_CALLS:
        try:
            data[call.name] = call_rpc(indy, call, timeout)
        except Exception as exc:  # noqa: BLE001 - 선택 진단 정보
            data[call.name] = {"error": format_rpc_error(exc)}
    return data


def response_warnings(data: Dict[str, Dict[str, Any]]) -> List[str]:
    warnings = []
    for name, payload in data.items():
        response = payload.get("response")
        if not isinstance(response, dict):
            continue
        code = as_int(response.get("code"))
        msg = response.get("msg") or response.get("message") or ""
        if code not in (None, 0):
            warnings.append(f"{name} 응답 code={code} msg={msg}")
    return warnings


def violation_messages(
    violation: Dict[str, Any],
    queue: Dict[str, Any],
) -> List[str]:
    """현재 violation과 큐에 남은 violation 메시지를 뽑는다."""
    messages = []
    direct_msg = violation.get("violation_str", "")
    if direct_msg:
        code = violation.get("violation_code", "")
        joint = violation.get("j_index", "")
        messages.append(
            f"violation 코드={code} 관절_index={joint}: {direct_msg}"
        )

    for item in queue.get("violation_queue", []):
        if not isinstance(item, dict):
            continue
        msg = item.get("violation_str", "")
        if not msg:
            continue
        code = item.get("violation_code", "")
        joint = item.get("j_index", "")
        text = f"큐 violation 코드={code} 관절_index={joint}: {msg}"
        if text not in messages:
            messages.append(text)
    return messages[:4]


def servo_findings(servo: Dict[str, Any]) -> List[str]:
    """서보 꺼짐, 브레이크 활성, 관절별 상태 코드 차이를 찾는다."""
    findings = []
    servo_actives = servo.get("servo_actives", [])
    brake_actives = servo.get("brake_actives", [])
    status_codes = servo.get("status_codes", [])

    off_joints = [
        index + 1
        for index, active in enumerate(servo_actives)
        if not as_bool(active)
    ]
    braked_joints = [
        index + 1
        for index, active in enumerate(brake_actives)
        if as_bool(active)
    ]

    if off_joints:
        findings.append(f"서보 꺼짐 관절 {off_joints}")
    if braked_joints:
        findings.append(f"브레이크 활성 관절 {braked_joints}")
    if status_codes and len(set(status_codes)) > 1:
        findings.append(f"관절별 서보 상태 코드가 다름: {status_codes}")
    return findings


def analyze_snapshot(
    data: Dict[str, Dict[str, Any]],
    errors: Dict[str, str],
) -> Tuple[str, List[str], List[str]]:
    """수집한 상태를 OK/WARN/ALERT로 해석한다."""
    alerts: List[str] = []
    warnings: List[str] = []

    for name, error in errors.items():
        if name == "boot" and "robot" in data:
            warnings.append(
                "boot RPC 실패: 컨트롤/RTDE는 연결됨. "
                f"컨트롤러 버전에서 boot service 미지원/차단 가능: {error}"
            )
        else:
            alerts.append(f"{name} RPC 실패: {error}")

    boot = data.get("boot", {})
    for field in BOOT_CRITICAL_FIELDS:
        if field in boot and not as_bool(boot.get(field)):
            alerts.append(f"부팅 상태 {field}=False")
    if "safety_connected" in boot and not as_bool(boot.get("safety_connected")):
        warnings.append("안전 컨트롤러가 연결되지 않음")

    robot = data.get("robot", {})
    op_state = as_int(robot.get("op_state"))
    if "is_robot_connected" in robot and not as_bool(
        robot.get("is_robot_connected")
    ):
        servo_actives = data.get("servo", {}).get("servo_actives", [])
        if servo_actives:
            warnings.append(
                "robot 데이터 is_robot_connected=False "
                "(서보 데이터는 수신됨. 구버전 필드 값일 수 있음)"
            )
        else:
            alerts.append("robot 데이터 is_robot_connected=False")
    if op_state in BAD_OP_STATES:
        alerts.append(
            "동작 상태가 "
            f"{state_label(op_state, OP_STATE_NAMES)}"
        )
    elif op_state in COMMAND_BLOCKING_OP_STATES:
        warnings.append(
            "외부 모션 명령이 막힐 수 있는 동작 상태: "
            f"{state_label(op_state, OP_STATE_NAMES)}"
        )
    elif op_state not in OK_OP_STATES and op_state is not None:
        warnings.append(
            f"동작 상태가 {state_label(op_state, OP_STATE_NAMES)}"
        )

    for msg in violation_messages(
        data.get("violation", {}),
        data.get("violation_queue", {}),
    ):
        alerts.append(short_text(msg, 180))

    for finding in servo_findings(data.get("servo", {})):
        alerts.append(finding)

    motion = data.get("motion", {})
    traj_state = as_int(motion.get("traj_state"))
    if traj_state == 9:
        alerts.append("trajectory 상태가 ERROR")
    if as_bool(motion.get("is_stopping")):
        warnings.append("모션 정지 중: is_stopping=True")
    if as_bool(motion.get("is_pausing")):
        warnings.append("모션 일시정지 중: is_pausing=True")

    program = data.get("program", {})
    program_alarm = program.get("program_alarm", "")
    if program_alarm:
        alerts.append(f"프로그램 알람: {program_alarm}")
    program_state = as_int(program.get("program_state"))
    if program_state in (1, 2, 3):
        warnings.append(
            "프로그램 상태가 "
            f"{state_label(program_state, PROGRAM_STATE_NAMES)}"
        )

    stop = data.get("stop", {})
    stop_category = as_int(stop.get("category"))
    if stop_category in (0, 1):
        alerts.append(
            f"정지 카테고리: {state_label(stop_category, STOP_CATEGORY_NAMES)}"
        )
    elif stop_category == 2:
        warnings.append(
            f"정지 카테고리: {state_label(stop_category, STOP_CATEGORY_NAMES)}"
        )

    safety = data.get("safety", {})
    safety_state = safety.get("safety_state")
    if isinstance(safety_state, dict):
        state = as_int(safety_state.get("state"))
        safety_id = as_int(safety_state.get("id"))
        if state == 255:
            warnings.append(
                f"안전 상태 safety_state id={safety_id} state=255 "
                "(알 수 없음/구버전 미지원 가능)"
            )
        elif state not in (None, 0):
            alerts.append(f"안전 상태 safety_state id={safety_id} state={state}")
    if "auto_mode" in safety and not as_bool(safety.get("auto_mode")):
        warnings.append("자동 모드 꺼짐: auto_mode=False")
    if "reduced_mode" in safety and as_bool(safety.get("reduced_mode")):
        warnings.append("감속 모드 켜짐: reduced_mode=True")

    auto_mode = data.get("auto_mode", {})
    if "on" in auto_mode and not as_bool(auto_mode.get("on")):
        warnings.append("자동 모드 확인 결과 on=False")

    reduced_mode = data.get("reduced_mode", {})
    if "on" in reduced_mode and as_bool(reduced_mode.get("on")):
        warnings.append("감속 모드 확인 결과 on=True")

    warnings.extend(response_warnings(data))

    level = "OK"
    if warnings:
        level = "WARN"
    if alerts:
        level = "ALERT"
    return level, alerts, warnings


def summary_line(
    data: Dict[str, Dict[str, Any]],
    errors: Dict[str, str],
    show_position: bool = False,
) -> str:
    """터미널 한 줄에 들어갈 핵심 상태 요약을 만든다."""
    robot = data.get("robot", {})
    motion = data.get("motion", {})
    servo = data.get("servo", {})
    program = data.get("program", {})
    safety = data.get("safety", {})
    stop = data.get("stop", {})
    violation = data.get("violation", {})

    servo_actives = servo.get("servo_actives", [])
    brake_actives = servo.get("brake_actives", [])
    servo_on = sum(1 for active in servo_actives if as_bool(active))
    brakes = sum(1 for active in brake_actives if as_bool(active))
    servo_total = len(servo_actives) if servo_actives else "-"

    violation_text = violation.get("violation_str", "")
    if not violation_text:
        violation_text = "-"

    fields = [
        f"op={state_label(robot.get('op_state'), OP_STATE_NAMES)}",
        f"motion_in={bool_label(motion.get('is_in_motion'))}",
        f"target={bool_label(motion.get('is_target_reached'))}",
        f"traj={state_label(motion.get('traj_state'), TRAJ_STATE_NAMES)}",
        f"queue={motion.get('motion_queue_size', '-')}",
        f"servo={servo_on}/{servo_total}",
        f"brake={brakes}",
        f"stop={state_label(stop.get('category'), STOP_CATEGORY_NAMES)}",
        f"auto={safety.get('auto_mode', '-')}",
        f"reduced={safety.get('reduced_mode', '-')}",
        f"program={state_label(program.get('program_state'), PROGRAM_STATE_NAMES)}",
        f"viol={short_text(violation_text, 80)}",
    ]

    if errors:
        fields.append("rpc실패=" + ",".join(sorted(errors.keys())))

    if show_position:
        q = robot.get("q") or []
        p = robot.get("p") or []
        fields.append(f"q={short_list(q)}")
        fields.append(f"p={short_list(p)}")

    return " | ".join(fields)


def colorize(text: str, level: str, enabled: bool) -> str:
    if not enabled:
        return text
    color = {
        "OK": "\033[32m",
        "WARN": "\033[33m",
        "ALERT": "\033[31m",
    }.get(level)
    if not color:
        return text
    return f"{color}{text}\033[0m"


def emit(text: str, log_file: Optional[str] = None) -> None:
    """터미널에 출력하고, 옵션이 있으면 로그 파일에도 남긴다."""
    print(text, flush=True)
    if log_file:
        with open(log_file, "a", encoding="utf-8") as handle:
            handle.write(strip_ansi(text) + "\n")


def print_static_info(
    data: Dict[str, Dict[str, Any]],
    log_file: Optional[str],
) -> None:
    control = data.get("control_info", {})
    device = data.get("device_info", {})
    lines = ["DCP3 모니터 초기화"]

    if "error" in control:
        lines.append(f"control_info 조회 실패: {control['error']}")
    else:
        version = control.get("control_version", "-")
        model = control.get("robot_model", "-")
        lines.append(f"컨트롤러: 모델={model} 버전={version}")

    if "error" in device:
        lines.append(f"device_info 조회 실패: {device['error']}")
    else:
        serial = device.get("robot_serial", "-")
        joints = device.get("num_joints", "-")
        payload = device.get("payload", "-")
        lines.append(
            f"장치: 시리얼={serial} 관절수={joints} 페이로드={payload}"
        )

    for line in lines:
        emit(line, log_file)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="IndyDCP3 실물 로봇 상태를 터미널에서 감시합니다.",
        add_help=False,
    )
    parser._positionals.title = "위치 인자"
    parser._optionals.title = "옵션"
    parser.add_argument(
        "-h",
        "--help",
        action="help",
        help="도움말을 출력하고 종료합니다.",
    )
    parser.add_argument("robot_ip_pos", nargs="?", help="로봇 컨트롤러 IP")
    parser.add_argument(
        "--robot-ip",
        default=None,
        help="로봇 컨트롤러 IP. 기본값: 166.104.214.96.",
    )
    parser.add_argument(
        "--index",
        type=int,
        default=0,
        choices=(0, 1),
        help="DCP3 포트 index. 기본값: 0.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="상태 조회 주기(초). 기본값: 1.0.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="RPC별 timeout(초). 기본값: 0.5.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="상태를 한 번만 출력하고 종료합니다.",
    )
    parser.add_argument(
        "--changes-only",
        action="store_true",
        help="상태 요약이나 진단 결과가 바뀔 때만 출력합니다.",
    )
    parser.add_argument(
        "--heartbeat-every",
        type=int,
        default=30,
        help="--changes-only 사용 중에도 N회마다 한 번 출력합니다. 기본값: 30.",
    )
    parser.add_argument(
        "--show-position",
        action="store_true",
        help="요약 줄에 현재 q/p 위치를 같이 표시합니다.",
    )
    parser.add_argument(
        "--control-state",
        action="store_true",
        help="GetControlState도 조회합니다. 토크/전류 디버깅 때 사용합니다.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="요약 진단 대신 원본 JSON 스냅샷을 출력합니다.",
    )
    parser.add_argument(
        "--log-file",
        default=None,
        help="모니터 출력을 지정한 파일에도 누적 저장합니다.",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="터미널 색상 출력을 끕니다.",
    )
    return parser


def run(args: argparse.Namespace) -> int:
    """모니터 메인 루프."""
    robot_ip = args.robot_ip or args.robot_ip_pos or "166.104.214.96"
    color_enabled = sys.stdout.isatty() and not args.no_color
    indy = IndyDCP3(robot_ip=robot_ip, index=args.index)

    emit(
        f"DCP3 모니터 시작: ip={robot_ip} "
        f"index={args.index} interval={args.interval}s timeout={args.timeout}s",
        args.log_file,
    )
    print_static_info(collect_static(indy, args.timeout), args.log_file)

    last_signature = None
    loop_index = 0
    while True:
        timestamp = datetime.now().strftime("%H:%M:%S")
        data, errors = collect_snapshot(
            indy,
            args.timeout,
            include_control_state=args.control_state,
        )

        if args.json:
            payload = {
                "time": timestamp,
                "data": data,
                "errors": errors,
            }
            emit(json.dumps(payload, ensure_ascii=False), args.log_file)
        else:
            level, alerts, warnings = analyze_snapshot(data, errors)
            summary = summary_line(data, errors, args.show_position)
            signature = (level, summary, tuple(alerts), tuple(warnings))
            heartbeat = (
                args.heartbeat_every > 0
                and loop_index % args.heartbeat_every == 0
            )
            should_print = (
                not args.changes_only
                or signature != last_signature
                or heartbeat
            )

            if should_print:
                level_text = DISPLAY_LEVEL_NAMES.get(level, level)
                header = colorize(
                    f"[{timestamp}] {level_text}",
                    level,
                    color_enabled,
                )
                emit(f"{header} {summary}", args.log_file)
                for item in alerts:
                    emit(
                        colorize(f"  위험: {item}", "ALERT", color_enabled),
                        args.log_file,
                    )
                for item in warnings:
                    emit(
                        colorize(f"  주의: {item}", "WARN", color_enabled),
                        args.log_file,
                    )
                last_signature = signature

        if args.once:
            return 0

        loop_index += 1
        try:
            time.sleep(max(args.interval, 0.05))
        except KeyboardInterrupt:
            emit("사용자 입력으로 DCP3 모니터를 종료합니다.", args.log_file)
            return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except KeyboardInterrupt:
        print("사용자 입력으로 DCP3 모니터를 종료합니다.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
