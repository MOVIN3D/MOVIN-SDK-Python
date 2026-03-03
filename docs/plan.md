# 계획: mocap_to_robot_mujoco_stream.py + E2E 진단 (GR00T 코드 수정 없음)

## 상태
- 작성일: 2026-02-27
- 방향: GR00T-WholeBodyControl 코드는 직접 수정하지 않고, MOVIN 송신/수신 파이프라인 정합성 중심으로 검증한다.
- 대상:
  - `/home/dkjang/Workspace/Code/DK/MOVIN-SDK-Python/docs/plan.md`
  - `/home/dkjang/Workspace/Code/DK/MOVIN-SDK-Python/examples/mocap_to_robot_mujoco_stream.py`
  - `/home/dkjang/Workspace/Code/DK/MOVIN-SDK-Python/examples/mocap_to_robot_mujoco_zmq_viewer.py`
  - `/home/dkjang/Workspace/Code/DK/MOVIN-SDK-Python/examples/zmq_pose_stream_visualizer.py`

## 구현/결정사항
- ZMQ 프로토콜은 `v1` 고정.
- 필수 필드: `joint_pos [N,29]`, `joint_vel [N,29]`, `body_quat [N,4]`, `frame_index [N]`, `catch_up [1]`.
- 루트 이동 검증을 위해 `root_pos [N,3]`를 항상 송출하고, 하위 호환성으로 `body_pos [N,3]`를 함께 송출.
- `send_fps=0`이면 BVH 헤더의 `Frame Time`으로 자동 추론(Locomotion은 60Hz 근접).
- `frame_index`는 int64 연속 증가(스트리밍 정합성용).
- `batch_size` 기본값 1.
- GR00T C++ 쪽 root_pos/body_pos 처리 임시 패치는 이전 상태에서 모두 원복함.
- 현재는 GR00T 코드 변경 없이 원인 분리를 진행한다.

## 공개 CLI (현재)
- `--bvh_file` (기본 `examples/test.bvh`)
- `--robot` (`unitree_g1`, `unitree_g1_with_hands`)
- `--human_height`
- `--send_fps` (`0`이면 BVH frame time 사용)
- `--loop_bvh`, `--no_loop_bvh`
- `--zmq_protocol`(현재 `1`)
- `--zmq_host`, `--zmq_port`, `--zmq_topic`
- `--zmq_conflate`
- `--batch_size`
- `--catch_up`, `--no_catch_up`
- `--show_viewer`, `--print_fps`, `--debug`

## 테스트 및 재현 절차 (현재 기준)
### 1) 송신기 단독 + 스키마 정합성
```bash
cd /home/dkjang/Workspace/Code/DK/MOVIN-SDK-Python
/home/dkjang/Workspace/Code/DK/MOVIN-SDK-Python/.venv/bin/python -u examples/mocap_to_robot_mujoco_stream.py \
  --bvh_file examples/Locomotion.bvh \
  --send_fps 0 \
  --batch_size 1 \
  --print_fps

/home/dkjang/Workspace/Code/DK/MOVIN-SDK-Python/.venv/bin/python -u examples/zmq_pose_stream_visualizer.py \
  --host localhost \
  --port 5556 \
  --topic pose \
  --duration_sec 10 \
  --print_every 1
```

### 2) 로컬 viewer 재생
```bash
cd /home/dkjang/Workspace/Code/DK/MOVIN-SDK-Python
/home/dkjang/Workspace/Code/DK/MOVIN-SDK-Python/.venv/bin/python -u examples/mocap_to_robot_mujoco_zmq_viewer.py \
  --host localhost --port 5556 --topic pose --robot unitree_g1 --print_fps
```

### 3) 경계/예외 케이스
- `--batch_size 3`
- `--no_catch_up`
- `--no_loop_bvh`
- `--no_loop_bvh --batch_size 1`로 마지막 프레임 종료 확인
- 잘못된 BVH 경로 예외 메시지 확인

### 4) GR00T E2E(참조, GR00T 수정 없음)
- 기존 가이드 기준:
  - `cd /home/dkjang/Workspace/Code/Others/GR00T-WholeBodyControl`
  - `source .venv_sim/bin/activate && python gear_sonic/scripts/run_sim_loop.py`
  - `cd gear_sonic_deploy && bash deploy.sh sim --input-type zmq --zmq-host localhost --zmq-port 5556 --zmq-topic pose`
  - 송신기 실행
- 실패 시 분기:
  1) 송신 측 필드/속도/연속성 로그 확인  
  2) GR00T 토픽/포트/인풋타입 매칭 확인  
  3) GR00T가 실제 반영하는 필드만 사용되고 있는지 확인

## 현재 확인 결과 (요약)
- Locomotion 송신 성공:
  - `send_fps=59.9988` (BVH frame time 추론)
  - 메시지 필드에 `joint_pos`, `joint_vel`, `body_quat`, `root_pos`, `body_pos`, `frame_index`, `catch_up` 모두 존재
  - `header=1280B`, `used_payload=281/281 bytes`(N=1)
  - `frame_index` 연속성 및 수신 FPS가 정상(약 60fps)
- `matplotlib unavailable` 경고는 시각화 스크립트 제한 경고일 뿐, 패킷 파싱/수신 자체 실패와는 무관.
- `--send_fps` mismatch로 인한 정적 실패 가설은 낮아짐(Locomotion은 자동 추론으로 해결).
- `GR00T에서 참조 추적이 안 되는 경우`는 송신기 실패보다 GR00T가 root/velocity를 어떻게 소비하는지가 주 원인으로 보임.

## 보류 항목
- GR00T 소스 변경 없이 완전 추적 정합성 판단은 제한적임.
- 로컬 sender/visualizer는 정합된 값으로 동작하므로, GR00T E2E는 별도 로그/입력 스키마 확인이 필요.

## 2026-02-27 테스트 실행 기록 (요청한 3단계 절차 기반)

### 1) 단독 송신 + 스키마 정합성
- 상태: **완료 (PASS)**
- 실행: `mocap_to_robot_mujoco_stream.py --bvh_file examples/Locomotion.bvh --send_fps 0 --loop_bvh`
- 관측:
  - `protocol=v1`, `topic=pose`, `zmq_host=*`, `batch_size=1` 전송
  - `frame_index` 연속 증가
  - `used_payload=281/281`, `header=1280B`
  - 필수 필드 존재: `joint_pos`, `joint_vel`, `body_quat`, `frame_index`, `catch_up`
  - 보조 필드 존재: `root_pos`, `body_pos`
  - `matplotlib` 미설치 시 `zmq_pose_stream_visualizer`에서 경고만 출력되나 파싱 실패는 없음
- 보강 확인:
  - `visualizer` 동시 구독 테스트에서 `frame_diff=0`, `status=OK` 지속
  - 포지션/속도/루트 이동이 연속적으로 정상 범위 내 수신(`joint_vel` 피크 수치 포함)
  - 패킷당 `used_payload=281/281 bytes`로 일관

### 2) run_sim_loop + deploy(sim)
- 상태: **차단 (BLOCKED)**
- 실패 원인:
  - 최초 시도: 기본 실행은 `DISPLAY` 미설정으로 `GLFW` 초기화 실패(`glfw` X11 오류)
  - 최신 시도: `--no-enable-onscreen --enable-offscreen` 옵션으로 시뮬레이터는 기동됨(`run_sim_loop.py` PID 400868 유지)
    - 예: `cd /home/dkjang/Workspace/Code/Others/GR00T-WholeBodyControl && source .venv_sim/bin/activate && python gear_sonic/scripts/run_sim_loop.py --no-enable-onscreen --enable-offscreen`
  - `deploy.sh sim --input-type zmq --zmq-host localhost --zmq-port 5556 --zmq-topic pose`는 아래 이유로 BLOCKED
    - 필수 모델 파일 미존재: `policy/release/model_decoder.onnx`, `policy/release/model_encoder.onnx`, `planner/target_vel/V2/planner_sonic.onnx`
    - 설치 루틴에서 `sudo` 비대화형 권한 처리 실패
  - 추가: GR00T venv를 새로 구성해 `gear_sonic` 및 `unitree_sdk2py` import 실행 환경을 확보함
    - `uv venv --python 3.10 .venv_sim`
    - `pip install -e "gear_sonic[sim,teleop]" && pip install -e external_dependencies/unitree_sdk2_python`

### 3) GR00T 제어 파이프라인 전체 연동
- 상태: **차단 (BLOCKED)**
- 원인:
  - 2단계가 배포 전제(모델 파일/권한) 미충족으로 차단
  - 로컬에서는 run_sim_loop은 온스크린 없이 유지되고 있으나, deploy 없이 C++ side 수신 로그를 직접 검증할 수 없음

### 4) 실행 시도 보충 기록
- `run_sim_loop.py --help`는 `--no-enable-onscreen` / `--enable-offscreen` 플래그 확인(tyro)
- `deploy.sh` 단계에서는 정책 파일 누락/권한 이슈로 실패

### 추가 진단(요청사항 반영)
- `zmq_endpoint_interface.hpp` 기준으로도 송신 포맷은 호환:
  - `protocol=1`에서 `joint_pos/joint_vel/body_quat/frame_index` 필수, `catch_up` 기본 true 처리
  - `body_quat`는 `w,x,y,z` 순서 허용
- `root_pos`는 현재 GR00T v1 필수 필드가 아님(참조용 부가 필드)
- 로컬 무드 뷰어(`mocap_to_robot_mujoco_zmq_viewer.py`)는 머리말: 현재 머신이 DISPLAY 미설정되어 `glfw` 초기화 실패. 즉, 로컬 GUI 렌더링은 별도 그래픽 환경에서 재실행 필요.
