# 검증 기준과 기록 방법

아래는 **초기 인수 기준**이며 성능 측정 결과가 아닙니다. 하드웨어·제어 계층의 기준 규격은 [기술 기준](baseline.md)에 정의합니다.

## 단계별 최소 검증

| 검증 | 대상 | 한 번의 묶음에 포함할 항목 | 통과 기준 |
| --- | --- | --- | --- |
| V1 | ROS base·mock | 기구학·부호·명령 수명·큐 지연·fault·원시 엔코더 적분·설치본 | 정해진 입력의 계산값 일치, 원본 age를 재송신으로 늘리지 않음, stale 명령 재활성화 0회 |
| V2 | Sim base·센서 | 짧은 전진/회전, TF·시간·타입·pause/resume | 전진 +X/좌회전 +yaw, TF 각 edge 단일 발행, `/clock` 혼용 없음, resume 후 새 명령 요구 |
| V3 | Sim mapping/localization | 폐루프·map/graph 저장·AMCL 재시작 | 중복 벽/지속적 TF 오류 없음, 저장 파일 재로딩 성공, truth 대비 위치 ≤0.15 m·yaw ≤0.20 rad |
| V4 | Sim Nav2 | 도착/취소, 코너·좁은 통로, 낮은 장애물, 막힌 경로 | 위치 ≤0.15 m·yaw ≤0.20 rad, 충돌 0회, 통과 불가 시 정지 후 제한된 실패 반환 |
| V5 | Real base·odom/IMU | 새 ROS 경로 구동, 1 m 직진·양방향 회전·짧은 폐루프, 저속/정지 | 직진 거리 오차 ≤5%, 360° 회전 오차 ≤5°, 부호·축 정상, 원본 명령 timeout과 fault 처리 확인 |
| V6 | Real SLAM/AMCL | 실제 폐루프·저장·두 위치 재초기화 | 기준 표식 대비 위치 ≤0.15 m·yaw ≤5°, 벽 이중화/지속적 scan drop 없음 |
| V7 | Real 저장 지도 Nav2 | 3개 목표·취소/재출발·장애물·센서중단 | 위치 ≤0.15 m·yaw ≤0.20 rad, 접촉 0회, 실제 여유가 설정 footprint+padding 만족 |
| V8 | Real online SLAM+Nav2 | 지도 확장·loop closure·목표·저장/재시작 | V7 목표 기준 유지, odom 연속, map 보정 뒤 경로/메모리 정합, 모드 전환 중 비의도 구동 0회 |

V5의 양방향 회전은 방향별 유효 윤거/미끄러짐 차이를 확인하기 위한 것입니다. 여러 속도·모든 센서의 무차별 전체 조합 시험으로 확대하지 않습니다. 필요한 저속 범위는 Nav2가 실제 사용하는 값과 관측된 문제를 기준으로 선택합니다.

V1의 합성 수학 시험은 부동소수점 허용오차를 코드에서 명시합니다. V3–V8의 위치·각도 기준은 결과를 본 뒤 임의로 완화하지 않으며, 변경이 필요하면 이유와 적용 범위를 계획 문서에 기록합니다.

## 시간·정지 판정

명령 처리와 실제 로봇 정지는 다음처럼 나눕니다.

| 측정 | 기준/해석 |
| --- | --- |
| 원본 입력 deadline | 기본 300 ms. 원본 stamp·수신 monotonic 시각·MCU lease를 함께 기록 |
| 모의 worker의 출력 해제 | deadline 후 1 worker 주기(20 ms) 이내 `STOP` 요청; scheduling 영향은 별도 |
| 실제 MCU 출력 해제 | 300 ms 명령 만료를 기준으로 UART·MCU tick·측정 시각 오차를 기록. 초기 목표 350 ms 이내이며 반드시 실제 관측으로 판정 |
| 실제 정지 완료 | 최신 측정 속도·상태와 관찰을 함께 확인. ACK만으로 정지 판정하지 않음 |
| 제동 거리 | 시험 속도별 거리 측정. 충돌 감시 거리에는 반응 지연 중 이동거리와 제동거리·padding 포함 |
| feedback freshness | 기존 400 ms 제한 보존. bridge가 stale 피드백을 새 stamp로 재발행하지 않음 |
| 정상 제어 주기 | 50 Hz 목표 송신, 실제 telemetry 약 20 Hz. p50/p95/p99와 긴 gap 기록 |
| planner/controller 부하 | 필요 경로가 정해진 시각 내 갱신되는지와 command age를 함께 확인. CPU 비율만으로 통과 판정하지 않음 |

이 숫자는 기능의 보장이나 안전 인증을 의미하지 않습니다. firmware watchdog의 출력 해제와 차체의 물리적 정지 시간을 서로 바꿔 쓰지 않습니다. 시뮬레이션 정지로 실물 차단 시간을 검증했다고 보고하지 않습니다.

## 필수 실패 주입의 범위

V1의 모의 환경에서 한 번 확인할 항목:

- 원본 publisher가 멈췄지만 smoother가 같은 명령을 계속 출력하는 경우.
- 원본 시각이 유지된 여러 평활 출력, 큐에서 지연된 명령, 오래된 메시지의 지연 도착.
- zero 명령 이후 늦게 도착한 nonzero, mode 전환 전의 저장 명령.
- 원본 age가 200 ms를 넘긴 뒤 들어온 Collision Monitor의 zero: 최소 lease 때문에 생략하지 않고 즉시 `STOP`하는지.
- 피드백 단절, CRC/프레임 오류, MCU fault, MCU clock 재시작·정상 wrap.
- deactivate/reconnect 뒤 fresh 명령 없이 이동하지 않는지.

실물에서는 이 모의 조합 전체를 반복하지 않습니다. 새 하드웨어 경로의 입력 중단·브리지 중단·피드백/센서 중단 중 실제 연결이 필요한 항목만 준비된 V5/V7 세션에서 확인합니다.

## TF·시간·센서 판정

- `map → odom`과 `odom → base_footprint`의 authority를 기록하고 각각 하나인지 확인합니다.
- 센서 transform은 실제 장착 방향과 대조합니다. URDF 파일의 숫자가 서로 맞는 것만으로 실물 축 정합을 판정하지 않습니다.
- IMU의 중력과 좌회전 yaw rate는 base 좌표계로 변환한 뒤 판단합니다.
- wheel odom과 `/joint_states`는 실제 sample ID·MCU tick이 바뀔 때 갱신되어야 합니다.
- sim은 모든 관련 노드가 sim time, real은 wall/system time이어야 합니다. 실물 노트북/Orin의 시각 동기화 상태를 확인합니다.
- 카메라 depth scale·intrinsics·optical frame·TF 시각·장애물 기억의 좌표계를 확인합니다. 빈 depth와 관측된 자유 공간을 구별합니다.
- 실물 지도와 sim 지도는 metadata의 backend로 구분하고 잘못된 backend의 지도를 실물 launch 기본값으로 고르지 않습니다.

## 빌드와 실행 근거

구현 뒤에는 각 단계에 필요한 패키지 graph를 `colcon list`로 확인하고 `colcon build --packages-up-to <대상>`으로 빌드합니다. 빌드가 성공한 뒤 새 셸에서 underlay와 이번 install을 source하고 `ros2 pkg prefix <대상>`을 확인합니다.

테스트는 새 결과 디렉터리에서 실행하고 **실제로 실행된 비-skipped 검사 ID와 개수**를 남깁니다. `colcon test-result`의 성공 코드나 빈 테스트 결과만으로 통과시키지 않습니다. 변경이 launch·설치 경로뿐이면 관련 실행 확인을 사용하며 기존 동작 검사 전체를 반복하지 않습니다.

아직 패키지가 없는 계획 단계에 가짜 테스트나 빈 colcon 성공 기록을 만들지 않습니다.

## 기록 구조

실행 도구는 구현 시 다음 형태의 run 디렉터리를 생성하도록 합니다.

```text
reports/<stage>/<run-id>/
  manifest.json       # commit, 의존성 commit, config/calibration hash, backend, 장비/버전
  commands.json       # 실제 실행 명령, cwd, 종료 코드, 대상 package prefix
  results.json        # 검사 ID, 측정값, 기준, pass/fail/inconclusive
  summary.md          # 이번 범위의 결과와 실제 한계
  rosbag/             # 필요한 토픽만; 무거운 RGB 원본은 목적이 있을 때만
  logs/
```

입력 명령·wheel odom·filtered odom·IMU·TF·진단·scan은 측정 목적에 맞게 선택합니다. 제어부하가 달라질 정도로 모든 영상 토픽을 무조건 녹화하지 않습니다.

공개 저장소에는 실내 지도, 원본 rosbag, 장비 주소·인증정보를 넣지 않습니다. 필요할 때 사용자가 공개 범위를 정한 요약·그림만 게시합니다.

## 중단과 복구

- build 실패: 의존 실행 검사 중단, 이전 install로 대체 검증하지 않음.
- 실제 방향/TF 오류·예기치 않은 재출발·정지 불응: 해당 실물 단계를 중단하고 원인 수정 후 영향을 받은 검사만 재수행.
- 지도 정합 실패: Nav2를 얹어 숨기지 않고 odom·센서 frame·시각부터 근거를 확인.
- rollback: 소스 commit, calibration hash, map metadata를 함께 선택. 기존 `ros2_dev_ws`/`robot_mapping_ws`와 새 workspace를 동시에 구동하지 않음.

필요한 구현과 해당 단계의 검증이 끝나면 그 단계는 완료입니다. 이후 단계나 이번 범위 밖 기능을 현재 단계의 미완성으로 취급하지 않습니다.
