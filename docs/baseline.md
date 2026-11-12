# 기술 기준과 호환성

대상 환경: **Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic**. 기준일: 2026-11-12.

## 시스템 기준

| 구성 요소 | 기준 | 적용 조건 |
| --- | --- | --- |
| 메가/Orin 공통 제어 | `Oh-My-Control@c703f130c14388b4b7845a25b900d726f8c9a71d` | 저수준 구동 기준. 새 ROS 연결은 별도 검증 |
| 실물 컴퓨터 | Jetson Orin, aarch64, Ubuntu 24.04.5 LTS, ROS 2 Jazzy | ARM64에서 별도 빌드 |
| 기존 실물 구성 | `robot_mapping_ws`, 수동 mapping 및 저장 지도 navigation | 새 bridge·odom/TF 계약으로 이관 |
| 기존 시뮬레이션 | Jazzy + Harmonic, NavFn + MPPI, 보로노이 선호 cost·깊이 장애물 기억 | 새 공통 기준은 NavFn + RPP |
| 모델 저장소 | `Oh-My-MobileRobot@f85154b53ea57b80e8a8ff6e4d6d8d38a8a0c35b` | description·CAD/mesh의 출처와 라이선스 유지 |
| 소스 구성 | 기존 sim은 독립 Git 이력 없음 | 이관 파일의 SHA256·NOTICE로 출처 고정 |

저수준 제어의 기본 실물 구동 확인은 ROS bridge·오도메트리·SLAM·Nav2의 인수 검증을 대체하지 않습니다. 실물 센서 드라이버의 정확한 설치 버전·토픽·frame·시간/QoS 설정은 P5의 배치 기준에 포함합니다.

## 개발 PC 설치 버전

개발 기준의 설치 버전은 다음과 같습니다. Orin의 호환 버전은 장비별 배치 manifest에 별도로 고정합니다.

| 패키지 | 기준 버전 |
| --- | --- |
| `ros_gz_bridge` | 1.0.24 |
| `nav2_bringup`, `nav2_controller`, `nav2_mppi_controller` | 1.3.13 |
| `slam_toolbox` | 2.8.5 |
| `robot_localization` | 3.8.3 |
| `diff_drive_controller` | 4.42.1 |

문서의 최신 Jazzy branch와 설치된 바이너리가 완전히 같다고 가정하지 않습니다. 특히 stamped velocity 옵션·smoother timestamp 보존 설정은 P1에서 설치본의 선언 파라미터와 실제 메시지로 확인합니다.

## 제어 계층 규격

| 항목 | 값/의미 |
| --- | --- |
| 프로토콜 | Mega v8, `signed_pwm_v1`, OMR1 CRC/sequence 프레임 |
| USB 시리얼 | 115200 bps |
| MCU 속도 제어 | 1 kHz |
| 호스트 송신 / 상태 수신 | 최대 50 Hz / 약 20 Hz |
| command lease | 100–300 ms, 원본 생성 시각의 유효기간 보존 |
| 피드백 중단 | 호스트 400 ms 기준 |
| 바퀴 상한 | 실물 운용 95 RPM / MCU 보호 100 RPM |
| 정상 0 속도 | 제동/HOLD |
| `STOP` | 제어·출력 해제 |

근거: [제어 소스 고정 커밋](https://github.com/Leehyunbin0131/Oh-My-Control/tree/c703f130c14388b4b7845a25b900d726f8c9a71d), 특히 `tools/wheel_drive.py`, `motor_link.py`, `motor_pid.py`, `drive_calibration.py`, `docs/control_rebuild.md`.

## 모델과 실측값의 구분

| 항목 | 현재 모델 값 | 적용 방식 |
| --- | --- | --- |
| 바퀴 반경 | 0.068 m | sim 초기값; 실물 유효값은 지면 보정 |
| 좌우 구동축 간격 | 0.480 m | 실물 유효 윤거와 구분 |
| `base_footprint → base_link` | `(0, 0, 0.101)` m | 원본 모델 구조 유지 |
| wheel joints | `left_wheel_joint`, `right_wheel_joint` | 위치 `(0, ±0.24, −0.033)` m, 축 +Y |
| 기존 sim footprint | 앞 +0.09 / 뒤 −0.39 / 좌우 ±0.26 m | 기준 원점은 구동축; 실물 외곽을 확인해 갱신 |
| 기존 padding | 0.02 m | 초기값; 실제 외곽·제동/관측 오차와 별도 검토 |
| lidar | `base_link → lidar_link=(-0.14,0,0.143)` m, 이후 `laser_frame` z=0.0303 m | CAD 위치이며 실물 장착 TF를 확인 |
| camera | `base_link → camera_link=(0.06505,0,0.1035)` m | optical transform 포함; 실물 높이/각도와 대조 |

기준 xacro SHA256: `5290010ef531becc2b4c5e3db8f64dff8692b4000129714ee13cbbf37a389009`. 원본 저장소와 기존 sim의 모델 내용은 동일합니다. 버전 표기에서는 반입 당시 commit과 현재 원본 저장소 HEAD를 구분합니다.

## 재사용과 변경

| 기존 자산 | 새 저장소의 처리 |
| --- | --- |
| Mega 펌웨어·공통 통신/보정 | 버전 고정 재사용. ROS 명령 시각을 전달하는 작은 호스트 API 확장만 P1에 계획 |
| 원본 description·mesh | provenance·라이선스 유지, 필요한 경로만 설치 |
| Gazebo 모델·센서·월드 | 공통 ROS 계약과 95 RPM 운용 한도로 이관 |
| `omm_perception`, `omm_perception_msgs` | 기존 바닥 분리·낮은 장애물 기억 기능 이관 |
| `omm_costmap_layers` | camera memory와 보로노이 선호 비용 이관 |
| 기존 MPPI critic·State Lattice | 공통 NavFn+RPP 기준 구현 이후 실제 요구가 생길 때만 검토 |
| 기존 실물 bridge | 통신·정지·원본 수명 계약을 새 base에 맞춰 구현. 소스를 확인 없이 복제하지 않음 |
| ICP odometry | 새 기본 구성은 측정 wheel odom+IMU. 기존 ICP TF와 동시 발행하지 않음 |
| 기존 실물 지도·보정 | 원본 보존, 장비별 설정으로 관리. 새 구성의 통과 근거로 자동 승계하지 않음 |

## 레거시 구성과의 호환성

| 구분 | 레거시 특성 | 이관 요구사항 |
| --- | --- | --- |
| 실물 controller | 운용 문서는 NavFn + RPP, 일부 보관 검증은 NavFn + MPPI | 실제 controller 설정을 고정하고 서로 다른 구성의 결과를 혼용하지 않음 |
| 실물 odom | 보관 구성은 `icp_odometry`, 약 12 Hz | 새 측정 wheel odom/EKF로 전환, 기존 TF authority 비활성화 |
| sim odom | Gazebo가 odom과 odom TF를 함께 제공 | EKF 적용 시 Gazebo TF를 제거 |
| sim 명령 | `Twist`를 guard에서 주기적으로 재발행 | stamped 원본 수명·공통 명령 계약으로 변경 |
| 실물 입력 timeout | 레거시 bridge는 지속적인 0 RPM/HOLD | 정상 zero와 입력 단절을 구분하고 단절 시 `STOP` |
| 기존 인수 근거 | 일부 실행은 `motor:=false`, 경로 생성 후 취소 | 새 실물 목표 도착의 검증 근거로 사용하지 않음 |

정확한 명령·TF 계약은 [통합 설계](architecture.md), 단계별 판정은 [검증 기준](acceptance.md)을 따릅니다.

## 공식 자료

설계 선택은 현재 로봇의 제약을 바탕으로 한 프로젝트 판단입니다. 아래 자료는 ROS 인터페이스와 동작 규약의 근거입니다.

1. [Nav2 Jazzy: Gazebo 구성 순서](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/gazebo/) — TF·모델·odom·센서·mapping·navigation의 연결 순서.
2. [Nav2 Jazzy: robot_localization](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization/) — wheel/IMU 결합과 odom TF의 단일 발행.
3. [Nav2 Jazzy: Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/) — stamped 명령, 입력/출력 토픽, 센서 timeout.
4. [Nav2 Jazzy: Collision Monitor 연결](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/) — 속도 처리 체인의 마지막 단계에 배치.
5. [Nav2 Jazzy smoother 소스](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_velocity_smoother/src/velocity_smoother.cpp) 및 [collision monitor 소스](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_collision_monitor/src/collision_monitor_node.cpp) — stamp 보존과 timeout 뒤 메시지의 의미 확인.
6. [SLAM Toolbox Jazzy README](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/README.md) — scan/odom 입력, map 저장과 pose graph 직렬화의 차이.
7. [REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst), [REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst) — 좌표축·SI 단위와 map/odom/base 프레임 책임.
8. [ros2_control Jazzy diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) — 대안 검토 근거. 현재 계획에서 이 controller를 사용한다는 뜻은 아님.
