# 현재 기준과 출처

확인일: **2026-11-12**. 이 문서는 관측한 현재 상태와 앞으로 구현할 설계를 구분합니다.

## 확인한 현재 상태

| 항목 | 관측 결과 | 근거·한계 |
| --- | --- | --- |
| 메가/Orin 공통 제어 | `Oh-My-Control@c703f130c14388b4b7845a25b900d726f8c9a71d` | 이전 작업에서 GitHub와 Orin `ros2_dev_ws`의 commit·소스 일치 확인 |
| 기본 실물 구동 | 무부하·지면 전후진·좌우 회전·정지·재출발 정상 | 2026-11-12 사용자 보고; 계측 정지거리나 ROS Nav2 도착 결과는 아님 |
| Orin 환경 | aarch64, Ubuntu 24.04.5 LTS, `/opt/ros/jazzy` | 이번 계획 시작 시 SSH 읽기로 확인 |
| 기존 실물 workspace | `robot_mapping_ws` | 현재 README의 수동 mapping·저장 지도 navigation 설명을 읽음 |
| 개발 PC 소스 위치 | `~/MyRos/Oh-My-Control`, `~/MyRos/mobile_robot_sim`, `~/MyRos/Oh-My-MobileRobot` | 조사 도중 홈에서 MyRos 아래로 이동된 새 위치 확인 |
| 기존 시뮬레이션 | Jazzy + Harmonic, 기본 NavFn + MPPI, 보로노이 선호 cost·깊이 장애물 기억 | 소스·README 확인. sim 폴더 자체는 Git 저장소가 아님 |
| 모델 저장소 | `Oh-My-MobileRobot@f85154b53ea57b80e8a8ff6e4d6d8d38a8a0c35b` | 작업 트리 clean, sim의 xacro와 내용 일치 |

후속 SSH 소스 수집은 네트워크 단절(`No route to host`)로 수행하지 못했습니다. 따라서 현재 실물 ROS bridge의 상세 소스·실행 중 graph는 이 계획에서 확인했다고 주장하지 않습니다. P5에서 연결 후 최신 버전을 다시 대조합니다. 계획 수립 중 모터 구동·센서 장치 open·ROS launch는 실행하지 않았습니다.

## 개발 PC 설치 버전

아래는 `/opt/ros/jazzy/share/*/package.xml`에서 읽은 값입니다. Orin의 동일 버전을 의미하지 않으며 배치 시 장비별로 기록합니다.

| 패키지 | 관측 버전 |
| --- | --- |
| `ros_gz_bridge` | 1.0.24 |
| `nav2_bringup`, `nav2_controller`, `nav2_mppi_controller` | 1.3.13 |
| `slam_toolbox` | 2.8.5 |
| `robot_localization` | 3.8.3 |
| `diff_drive_controller` | 4.42.1 |

문서의 최신 Jazzy branch와 설치된 바이너리가 완전히 같다고 가정하지 않습니다. 특히 stamped velocity 옵션·smoother timestamp 보존 설정은 P1에서 설치본의 선언 파라미터와 실제 메시지로 확인합니다.

## 제어층의 현재 계약

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
| 기존 자동 검증 | 초기 공통계층·펌웨어 검증 후, 최신 WheelDrive의 관련 6개 검사 통과 |

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

원본과 sim 복사본 xacro의 SHA256는 `5290010ef531becc2b4c5e3db8f64dff8692b4000129714ee13cbbf37a389009`로 일치했습니다. 기존 sim NOTICE의 과거 반입 commit과 현재 원본 저장소 HEAD는 다른 개념입니다.

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

## 기존 검증을 새 완료 근거로 오인하지 않기

- 현재 실물 README는 **NavFn + RPP**, 0.30 m/s 목표를 설명합니다. 기존 보관 Nav2 검증 JSON은 **NavFn + MPPI**, `motor:=false` 조건입니다.
- 보관 검증의 `/odom` 발행자는 `icp_odometry`, 약 12 Hz입니다. 새 wheel odom/EKF의 검증이 아닙니다.
- 기존 검증에서 목표는 도착한 것이 아니라 취소됐습니다. 센서·lifecycle·경로 생성 성공을 새 실물 목표 도착 성공으로 쓰지 않습니다.
- 기존 sim은 Gazebo가 `/odom`·odom TF를 함께 발행하고, 명령 guard는 `Twist`를 주기적으로 재발행합니다. 새 stamped 계약·단일 TF authority로 정리해야 합니다.
- 기존 실물 README의 입력 timeout 뒤 지속적인 0 RPM/HOLD는 새 `WheelDrive`의 원본 만료 후 `STOP`과 다릅니다. [통합 설계](architecture.md)의 계약으로 통일합니다.

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
