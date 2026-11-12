# 계획된 패키지 구조

아래 패키지는 구현 예정입니다. 이 문서만 있는 현재 상태에서는 colcon 빌드·실행 대상이 없습니다.

| 패키지 | 책임 | 첫 구현 단계 |
| --- | --- | --- |
| `omslam_control_vendor` | 고정 `Oh-My-Control` 모듈의 설치·버전 확인 | P1 |
| `omslam_base` | ROS 구동 어댑터·기구학·원본 명령 수명·측정 odom·진단 | P1 |
| `omslam_description` | 원본 CAD 모델 연계, joint/frame, 장비별 extrinsic | P2 |
| `omslam_sim` | Gazebo 하단 어댑터·가상 센서·월드 | P2 |
| `omslam_localization` | wheel-only/EKF 설정, IMU 입력과 TF 발행 선택 | P2–P3 |
| `omslam_mapping` | SLAM·map/graph 저장·map_server/AMCL | P3 |
| `omslam_navigation` | Nav2 공통 설정·속도 체인·BT·실행 모드 | P4 |
| `omm_perception`, `omm_perception_msgs`, `omm_costmap_layers` | 기존 인지·메모리·보로노이 cost 기능 이관 | P4 |
| `omslam_bringup` | sim/real·mapping/localization/navigation 모드 조합 | P2부터 단계별 |

P1의 순수 기구학·시간 처리는 `omslam_base` 내부 모듈로 두며 불필요하게 많은 패키지로 나누지 않습니다. 기존 인지 패키지는 이름을 바꾸기 위해 재작성하지 않고 원래 이름과 출처를 유지합니다.

launch/config/RViz/BT는 해당 패키지의 share 경로에 설치합니다. 실행은 source tree의 파일 경로가 아닌 설치된 패키지 이름을 기준으로 합니다.
