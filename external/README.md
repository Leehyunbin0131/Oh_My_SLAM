# 외부 소스

`../dependencies.repos`의 고정 커밋을 이 위치로 가져오도록 계획합니다. 상위 저장소 자체에는 외부 Git checkout을 커밋하지 않습니다.

이 디렉터리의 `COLCON_IGNORE`는 외부 저장소 안의 ROS 패키지나 기존 workspace가 자동으로 중복 빌드되는 것을 막습니다. 이후 vendor/description 패키지가 필요한 부분을 명시적으로 설치합니다.

현재 계획 단계에서는 외부 소스를 import하거나 빌드하지 않았습니다.
