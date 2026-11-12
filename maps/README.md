# 지도 보관

실제 지도는 Git에 포함하지 않습니다. 구현할 저장 도구는 실행별 하위 폴더에 occupancy map, SLAM pose graph와 metadata를 함께 저장합니다.

metadata에는 backend(`sim`/`real`), 소스·의존성 commit, frame 이름, geometry/calibration hash, 생성 시각을 기록합니다. sim 지도와 실제 공간 지도를 구분하며, 공개가 필요한 예시는 별도로 선정합니다.
