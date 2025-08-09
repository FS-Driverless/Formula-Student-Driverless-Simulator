# KSAE FSD SW Platform

## 개요

**Formula Student 자율주행 경기를 위한 ROS1 기반 통합 소프트웨어 플랫폼**

이 패키지는 Formula Student Driverless 대회 참가를 위한 완전한 자율주행 시스템을 제공합니다. C++과 Python을 모두 지원하며, 표준 Formula Student 메시지 타입([fs_msgs](https://github.com/FS-Driverless/fs_msgs))을 사용합니다.

## 주요 기능

- **인식 시스템**: LiDAR, 카메라를 통한 콘 감지 및 트랙 인식
- **경로 계획**: 실시간 최적 경로 생성 및 장애물 회피
- **차량 제어**: Pure Pursuit, Stanley 제어기를 통한 정밀한 차량 조향
- **Formula Student 표준**: fs_msgs 패키지를 통한 대회 표준 메시지 지원

## 시스템 요구사항

- **OS**: Ubuntu 20.04
- **ROS**: ROS Noetic
- **컴파일러**: GCC 7.0+ (C++11)
- **Python**: 3.6+

## 설치 방법

### 1. 저장소 클론 (서브모듈 포함)
```bash
# 프로젝트 클론 (fs_msgs 서브모듈 자동 포함)
git clone --recurse-submodules https://github.com/your-org/KSAE_FSD_SW_Platform.git

# 또는 기존 클론된 저장소에서 서브모듈 초기화
cd KSAE_FSD_SW_Platform
git submodule update --init --recursive
```

### 2. 의존성 설치 및 빌드
```bash
catkin_make
source devel/setup.bash
```

## 사용 방법

### Formula Student 자율주행 시스템 실행
```bash
# 전체 시스템 실행
roslaunch formula_autonomous_system formula_autonomous_system.launch
```

## 서브모듈 관리

### 서브모듈 업데이트
```bash
# fs_msgs 서브모듈 최신 버전으로 업데이트
git submodule update --remote src/msgs/fs_msgs

# 모든 서브모듈 최신 버전으로 업데이트
git submodule update --remote --recursive

```

## fs_msgs

이 프로젝트는 [FS-Driverless/fs_msgs](https://github.com/FS-Driverless/fs_msgs) 패키지를 서브모듈로 사용합니다.