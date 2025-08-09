#!/usr/bin/env python3
import sys
import os

# FSDS Python 라이브러리의 경로를 시스템에 추가
# 이 스크립트가 어디서 실행되든 fsds 모듈을 찾을 수 있게 해줍니다.
try:
    # Formula-Student-Driverless-Simulator 폴더가 FSDS라는 이름의 폴더 안에 있을 경우
    fsds_lib_path = os.path.join(os.path.expanduser("~"), "FSDS/Formula-Student-Driverless-Simulator", "python")
    sys.path.insert(0, fsds_lib_path)
    import fsds
except ImportError:
    # Formula-Student-Driverless-Simulator 폴더가 홈 디렉토리에 바로 있을 경우
    fsds_lib_path = os.path.join(os.path.expanduser("~"), "Formula-Student-Driverless-Simulator", "python")
    sys.path.insert(0, fsds_lib_path)
    import fsds

# --- 메인 코드 ---
def main():
    """
    시뮬레이터에 연결하여 차량을 리셋합니다.
    """
    try:
        # FSDS 클라이언트에 연결
        client = fsds.FSDSClient()
        client.confirmConnection()
        print("시뮬레이터에 성공적으로 연결되었습니다.")

        # 차량을 초기 위치로 리셋
        print("차량을 초기 위치로 리셋하는 중...")
        client.reset()
        print("✅ 리셋 완료.")

    except Exception as e:
        print(f"오류 발생: {e}")
        print("시뮬레이터가 실행 중인지 확인해주세요.")

if __name__ == '__main__':
    main()