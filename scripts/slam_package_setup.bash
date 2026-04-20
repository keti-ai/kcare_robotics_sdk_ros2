#!/bin/bash

# 에러 발생 시 중단
set -e

# 1. 경로 설정 (스크립트 위치와 상관없이 프로젝트 루트 기준)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( dirname "$SCRIPT_DIR" )"
cd "$PROJECT_ROOT"

# 2. 아키텍처 확인
ARCH=$(uname -m)
PKG_DIR="src/third_party/slamware_pkgs"

if [ "$ARCH" = "x86_64" ]; then
    TARGET_FILE="slamware_ros2_sdk_linux-x86_64-gcc11.tar.gz"
elif [ "$ARCH" = "aarch64" ]; then
    TARGET_FILE="slamware_ros2_sdk_linux-aarch64-gcc11.tar.gz"
else
    echo "❌ Error: Unsupported architecture ($ARCH)"
    exit 1
fi

FULL_PATH="$PKG_DIR/$TARGET_FILE"

# 3. Git LFS 파일 실제 데이터 확인/다운로드
if [ -f "$FULL_PATH" ]; then
    if [ $(stat -c%s "$FULL_PATH") -lt 1000 ]; then
        echo "📥 Git LFS pointer detected. Pulling actual binary..."
        git lfs pull --include="$FULL_PATH"
    fi
else
    echo "❌ Error: $FULL_PATH not found."
    exit 1
fi

echo "📦 Extracting SDKs for $ARCH in $PKG_DIR..."

# 4. 압축 파일이 있는 경로($PKG_DIR)로 이동해서 작업
cd "$PKG_DIR"

# 기존에 풀려있던 폴더가 있다면 정리 (깔끔한 업데이트를 위해)
rm -rf slamware_sdk slamware_ros_sdk

# 현재 위치(PKG_DIR)에서 바로 압축 해제
tar -xvzf "$TARGET_FILE"

# 5. 복잡하게 얽힌 경로 내부에서 필요한 폴더만 현재 위치(PKG_DIR)로 꺼내기
# 압축 파일명이 폴더명이 되므로 와일드카드로 접근하여 src 내부 폴더를 꺼냄
echo "🚚 Organizing SDK folders..."
mv */src/slamware_sdk .
mv */src/slamware_ros_sdk .

# 6. 압축 해제로 생긴 나머지 불필요한 폴더(상위 폴더) 삭제
# 타겟 파일명으로 시작하는 디렉토리를 찾아 삭제합니다.
rm -rf slamware_ros2_sdk_linux-*/

echo "✅ Setup complete."
echo "📍 Contents in $PKG_DIR:"
ls -F