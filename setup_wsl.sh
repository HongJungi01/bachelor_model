#!/bin/bash
# setup_wsl.sh ??WSL2 Ubuntu 22.04 ?섍꼍?먯꽌 RTAB-Map + 而ㅼ뒪? ?뚯씠?꾨씪??鍮뚮뱶
# ?ъ슜踰? bash setup_wsl.sh
set -euo pipefail

echo "=== [1/6] apt ?⑦궎吏 ?낅뜲?댄듃 ==="
sudo apt-get update
sudo apt-get upgrade -y

echo "=== [2/6] 鍮뚮뱶 ?꾧뎄 + 湲곕낯 ?섏〈???ㅼ튂 ==="
sudo apt-get install -y \
    build-essential cmake git pkg-config \
    libopencv-dev \
    libpcl-dev \
    libtbb-dev \
    zlib1g-dev \
    liboctomap-dev \
    libproj-dev \
    libsqlite3-dev \
    libgtsam-dev \
    libg2o-dev

echo "=== [3/6] Intel RealSense SDK apt ??μ냼 ?깅줉 ==="
if ! apt-cache policy 2>/dev/null | grep -q "packages.microsoft.com"; then
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
        | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/librealsense.list
    sudo apt-get update
fi

echo "=== [4/6] librealsense2 ?ㅼ튂 ==="
sudo apt-get install -y librealsense2-dev librealsense2-utils

echo "=== [5/6] RTAB-Map ?뚯뒪 鍮뚮뱶 ==="
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RTABMAP_SRC="${SCRIPT_DIR}/rtabmap"
if [ ! -d "${RTABMAP_SRC}" ]; then
    echo "ERROR: rtabmap ?뚯뒪媛 ${RTABMAP_SRC} ???놁뒿?덈떎."
    echo "  cp -r /mnt/c/dev/rtabmap ~/dev/rtabmap  濡?蹂듭궗 ???ㅼ떆 ?ㅽ뻾?섏꽭??"
    exit 1
fi

mkdir -p "${RTABMAP_SRC}/build"
cd "${RTABMAP_SRC}/build"

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DWITH_QT=OFF \
    -DWITH_REALSENSE2=ON \
    -DBUILD_APP=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_EXAMPLES=OFF

echo "Building RTAB-Map with $(nproc) threads..."
make -j$(nproc)
sudo make install
sudo ldconfig
echo "RTAB-Map installed to /usr/local"

echo "=== [6/6] 而ㅼ뒪? ?뚯씠?꾨씪??鍮뚮뱶 ==="
PIPELINE_SRC="${SCRIPT_DIR}/rtabmap_2d_pipeline"
if [ ! -d "${PIPELINE_SRC}" ]; then
    echo "ERROR: rtabmap_2d_pipeline ?뚯뒪媛 ${PIPELINE_SRC} ???놁뒿?덈떎."
    exit 1
fi

mkdir -p "${PIPELINE_SRC}/build"
cd "${PIPELINE_SRC}/build"

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo ""
echo "========================================="
echo " 鍮뚮뱶 ?꾨즺!"
echo " ?뚯씠?꾨씪???ㅽ뻾: ${PIPELINE_SRC}/build/rtabmap_2d_pipeline"
echo "========================================="
