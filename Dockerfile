FROM ubuntu:24.04

ARG TARGETOS
ARG TARGETARCH
ARG PFACES_RELEASE_TAG=Release_1.4.0d

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    build-essential \
    ca-certificates \
    clinfo \
    cmake \
    ffmpeg \
    git \
    libcpprest-dev \
    libgl1 \
    libglib2.0-0 \
    libnlopt-cxx-dev \
    libssl-dev \
    ocl-icd-libopencl1 \
    ocl-icd-opencl-dev \
    opencl-clhpp-headers \
    opencl-headers \
    pkg-config \
    pocl-opencl-icd \
    python3 \
    python3-pip \
    unzip \
    wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY docker/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --break-system-packages --no-cache-dir -r /tmp/requirements.txt

RUN mkdir -p /opt/pfaces && \
    cd /opt/pfaces && \
    case "${TARGETOS:-linux}/${TARGETARCH:-amd64}" in \
        linux/amd64) pf_asset="pFaces-1.4-Ubuntu24.04.zip" ;; \
        *) \
            echo "ERROR: Unsupported Docker target ${TARGETOS:-unknown}/${TARGETARCH:-unknown} for pFaces 1.4." >&2; \
            echo "Use --platform linux/amd64 (for example on Apple Silicon)." >&2; \
            exit 1 ;; \
    esac && \
    wget -q "https://github.com/parallall/pFaces/releases/download/${PFACES_RELEASE_TAG}/${pf_asset}" && \
    unzip -q "${pf_asset}" && \
    rm -f "${pf_asset}" && \
    ln -sf /opt/pfaces/bin/pfaces /usr/local/bin/pfaces

RUN mkdir -p /etc/OpenCL/vendors && \
    printf "libnvidia-opencl.so.1\n" > /etc/OpenCL/vendors/nvidia.icd

RUN git clone --depth 1 https://github.com/nicolapiccinelli/libmpc.git /opt/libmpc-src && \
    git -C /opt/libmpc-src fetch --depth 1 origin d2c0352ced8e41d92f9d658a3c8d91693920f261 && \
    git -C /opt/libmpc-src checkout d2c0352ced8e41d92f9d658a3c8d91693920f261 && \
    sed -i '/#include <Eigen\/Sparse>/a #include <optional>' /opt/libmpc-src/include/mpc/Types.hpp

ENV PFACES_SDK_ROOT=/opt/pfaces/pfaces-sdk
ENV FETCHCONTENT_SOURCE_DIR_LIBMPC=/opt/libmpc-src
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV MPLBACKEND=Agg
ENV PYTHONUNBUFFERED=1

COPY . /workspace

RUN chmod +x /workspace/build.sh && \
    chmod +x /workspace/run_threshold_synthesis.sh && \
    chmod +x /workspace/run_two_oncoming_threshold_rt.sh && \
    chmod +x /workspace/tools/rt_controller/scripts/run_two_oncoming_threshold_rt.sh && \
    chmod +x /workspace/docker/run_all_videos.sh

RUN /workspace/build.sh
RUN cmake -S /workspace/tools/rt_controller -B /workspace/tools/rt_controller/build_rt_threshold \
    -DCMAKE_BUILD_TYPE=Release -DUSE_PFACES_SDK=ON \
    -DFETCHCONTENT_SOURCE_DIR_LIBMPC=/opt/libmpc-src && \
    cmake --build /workspace/tools/rt_controller/build_rt_threshold -j"$(nproc)"

VOLUME ["/outputs"]

ENTRYPOINT ["/workspace/docker/run_all_videos.sh"]
    
