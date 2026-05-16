FROM ubuntu:24.04

ARG TARGETOS
ARG TARGETARCH
ARG PFACES_RELEASE_TAG=Release_1.4.0d

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV PYTHONUNBUFFERED=1
ENV MPLBACKEND=Agg

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    build-essential \
    ca-certificates \
    clinfo \
    cmake \
    ffmpeg \
    git \
    ocl-icd-libopencl1 \
    ocl-icd-opencl-dev \
    opencl-clhpp-headers \
    opencl-headers \
    pocl-opencl-icd \
    python3 \
    python3-pip \
    unzip \
    wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --break-system-packages --no-cache-dir -r /tmp/requirements.txt

RUN mkdir -p /opt/pfaces && \
    cd /opt/pfaces && \
    case "${TARGETOS:-linux}/${TARGETARCH:-amd64}" in \
        linux/amd64) pf_asset="pFaces-1.4-Ubuntu24.04.zip" ;; \
        *) \
            echo "ERROR: Unsupported Docker target ${TARGETOS:-unknown}/${TARGETARCH:-unknown} for pFaces 1.4." >&2; \
            echo "Hugging Face Space must run on linux/amd64 hardware for this build." >&2; \
            exit 1 ;; \
    esac && \
    wget -q "https://github.com/parallall/pFaces/releases/download/${PFACES_RELEASE_TAG}/${pf_asset}" && \
    unzip -q "${pf_asset}" && \
    rm -f "${pf_asset}" && \
    ln -sf /opt/pfaces/bin/pfaces /usr/local/bin/pfaces

RUN mkdir -p /etc/OpenCL/vendors && \
    printf "libnvidia-opencl.so.1\n" > /etc/OpenCL/vendors/nvidia.icd

ENV PFACES_SDK_ROOT=/opt/pfaces/pfaces-sdk
ENV PATH=/opt/pfaces/bin:${PATH}

COPY . /app

RUN chmod +x /app/project/build.sh /app/project/run_threshold_synthesis.sh && \
    /app/project/build.sh

RUN useradd -m -u 1001 appuser && chown -R appuser:appuser /app
USER appuser

ENV GRADIO_SERVER_NAME=0.0.0.0
ENV GRADIO_SERVER_PORT=7860
EXPOSE 7860

CMD ["python3", "/app/app.py"]
