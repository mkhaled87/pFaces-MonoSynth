# Image
FROM --platform=linux/amd64 ubuntu:24.04

# update image
RUN apt-get update
RUN apt-get install -y tzdata
RUN apt-get -y upgrade

# install required libs/tools
RUN apt-get install -y cmake wget git unzip build-essential cmake libcpprest-dev opencl-c-headers opencl-clhpp-headers ocl-icd-opencl-dev clinfo oclgrind

# install pFaces 1.3
RUN mkdir pfaces && cd pfaces && wget https://github.com/parallall/pFaces/releases/download/Release_1.3.0d/pFaces-1.3.0-Ubuntu22.04.zip && unzip pFaces-1.3.0-Ubuntu22.04.zip && sh ./install.sh

# install a binary version of pFaces-MonoSynth (version that supports pFaces v1.3)
RUN git clone https://github.com/mkhaled87/pFaces-MonoSynth.git && \
    cd pFaces-MonoSynth && \
    export PFACES_SDK_ROOT=$PWD/../pfaces/pfaces-sdk/ && \
    sh build.sh
    