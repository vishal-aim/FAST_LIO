FROM nvidia/cuda:12.2.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH="/opt/fastlio/build:/usr/local/lib:${LD_LIBRARY_PATH:-}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    libboost-all-dev \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    libzstd-dev \
    liblz4-dev \
    zenity \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/fastlio
COPY . /opt/fastlio

WORKDIR /opt/fastlio/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release \
    && cmake --build . -j$(nproc) \
    && ln -s /opt/fastlio/build/fastlio_mcap /usr/local/bin/fastlio_mcap \
    && chmod -R a+rX /opt/fastlio

WORKDIR /opt/fastlio

ENTRYPOINT ["/usr/local/bin/fastlio_mcap"]
CMD ["--help"]
