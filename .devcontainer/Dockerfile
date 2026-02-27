ARG VARIANT=bookworm-slim
FROM debian:${VARIANT}
ENV DEBIAN_FRONTEND=noninteractive
ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

# Arguments
ARG CONTAINER_USER=esp
ARG CONTAINER_GROUP=esp
ARG ESP_BOARD=all
ARG GITHUB_TOKEN

# Install dependencies
RUN apt-get update \
    && apt-get install -y pkg-config curl gcc clang libudev-dev unzip xz-utils libsdl2-dev \
    git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/* /tmp/library-scripts

# Set users
RUN adduser --disabled-password --gecos "" -u 1000 ${CONTAINER_USER}
RUN usermod -aG $(getent group 27  | cut -d: -f1) ${CONTAINER_USER}
USER ${CONTAINER_USER}
WORKDIR /home/${CONTAINER_USER}

# Install rustup
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- \
     -y --profile default

RUN curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh

# Update envs
ENV PATH=${PATH}:/home/${CONTAINER_USER}/.cargo/bin

# Install esp specific crates
RUN cargo install espup cargo-espflash espflash ldproxy trunk

# Install Xtensa Rust
RUN espup install\
  --targets "${ESP_BOARD}" \
  --log-level debug \
  --export-file /home/${CONTAINER_USER}/export-esp.sh  
  
  
# Set default toolchain
RUN rustup target add wasm32-unknown-unknown

# Activate ESP environment
RUN echo "source /home/${CONTAINER_USER}/export-esp.sh" >> /home/${CONTAINER_USER}/.zshrc \
  && echo "source /home/${CONTAINER_USER}/export-esp.sh" >> /home/${CONTAINER_USER}/.bashrc

CMD [ "/bin/bash" ]
