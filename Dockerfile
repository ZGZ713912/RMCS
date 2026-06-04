# 23.0 is the minimum docker engine version that required to build.
# If your docker engine lower than this version, please upgrade it or manually enable the BuildKit.



# Sysroot source images for rmcs-develop-full.
# CI overrides these with digest-pinned references. For local builds, build
# rmcs-base for both architectures first and pass --build-arg explicitly;
# see docs/zh-cn/build_docker_image.md.
ARG SYSROOT_IMAGE_AMD64=rmcs-base:latest
ARG SYSROOT_IMAGE_ARM64=rmcs-base:latest

# Base container, provides a runtime environment
FROM ros:jazzy AS rmcs-base
ARG TARGETARCH

ARG RMCS_INSTALL_OPENVINO=1
ARG RMCS_INSTALL_LIVOX_SDK=1
ARG RMCS_ROS_APT_URI=http://packages.ros.org/ros2/ubuntu
ARG RMCS_UBUNTU_APT_URI=https://ports.ubuntu.com/ubuntu-ports

# Change bash as default shell instead of sh
SHELL ["/bin/bash", "-c"]

# Set timezone and non-interactive mode
ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive

# Make long dependency installs more tolerant of flaky networks.
RUN printf 'Acquire::Retries "5";\nAcquire::http::Timeout "30";\nAcquire::https::Timeout "30";\n' \
    > /etc/apt/apt.conf.d/80-retries

# Allow the Ubuntu and ROS apt endpoints to be profile-specific.
RUN grep -Rrl 'ports.ubuntu.com/ubuntu-ports' /etc/apt | \
    xargs -r sed -Ei "s|https?://ports\\.ubuntu\\.com/ubuntu-ports|${RMCS_UBUNTU_APT_URI}|g" && \
    grep -Rrl 'packages.ros.org/ros2/ubuntu' /etc/apt | \
    xargs -r sed -Ei "s|https?://packages\\.ros\\.org/ros2/ubuntu|${RMCS_ROS_APT_URI}|g"

# Disable source repositories during image builds to avoid flaky deb-src index fetches.
RUN grep -Rrl '^[[:space:]]*deb-src[[:space:]]' /etc/apt | \
    xargs -r sed -i 's|^[[:space:]]*deb-src|# deb-src|g' && \
    grep -Rrl '^Types:[[:space:]].*deb-src' /etc/apt | \
    xargs -r sed -Ei 's/^Types:[[:space:]]*deb[[:space:]]+deb-src$/Types: deb/'

# Install command-line tools.
RUN set -eux; \
    restore_ros=0; \
    if [ -f /etc/apt/sources.list.d/ros2.sources ]; then \
        mv /etc/apt/sources.list.d/ros2.sources /tmp/ros2.sources.disabled; \
        restore_ros=1; \
    fi; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        vim wget curl unzip \
        zsh screen tmux \
        usbutils net-tools iputils-ping \
        ripgrep htop fzf \
        unison; \
    if [ "${restore_ros}" = "1" ]; then \
        mv /tmp/ros2.sources.disabled /etc/apt/sources.list.d/ros2.sources; \
    fi; \
    apt-get autoremove -y; \
    apt-get clean; \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Install common development libraries.
RUN set -eux; \
    restore_ros=0; \
    if [ -f /etc/apt/sources.list.d/ros2.sources ]; then \
        mv /etc/apt/sources.list.d/ros2.sources /tmp/ros2.sources.disabled; \
        restore_ros=1; \
    fi; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        libusb-1.0-0-dev \
        libeigen3-dev \
        libopencv-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libatlas-base-dev \
        libsuitesparse-dev \
        libceres-dev; \
    if [ "${restore_ros}" = "1" ]; then \
        mv /tmp/ros2.sources.disabled /etc/apt/sources.list.d/ros2.sources; \
    fi; \
    apt-get autoremove -y; \
    apt-get clean; \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Install ROS desktop dependencies and other large packages.
RUN set -eux; \
    for attempt in 1 2 3 4 5; do \
        if apt-get update; then \
            break; \
        fi; \
        if [ "${attempt}" -eq 5 ]; then \
            echo "apt-get update failed after ${attempt} attempts." >&2; \
            exit 1; \
        fi; \
        sleep $((attempt * 2)); \
    done; \
    apt-get install -y --no-install-recommends \
        gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
        ros-$ROS_DISTRO-rviz2 \
        ros-$ROS_DISTRO-foxglove-bridge \
        dotnet-sdk-8.0 \
        ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions ros-$ROS_DISTRO-pcl-msgs; \
    apt-get autoremove -y; \
    apt-get clean; \
    rm -rf /tmp/*


# Install openvino runtime when available for the target architecture.
RUN if [ "${RMCS_INSTALL_OPENVINO}" = "1" ]; then \
        wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
        apt-key add ./GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
        rm ./GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
        echo "deb https://apt.repos.intel.com/openvino ubuntu24 main" > /etc/apt/sources.list.d/intel-openvino.list && \
        apt-get update && \
        apt-get install -y --no-install-recommends openvino-2025.2.0 && \
        apt-get autoremove -y && apt-get clean && \
        rm -rf /var/lib/apt/lists/* /tmp/*; \
    else \
        echo "Skipping OpenVINO install for this development profile."; \
    fi

# Install Livox SDK when needed for the development profile.
RUN if [ "${RMCS_INSTALL_LIVOX_SDK}" = "1" ]; then \
        set -eux; \
        livox_dir="/tmp/Livox-SDK2"; \
        archive="/tmp/Livox-SDK2.tar.gz"; \
        for attempt in 1 2 3 4 5; do \
            rm -rf "${livox_dir}" "${archive}"; \
            if curl -fsSL --retry 5 --retry-delay 2 --retry-all-errors \
                https://codeload.github.com/Livox-SDK/Livox-SDK2/tar.gz/refs/heads/master \
                -o "${archive}"; then \
                break; \
            fi; \
            if [ "${attempt}" -eq 5 ]; then \
                echo "Failed to download Livox SDK after ${attempt} attempts." >&2; \
                exit 1; \
            fi; \
            sleep $((attempt * 2)); \
        done; \
        mkdir -p "${livox_dir}"; \
        tar -xzf "${archive}" -C /tmp; \
        mv /tmp/Livox-SDK2-master "${livox_dir}"; \
        sed -i '6iset(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-c++20-compat -include cstdint")' "${livox_dir}/CMakeLists.txt"; \
        mkdir -p "${livox_dir}/build"; \
        cd "${livox_dir}/build"; \
        cmake -DCMAKE_BUILD_TYPE=Release ..; \
        make -j; \
        make install; \
        rm -rf "${livox_dir}" "${archive}"; \
    else \
        echo "Skipping Livox SDK install for this development profile."; \
    fi

# Mount rmcs source and install dependencies
RUN --mount=type=bind,target=/rmcs_ws/src,source=rmcs_ws/src,readonly \
    set -eux; \
    if ! find /var/lib/apt/lists -name '*Packages*' -print -quit | grep -q .; then \
        for attempt in 1 2 3 4 5; do \
            if apt-get update; then \
                break; \
            fi; \
            if [ "${attempt}" -eq 5 ]; then \
                echo "apt-get update failed after ${attempt} attempts." >&2; \
                exit 1; \
            fi; \
            sleep $((attempt * 2)); \
        done; \
    fi; \
    rosdep install --from-paths /rmcs_ws/src --ignore-src -r -y; \
    apt-get autoremove -y; \
    apt-get clean; \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Developing container, works with devcontainer
FROM rmcs-base AS rmcs-develop
ARG TARGETARCH

ARG RMCS_DEV_PROFILE=linux

# Install develop tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    libc6-dev gcc-14 g++-14 \
    cmake make ninja-build \
    openssh-client \
    lsb-release software-properties-common gnupg sudo \
    python3-colorama python3-dpkt && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 50 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 50 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Install Node.js 24 LTS (required by agent CLIs)
RUN curl -fsSL https://deb.nodesource.com/setup_24.x | bash - && \
    apt-get install -y --no-install-recommends nodejs && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Install llvm-toolchain
ARG LLVM_VERSION=22
RUN mkdir -p /etc/apt/keyrings && \
    wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | gpg --dearmor -o /etc/apt/keyrings/apt.llvm.org.gpg && \
    chmod 644 /etc/apt/keyrings/apt.llvm.org.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/apt.llvm.org.gpg] https://apt.llvm.org/noble/ llvm-toolchain-noble-${LLVM_VERSION} main" \
    > /etc/apt/sources.list.d/llvm.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    libomp-${LLVM_VERSION}-dev \
    clang-${LLVM_VERSION} clangd-${LLVM_VERSION} clang-format-${LLVM_VERSION} clang-tidy-${LLVM_VERSION} \
    lldb-${LLVM_VERSION} lld-${LLVM_VERSION} llvm-${LLVM_VERSION} && \
    update-alternatives --install /usr/bin/clang clang /usr/bin/clang-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/lldb lldb /usr/bin/lldb-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/llvm-ar llvm-ar /usr/bin/llvm-ar-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/llvm-ranlib llvm-ranlib /usr/bin/llvm-ranlib-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/ld.lld ld.lld /usr/bin/ld.lld-${LLVM_VERSION} 50 && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

# Generate/load ssh key and setup unison
RUN --mount=type=bind,target=/tmp/.ssh,source=.ssh,readonly=false \
    cd /home/ubuntu && mkdir -p .ssh && \
    if [ ! -f "/tmp/.ssh/id_rsa" ]; then ssh-keygen -N "" -f "/tmp/.ssh/id_rsa"; fi && \
    cp -r /tmp/.ssh/* .ssh && \
    chown -R 1000:1000 .ssh && chmod 600 .ssh/id_rsa && \
    mkdir -p .unison && \
    echo 'confirmbigdel = false' >> ".unison/default.prf" && \
    chown -R 1000:1000 .unison

# Install latest neovim
RUN arch="$(dpkg --print-architecture)" && \
    case "$arch" in \
        amd64) nvim_arch="x86_64" ;; \
        arm64) nvim_arch="arm64" ;; \
        *) echo "Unsupported architecture: $arch" >&2; exit 1 ;; \
    esac && \
    curl -LO "https://github.com/neovim/neovim/releases/latest/download/nvim-linux-${nvim_arch}.tar.gz" && \
    rm -rf /opt/nvim && \
    tar -C /opt -xzf "nvim-linux-${nvim_arch}.tar.gz" && \
    mv "/opt/nvim-linux-${nvim_arch}" /opt/nvim && \
    rm "nvim-linux-${nvim_arch}.tar.gz"

# Change user
RUN chsh -s /bin/zsh ubuntu && \
    echo "ubuntu ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

# Precreate generic XDG-style parent directories for direct bind mounts under ubuntu's home.
RUN mkdir -p \
        /home/ubuntu/.agents \
        /home/ubuntu/.cache \
        /home/ubuntu/.config \
        /home/ubuntu/.local/share \
        /home/ubuntu/.local/state && \
    chown -R ubuntu:ubuntu /home/ubuntu/.agents /home/ubuntu/.cache /home/ubuntu/.config /home/ubuntu/.local
WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
ENV RMCS_DEV_PROFILE=${RMCS_DEV_PROFILE}
USER ubuntu

# Install oh my zsh, change theme to af-magic and setup environment of zsh
RUN git config --global http.proxy http://host.docker.internal:7890 && \
    git config --global https.proxy http://host.docker.internal:7890 && \
    sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    echo 'source ~/env_setup.zsh' >> ~/.zshrc && \
    echo 'export PATH="${PATH}:/opt/nvim/bin"' >> ~/.zshrc && \
    echo 'export PATH="${PATH}:${RMCS_PATH}/.script"' >> ~/.zshrc

# Copy environment setup scripts
COPY --chown=1000:1000 .script/template/env_setup.bash env_setup.bash
COPY --chown=1000:1000 .script/template/env_setup.zsh env_setup.zsh

FROM --platform=linux/amd64 ${SYSROOT_IMAGE_AMD64} AS rmcs-sysroot-amd64
FROM --platform=linux/arm64 ${SYSROOT_IMAGE_ARM64} AS rmcs-sysroot-arm64

# Developing container with cross toolchains and opposite-arch sysroot.
FROM rmcs-develop AS rmcs-develop-full
ARG TARGETARCH

USER root

RUN apt-get update && \
    case "${TARGETARCH}" in \
        amd64) cross_triplet=aarch64-linux-gnu; cross_pkg_triplet=aarch64-linux-gnu ;; \
        arm64) cross_triplet=x86_64-linux-gnu; cross_pkg_triplet=x86-64-linux-gnu ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    apt-get install -y --no-install-recommends \
        "gcc-14-${cross_pkg_triplet}" "g++-14-${cross_pkg_triplet}" \
        "binutils-${cross_pkg_triplet}" && \
    update-alternatives --install "/usr/bin/${cross_triplet}-gcc" "${cross_triplet}-gcc" "/usr/bin/${cross_triplet}-gcc-14" 50 && \
    update-alternatives --install "/usr/bin/${cross_triplet}-g++" "${cross_triplet}-g++" "/usr/bin/${cross_triplet}-g++-14" 50 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

RUN mkdir -p /opt/sysroots && \
    case "${TARGETARCH}" in \
        amd64) mkdir -p /opt/sysroots/arm64 ;; \
        arm64) mkdir -p /opt/sysroots/amd64 ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac

RUN --mount=from=rmcs-sysroot-amd64,target=/mnt/sysroot-amd64,readonly \
    --mount=from=rmcs-sysroot-arm64,target=/mnt/sysroot-arm64,readonly \
    set -euo pipefail && \
    case "${TARGETARCH}" in \
        amd64) tar \
            --exclude='./dev/*' \
            --exclude='./proc/*' \
            --exclude='./sys/*' \
            --exclude='./run/*' \
            --exclude='./tmp/*' \
            -C /mnt/sysroot-arm64 -cf - . | tar -C /opt/sysroots/arm64 -xf - ;; \
        arm64) tar \
            --exclude='./dev/*' \
            --exclude='./proc/*' \
            --exclude='./sys/*' \
            --exclude='./run/*' \
            --exclude='./tmp/*' \
            -C /mnt/sysroot-amd64 -cf - . | tar -C /opt/sysroots/amd64 -xf - ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac

WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
USER ubuntu


# Runtime container, will automatically launch the main program
FROM rmcs-base AS rmcs-runtime

# Install runtime tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends tini openssh-server avahi-daemon orphan-sysvinit-scripts && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* && \
    echo 'Port 2022' >> /etc/ssh/sshd_config && \
    echo 'PermitRootLogin yes' >> /etc/ssh/sshd_config && \
    echo 'PasswordAuthentication no' >> /etc/ssh/sshd_config && \
    sed -i 's/#enable-dbus=yes/enable-dbus=no/g' /etc/avahi/avahi-daemon.conf

# Install oh my zsh, disable plugins and update, setup environment and set zsh as default shell
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/plugins=(git)/plugins=()/g' ~/.zshrc && \
    sed -i "s/# zstyle ':omz:update' mode disabled/zstyle ':omz:update' mode disabled/g" ~/.zshrc && \
    echo 'source ~/env_setup.zsh' >> ~/.zshrc && \
    echo 'export PATH=${PATH}:/rmcs_install/lib/rmcs_cli' >> ~/.zshrc && \
    chsh -s /bin/zsh root

RUN mkdir -p /workspaces/RMCS/rmcs_ws/log \
    /workspaces/RMCS/rmcs_ws/build \
    /workspaces/RMCS/rmcs_ws/install \
    && chown -R ubuntu:ubuntu /workspaces/RMCS/rmcs_ws

COPY --chown=root:root .script/set-robot /usr/local/bin/set-robot
COPY --chown=root:root .script/template/set-hostname /usr/local/bin/set-hostname

COPY --chown=root:root .script/template/entrypoint /entrypoint
COPY --chown=root:root .script/template/rmcs-service /etc/init.d/rmcs

COPY --from=rmcs-develop --chown=root:root /home/ubuntu/.ssh/id_rsa.pub /root/.ssh/authorized_keys

WORKDIR /root/
COPY --chown=root:root .script/template/env_setup.bash env_setup.bash
COPY --chown=root:root .script/template/env_setup.zsh env_setup.zsh

ENTRYPOINT ["tini", "--"]
CMD [ "/entrypoint" ]
