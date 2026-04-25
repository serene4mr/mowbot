# hadolint global ignore=DL3006,DL3008
# Unified mowbot image: runtime (deploy) and devel (full toolchain).
# Build with docker/build.sh or docker-bake; set BASE_IMAGE via env/amd64.env or env/jetson-l4t-r36.4.env

ARG BASE_IMAGE=ros:humble-ros-base-jammy
ARG ROS_DISTRO=humble

# ============================================================
# runtime — minimal app image: no build_tools, no dev_tools (see ansible build_profile)
# ============================================================
FROM $BASE_IMAGE AS runtime
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ENV CCACHE_DIR="/root/.ccache"
ENV ROS_DISTRO=${ROS_DISTRO}

COPY setup-dev-env.sh ansible-galaxy-requirements.yaml /mowbot/
COPY env/ /mowbot/env/
RUN chmod +x /mowbot/setup-dev-env.sh
COPY ansible/ /mowbot/ansible/
COPY docker/scripts/cleanup_apt.sh /mowbot/cleanup_apt.sh
RUN chmod +x /mowbot/cleanup_apt.sh
COPY docker/scripts/cleanup_system.sh /mowbot/cleanup_system.sh
RUN chmod +x /mowbot/cleanup_system.sh
COPY docker/scripts/resolve_rosdep_keys.sh /mowbot/resolve_rosdep_keys.sh
RUN chmod +x /mowbot/resolve_rosdep_keys.sh
COPY docker/scripts/build_and_clean.sh /mowbot/build_and_clean.sh
RUN chmod +x /mowbot/build_and_clean.sh
COPY docker/scripts/resolve_pip_requirements.sh /mowbot/resolve_pip_requirements.sh
RUN chmod +x /mowbot/resolve_pip_requirements.sh
WORKDIR /mowbot

RUN rm -f /etc/apt/apt.conf.d/docker-clean \
  && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  gosu \
  ssh \
  apt-utils \
  && /mowbot/cleanup_apt.sh \
  && mkdir -p ~/.ssh \
  && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Ansible: build_profile=runtime, skip artifacts, no driver install (container)
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  /mowbot/setup-dev-env.sh -y --skip-artifacts --no-cuda-drivers --profile runtime --install-build-tools \
  && pip uninstall -y ansible ansible-core 2>/dev/null || true \
  && /mowbot/cleanup_apt.sh \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" > /etc/bash.bashrc

# Extra runtime lib (parity with previous base-cuda)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends libcufft10 \
  && /mowbot/cleanup_apt.sh || true

COPY src /mowbot/src
RUN /mowbot/resolve_rosdep_keys.sh /mowbot/src "${ROS_DISTRO}" \
  > /tmp/rosdep-main-depend-packages.txt

# hadolint ignore=SC2002
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  apt-get update \
  && cat /tmp/rosdep-main-depend-packages.txt | xargs apt-get install -y --no-install-recommends \
  && /mowbot/cleanup_apt.sh

# PyTorch cu124 index matches cuda_version in env/amd64.env (12.4).
# Using it avoids pip resolving PyPI torch which pulls nvidia-cudnn-cu1x as a
# separate large wheel, duplicating what ansible already installed.
RUN /mowbot/resolve_pip_requirements.sh src > /tmp/pip_requirements.txt \
  && pip install --no-cache-dir \
    --timeout 300 \
    --retries 5 \
    -r /tmp/pip_requirements.txt

# hadolint ignore=SC1091
RUN --mount=type=cache,target=${CCACHE_DIR} \
  source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && /mowbot/build_and_clean.sh ${CCACHE_DIR} /opt/mowbot

COPY docker/etc/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

# ============================================================
# devel — runtime + build_tools, dev_tools, CUDA/TRT devel packages
# ============================================================
FROM runtime AS devel
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  /mowbot/setup-dev-env.sh -y --skip-artifacts --no-cuda-drivers --profile devel \
  && pip uninstall -y ansible ansible-core 2>/dev/null || true \
  && /mowbot/cleanup_apt.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
