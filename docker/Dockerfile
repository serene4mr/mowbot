ARG MOWBOT_BASE_IMAGE
ARG MOWBOT_BASE_CUDA_IMAGE

# hadolint ignore=DL3006
FROM $MOWBOT_BASE_IMAGE AS rosdep-depend
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO

COPY setup-dev-env.sh ansible-galaxy-requirements.yaml amd64.env arm64.env /mowbot/
COPY ansible/ /mowbot/ansible/
COPY docker/scripts/cleanup_apt.sh /mowbot/cleanup_apt.sh
RUN chmod +x /mowbot/cleanup_apt.sh
COPY docker/scripts/resolve_rosdep_keys.sh /mowbot/resolve_rosdep_keys.sh
RUN chmod +x /mowbot/resolve_rosdep_keys.sh
WORKDIR /mowbot

RUN rm -f /etc/apt/apt.conf.d/docker-clean \
  && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' >/etc/apt/apt.conf.d/keep-cache
RUN --mount=type=ssh \
  --mount=type=cache,target=/var/cache/apt,sharing=locked \
  ./setup-dev-env.sh -y rosdep \
  && pip uninstall -y ansible ansible-core \
  && /mowbot/cleanup_apt.sh

# Generate install package lists
# ....

##########################################################################

# hadolint ignore=DL3006
FROM $MOWBOT_BASE_IMAGE AS main-dev
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

##########################################################################

# hadolint ignore=DL3006
FROM $MOWBOT_BASE_CUDA_IMAGE AS main-dev-cuda
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

##########################################################################

# hadolint ignore=DL3006
FROM $MOWBOT_BASE_IMAGE AS main
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

##########################################################################

# hadolint ignore=DL3006
FROM $MOWBOT_BASE_CUDA_IMAGE AS main-cuda
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
ARG LIB_DIR

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

