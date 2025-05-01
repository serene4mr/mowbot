# Containerized workloads for Mowbot

These containers are offered to simplify the development and deployment of Mowbot and its dependencies. This directory contains scripts to build and run the containers.

## Development containers

## Runtime containers

## Multi-stage Dockerfile structure

### `$BASE_IMAGE`

This is a base image of this Dockerfile. [`ros:humble-ros-base-jammy`](https://hub.docker.com/_/ros/tags?page=&page_size=&ordering=&name=humble-ros-base-jammy) will be given.

### `$MOWBOT_BASE_IMAGE` (from Dockerfile.base)

This stage performs only the basic setup required for all Autoware images.

### `$MOWBOT_BASE_CUDA_IMAGE` (from Dockerfile.base)

This stage is built on top of `$AUTOWARE_BASE_IMAGE` and adds the CUDA runtime environment and artifacts.

### `rosdep-depend`

The ROS dependency package list files will be generated.
These files will be used in the subsequent stages:

- `mowbot-devel`
- `mowbot`

By generating only the package list files and copying them to the subsequent stages, the dependency packages will not be reinstalled during the container build process unless the dependency packages change.

### `mowbot-devel`


### `mowbot`