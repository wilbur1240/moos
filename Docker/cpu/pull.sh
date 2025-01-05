#!/usr/bin/env bash

REPOSITORY="laipi1240/moos-test"
TAG="ubuntu22.04"

IMG="${REPOSITORY}:${TAG}"

docker pull "${IMG}"
