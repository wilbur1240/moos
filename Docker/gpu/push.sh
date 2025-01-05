#!/usr/bin/env bash

REPOSITORY="laipi1240/moos-test"
TAG="ubuntu22.04-gpu"

IMG="${REPOSITORY}:${TAG}"

docker image push "${IMG}"
