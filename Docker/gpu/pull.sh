#!/usr/bin/env bash

REPOSITORY="wilbur1240/moos-test"
TAG="ubuntu20.04-gpu"

IMG="${REPOSITORY}:${TAG}"

docker pull "${IMG}"
