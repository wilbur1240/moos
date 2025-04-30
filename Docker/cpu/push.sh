#!/usr/bin/env bash

REPOSITORY="wilbur1240/moos-test"
TAG="ubuntu20.04"

IMG="${REPOSITORY}:${TAG}"

docker image push "${IMG}"
