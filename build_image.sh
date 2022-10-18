#!/bin/bash

docker build --file dockerfiles/Dockerfile.aarch64 \
             --tag mrp_dev_image:latest \
             .