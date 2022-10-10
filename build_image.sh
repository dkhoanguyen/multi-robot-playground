#!/bin/bash

docker build --file dockerfiles/Dockerfile \
             --tag mrp_dev_image:latest \
             .