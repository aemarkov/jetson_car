#!/bin/bash
WEB_DIR=$(realpath $(dirname $0)/../../web_control)
cd "$WEB_DIR"
python3 -m http.server 8080
