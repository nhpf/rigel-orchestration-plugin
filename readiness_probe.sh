#!/bin/bash
if [ -f /tmp/ready ]; then
    exit 0
else
    exit 1
fi
