#!/bin/bash

ROOT="$(cd $(dirname $0); pwd -P)"

sudo cp "${ROOT}"/udev/rules.d/* /usr/lib/udev/rules.d
