#!/usr/bin/env bash
set -ex
if [ -z "$1" ]; then
	echo "design must be specified"
	exit 1
fi
rm -rf "$1/runs/latest"
docker run --rm -v ${OPENLANE_ROOT}:/openlane -v ${PDK_ROOT}:${PDK_ROOT} -v `pwd`:/work -e PDK_ROOT=${PDK_ROOT} -e PDK=sky130A -u `id -u`:`id -g` ${OPENLANE_IMAGE_NAME} /bin/bash -c "./flow.tcl -overwrite -ignore_mismatches -design /work/openlane/$1 -run_path /work/openlane/$1/runs -tag latest"
mkdir -p gds
mkdir -p lef
mkdir -p lib
cp openlane/$1/runs/latest/results/final/lef/$1.lef lef/
cp openlane/$1/runs/latest/results/final/gds/$1.gds gds/
cp openlane/$1/runs/latest/results/final/lib/$1.lib lib/
mkdir -p verilog/gl
cp openlane/$1/runs/latest/results/final/verilog/gl/$1.v verilog/gl/
mkdir -p signoff/$1/openlane-signoff/spef
cp openlane/$1/runs/latest/results/final/spef/multicorner/$1.* signoff/$1/openlane-signoff/spef
