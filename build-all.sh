#!/bin/bash

# This script build u-boot.imx for all imx6 based SolidRun machines.
# Usage -
# ./build.all - this will build all models
#   or
# ./build.all cubox-i2ultra - this will build u-boot for CuBox-i2ultra only


if [ "x$1" != "x" ]; then
	VECTOR=$1
else
	VECTOR="c1solo c1dl c1d c1q cubox-i1 cubox-i2 cubox-i2ultra cubox-i4pro"
	VECTOR="cubox-i1 cubox-i2 cubox-i2ultra cubox-i4pro"
fi

for i in $VECTOR; do
	echo "Building $i version"
	make clean
	echo "Running make mx6_${i}_config"
	make "mx6_${i}_config"
	if [ $? != 0 ]; then
		echo "Error building $i version"
		exit
	fi
	make -j8
        if [ $? != 0 ]; then
                echo "Error building $i version"
                exit
        fi
        make -j8
	cp u-boot.imx u-boot_mx6_$i.imx
done
