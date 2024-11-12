#!/bin/bash

dbger_path=`find /sys/devices/pci* -name al_dbger`

if [ ! -n "$1" ]; then
    echo -t list > $dbger_path
else
    echo -t $1 > $dbger_path
fi
