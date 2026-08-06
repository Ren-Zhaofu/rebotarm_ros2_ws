#!/usr/bin/env bash

# Use "auto" to detect one connected dedicated Ethernet interface.
RS_ETHERCAT_INTERFACE="${RS_ETHERCAT_INTERFACE:-auto}"

# Directory containing libsoem.so, libcyhcs_log.so, and libeu_ethercat.so.
# Export EYOU_SDK_LIB before running an RS SDK command.
EYOU_SDK_LIB="${EYOU_SDK_LIB:-}"
