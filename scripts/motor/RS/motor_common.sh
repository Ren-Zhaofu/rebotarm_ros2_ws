#!/usr/bin/env bash

RS_MOTOR_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=motor_config.sh
source "${RS_MOTOR_SCRIPT_DIR}/motor_config.sh"

rs_detect_interface() {
    local path interface type carrier default_interface
    local -a candidates=()

    default_interface=$(ip route show default 2>/dev/null | awk 'NR == 1 { print $5 }')
    for path in /sys/class/net/*; do
        [[ -e $path ]] || continue
        interface=${path##*/}
        type=$(<"${path}/type")
        carrier=$(cat "${path}/carrier" 2>/dev/null || printf '0')
        [[ $type == 1 && $carrier == 1 ]] || continue
        [[ ! -d "${path}/wireless" && $interface != "$default_interface" ]] || continue
        candidates+=("$interface")
    done

    case ${#candidates[@]} in
        1) printf '%s\n' "${candidates[0]}" ;;
        0)
            echo "No connected dedicated Ethernet interface was detected for RS EtherCAT." >&2
            return 1
            ;;
        *)
            echo "Multiple RS EtherCAT interface candidates: ${candidates[*]}" >&2
            echo "Set RS_ETHERCAT_INTERFACE in ${RS_MOTOR_SCRIPT_DIR}/motor_config.sh." >&2
            return 1
            ;;
    esac
}

rs_resolve_interface() {
    local interface=${RS_ETHERCAT_INTERFACE:-auto}
    RS_MOTOR_ARGS=("$@")

    if [[ ${RS_MOTOR_ARGS[0]:-} == "--interface" ]]; then
        if [[ -z ${RS_MOTOR_ARGS[1]:-} ]]; then
            echo "--interface requires an Ethernet interface name." >&2
            return 2
        fi
        interface=${RS_MOTOR_ARGS[1]}
        RS_MOTOR_ARGS=("${RS_MOTOR_ARGS[@]:2}")
    fi

    if [[ $interface == "auto" ]]; then
        interface=$(rs_detect_interface) || return 2
    elif [[ -z $interface ]]; then
        echo "RS_ETHERCAT_INTERFACE is empty." >&2
        return 2
    fi
    if [[ ! -d /sys/class/net/$interface ]]; then
        echo "RS EtherCAT interface '$interface' does not exist." >&2
        return 2
    fi
    RS_MOTOR_INTERFACE=$interface
}

rs_require_root() {
    if (( EUID != 0 )); then
        echo "RS EtherCAT raw network access requires root privileges." >&2
        echo "Run again with sudo -E to preserve EYOU_SDK_LIB." >&2
        return 2
    fi
}

rs_require_sdk() {
    local library
    if [[ -z ${EYOU_SDK_LIB:-} ]]; then
        echo "EYOU_SDK_LIB is not set." >&2
        echo "Export the directory containing the EYou SDK shared libraries." >&2
        return 2
    fi
    if [[ ${EYOU_SDK_LIB} != /* ]]; then
        echo "EYOU_SDK_LIB must be an absolute path." >&2
        return 2
    fi
    for library in libsoem.so libcyhcs_log.so libeu_ethercat.so; do
        if [[ ! -f ${EYOU_SDK_LIB}/${library} ]]; then
            echo "Missing EYou SDK library: ${EYOU_SDK_LIB}/${library}" >&2
            return 2
        fi
    done
}
