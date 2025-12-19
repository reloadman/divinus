#!/bin/sh

# Divinus build helper (offline, fixed toolchain list)
# Usage:
#   ./build.sh [PLATFORM] [debug]
# If PLATFORM is omitted, builds all toolchains from TOOLCHAIN_PAIRS.
# Toolchains are expected to be already unpacked.

set -e

TOOLCHAIN_ROOT="${TOOLCHAIN_ROOT:-$HOME/openipc/toolchain}"
NPROC=$(command -v nproc >/dev/null 2>&1 && nproc || sysctl -n hw.ncpu || echo 1)

TOOLCHAIN_PAIRS="
  sigmastar-infinity6b0:arm-openipc-linux-musleabihf
  goke-gk7205v200:arm-openipc-linux-musleabi
  hisilicon-hi3516ev200:arm-openipc-linux-musleabi
  rockchip-rv1106:arm-openipc-linux-gnueabihf
  sigmastar-infinity6c:arm-openipc-linux-musleabihf
  sigmastar-infinity6e:arm-openipc-linux-gnueabihf
  allwinner-v85x:arm-openipc-linux-musleabihf
  novatek-nt9856x:arm-openipc-linux-musleabihf
  fullhan-fh8852v200:arm-openipc-linux-musleabi
  ingenic-t31:mipsel-openipc-linux-musl
  ingenic-t40:mipsel-openipc-linux-musl
"

if [ "$2" = "debug" ]; then
    OPT="-DDEBUG -gdwarf-3"
    LTO=0
else
    OPT="-Os -pipe -s"
    LTO=1
fi

build_one() {
    tc="$1"
    comp="$2"
    plat_name="${tc#*-}"

    echo "==> Building for ${tc} (${comp})"

    dir="${TOOLCHAIN_ROOT}/${tc}"
    cc="${dir}/bin/${comp}-gcc"
    strip_bin="${dir}/bin/${comp}-strip"

    if [ ! -d "$dir" ]; then
        echo "Toolchain directory not found: $dir" >&2
        return 1
    fi
    if [ ! -x "$cc" ]; then
        echo "Compiler not found: $cc" >&2
        return 1
    fi

    make -C src clean CC="$cc"
    make -j "$NPROC" -C src CC="$cc" OPT="$OPT" LTO="$LTO"

    [ -x "$strip_bin" ] && "$strip_bin" divinus 2>/dev/null || true

    mkdir -p builds
    cp -f divinus "builds/divinus.${plat_name}"
    echo "✅ builds/divinus.${plat_name}"
}

run_platforms() {
    target="$1"
    found=0
    for pair in $TOOLCHAIN_PAIRS; do
        IFS=':' read -r tc comp <<EOF
$pair
EOF
        if [ -z "$target" ] || [ "$target" = "$tc" ]; then
            build_one "$tc" "$comp" && found=1 || return 1
        fi
    done
    if [ -n "$target" ] && [ $found -eq 0 ]; then
        echo "Platform not found: $target" >&2
        return 1
    fi
}

run_platforms "$1"
