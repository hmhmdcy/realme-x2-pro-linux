# Realme X2 Pro Linux kernel fork

This repository is the Linux kernel source fork for the Realme X2 Pro
(RMX1931 / RMX1931CN, codename `samurai`, SM8150-AC).

## Public entry points

- System releases (boot/kernel/DTB/recovery):
  https://github.com/hmhmdcy/realme-x2-pro-releases
- Signed userspace APK repository:
  https://github.com/hmhmdcy/realme-x2-pro-apk
- Package recipes:
  https://github.com/hmhmdcy/realme-x2-pro-pmaports
- Integration/docs/patches:
  https://github.com/hmhmdcy/realme-x2-pro-edk2

## Branches

`stable` is the public snapshot of the standard ARM64 tree.  `master`
is the upstream Linux baseline; `samurai-bringup`, camera and USB
experiment branches are historical/development branches.

The current published kernel is `7.3.1-rmx1931-samurai-v731-usbwake-otg`
build `#8`.  Its source patch set is recorded in the integration
repository; do not assume that a newer upstream kernel or a different
module tree is compatible.

## Kernel #8 build inputs

The published release is `v2026.11.14-kernel8`.  Use the full
configuration committed as `samurai.config.kernel8`; it is the
generated `.config` from the build that produced the published Image
(`29c8d27c64b9f3bb11bbf6b94bcd211e88aa3ad4a8d31955b37b54d622d8a739`).

Reproduce the compile environment as closely as possible:

```sh
git clone --branch stable \
  https://github.com/hmhmdcy/realme-x2-pro-linux.git
cd realme-x2-pro-linux
make O=/fixed/linux-kernel8-build ARCH=arm64 \
  CROSS_COMPILE=aarch64-linux-gnu- LOCALVERSION=-usbwake-otg \
  KCONFIG_ALLCONFIG=$PWD/samurai.config.kernel8 \
  allnoconfig
make O=/fixed/linux-kernel8-build ARCH=arm64 \
  CROSS_COMPILE=aarch64-linux-gnu- LOCALVERSION=-usbwake-otg \
  -j"$(nproc)" Image modules
```

The release name is not fully represented by `CONFIG_LOCALVERSION` alone.
The build used `LOCALVERSION=-usbwake-otg` as an environment override, so
the resulting release is
`7.3.1-rmx1931-samurai-v731-usbwake-otg`.  Without that override the tree
builds `7.3.1-rmx1931-samurai-v731+`, which does not match the published
module archive.

The recorded compiler was GCC 15.2.0 with GNU ld 2.46; a different
toolchain can produce a different Image hash even with identical source
and configuration.  Use a fixed absolute `O=` path.  This is a
source/config reproduction recipe, not a promise of bit-identical
output.  Verify the result against the release manifest rather than
publishing a local Image as kernel #8.

The matching module archive is a separate release asset and must be
installed under `/lib/modules/7.3.1-rmx1931-samurai-v731-usbwake-otg`.

## Boundaries

- Kernel build must match the deployed module release and DTB.
- Boot carrier and kernel FAT are distributed through the releases
  repository, not through `apk`.
- No vendor firmware, calibration, NV, persist or userdata belongs here.