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

## Boundaries

- Kernel build must match the deployed module release and DTB.
- Boot carrier and kernel FAT are distributed through the releases
  repository, not through `apk`.
- No vendor firmware, calibration, NV, persist or userdata belongs here.