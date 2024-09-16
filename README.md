# android_kernel_lenovo_mt8765_P
Kernel for Lenovo Tab M7 (TB-7305X)

Looks like the source code's kernel is broken as it boots but problems opening most apps: SIGABRT from hwcomposer.mt6739.so

Didn't check on any GSI

Stock kernel flashable zip: https://github.com/mlm-games/android_kernel_lenovo_mt8765_P/releases/download/v1.2/original-boot-kernel.zip

this source will NOT produce a usable image, you will need to replace the DTBs with the ones from the stock kernel as the resulting image has completely different DTBs and does not boot ( from tb7304f kernel)
