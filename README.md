lk-ef65l
========
for ef65l:

make msm8660_surf EMMC_BOOT=1 



boot flow:
POWER : boot from emmc
VOL UP + POWER : boot from TF card (need extended 'boot' partion on the card), But if no TF card boot from emmc, if no 'boot' partion boot into fastboot
VOL DOWN + POWER : recovery
VOL UP + VOL DOWN + POWER : fastboot


