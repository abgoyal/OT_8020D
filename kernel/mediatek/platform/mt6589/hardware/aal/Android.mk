ifeq ($(MTK_AAL_SUPPORT),yes)

PRODUCT_COPY_FILES += \
    $(MTK_PATH_PLATFORM)/hardware/aal/inc/aal_lcm_cal.cfg:system/etc/aal_lcm_cal.cfg \
    $(MTK_PATH_PLATFORM)/hardware/aal/inc/aal_als_cal.cfg:system/etc/aal_als_cal.cfg \
    $(MTK_PATH_PLATFORM)/hardware/aal/inc/aal.cfg:system/etc/aal.cfg \
	$(MTK_PATH_PLATFORM)/hardware/aal/inc/mtk_aal_config.txt:system/etc/mtk_aal_config.txt

include $(call all-subdir-makefiles)
endif
