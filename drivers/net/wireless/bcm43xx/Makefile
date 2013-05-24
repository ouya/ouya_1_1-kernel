# bcm43xx
#	-DDHDTHREAD -DDHD_GPL -DDHD_SCHED -DDHD_DEBUG -DSDTEST -DBDC -DTOE    \
#	-DDHD_BCMEVENTS -DSHOW_EVENTS -DDONGLEOVERLAYS -DBCMDBG               \
#	-DCUSTOMER_HW2 -DOOB_INTR_ONLY -DHW_OOB       \
#	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
#	-DNEW_COMPAT_WIRELESS -DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT          \
#	-DKEEP_ALIVE -DCSCAN -DGET_CUSTOM_MAC_ENABLE -DPKT_FILTER_SUPPORT     \
#	-DEMBEDDED_PLATFORM -DENABLE_INSMOD_NO_FW_LOAD -DPNO_SUPPORT          \

DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                     \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DBCMFILEIMAGE            \
	-DDHDTHREAD -DDHD_DEBUG -DSDTEST -DBDC -DTOE                          \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DPROP_TXSTATUS -DBCMDBG                \
	-DCUSTOMER_HW2                               \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
	-DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT                                \
	-DKEEP_ALIVE -DGET_CUSTOM_MAC_ENABLE -DPKT_FILTER_SUPPORT             \
	-DEMBEDDED_PLATFORM -DPNO_SUPPORT                                     \
	-DDHD_USE_IDLECOUNT -DSET_RANDOM_MAC_SOFTAP -DROAM_ENABLE -DVSDB      \
	-DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST -DAMPDU_HOSTREORDER        \
	-DESCAN_RESULT_PATCH -DHT40_GO -DPASS_ARP_PACKET                      \
	-Idrivers/net/wireless/bcm43xx -Idrivers/net/wireless/bcm43xx/include

ifeq ($(CONFIG_BCM43XX_WIFI_CONTROL_FUNC),y)
DHDCFLAGS += -DCONFIG_WIFI_CONTROL_FUNC
else
DHDCFLAGS += -DCUSTOM_OOB_GPIO_NUM=2
endif

ifeq ($(CONFIG_BCM43XX_HW_OOB),y)
DHDCFLAGS += -DHW_OOB -DOOB_INTR_ONLY
else
DHDCFLAGS += -DSDIO_ISR_THREAD
endif

ifeq ($(CONFIG_BCM43XX_INSMOD_NO_FW_LOAD),y)
DHDCFLAGS += -DENABLE_INSMOD_NO_FW_LOAD
endif

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o dhd_sdio.o bcmwifi_channels.o bcmevent.o hndpmu.o   \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o linux_osl.o            \
	bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o

obj-$(CONFIG_BCM43XX) += bcm43xx.o
bcm43xx-objs += $(DHDOFILES)

ifeq ($(CONFIG_BCM43XX_WEXT),y)
bcm43xx-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP -DWL_WIRELESS_EXT -DUSE_IW
endif

ifneq ($(CONFIG_BCM43XX_CFG80211),)
bcm43xx-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o dhd_cfg80211.o
DHDCFLAGS += -DWL_CFG80211 -DWL_CFG80211_STA_EVENT -DWL_ENABLE_P2P_IF
DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-65
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=15
endif
ifneq ($(CONFIG_DHD_USE_SCHED_SCAN),)
DHDCFLAGS += -DWL_SCHED_SCAN
endif
EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCM43XX),m)
EXTRA_LDFLAGS += --strip-debug
endif
