/*
 * linux/sound/soc/codecs/tlv320aic326x.c
 *
 * Copyright (C) 2011 Texas Instruments Inc.,
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * The TLV320AIC3262 is a flexible, low-power, low-voltage stereo audio
 * codec with digital microphone inputs and programmable outputs.
 *
 * History:
 *
 * Rev 0.1   ASoC driver support    TI	20-01-2011
 *
 *		The AIC325x ASoC driver is ported for the codec AIC3262.
 * Rev 0.2   ASoC driver support    TI	21-03-2011
 *		The AIC326x ASoC driver is updated for linux 2.6.32 Kernel.
 * Rev 0.3   ASoC driver support    TI	   20-04-2011
 *		The AIC326x ASoC driver is ported to 2.6.35 omap4 kernel
 */

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <sound/jack.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/input.h>

#include <sound/tlv.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/mfd/tlv320aic3262-registers.h>
#include <linux/mfd/tlv320aic3262-core.h>
#include "aic3xxx_cfw.h"
#include "aic3xxx_cfw_ops.h"

#include "tlv320aic326x.h"
#include "aic3262_codec_ops.h"
#include "tlv320aic3262_default_fw.h"


#define SOC_DOUBLE_R_SX_TLV3262(xname, xreg_left, xreg_right, xshift,\
			xmin, xmax, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
	SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw_2r_sx, \
	.get = snd_soc_get_volsw_2r_sx, \
	.put = snd_soc_put_volsw_2r_sx_aic3262, \
	.private_value = (unsigned long) &(struct soc_mixer_control) \
			{.reg = xreg_left, \
	.rreg = xreg_right, .shift = xshift, \
	.min = xmin, .max = xmax} }

/*****************************************************************************
			 Macros
******************************************************************************

******************************************************************************
		  Function Prototype
******************************************************************************/

static int aic3262_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai);

static int aic3262_mute(struct snd_soc_dai *dai, int mute);

static int aic3262_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir);

static int aic3262_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt);

static int aic3262_dai_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
				unsigned int Fin, unsigned int Fout);

static int aic3262_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level);

static int aic3262_set_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int aic3262_set_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);

static int aic326x_adc_dsp_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event);

static long debug_level;
module_param(debug_level, long, 0);
MODULE_PARM_DESC(debug_level, "Debug level for printing");

/**
 * snd_soc_put_volsw_2r_sx - double with tlv and variable data size
 *		mixer put callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Returns 0 for success.
 */
int snd_soc_put_volsw_2r_sx_aic3262(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int mask = (1 << mc->shift) - 1;
	int min = mc->min;
	int ret;
	unsigned int val, valr;

	val = ((ucontrol->value.integer.value[0] + min) & 0xff);
	val &= mask;
	valr = ((ucontrol->value.integer.value[1] + min) & 0xff);
	valr &= mask;

	ret = 0;
	ret = snd_soc_update_bits_locked(codec, mc->reg, mask, val);
	if (ret < 0)
		return ret;
	ret = snd_soc_update_bits_locked(codec, mc->rreg, mask, valr);
	if (ret < 0)
		return ret;
	return 0;
}

static ssize_t debug_level_show(struct device *dev,
		struct device_attribute *attr,
		char *buf, size_t count)
{
	return sprintf(buf, "%ld\n", debug_level);
}

static ssize_t debug_level_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;

	ret = kstrtol(buf, 10, &debug_level);
	if (ret)
		return ret;
	return count;
}

static DEVICE_ATTR(debug_level, 0644, debug_level_show, debug_level_set);

static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -6350, 50, 0);
static const DECLARE_TLV_DB_SCALE(adc_vol_tlv, -1200, 50, 0);
static const DECLARE_TLV_DB_SCALE(spk_gain_tlv, 600, 600, 0);
static const DECLARE_TLV_DB_SCALE(output_gain_tlv, -600, 100, 1);
static const DECLARE_TLV_DB_SCALE(micpga_gain_tlv, 0, 50, 0);
static const DECLARE_TLV_DB_SCALE(adc_fine_gain_tlv, -40, 10, 0);
static const DECLARE_TLV_DB_SCALE(beep_gen_volume_tlv, -6300, 100, 0);

/* Chip-level Input and Output CM Mode Controls */
static const char * const input_common_mode_text[] = {
	"0.9v", "0.75v"
};

static const char * const output_common_mode_text[] = {
	"Input CM", "1.25v", "1.5v", "1.65v"
};

static const struct soc_enum input_cm_mode =
SOC_ENUM_SINGLE(AIC3262_CM_REG, 2, 2, input_common_mode_text);

static const struct soc_enum output_cm_mode =
SOC_ENUM_SINGLE(AIC3262_CM_REG, 0, 4, output_common_mode_text);
/*
 *****************************************************************************
 * Structure Initialization
 *****************************************************************************
 */
static const struct snd_kcontrol_new aic3262_snd_controls[] = {
	/* Output */
#ifndef DAC_INDEPENDENT_VOL
	/* sound new kcontrol for PCM Playback volume control */

	SOC_DOUBLE_R_SX_TLV3262("PCM Playback Volume",
				AIC3262_DAC_LVOL, AIC3262_DAC_RVOL, 8,
				0xffffff81,
				0x30, dac_vol_tlv),
#endif
	/*HP Driver Gain Control */
	SOC_DOUBLE_R_SX_TLV3262("HeadPhone Driver Amplifier Volume",
				AIC3262_HPL_VOL, AIC3262_HPR_VOL, 6, 0xffffffb9,
				0xffffffce, output_gain_tlv),
	/*LO Driver Gain Control */
	SOC_DOUBLE_TLV("Speaker Amplifier Volume", AIC3262_SPK_AMP_CNTL_R4, 4,
			0, 5, 0, spk_gain_tlv),

	SOC_DOUBLE_R_SX_TLV3262("Receiver Amplifier Volume",
				AIC3262_REC_AMP_CNTL_R5, AIC3262_RAMPR_VOL, 6,
				0xffffffb9, 0xffffffd6, output_gain_tlv),

	SOC_DOUBLE_R_SX_TLV3262("PCM Capture Volume", AIC3262_LADC_VOL,
				AIC3262_RADC_VOL, 7, 0xffffff68, 0xffffffa8,
				adc_vol_tlv),

	SOC_DOUBLE_R_TLV("MicPGA Volume Control", AIC3262_MICL_PGA,
			 AIC3262_MICR_PGA, 0, 0x5F, 0, micpga_gain_tlv),

	SOC_DOUBLE_TLV("PCM Capture Fine Gain Volume", AIC3262_ADC_FINE_GAIN,
			4, 0, 5, 1, adc_fine_gain_tlv),

	SOC_DOUBLE("ADC channel mute", AIC3262_ADC_FINE_GAIN, 7, 3, 1, 0),

	SOC_DOUBLE("DAC MUTE", AIC3262_DAC_MVOL_CONF, 2, 3, 1, 1),

	SOC_SINGLE("RESET", AIC3262_RESET_REG, 0, 1, 0),

	SOC_SINGLE("DAC VOL SOFT STEPPING", AIC3262_DAC_MVOL_CONF, 0, 2, 0),

	SOC_SINGLE("DAC AUTO MUTE CONTROL", AIC3262_DAC_MVOL_CONF, 4, 7, 0),

	SOC_SINGLE("RIGHT MODULATOR SETUP", AIC3262_DAC_MVOL_CONF, 7, 1, 0),

	SOC_SINGLE("ADC Volume soft stepping", AIC3262_ADC_CHANNEL_POW,
		   0, 3, 0),

	SOC_SINGLE("Mic Bias ext independent enable", AIC3262_MIC_BIAS_CNTL,
		   7, 1, 0),

	SOC_SINGLE("MICBIAS EXT Power Level", AIC3262_MIC_BIAS_CNTL, 4, 3, 0),

	SOC_SINGLE("MICBIAS INT Power Level", AIC3262_MIC_BIAS_CNTL, 0, 3, 0),

	SOC_SINGLE("BEEP_GEN_EN", AIC3262_BEEP_CNTL_R1, 7, 1, 0),

	SOC_DOUBLE_R("BEEP_VOL_CNTL", AIC3262_BEEP_CNTL_R1,
		     AIC3262_BEEP_CNTL_R2, 0, 0x0F, 1),

	SOC_SINGLE("BEEP_MAS_VOL", AIC3262_BEEP_CNTL_R2, 6, 3, 0),

	SOC_SINGLE("DAC PRB Selection", AIC3262_DAC_PRB, 0, 26, 0),

	SOC_SINGLE("ADC PRB Selection", AIC3262_ADC_PRB, 0, 18, 0),

	SOC_ENUM("Input CM mode", input_cm_mode),

	SOC_ENUM("Output CM mode", output_cm_mode),

	SOC_SINGLE_EXT("FIRMWARE SET MODE", SND_SOC_NOPM, 0, 0xffff, 0,
			aic3262_set_mode_get, aic3262_set_mode_put),
};

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_dai |
 *	It is SoC Codec DAI structure which has DAI capabilities viz.,
 *	playback and capture, DAI runtime information viz. state of DAI
 *			and pop wait state, and DAI private data.
 *	The AIC3262 rates ranges from 8k to 192k
 *	The PCM bit format supported are 16, 20, 24 and 32 bits
 *----------------------------------------------------------------------------
 */
struct snd_soc_dai_ops aic3262_asi1_dai_ops = {
	.hw_params = aic3262_hw_params,
	.digital_mute = aic3262_mute,
	.set_sysclk = aic3262_set_dai_sysclk,
	.set_fmt = aic3262_set_dai_fmt,
	.set_pll = aic3262_dai_set_pll,
};

struct snd_soc_dai_ops aic3262_asi2_dai_ops = {
	.hw_params = aic3262_hw_params,
	.digital_mute = aic3262_mute,
	.set_sysclk = aic3262_set_dai_sysclk,
	.set_fmt = aic3262_set_dai_fmt,
	.set_pll = aic3262_dai_set_pll,
};

struct snd_soc_dai_ops aic3262_asi3_dai_ops = {
	.hw_params = aic3262_hw_params,
	.digital_mute = aic3262_mute,
	.set_sysclk = aic3262_set_dai_sysclk,
	.set_fmt = aic3262_set_dai_fmt,
	.set_pll = aic3262_dai_set_pll,
};

struct snd_soc_dai_driver aic326x_dai_driver[] = {
	{
	 .name = "aic326x-asi1",
	 .playback = {
		      .stream_name = "ASI1 Playback",
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = AIC3262_RATES,
		      .formats = AIC3262_FORMATS,
		      },
	 .capture = {
		     .stream_name = "ASI1 Capture",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = AIC3262_RATES,
		     .formats = AIC3262_FORMATS,
		     },
	 .ops = &aic3262_asi1_dai_ops,
	 },
	{
	 .name = "aic326x-asi2",
	 .playback = {
		      .stream_name = "ASI2 Playback",
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = AIC3262_RATES,
		      .formats = AIC3262_FORMATS,
		      },
	 .capture = {
		     .stream_name = "ASI2 Capture",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = AIC3262_RATES,
		     .formats = AIC3262_FORMATS,
		     },
	 .ops = &aic3262_asi2_dai_ops,
	 },
	{
	 .name = "aic326x-asi3",
	 .playback = {
		      .stream_name = "ASI3 Playback",
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = AIC3262_RATES,
		      .formats = AIC3262_FORMATS,
		      },
	 .capture = {
		     .stream_name = "ASI3 Capture",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = AIC3262_RATES,
		     .formats = AIC3262_FORMATS,
		     },
	 .ops = &aic3262_asi3_dai_ops,
	 },

};


static const unsigned int adc_ma_tlv[] = {
	TLV_DB_RANGE_HEAD(4),
	0, 29, TLV_DB_SCALE_ITEM(-1450, 500, 0),
	30, 35, TLV_DB_SCALE_ITEM(-2060, 1000, 0),
	36, 38, TLV_DB_SCALE_ITEM(-2660, 2000, 0),
	39, 40, TLV_DB_SCALE_ITEM(-3610, 5000, 0),
};

static const DECLARE_TLV_DB_SCALE(lo_hp_tlv, -7830, 50, 0);
static const struct snd_kcontrol_new mal_pga_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1L Switch", AIC3262_MA_CNTL, 5, 1, 0),
	SOC_DAPM_SINGLE_TLV("Left MicPGA Volume", AIC3262_LADC_PGA_MAL_VOL,
				0, 0x3f, 1, adc_ma_tlv),
};

static const struct snd_kcontrol_new mar_pga_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1R Switch", AIC3262_MA_CNTL, 4, 1, 0),
	SOC_DAPM_SINGLE_TLV("Right MicPGA Volume", AIC3262_RADC_PGA_MAR_VOL,
				0, 0x3f, 1, adc_ma_tlv),
};

/* Left HPL Mixer */
static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("MAL Switch", AIC3262_HP_AMP_CNTL_R1, 7, 1,
			0),
	SOC_DAPM_SINGLE("LDAC Switch", AIC3262_HP_AMP_CNTL_R1,
			5, 1, 0),
	SOC_DAPM_SINGLE_TLV("LOL-B1 Volume",
				AIC3262_HP_AMP_CNTL_R2, 0, 0x7f, 1, lo_hp_tlv),
};

/* Right HPR Mixer */
static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {
	SOC_DAPM_SINGLE_TLV("LOR-B1 Volume",
				AIC3262_HP_AMP_CNTL_R3, 0, 0x7f, 1, lo_hp_tlv),
	SOC_DAPM_SINGLE("LDAC Switch", AIC3262_HP_AMP_CNTL_R1,
			2, 1, 0),
	SOC_DAPM_SINGLE("RDAC Switch", AIC3262_HP_AMP_CNTL_R1,
			4, 1, 0),
	SOC_DAPM_SINGLE("MAR Switch", AIC3262_HP_AMP_CNTL_R1,
			6, 1, 0),
};

/* Left LOL Mixer */
static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("MAL Switch", AIC3262_LINE_AMP_CNTL_R2,
			7, 1, 0),
	SOC_DAPM_SINGLE("IN1L-B Switch", AIC3262_LINE_AMP_CNTL_R2,
			3, 1, 0),
	SOC_DAPM_SINGLE("LDAC Switch", AIC3262_LINE_AMP_CNTL_R1,
			7, 1, 0),
	SOC_DAPM_SINGLE("RDAC Switch", AIC3262_LINE_AMP_CNTL_R1,
			5, 1, 0),
};

/* Right LOR Mixer */
static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("LOL Switch", AIC3262_LINE_AMP_CNTL_R1,
			2, 1, 0),
	SOC_DAPM_SINGLE("RDAC Switch", AIC3262_LINE_AMP_CNTL_R1,
			6, 1, 0),
	SOC_DAPM_SINGLE("MAR Switch", AIC3262_LINE_AMP_CNTL_R2,
			6, 1, 0),
	SOC_DAPM_SINGLE("IN1R-B Switch", AIC3262_LINE_AMP_CNTL_R2,
			0, 1, 0),
};

/* Left SPKL Mixer */
static const struct snd_kcontrol_new spkl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("MAL Switch", AIC3262_SPK_AMP_CNTL_R1,
			7, 1, 0),
	SOC_DAPM_SINGLE_TLV("LOL Volume",
				AIC3262_SPK_AMP_CNTL_R2, 0, 0x7f, 1, lo_hp_tlv),
	SOC_DAPM_SINGLE("SPR_IN Switch", AIC3262_SPK_AMP_CNTL_R1, 2, 1, 0),
};

/* Right SPKR Mixer */
static const struct snd_kcontrol_new spkr_output_mixer_controls[] = {
	SOC_DAPM_SINGLE_TLV("LOR Volume",
				AIC3262_SPK_AMP_CNTL_R3, 0, 0x7f, 1, lo_hp_tlv),
	SOC_DAPM_SINGLE("MAR Switch",
			AIC3262_SPK_AMP_CNTL_R1, 6, 1, 0),
};

/* REC Mixer */
static const struct snd_kcontrol_new rec_output_mixer_controls[] = {
	SOC_DAPM_SINGLE_TLV("LOL-B2 Volume",
				AIC3262_RAMP_CNTL_R1, 0, 0x7f, 1, lo_hp_tlv),
	SOC_DAPM_SINGLE_TLV("IN1L Volume",
				AIC3262_IN1L_SEL_RM, 0, 0x7f, 1, lo_hp_tlv),
	SOC_DAPM_SINGLE_TLV("IN1R Volume",
				AIC3262_IN1R_SEL_RM, 0, 0x7f, 1, lo_hp_tlv),
	SOC_DAPM_SINGLE_TLV("LOR-B2 Volume",
				AIC3262_RAMP_CNTL_R2, 0, 0x7f, 1, lo_hp_tlv),
};

/* Left Input Mixer */
static const struct snd_kcontrol_new left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1L Switch", AIC3262_LMIC_PGA_PIN,
			6, 3, 0),
	SOC_DAPM_SINGLE("IN2L Switch", AIC3262_LMIC_PGA_PIN,
			4, 3, 0),
	SOC_DAPM_SINGLE("IN3L Switch", AIC3262_LMIC_PGA_PIN,
			2, 3, 0),
	SOC_DAPM_SINGLE("IN4L Switch", AIC3262_LMIC_PGA_PM_IN4,
			5, 1, 0),
	SOC_DAPM_SINGLE("IN1R Switch", AIC3262_LMIC_PGA_PIN,
			0, 3, 0),
	SOC_DAPM_SINGLE("IN2R Switch", AIC3262_LMIC_PGA_MIN,
			4, 3, 0),
	SOC_DAPM_SINGLE("IN3R Switch", AIC3262_LMIC_PGA_MIN,
			2, 3, 0),
	SOC_DAPM_SINGLE("IN4R Switch", AIC3262_LMIC_PGA_PM_IN4,
			4, 1, 0),
	SOC_DAPM_SINGLE("CM2L Switch", AIC3262_LMIC_PGA_MIN,
			0, 3, 0),
	SOC_DAPM_SINGLE("CM1L Switch", AIC3262_LMIC_PGA_MIN,
			6, 3, 0),
};

/* Right Input Mixer */
static const struct snd_kcontrol_new right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1R Switch", AIC3262_RMIC_PGA_PIN,
			6, 3, 0),
	SOC_DAPM_SINGLE("IN2R Switch", AIC3262_RMIC_PGA_PIN,
			4, 3, 0),
	SOC_DAPM_SINGLE("IN3R Switch", AIC3262_RMIC_PGA_PIN,
			2, 3, 0),
	SOC_DAPM_SINGLE("IN4R Switch", AIC3262_RMIC_PGA_PM_IN4,
			5, 1, 0),
	SOC_DAPM_SINGLE("IN2L Switch", AIC3262_RMIC_PGA_PIN,
			0, 3, 0),
	SOC_DAPM_SINGLE("IN1L Switch", AIC3262_RMIC_PGA_MIN,
			4, 3, 0),
	SOC_DAPM_SINGLE("IN3L Switch", AIC3262_RMIC_PGA_MIN,
			2, 3, 0),
	SOC_DAPM_SINGLE("IN4L Switch", AIC3262_RMIC_PGA_PM_IN4,
			4, 1, 0),
	SOC_DAPM_SINGLE("CM1R Switch", AIC3262_RMIC_PGA_MIN,
			6, 3, 0),
	SOC_DAPM_SINGLE("CM2R Switch", AIC3262_RMIC_PGA_MIN,
			0, 3, 0),
};

static const char * const asi1lin_text[] = {
	"Off", "ASI1 Left In", "ASI1 Right In", "ASI1 MonoMix In"
};

SOC_ENUM_SINGLE_DECL(asi1lin_enum, AIC3262_ASI1_DAC_OUT_CNTL, 6, asi1lin_text);

static const struct snd_kcontrol_new asi1lin_control =
SOC_DAPM_ENUM("ASI1LIN Route", asi1lin_enum);

static const char * const asi1rin_text[] = {
	"Off", "ASI1 Right In", "ASI1 Left In", "ASI1 MonoMix In"
};

SOC_ENUM_SINGLE_DECL(asi1rin_enum, AIC3262_ASI1_DAC_OUT_CNTL, 4, asi1rin_text);

static const struct snd_kcontrol_new asi1rin_control =
SOC_DAPM_ENUM("ASI1RIN Route", asi1rin_enum);

static const char * const asi2lin_text[] = {
	"Off", "ASI2 Left In", "ASI2 Right In", "ASI2 MonoMix In"
};

SOC_ENUM_SINGLE_DECL(asi2lin_enum, AIC3262_ASI2_DAC_OUT_CNTL, 6, asi2lin_text);

static const struct snd_kcontrol_new asi2lin_control =
SOC_DAPM_ENUM("ASI2LIN Route", asi2lin_enum);

static const char * const asi2rin_text[] = {
	"Off", "ASI2 Right In", "ASI2 Left In", "ASI2 MonoMix In"
};

SOC_ENUM_SINGLE_DECL(asi2rin_enum, AIC3262_ASI2_DAC_OUT_CNTL, 4, asi2rin_text);

static const struct snd_kcontrol_new asi2rin_control =
SOC_DAPM_ENUM("ASI2RIN Route", asi2rin_enum);

static const char * const asi3lin_text[] = {
	"Off", "ASI3 Left In", "ASI3 Right In", "ASI3 MonoMix In"
};

SOC_ENUM_SINGLE_DECL(asi3lin_enum, AIC3262_ASI3_DAC_OUT_CNTL, 6, asi3lin_text);

static const struct snd_kcontrol_new asi3lin_control =
SOC_DAPM_ENUM("ASI3LIN Route", asi3lin_enum);

static const char * const asi3rin_text[] = {
	"Off", "ASI3 Right In", "ASI3 Left In", "ASI3 MonoMix In"
};

SOC_ENUM_SINGLE_DECL(asi3rin_enum, AIC3262_ASI3_DAC_OUT_CNTL, 4, asi3rin_text);

static const struct snd_kcontrol_new asi3rin_control =
SOC_DAPM_ENUM("ASI3RIN Route", asi3rin_enum);

static const char * const dacminidspin1_text[] = {
	"ASI1 In", "ASI2 In", "ASI3 In", "ADC MiniDSP Out"
};

SOC_ENUM_SINGLE_DECL(dacminidspin1_enum, AIC3262_MINIDSP_DATA_PORT_CNTL, 4,
		     dacminidspin1_text);

static const struct snd_kcontrol_new dacminidspin1_control =
SOC_DAPM_ENUM("DAC MiniDSP IN1 Route", dacminidspin1_enum);

static const char * const dacminidspin2_text[] = {
	"ASI1 In", "ASI2 In", "ASI3 In"
};

SOC_ENUM_SINGLE_DECL(dacminidspin2_enum, AIC3262_MINIDSP_DATA_PORT_CNTL, 2,
		     dacminidspin2_text);

static const struct snd_kcontrol_new dacminidspin2_control =
SOC_DAPM_ENUM("DAC MiniDSP IN2 Route", dacminidspin2_enum);

static const char * const dacminidspin3_text[] = {
	"ASI1 In", "ASI2 In", "ASI3 In"
};

SOC_ENUM_SINGLE_DECL(dacminidspin3_enum, AIC3262_MINIDSP_DATA_PORT_CNTL, 0,
		     dacminidspin3_text);

static const struct snd_kcontrol_new dacminidspin3_control =
SOC_DAPM_ENUM("DAC MiniDSP IN3 Route", dacminidspin3_enum);

static const char * const adcdac_route_text[] = {
	"Off",
	"On",
};

SOC_ENUM_SINGLE_DECL(adcdac_enum, 0, 2, adcdac_route_text);

static const struct snd_kcontrol_new adcdacroute_control =
SOC_DAPM_ENUM_VIRT("ADC DAC Route", adcdac_enum);

static const char * const dout1_text[] = {
	"ASI1 Out",
	"DIN1 Bypass",
	"DIN2 Bypass",
	"DIN3 Bypass",
};

SOC_ENUM_SINGLE_DECL(dout1_enum, AIC3262_ASI1_DOUT_CNTL, 0, dout1_text);
static const struct snd_kcontrol_new dout1_control =
SOC_DAPM_ENUM("DOUT1 Route", dout1_enum);

static const char * const dout2_text[] = {
	"ASI2 Out",
	"DIN1 Bypass",
	"DIN2 Bypass",
	"DIN3 Bypass",
};

SOC_ENUM_SINGLE_DECL(dout2_enum, AIC3262_ASI2_DOUT_CNTL, 0, dout2_text);
static const struct snd_kcontrol_new dout2_control =
SOC_DAPM_ENUM("DOUT2 Route", dout2_enum);

static const char * const dout3_text[] = {
	"ASI3 Out",
	"DIN1 Bypass",
	"DIN2 Bypass",
	"DIN3 Bypass",
};

SOC_ENUM_SINGLE_DECL(dout3_enum, AIC3262_ASI3_DOUT_CNTL, 0, dout3_text);
static const struct snd_kcontrol_new dout3_control =
SOC_DAPM_ENUM("DOUT3 Route", dout3_enum);

static const char * const asi1out_text[] = {
	"Off",
	"ADC MiniDSP Out1",
	"ASI1In Bypass",
	"ASI2In Bypass",
	"ASI3In Bypass",
};

SOC_ENUM_SINGLE_DECL(asi1out_enum, AIC3262_ASI1_ADC_INPUT_CNTL,
		     0, asi1out_text);
static const struct snd_kcontrol_new asi1out_control =
SOC_DAPM_ENUM("ASI1OUT Route", asi1out_enum);

static const char * const asi2out_text[] = {
	"Off",
	"ADC MiniDSP Out1",
	"ASI1In Bypass",
	"ASI2In Bypass",
	"ASI3In Bypass",
	"ADC MiniDSP Out2",
};

SOC_ENUM_SINGLE_DECL(asi2out_enum, AIC3262_ASI2_ADC_INPUT_CNTL,
		     0, asi2out_text);
static const struct snd_kcontrol_new asi2out_control =
SOC_DAPM_ENUM("ASI2OUT Route", asi2out_enum);
static const char * const asi3out_text[] = {
	"Off",
	"ADC MiniDSP Out1",
	"ASI1In Bypass",
	"ASI2In Bypass",
	"ASI3In Bypass",
	"Reserved",
	"ADC MiniDSP Out3",
};

SOC_ENUM_SINGLE_DECL(asi3out_enum, AIC3262_ASI3_ADC_INPUT_CNTL,
		     0, asi3out_text);
static const struct snd_kcontrol_new asi3out_control =
SOC_DAPM_ENUM("ASI3OUT Route", asi3out_enum);
static const char * const asibclk_text[] = {
	"DAC_CLK",
	"DAC_MOD_CLK",
	"ADC_CLK",
	"ADC_MOD_CLK",
};

SOC_ENUM_SINGLE_DECL(asi1bclk_enum, AIC3262_ASI1_BCLK_N_CNTL, 0, asibclk_text);
static const struct snd_kcontrol_new asi1bclk_control =
SOC_DAPM_ENUM("ASI1_BCLK Route", asi1bclk_enum);

SOC_ENUM_SINGLE_DECL(asi2bclk_enum, AIC3262_ASI2_BCLK_N_CNTL, 0, asibclk_text);
static const struct snd_kcontrol_new asi2bclk_control =
SOC_DAPM_ENUM("ASI2_BCLK Route", asi2bclk_enum);
SOC_ENUM_SINGLE_DECL(asi3bclk_enum, AIC3262_ASI3_BCLK_N_CNTL, 0, asibclk_text);
static const struct snd_kcontrol_new asi3bclk_control =
SOC_DAPM_ENUM("ASI3_BCLK Route", asi3bclk_enum);

static const char * const adc_mux_text[] = {
	"Analog",
	"Digital",
};

SOC_ENUM_SINGLE_DECL(adcl_enum, AIC3262_ADC_CHANNEL_POW, 4, adc_mux_text);
SOC_ENUM_SINGLE_DECL(adcr_enum, AIC3262_ADC_CHANNEL_POW, 2, adc_mux_text);

static const struct snd_kcontrol_new adcl_mux =
SOC_DAPM_ENUM("Left ADC Route", adcl_enum);

static const struct snd_kcontrol_new adcr_mux =
SOC_DAPM_ENUM("Right ADC Route", adcr_enum);

/**
 * aic326x_hp_event: - To handle headphone related task before and after
 *			headphone powrup and power down
 * @w: pointer variable to dapm_widget
 * @kcontrol: mixer control
 * @event: event element information
 *
 * Returns 0 for success.
 */
static int aic326x_hp_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	int reg_mask = 0;
	int ret_wbits = 0;

	if (w->shift == 1)
		reg_mask = AIC3262_HPL_POWER_MASK;
	if (w->shift == 0)
		reg_mask = AIC3262_HPR_POWER_MASK;
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret_wbits = aic3262_wait_bits(w->codec->control_data,
					      AIC3262_HP_FLAG, reg_mask,
					      reg_mask, TIME_DELAY,
					      DELAY_COUNTER);
		if (!ret_wbits) {
			dev_err(w->codec->dev, "HP POST_PMU timedout\n");
			return -1;
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret_wbits = aic3262_wait_bits(w->codec->control_data,
					      AIC3262_HP_FLAG, reg_mask, 0,
					      TIME_DELAY, DELAY_COUNTER);
		if (!ret_wbits) {
			dev_err(w->codec->dev, "HP POST_PMD timedout\n");
			return -1;
		}
		break;
	default:
		BUG();
		return -EINVAL;
	}
	return 0;
}

/**
 *aic326x_dac_event: Headset popup reduction and powering up dsps together
 *			when they are in sync mode
 * @w: pointer variable to dapm_widget
 * @kcontrol: pointer to sound control
 * @event: event element information
 *
 * Returns 0 for success.
 */
static int aic326x_dac_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	int reg_mask = 0;
	int ret_wbits = 0;
	int run_state_mask;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(w->codec);
	int sync_needed = 0, non_sync_state = 0;
	int other_dsp = 0, run_state = 0;

	if (w->shift == 7) {
		reg_mask = AIC3262_LDAC_POWER_MASK;
		run_state_mask = AIC3262_COPS_MDSP_D_L;
	}
	if (w->shift == 6) {
		reg_mask = AIC3262_RDAC_POWER_MASK;
		run_state_mask = AIC3262_COPS_MDSP_D_R;
	}
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:

		ret_wbits = aic3262_wait_bits(w->codec->control_data,
					      AIC3262_DAC_FLAG, reg_mask,
					      reg_mask, TIME_DELAY,
					      DELAY_COUNTER);

		sync_needed = SYNC_STATE(aic3262);
		non_sync_state = DSP_NON_SYNC_MODE(aic3262->dsp_runstate);
		other_dsp = aic3262->dsp_runstate & AIC3262_COPS_MDSP_A;

		if (sync_needed && non_sync_state && other_dsp) {
			run_state = get_runstate(aic3262->codec->control_data);
			aic3262_dsp_pwrdwn_status(aic3262);
			aic3262_dsp_pwrup(aic3262, run_state);
		}
		aic3262->dsp_runstate |= run_state_mask;

		if (!ret_wbits) {
			dev_err(w->codec->dev, "DAC POST_PMU timedout\n");
			return -1;
		}
		break;
	case SND_SOC_DAPM_POST_PMD:

		ret_wbits = aic3262_wait_bits(w->codec->control_data,
					      AIC3262_DAC_FLAG, reg_mask, 0,
					      TIME_DELAY, DELAY_COUNTER);

		aic3262->dsp_runstate = (aic3262->dsp_runstate &
					 ~run_state_mask);
		if (!ret_wbits) {
			dev_err(w->codec->dev, "DAC POST_PMD timedout\n");
			return -1;
		}
		break;
	default:
		BUG();
		return -EINVAL;
	}
	return 0;
}

/**
 * aic326x_spk_event: Speaker related task before and after
 *			 headphone powrup and power down$
 * @w: pointer variable to dapm_widget,
 * @kcontrolr: pointer variable to sound control,
 * @event:	integer to event,
 *
 * Return value: 0 for success
 */
static int aic326x_spk_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	int reg_mask;

	if (w->shift == 1)
		reg_mask = AIC3262_SPKL_POWER_MASK;
	if (w->shift == 0)
		reg_mask = AIC3262_SPKR_POWER_MASK;
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mdelay(1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		mdelay(1);
		break;
	default:
		BUG();
		return -EINVAL;
	}
	return 0;
}

/**$
 * pll_power_on_event: provide delay after widget  power up
 * @w:  pointer variable to dapm_widget,
 * @kcontrolr: pointer variable to sound control,
 * @event:	integer to event,
 *
 * Return value: 0 for success
 */
static int pll_power_on_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	if (event == SND_SOC_DAPM_POST_PMU)
		mdelay(10);
	return 0;
}

/**
 * aic3262_set_mode_get: To get different mode of Firmware through tinymix
 * @kcontrolr: pointer to sound control,
 * ucontrol: pointer to control element value,
 *
 * Return value: 0 for success
 */
static int aic3262_set_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic3262_priv *priv_ds = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = ((priv_ds->cfw_p->cur_mode << 8)
					    | priv_ds->cfw_p->cur_cfg);

	return 0;
}

/**
 * aic3262_set_mode_put: To set different mode of Firmware through tinymix
 * @kcontrolr: pointer to sound control,
 * ucontrol: pointer to control element value,
 *
 * Return value: 0 for success
 */
static int aic3262_set_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic3262_priv *priv_ds = snd_soc_codec_get_drvdata(codec);

	int next_mode = 0, next_cfg = 0;
	int ret = 0;

	next_mode = (ucontrol->value.integer.value[0] >> 8);
	next_cfg = (ucontrol->value.integer.value[0]) & 0xFF;
	if (priv_ds == NULL)
		dev_err(codec->dev, "failed to load firmware\n");
	else
		ret = aic3xxx_cfw_setmode_cfg(priv_ds->cfw_p,
					      next_mode, next_cfg);

	return ret;
}

/**
 * aic326x_adc_dsp_event: To get DSP run state to perform synchronization
 * @w: pointer variable to dapm_widget
 * @kcontrol: pointer to sound control
 * @event: event element information
 *
 * Returns 0 for success.
 */
static int aic326x_adc_dsp_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	int run_state = 0;
	int non_sync_state = 0, sync_needed = 0;
	int other_dsp = 0;
	int run_state_mask = 0;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(w->codec);
	int reg_mask = 0;
	int ret_wbits = 0;

	if (w->shift == 7) {
		reg_mask = AIC3262_LADC_POWER_MASK;
		run_state_mask = AIC3262_COPS_MDSP_A_L;
	}
	if (w->shift == 6) {
		reg_mask = AIC3262_RADC_POWER_MASK;
		run_state_mask = AIC3262_COPS_MDSP_A_R;
	}
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret_wbits = aic3262_wait_bits(w->codec->control_data,
					      AIC3262_ADC_FLAG, reg_mask,
					      reg_mask, TIME_DELAY,
					      DELAY_COUNTER);
		sync_needed = SYNC_STATE(aic3262);
		non_sync_state = DSP_NON_SYNC_MODE(aic3262->dsp_runstate);
		other_dsp = aic3262->dsp_runstate & AIC3262_COPS_MDSP_D;
		if (sync_needed && non_sync_state && other_dsp) {
			run_state = get_runstate(aic3262->codec->control_data);
			aic3262_dsp_pwrdwn_status(aic3262);
			aic3262_dsp_pwrup(aic3262, run_state);
		}
		aic3262->dsp_runstate |= run_state_mask;
		if (!ret_wbits) {
			dev_err(w->codec->dev, "ADC POST_PMU timedout\n");
			return -1;
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret_wbits = aic3262_wait_bits(w->codec->control_data,
					      AIC3262_ADC_FLAG, reg_mask, 0,
					      TIME_DELAY, DELAY_COUNTER);
		aic3262->dsp_runstate = (aic3262->dsp_runstate &
					 ~run_state_mask);
		if (!ret_wbits) {
			dev_err(w->codec->dev, "ADC POST_PMD timedout\n");
			return -1;
		}
		break;
	default:
		BUG();
		return -EINVAL;
	}
	return 0;
}

static const struct snd_soc_dapm_widget aic3262_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("DIN1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DIN2", "ASI2 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DIN3", "ASI3 Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_DAC_E("Left DAC", NULL, AIC3262_PASI_DAC_DP_SETUP, 7, 0,
			   aic326x_dac_event, SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("Right DAC", NULL, AIC3262_PASI_DAC_DP_SETUP, 6, 0,
			   aic326x_dac_event, SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),

	/* dapm widget (path domain) for HPL Output Mixer */
	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
				&hpl_output_mixer_controls[0],
				ARRAY_SIZE(hpl_output_mixer_controls)),

	/* dapm widget (path domain) for HPR Output Mixer */
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
				&hpr_output_mixer_controls[0],
				ARRAY_SIZE(hpr_output_mixer_controls)),


	SND_SOC_DAPM_PGA_E("HPL Driver", AIC3262_HP_AMP_CNTL_R1,
				1, 0, NULL, 0, aic326x_hp_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("HPR Driver", AIC3262_HP_AMP_CNTL_R1,
				0, 0, NULL, 0, aic326x_hp_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),


	/* dapm widget (path domain) for LOL Output Mixer */
	SND_SOC_DAPM_MIXER("LOL Output Mixer", SND_SOC_NOPM, 0, 0,
				&lol_output_mixer_controls[0],
				ARRAY_SIZE(lol_output_mixer_controls)),

	/* dapm widget (path domain) for LOR Output Mixer mixer */
	SND_SOC_DAPM_MIXER("LOR Output Mixer", SND_SOC_NOPM, 0, 0,
				&lor_output_mixer_controls[0],
				ARRAY_SIZE(lor_output_mixer_controls)),

	SND_SOC_DAPM_PGA("LOL Driver", AIC3262_LINE_AMP_CNTL_R1,
				1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LOR Driver", AIC3262_LINE_AMP_CNTL_R1,
				0, 0, NULL, 0),


	/* dapm widget (path domain) for SPKL Output Mixer */
	SND_SOC_DAPM_MIXER("SPKL Output Mixer", SND_SOC_NOPM, 0, 0,
				&spkl_output_mixer_controls[0],
				ARRAY_SIZE(spkl_output_mixer_controls)),

	/* dapm widget (path domain) for SPKR Output Mixer */
	SND_SOC_DAPM_MIXER("SPKR Output Mixer", SND_SOC_NOPM, 0, 0,
				&spkr_output_mixer_controls[0],
				ARRAY_SIZE(spkr_output_mixer_controls)),

	SND_SOC_DAPM_PGA_E("SPKL Driver", AIC3262_SPK_AMP_CNTL_R1,
				1, 0, NULL, 0, aic326x_spk_event,
				SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA_E("SPKR Driver", AIC3262_SPK_AMP_CNTL_R1,
				0, 0, NULL, 0, aic326x_spk_event,
				SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU),


	/* dapm widget (path domain) for SPKR Output Mixer */
	SND_SOC_DAPM_MIXER("REC Output Mixer", SND_SOC_NOPM, 0, 0,
				&rec_output_mixer_controls[0],
				ARRAY_SIZE(rec_output_mixer_controls)),

	SND_SOC_DAPM_PGA("RECP Driver", AIC3262_REC_AMP_CNTL_R5,
				7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("RECM Driver", AIC3262_REC_AMP_CNTL_R5,
				6, 0, NULL, 0),


	SND_SOC_DAPM_MUX("ASI1LIN Route",
			 SND_SOC_NOPM, 0, 0, &asi1lin_control),
	SND_SOC_DAPM_MUX("ASI1RIN Route",
			 SND_SOC_NOPM, 0, 0, &asi1rin_control),
	SND_SOC_DAPM_MUX("ASI2LIN Route",
			 SND_SOC_NOPM, 0, 0, &asi2lin_control),
	SND_SOC_DAPM_MUX("ASI2RIN Route",
			 SND_SOC_NOPM, 0, 0, &asi2rin_control),
	SND_SOC_DAPM_MUX("ASI3LIN Route",
			 SND_SOC_NOPM, 0, 0, &asi3lin_control),
	SND_SOC_DAPM_MUX("ASI3RIN Route",
			 SND_SOC_NOPM, 0, 0, &asi3rin_control),

	SND_SOC_DAPM_PGA("ASI1LIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI1RIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI2LIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI2RIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI3LIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI3RIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI1MonoMixIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI2MonoMixIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI3MonoMixIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	/* TODO: Can we switch the ASIxIN off? */
	SND_SOC_DAPM_PGA("ASI1IN Port", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI2IN Port", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI3IN Port", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("DAC MiniDSP IN1 Route",
			 SND_SOC_NOPM, 0, 0, &dacminidspin1_control),
	SND_SOC_DAPM_MUX("DAC MiniDSP IN2 Route",
			 SND_SOC_NOPM, 0, 0, &dacminidspin2_control),
	SND_SOC_DAPM_MUX("DAC MiniDSP IN3 Route",
			 SND_SOC_NOPM, 0, 0, &dacminidspin3_control),

	SND_SOC_DAPM_VIRT_MUX("ADC DAC Route",
			      SND_SOC_NOPM, 0, 0, &adcdacroute_control),

	SND_SOC_DAPM_PGA("CM", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("CM1L", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("CM2L", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("CM1R", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("CM2R", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* TODO: Can we switch these off ? */
	SND_SOC_DAPM_AIF_OUT("DOUT1", "ASI1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("DOUT2", "ASI2 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("DOUT3", "ASI3 Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MUX("DOUT1 Route",
			 SND_SOC_NOPM, 0, 0, &dout1_control),
	SND_SOC_DAPM_MUX("DOUT2 Route",
			 SND_SOC_NOPM, 0, 0, &dout2_control),
	SND_SOC_DAPM_MUX("DOUT3 Route",
			 SND_SOC_NOPM, 0, 0, &dout3_control),

	SND_SOC_DAPM_PGA("ASI1OUT", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI2OUT", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI3OUT", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("ASI1OUT Route",
			 SND_SOC_NOPM, 0, 0, &asi1out_control),
	SND_SOC_DAPM_MUX("ASI2OUT Route",
			 SND_SOC_NOPM, 0, 0, &asi2out_control),
	SND_SOC_DAPM_MUX("ASI3OUT Route",
			 SND_SOC_NOPM, 0, 0, &asi3out_control),

	/* TODO: Can we switch the ASI1 OUT1 off? */
	/* TODO: Can we switch them off? */
	SND_SOC_DAPM_PGA("ADC MiniDSP OUT1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ADC MiniDSP OUT2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ADC MiniDSP OUT3", SND_SOC_NOPM, 0, 0, NULL, 0),

/*	SND_SOC_DAPM_MUX("DMICDAT Input Route",
			SND_SOC_NOPM, 0, 0, &dmicinput_control),*/

	SND_SOC_DAPM_MUX("Left ADC Route", SND_SOC_NOPM, 0, 0, &adcl_mux),
	SND_SOC_DAPM_MUX("Right ADC Route", SND_SOC_NOPM, 0, 0, &adcr_mux),

	SND_SOC_DAPM_ADC_E("Left ADC", NULL, AIC3262_ADC_CHANNEL_POW, 7, 0,
			   aic326x_adc_dsp_event, SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("Right ADC", NULL, AIC3262_ADC_CHANNEL_POW, 6, 0,
			   aic326x_adc_dsp_event, SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA("Left MicPGA", AIC3262_MICL_PGA, 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right MicPGA", AIC3262_MICR_PGA, 7, 1, NULL, 0),

	SND_SOC_DAPM_PGA("MAL PGA", AIC3262_MA_CNTL,
				3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MAR PGA", AIC3262_MA_CNTL,
				2, 0, NULL, 0),

	/* dapm widget for MAL PGA Mixer */
	SND_SOC_DAPM_MIXER("MAL PGA Mixer", SND_SOC_NOPM, 0, 0,
				&mal_pga_mixer_controls[0],
				ARRAY_SIZE(mal_pga_mixer_controls)),

	/* dapm widget for MAR PGA Mixer */
	SND_SOC_DAPM_MIXER("MAR PGA Mixer", SND_SOC_NOPM, 0, 0,
				&mar_pga_mixer_controls[0],
				ARRAY_SIZE(mar_pga_mixer_controls)),

	/* dapm widget for Left Input Mixer */
	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
				&left_input_mixer_controls[0],
				ARRAY_SIZE(left_input_mixer_controls)),

	/* dapm widget for Right Input Mixer */
	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
				&right_input_mixer_controls[0],
				ARRAY_SIZE(right_input_mixer_controls)),

	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_OUTPUT("LOL"),
	SND_SOC_DAPM_OUTPUT("LOR"),
	SND_SOC_DAPM_OUTPUT("SPKL"),
	SND_SOC_DAPM_OUTPUT("SPKR"),
	SND_SOC_DAPM_OUTPUT("RECP"),
	SND_SOC_DAPM_OUTPUT("RECM"),

	SND_SOC_DAPM_INPUT("IN1L"),
	SND_SOC_DAPM_INPUT("IN2L"),
	SND_SOC_DAPM_INPUT("IN3L"),
	SND_SOC_DAPM_INPUT("IN4L"),
	SND_SOC_DAPM_INPUT("IN1R"),
	SND_SOC_DAPM_INPUT("IN2R"),
	SND_SOC_DAPM_INPUT("IN3R"),
	SND_SOC_DAPM_INPUT("IN4R"),
	SND_SOC_DAPM_INPUT("Left DMIC"),
	SND_SOC_DAPM_INPUT("Right DMIC"),

	SND_SOC_DAPM_MICBIAS("Mic Bias Ext", AIC3262_MIC_BIAS_CNTL, 6, 0),
	SND_SOC_DAPM_MICBIAS("Mic Bias Int", AIC3262_MIC_BIAS_CNTL, 2, 0),

	SND_SOC_DAPM_SUPPLY("PLLCLK", AIC3262_PLL_PR_POW_REG, 7, 0,
			    pll_power_on_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_SUPPLY("DACCLK", AIC3262_NDAC_DIV_POW_REG, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("CODEC_CLK_IN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC_MOD_CLK", AIC3262_MDAC_DIV_POW_REG,
			    7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADCCLK", AIC3262_NADC_DIV_POW_REG, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC_MOD_CLK", AIC3262_MADC_DIV_POW_REG,
			    7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI1_BCLK", AIC3262_ASI1_BCLK_N, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI1_WCLK", AIC3262_ASI1_WCLK_N, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI2_BCLK", AIC3262_ASI2_BCLK_N, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI2_WCLK", AIC3262_ASI2_WCLK_N, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI3_BCLK", AIC3262_ASI3_BCLK_N, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ASI3_WCLK", AIC3262_ASI3_WCLK_N, 7, 0, NULL, 0),
	SND_SOC_DAPM_MUX("ASI1_BCLK Route",
			 SND_SOC_NOPM, 0, 0, &asi1bclk_control),
	SND_SOC_DAPM_MUX("ASI2_BCLK Route",
			 SND_SOC_NOPM, 0, 0, &asi2bclk_control),
	SND_SOC_DAPM_MUX("ASI3_BCLK Route",
			 SND_SOC_NOPM, 0, 0, &asi3bclk_control),
};

static const struct snd_soc_dapm_route aic3262_dapm_routes[] = {
	/* TODO: Do we need only DACCLK for ASIIN's and ADCCLK for ASIOUT??? */
	/* Clock portion */
	{"CODEC_CLK_IN", NULL, "PLLCLK"},
	{"DACCLK", NULL, "CODEC_CLK_IN"},
	{"ADCCLK", NULL, "CODEC_CLK_IN"},
	{"DAC_MOD_CLK", NULL, "DACCLK"},
#ifdef AIC3262_SYNC_MODE
	{"ADC_MOD_CLK", NULL, "DACCLK"},
#else
	{"ADC_MOD_CLK", NULL, "ADCCLK"},
#endif

	{"ASI1_BCLK Route", "DAC_CLK", "DACCLK"},
	{"ASI1_BCLK Route", "DAC_MOD_CLK", "DAC_MOD_CLK"},
	{"ASI1_BCLK Route", "ADC_CLK", "ADCCLK"},
	{"ASI1_BCLK Route", "ADC_MOD_CLK", "ADC_MOD_CLK"},

	{"ASI2_BCLK Route", "DAC_CLK", "DACCLK"},
	{"ASI2_BCLK Route", "DAC_MOD_CLK", "DAC_MOD_CLK"},
	{"ASI2_BCLK Route", "ADC_CLK", "ADCCLK"},
	{"ASI2_BCLK Route", "ADC_MOD_CLK", "ADC_MOD_CLK"},

	{"ASI3_BCLK Route", "DAC_CLK", "DACCLK"},
	{"ASI3_BCLK Route", "DAC_MOD_CLK", "DAC_MOD_CLK"},
	{"ASI3_BCLK Route", "ADC_CLK", "ADCCLK"},
	{"ASI3_BCLK Route", "ADC_MOD_CLK", "ADC_MOD_CLK"},

	{"ASI1_BCLK", NULL, "ASI1_BCLK Route"},
	{"ASI2_BCLK", NULL, "ASI2_BCLK Route"},
	{"ASI3_BCLK", NULL, "ASI3_BCLK Route"},

	{"DIN1", NULL, "PLLCLK"},
	{"DIN1", NULL, "DACCLK"},
	{"DIN1", NULL, "ADCCLK"},
	{"DIN1", NULL, "DAC_MOD_CLK"},
	{"DIN1", NULL, "ADC_MOD_CLK"},

	{"DOUT1", NULL, "PLLCLK"},
	{"DOUT1", NULL, "DACCLK"},
	{"DOUT1", NULL, "ADCCLK"},
	{"DOUT1", NULL, "DAC_MOD_CLK"},
	{"DOUT1", NULL, "ADC_MOD_CLK"},
#ifdef AIC3262_ASI1_MASTER
	{"DIN1", NULL, "ASI1_BCLK"},
	{"DOUT1", NULL, "ASI1_BCLK"},
	{"DIN1", NULL, "ASI1_WCLK"},
	{"DOUT1", NULL, "ASI1_WCLK"},
#else

#endif
	{"DIN2", NULL, "PLLCLK"},
	{"DIN2", NULL, "DACCLK"},
	{"DIN2", NULL, "ADCCLK"},
	{"DIN2", NULL, "DAC_MOD_CLK"},
	{"DIN2", NULL, "ADC_MOD_CLK"},

	{"DOUT2", NULL, "PLLCLK"},
	{"DOUT2", NULL, "DACCLK"},
	{"DOUT2", NULL, "ADCCLK"},
	{"DOUT2", NULL, "DAC_MOD_CLK"},
	{"DOUT2", NULL, "ADC_MOD_CLK"},

#ifdef AIC3262_ASI2_MASTER
	{"DIN2", NULL, "ASI2_BCLK"},
	{"DOUT2", NULL, "ASI2_BCLK"},
	{"DIN2", NULL, "ASI2_WCLK"},
	{"DOUT2", NULL, "ASI2_WCLK"},
#else

#endif
	{"DIN3", NULL, "PLLCLK"},
	{"DIN3", NULL, "DACCLK"},
	{"DIN3", NULL, "ADCCLK"},
	{"DIN3", NULL, "DAC_MOD_CLK"},
	{"DIN3", NULL, "ADC_MOD_CLK"},

	{"DOUT3", NULL, "PLLCLK"},
	{"DOUT3", NULL, "DACCLK"},
	{"DOUT3", NULL, "ADCCLK"},
	{"DOUT3", NULL, "DAC_MOD_CLK"},
	{"DOUT3", NULL, "ADC_MOD_CLK"},

#ifdef AIC3262_ASI3_MASTER
	{"DIN3", NULL, "ASI3_BCLK"},
	{"DOUT3", NULL, "ASI3_BCLK"},
	{"DIN3", NULL, "ASI3_WCLK"},
	{"DOUT3", NULL, "ASI3_WCLK"},
#else

#endif
	/* Playback (DAC) Portion */
	{"HPL Output Mixer", "LDAC Switch", "Left DAC"},
	{"HPL Output Mixer", "MAL Switch", "MAL PGA"},
	{"HPL Output Mixer", "LOL-B1 Volume", "LOL"},

	{"HPR Output Mixer", "LOR-B1 Volume", "LOR"},
	{"HPR Output Mixer", "LDAC Switch", "Left DAC"},
	{"HPR Output Mixer", "RDAC Switch", "Right DAC"},
	{"HPR Output Mixer", "MAR Switch", "MAR PGA"},

	{"HPL Driver", NULL, "HPL Output Mixer"},
	{"HPR Driver", NULL, "HPR Output Mixer"},

	{"HPL", NULL, "HPL Driver"},
	{"HPR", NULL, "HPR Driver"},

	{"LOL Output Mixer", "MAL Switch", "MAL PGA"},
	{"LOL Output Mixer", "IN1L-B Switch", "IN1L"},
	{"LOL Output Mixer", "LDAC Switch", "Left DAC"},
	{"LOL Output Mixer", "RDAC Switch", "Right DAC"},

	{"LOR Output Mixer", "LOL Switch", "LOL"},
	{"LOR Output Mixer", "RDAC Switch", "Right DAC"},
	{"LOR Output Mixer", "MAR Switch", "MAR PGA"},
	{"LOR Output Mixer", "IN1R-B Switch", "IN1R"},

	{"LOL Driver", NULL, "LOL Output Mixer"},
	{"LOR Driver", NULL, "LOR Output Mixer"},

	{"LOL", NULL, "LOL Driver"},
	{"LOR", NULL, "LOR Driver"},

	{"REC Output Mixer", "LOL-B2 Volume", "LOL"},
	{"REC Output Mixer", "IN1L Volume", "IN1L"},
	{"REC Output Mixer", "IN1R Volume", "IN1R"},
	{"REC Output Mixer", "LOR-B2 Volume", "LOR"},

	{"RECP Driver", NULL, "REC Output Mixer"},
	{"RECM Driver", NULL, "REC Output Mixer"},

	{"RECP", NULL, "RECP Driver"},
	{"RECM", NULL, "RECM Driver"},

	{"SPKL Output Mixer", "MAL Switch", "MAL PGA"},
	{"SPKL Output Mixer", "LOL Volume", "LOL"},
	{"SPKL Output Mixer", "SPR_IN Switch", "SPKR Output Mixer"},

	{"SPKR Output Mixer", "LOR Volume", "LOR"},
	{"SPKR Output Mixer", "MAR Switch", "MAR PGA"},


	{"SPKL Driver", NULL, "SPKL Output Mixer"},
	{"SPKR Driver", NULL, "SPKR Output Mixer"},

	{"SPKL", NULL, "SPKL Driver"},
	{"SPKR", NULL, "SPKR Driver"},
	/* ASI Input routing */
	{"ASI1LIN", NULL, "DIN1"},
	{"ASI1RIN", NULL, "DIN1"},
	{"ASI1MonoMixIN", NULL, "DIN1"},
	{"ASI2LIN", NULL, "DIN2"},
	{"ASI2RIN", NULL, "DIN2"},
	{"ASI2MonoMixIN", NULL, "DIN2"},
	{"ASI3LIN", NULL, "DIN3"},
	{"ASI3RIN", NULL, "DIN3"},
	{"ASI3MonoMixIN", NULL, "DIN3"},

	{"ASI1LIN Route", "ASI1 Left In", "ASI1LIN"},
	{"ASI1LIN Route", "ASI1 Right In", "ASI1RIN"},
	{"ASI1LIN Route", "ASI1 MonoMix In", "ASI1MonoMixIN"},

	{"ASI1RIN Route", "ASI1 Right In", "ASI1RIN"},
	{"ASI1RIN Route", "ASI1 Left In", "ASI1LIN"},
	{"ASI1RIN Route", "ASI1 MonoMix In", "ASI1MonoMixIN"},

	{"ASI2LIN Route", "ASI2 Left In", "ASI2LIN"},
	{"ASI2LIN Route", "ASI2 Right In", "ASI2RIN"},
	{"ASI2LIN Route", "ASI2 MonoMix In", "ASI2MonoMixIN"},

	{"ASI2RIN Route", "ASI2 Right In", "ASI2RIN"},
	{"ASI2RIN Route", "ASI2 Left In", "ASI2LIN"},
	{"ASI2RIN Route", "ASI2 MonoMix In", "ASI2MonoMixIN"},

	{"ASI3LIN Route", "ASI3 Left In", "ASI3LIN"},
	{"ASI3LIN Route", "ASI3 Right In", "ASI3RIN"},
	{"ASI3LIN Route", "ASI3 MonoMix In", "ASI3MonoMixIN"},

	{"ASI3RIN Route", "ASI3 Right In", "ASI3RIN"},
	{"ASI3RIN Route", "ASI3 Left In", "ASI3LIN"},
	{"ASI3RIN Route", "ASI3 MonoMix In", "ASI3MonoMixIN"},

	{"ASI1IN Port", NULL, "ASI1LIN Route"},
	{"ASI1IN Port", NULL, "ASI1RIN Route"},
	{"ASI2IN Port", NULL, "ASI2LIN Route"},
	{"ASI2IN Port", NULL, "ASI2RIN Route"},
	{"ASI3IN Port", NULL, "ASI3LIN Route"},
	{"ASI3IN Port", NULL, "ASI3RIN Route"},

	{"DAC MiniDSP IN1 Route", "ASI1 In", "ASI1IN Port"},
	{"DAC MiniDSP IN1 Route", "ASI2 In", "ASI2IN Port"},
	{"DAC MiniDSP IN1 Route", "ASI3 In", "ASI3IN Port"},
	{"DAC MiniDSP IN1 Route", "ADC MiniDSP Out", "ADC MiniDSP OUT1"},

	{"DAC MiniDSP IN2 Route", "ASI1 In", "ASI1IN Port"},
	{"DAC MiniDSP IN2 Route", "ASI2 In", "ASI2IN Port"},
	{"DAC MiniDSP IN2 Route", "ASI3 In", "ASI3IN Port"},

	{"DAC MiniDSP IN3 Route", "ASI1 In", "ASI1IN Port"},
	{"DAC MiniDSP IN3 Route", "ASI2 In", "ASI2IN Port"},
	{"DAC MiniDSP IN3 Route", "ASI3 In", "ASI3IN Port"},

	{"Left DAC", "NULL", "DAC MiniDSP IN1 Route"},
	{"Right DAC", "NULL", "DAC MiniDSP IN1 Route"},
	{"Left DAC", "NULL", "DAC MiniDSP IN2 Route"},
	{"Right DAC", "NULL", "DAC MiniDSP IN2 Route"},
	{"Left DAC", "NULL", "DAC MiniDSP IN3 Route"},
	{"Right DAC", "NULL", "DAC MiniDSP IN3 Route"},

	/* Mixer Amplifier */

	{"MAL PGA Mixer", "IN1L Switch", "IN1L"},
	{"MAL PGA Mixer", "Left MicPGA Volume", "Left MicPGA"},

	{"MAL PGA", NULL, "MAL PGA Mixer"},


	{"MAR PGA Mixer", "IN1R Switch", "IN1R"},
	{"MAR PGA Mixer", "Right MicPGA Volume", "Right MicPGA"},

	{"MAR PGA", NULL, "MAR PGA Mixer"},


	/* Virtual connection between DAC and ADC for miniDSP IPC */
	{"ADC DAC Route", "On", "Left ADC"},
	{"ADC DAC Route", "On", "Right ADC"},

	{"Left DAC", NULL, "ADC DAC Route"},
	{"Right DAC", NULL, "ADC DAC Route"},

	/* Capture (ADC) portions */
	/* Left Positive PGA input */
	{"Left Input Mixer", "IN1L Switch", "IN1L"},
	{"Left Input Mixer", "IN2L Switch", "IN2L"},
	{"Left Input Mixer", "IN3L Switch", "IN3L"},
	{"Left Input Mixer", "IN4L Switch", "IN4L"},
	{"Left Input Mixer", "IN1R Switch", "IN1R"},
	/* Left Negative PGA input */
	{"Left Input Mixer", "IN2R Switch", "IN2R"},
	{"Left Input Mixer", "IN3R Switch", "IN3R"},
	{"Left Input Mixer", "IN4R Switch", "IN4R"},
	{"Left Input Mixer", "CM2L Switch", "CM2L"},
	{"Left Input Mixer", "CM1L Switch", "CM1L"},


	/* Right Positive PGA Input */
	{"Right Input Mixer", "IN1R Switch", "IN1R"},
	{"Right Input Mixer", "IN2R Switch", "IN2R"},
	{"Right Input Mixer", "IN3R Switch", "IN3R"},
	{"Right Input Mixer", "IN4R Switch", "IN4R"},
	{"Right Input Mixer", "IN2L Switch", "IN2L"},
	/* Right Negative PGA Input */
	{"Right Input Mixer", "IN1L Switch", "IN1L"},
	{"Right Input Mixer", "IN3L Switch", "IN3L"},
	{"Right Input Mixer", "IN4L Switch", "IN4L"},
	{"Right Input Mixer", "CM1R Switch", "CM1R"},
	{"Right Input Mixer", "CM2R Switch", "CM2R"},


	{"CM1L", NULL, "CM"},
	{"CM2L", NULL, "CM"},
	{"CM1R", NULL, "CM"},
	{"CM2R", NULL, "CM"},

	{"Left MicPGA", NULL, "Left Input Mixer"},
	{"Right MicPGA", NULL, "Right Input Mixer"},

	{"Left ADC Route", "Analog", "Left MicPGA"},
	{"Left ADC Route", "Digital", "Left DMIC"},

	{"Right ADC Route", "Analog", "Right MicPGA"},
	{"Right ADC Route", "Digital", "Right DMIC"},

	{"Left ADC", NULL, "Left ADC Route"},
	{"Right ADC", NULL, "Right ADC Route"},

	/* ASI Output Routing */
	{"ADC MiniDSP OUT1", NULL, "Left ADC"},
	{"ADC MiniDSP OUT1", NULL, "Right ADC"},
	{"ADC MiniDSP OUT2", NULL, "Left ADC"},
	{"ADC MiniDSP OUT2", NULL, "Right ADC"},
	{"ADC MiniDSP OUT3", NULL, "Left ADC"},
	{"ADC MiniDSP OUT3", NULL, "Right ADC"},

	{"ASI1OUT Route", "ADC MiniDSP Out1", "ADC MiniDSP OUT1"},
	{"ASI1OUT Route", "ASI1In Bypass", "ASI1IN Port"},
	{"ASI1OUT Route", "ASI2In Bypass", "ASI2IN Port"},
	{"ASI1OUT Route", "ASI3In Bypass", "ASI3IN Port"},

	{"ASI2OUT Route", "ADC MiniDSP Out1", "ADC MiniDSP OUT1"},
	{"ASI2OUT Route", "ASI1In Bypass", "ASI1IN Port"},
	{"ASI2OUT Route", "ASI2In Bypass", "ASI2IN Port"},
	{"ASI2OUT Route", "ASI3In Bypass", "ASI3IN Port"},
	{"ASI2OUT Route", "ADC MiniDSP Out2", "ADC MiniDSP OUT2"},

	{"ASI3OUT Route", "ADC MiniDSP Out1", "ADC MiniDSP OUT1"},
	{"ASI3OUT Route", "ASI1In Bypass", "ASI1IN Port"},
	{"ASI3OUT Route", "ASI2In Bypass", "ASI2IN Port"},
	{"ASI3OUT Route", "ASI3In Bypass", "ASI3IN Port"},
	{"ASI3OUT Route", "ADC MiniDSP Out3", "ADC MiniDSP OUT3"},

	{"ASI1OUT", NULL, "ASI1OUT Route"},
	{"ASI2OUT", NULL, "ASI2OUT Route"},
	{"ASI3OUT", NULL, "ASI3OUT Route"},

	{"DOUT1 Route", "ASI1 Out", "ASI1OUT"},
	{"DOUT1 Route", "DIN1 Bypass", "DIN1"},
	{"DOUT1 Route", "DIN2 Bypass", "DIN2"},
	{"DOUT1 Route", "DIN3 Bypass", "DIN3"},

	{"DOUT2 Route", "ASI2 Out", "ASI2OUT"},
	{"DOUT2 Route", "DIN1 Bypass", "DIN1"},
	{"DOUT2 Route", "DIN2 Bypass", "DIN2"},
	{"DOUT2 Route", "DIN3 Bypass", "DIN3"},

	{"DOUT3 Route", "ASI3 Out", "ASI3OUT"},
	{"DOUT3 Route", "DIN1 Bypass", "DIN1"},
	{"DOUT3 Route", "DIN2 Bypass", "DIN2"},
	{"DOUT3 Route", "DIN3 Bypass", "DIN3"},

	{"DOUT1", NULL, "DOUT1 Route"},
	{"DOUT2", NULL, "DOUT2 Route"},
	{"DOUT3", NULL, "DOUT3 Route"},
};

#define AIC3262_DAPM_ROUTE_NUM (ARRAY_SIZE(aic3262_dapm_routes)/ \
					sizeof(struct snd_soc_dapm_route))
/* aic3262_firmware_load:   This function is called by the
 *		request_firmware_nowait function as soon
 *		as the firmware has been loaded from the file.
 *		The firmware structure contains the data and$
 *		the size of the firmware loaded.
 * @fw: pointer to firmware file to be dowloaded
 * @context: pointer variable to codec
 *
 * Returns 0 for success.
 */
void aic3262_firmware_load(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = context;
	struct aic3262_priv *private_ds = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	aic3xxx_cfw_lock(private_ds->cfw_p, 1);
	if (private_ds->cur_fw != NULL)
		release_firmware(private_ds->cur_fw);
	private_ds->cur_fw = NULL;

	if (fw != NULL) {
		dev_dbg(codec->dev, "Firmware binary load\n");
		private_ds->cur_fw = (void *)fw;
		ret = aic3xxx_cfw_reload(private_ds->cfw_p,
					 (void *)fw->data, fw->size);
		if (ret < 0) {	/* reload failed */
			dev_err(codec->dev, "Firmware binary load failed\n");
			release_firmware(private_ds->cur_fw);
			private_ds->cur_fw = NULL;
			fw = NULL;
		} else
			private_ds->isdefault_fw = 0;
	}

	if (fw == NULL) {
		/* either request_firmware or reload failed */
		dev_dbg(codec->dev, "Default firmware load\n");
		ret = aic3xxx_cfw_reload(private_ds->cfw_p, default_firmware,
					 sizeof(default_firmware));
		if (ret < 0)
			dev_err(codec->dev, "Default firmware load failed\n");
		else
			private_ds->isdefault_fw = 1;
	}
	aic3xxx_cfw_lock(private_ds->cfw_p, 0);
	if (ret >= 0) {
		/* init function for transition */
		aic3xxx_cfw_transition(private_ds->cfw_p, "INIT");
		if (!private_ds->isdefault_fw) {
			aic3xxx_cfw_add_modes(codec, private_ds->cfw_p);
			aic3xxx_cfw_add_controls(codec, private_ds->cfw_p);
		}
		aic3xxx_cfw_setmode_cfg(private_ds->cfw_p, 0, 0);
	}
}

/*=========================================================

 headset work and headphone/headset jack interrupt handlers

 ========================================================*/

/**
 * aic3262_hs_jack_report: Report jack notication to upper layor
 * @codec: pointer variable to codec having information related to codec
 * @jack: Pointer variable to snd_soc_jack having information of codec
 *		 and pin number$
 * @report: Provides informaton of whether it is headphone or microphone
 *
*/
static void aic3262_hs_jack_report(struct snd_soc_codec *codec,
				   struct snd_soc_jack *jack, int report)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	int status, state = 0;

	mutex_lock(&aic3262->mutex);

	/* Sync status */
	status = snd_soc_read(codec, AIC3262_DAC_FLAG);

	switch (status & AIC3262_JACK_TYPE_MASK) {
	case AIC3262_JACK_WITH_MIC:
		state |= SND_JACK_HEADSET;
		break;
	case AIC3262_JACK_WITHOUT_MIC:
		state |= SND_JACK_HEADPHONE;
		break;
	default:
		break;
	}

	mutex_unlock(&aic3262->mutex);

	snd_soc_jack_report(jack, state, report);

}

/**
 * aic3262_hs_jack_detect: Detect headphone jack during boot time
 * @codec: pointer variable to codec having information related to codec
 * @jack: Pointer variable to snd_soc_jack having information of codec
 *	     and pin number$
 * @report: Provides informaton of whether it is headphone or microphone
 *
*/
void aic3262_hs_jack_detect(struct snd_soc_codec *codec,
			    struct snd_soc_jack *jack, int report)
{
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	struct aic3262_jack_data *hs_jack = &aic3262->hs_jack;

	hs_jack->jack = jack;
	hs_jack->report = report;
	aic3262_hs_jack_report(codec, hs_jack->jack, hs_jack->report);
}
EXPORT_SYMBOL_GPL(aic3262_hs_jack_detect);
/**
 * aic3262_accessory_work: Finished bottom half work from headphone jack
 *		insertion interupt
 * @work: pionter variable to work_struct which is maintaining work queqe
 *
*/
static void aic3262_accessory_work(struct work_struct *work)
{
	struct aic3262_priv *aic3262 = container_of(work,
						    struct aic3262_priv,
						    delayed_work.work);
	struct snd_soc_codec *codec = aic3262->codec;
	struct aic3262_jack_data *hs_jack = &aic3262->hs_jack;
	aic3262_hs_jack_report(codec, hs_jack->jack, hs_jack->report);
}

/**
 * aic3262_audio_handler: audio interrupt handler called
 *		when interupt is generated
 * @irq: provides interupt number which is assigned by aic3262_request_irq,
 * @data having information of data passed by aic3262_request_irq last arg,
 *
 * Return IRQ_HANDLED(means interupt handeled successfully)
*/
static irqreturn_t aic3262_audio_handler(int irq, void *data)
{
	struct snd_soc_codec *codec = data;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	queue_delayed_work(aic3262->workqueue, &aic3262->delayed_work,
			   msecs_to_jiffies(200));
	return IRQ_HANDLED;
}

static irqreturn_t aic3262_button_handler(int irq, void *data)
{
	struct snd_soc_codec *codec = data;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	input_report_key(aic3262->idev, KEY_MEDIA, 1);
	mdelay(50);
	input_report_key(aic3262->idev, KEY_MEDIA, 0);
	input_sync(aic3262->idev);

	return IRQ_HANDLED;
}

/**
 * aic3262_codec_read: provide read api to read aic3262 registe space
 * @codec: pointer variable to codec having codec information,
 * @reg: register address,
 *
 * Return: Return value will be value read.
 */
unsigned int aic3262_codec_read(struct snd_soc_codec *codec, unsigned int reg)
{

	u8 value;

	union aic326x_reg_union *aic_reg = (union aic326x_reg_union *) &reg;
	value = aic3262_reg_read(codec->control_data, reg);
	dev_dbg(codec->dev, "p %d , r 30 %x %x\n",
		aic_reg->aic326x_register.page,
		aic_reg->aic326x_register.offset, value);
	return value;
}

/**
 * aic3262_codec_write: provide write api to write at aic3262 registe space
 * @codec: Pointer variable to codec having codec information,
 * @reg: Register address,
 * @value: Value to be written to address space
 *
 * Return: Total no of byte written to address space.
 */
int aic3262_codec_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int value)
{
	union aic326x_reg_union *aic_reg = (union aic326x_reg_union *) &reg;
	dev_dbg(codec->dev, "p %d, w 30 %x %x\n",
		aic_reg->aic326x_register.page,
		aic_reg->aic326x_register.offset, value);
	return aic3262_reg_write(codec->control_data, reg, value);
}

/**
 * aic3262_add_widget: This function is to add the dapm widgets
 *	  The following are the main widgets supported
 *	      # Left DAC to Left Outputs
 *	      # Right DAC to Right Outputs
 *		  # Left Inputs to Left ADC
 *		  # Right Inputs to Right ADC
 * @codec: pointer variable to codec having informaton related to codec,
 *
 * Return: return 0 on success.
 */
static int aic3262_add_widgets(struct snd_soc_codec *codec)
{

	snd_soc_dapm_new_controls(&codec->dapm, aic3262_dapm_widgets,
				  ARRAY_SIZE(aic3262_dapm_widgets));
	/* set up audio path interconnects */
	dev_dbg(codec->dev, "#Completed adding new dapm widget"
		 " controls size=%d\n", ARRAY_SIZE(aic3262_dapm_widgets));

	snd_soc_dapm_add_routes(&codec->dapm, aic3262_dapm_routes,
				ARRAY_SIZE(aic3262_dapm_routes));
	dev_dbg(codec->dev, "#Completed adding DAPM routes\n");
	snd_soc_dapm_new_widgets(&codec->dapm);
	dev_dbg(codec->dev, "#Completed updating dapm\n");

	return 0;
}

/**
 * aic3262_set_interface_fmt: Setting interface ASI1/2/3 data format
 * @dai: ponter to dai Holds runtime data for a DAI,
 * @fmt: asi format info,
 * @channel: number of channel,
 *
 * Return: On success return 0.
*/
static int aic3262_set_interface_fmt(struct snd_soc_dai *dai, unsigned int fmt,
				     unsigned int channel)
{
	int aif_interface_reg;
	int aif_bclk_offset_reg;
	struct snd_soc_codec *codec = dai->codec;
	u8 iface_val = 0;
	u8 dsp_a_val = 0;

	switch (dai->id) {
	case 0:
		aif_interface_reg = AIC3262_ASI1_BUS_FMT;
		aif_bclk_offset_reg = AIC3262_ASI1_LCH_OFFSET;
		break;
	case 1:
		aif_interface_reg = AIC3262_ASI2_BUS_FMT;
		aif_bclk_offset_reg = AIC3262_ASI2_LCH_OFFSET;
		break;
	case 2:
		aif_interface_reg = AIC3262_ASI3_BUS_FMT;
		aif_bclk_offset_reg = AIC3262_ASI3_LCH_OFFSET;
		break;
	default:
		return -EINVAL;

	}
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface_val = 0;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		dsp_a_val = 0x1;	/* Intentionally falling back
					   to following case */
	case SND_SOC_DAIFMT_DSP_B:
		switch (channel) {
		case 1:
			iface_val = 0x80;	/* Choose mono PCM */
			break;
		case 2:
			iface_val = 0x20;
			break;
		default:
			return -EINVAL;
		}
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_val = 0x40;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_val = 0x60;
		break;
	default:
		dev_err(codec->dev, "Invalid DAI interface format\n");
		return -EINVAL;
	}
	snd_soc_update_bits(codec, aif_interface_reg,
			    AIC3262_ASI_INTERFACE_MASK, iface_val);
	snd_soc_update_bits(codec, aif_bclk_offset_reg,
			    AIC3262_BCLK_OFFSET_MASK, dsp_a_val);
	return 0;

}

/**
 * aic3262_hw_params: This function is to set the hardware parameters
 *		for AIC3262.
 *		The functions set the sample rate and audio serial data word
 *		length.
 * @substream: pointer variable to sn_pcm_substream,
 * @params: pointer to snd_pcm_hw_params structure,
 * @dai: ponter to dai Holds runtime data for a DAI,
 *
 * Return: Return 0 on success.
 */
int aic3262_hw_params(struct snd_pcm_substream *substream,
		      struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	int asi_reg;
	u8 data = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		aic3262->stream_status = 1;
	else
		aic3262->stream_status = 0;

	switch (dai->id) {
	case 0:
		asi_reg = AIC3262_ASI1_BUS_FMT;
		break;
	case 1:
		asi_reg = AIC3262_ASI2_BUS_FMT;
		break;
	case 2:
		asi_reg = AIC3262_ASI3_BUS_FMT;
		break;
	default:
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		data = data | 0x00;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (0x08);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (0x10);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (0x18);
		break;
	}

	/* configure the respective Registers for the above configuration */
	snd_soc_update_bits(codec, asi_reg,
			    AIC3262_ASI_DATA_WORD_LENGTH_MASK, data);
	return aic3262_set_interface_fmt(dai, aic3262->asi_fmt[dai->id],
					 params_channels(params));
}

/**
 * aic3262_mute: This function is to mute or unmute the left and right DAC
 * @dai: ponter to dai Holds runtime data for a DAI,
 * @mute: integer value one if we using mute else unmute,
 *
 * Return: return 0 on success.
 */
static int aic3262_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "codec : %s : started\n", __func__);
	if (dai->id > 2)
		return -EINVAL;
	if (mute) {
		aic3262->mute_asi &= ~((0x1) << dai->id);
		if (aic3262->mute_asi == 0)
			/* Mute only when all asi's are muted */
			snd_soc_update_bits_locked(codec,
						   AIC3262_DAC_MVOL_CONF,
						   AIC3262_DAC_LR_MUTE_MASK,
						   AIC3262_DAC_LR_MUTE);

	} else {	/* Unmute */
		if (aic3262->mute_asi == 0)
			/* Unmute for the first asi that need to unmute.
			   rest unmute will pass */
			snd_soc_update_bits_locked(codec,
						   AIC3262_DAC_MVOL_CONF,
						   AIC3262_DAC_LR_MUTE_MASK,
						   0x0);
		aic3262->mute_asi |= ((0x1) << dai->id);
	}
	dev_dbg(codec->dev, "codec : %s : ended\n", __func__);
	return 0;
}

/**
 * aic3262_set_dai_sysclk: This function is to set the DAI system clock
 * @codec_dai: ponter to dai Holds runtime data for a DAI,
 * @freq: system clock to be set,
 * @dir: integer dir,
 *
 * Return: return 0 on success.
 */
static int aic3262_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct aic3262_priv *aic3262;
	struct snd_soc_codec *codec;

	codec = codec_dai->codec;
	aic3262 = snd_soc_codec_get_drvdata(codec);
	switch (freq) {
	case AIC3262_FREQ_12000000:
		aic3262->sysclk = freq;
		return 0;
	case AIC3262_FREQ_24000000:
		aic3262->sysclk = freq;
		return 0;
		break;
	case AIC3262_FREQ_19200000:
		aic3262->sysclk = freq;
		return 0;
		break;
	case AIC3262_FREQ_38400000:
		aic3262->sysclk = freq;
		dev_dbg(codec->dev, "codec: sysclk = %d\n", aic3262->sysclk);
		return 0;
		break;
	case AIC3262_FREQ_12288000:
		aic3262->sysclk = freq;
		dev_dbg(codec->dev, "codec: sysclk = %d\n", aic3262->sysclk);
		return 0;
		break;

	}
	dev_err(codec->dev, "Invalid frequency to set DAI system clock\n");

	return -EINVAL;
}

/**
 * aic3262_set_dai_fmt: This function is to set the DAI format
 * @codec_dai: ponter to dai Holds runtime data for a DAI,
 * @fmt: asi format info,
 *
 * return: return 0 on success.
 */
static int aic3262_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct aic3262_priv *aic3262;
	struct snd_soc_codec *codec;
	u8 iface_val, master;
	int aif_bclk_wclk_reg;

	codec = codec_dai->codec;
	aic3262 = snd_soc_codec_get_drvdata(codec);
	iface_val = 0x00;
	master = 0x0;

	switch (codec_dai->id) {
	case 0:
		aif_bclk_wclk_reg = AIC3262_ASI1_BWCLK_CNTL_REG;
		break;
	case 1:
		aif_bclk_wclk_reg = AIC3262_ASI2_BWCLK_CNTL_REG;
		break;
	case 2:
		aif_bclk_wclk_reg = AIC3262_ASI3_BWCLK_CNTL_REG;
		break;
	default:
		return -EINVAL;

	}
	aic3262->asi_fmt[codec_dai->id] = fmt;
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic3262->master = 1;
		master |= (AIC3262_WCLK_OUT_MASK | AIC3262_BCLK_OUT_MASK);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic3262->master = 0;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:	/* new case..just for debugging */
		master |= (AIC3262_WCLK_OUT_MASK);
		aic3262->master = 0;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		master |= (AIC3262_BCLK_OUT_MASK);
		aic3262->master = 0;
		break;

	default:
		dev_err(codec->dev, "Invalid DAI master/slave" " interface\n");

		return -EINVAL;
	}
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			break;
		case SND_SOC_DAIFMT_IB_NF:
			master |= AIC3262_BCLK_INV_MASK;
			break;
		default:
			return -EINVAL;
		}
		break;
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			break;
		case SND_SOC_DAIFMT_IB_NF:
			master |= AIC3262_BCLK_INV_MASK;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	snd_soc_update_bits(codec, aif_bclk_wclk_reg,
			    AIC3262_WCLK_BCLK_MASTER_MASK, master);
	return 0;
}

/**
 * aic3262_dai_set_pll: This function is to Set pll for aic3262 codec dai
 * @dai: ponter to dai Holds runtime data for a DAI,$
 * @pll_id: integer pll_id
 * @fin: frequency in,
 * @fout: Frequency out,
 *
 * Return: return 0 on success
*/
static int aic3262_dai_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
				unsigned int Fin, unsigned int Fout)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "In aic3262: dai_set_pll\n");
	dev_dbg(codec->dev, "%d, %s, dai->id = %d\n", __LINE__,
		__func__, dai->id);
	/* select the PLL_CLKIN */
	snd_soc_update_bits(codec, AIC3262_PLL_CLKIN_REG,
			    AIC3262_PLL_CLKIN_MASK, source <<
			    AIC3262_PLL_CLKIN_SHIFT);
	/* TODO: How to select low/high clock range? */

	aic3xxx_cfw_set_pll(aic3262->cfw_p, dai->id);
	return 0;
}

/**
 *
 * aic3262_set_bias_level: This function is to get triggered
 *			 when dapm events occurs.
 * @codec: pointer variable to codec having informaton related to codec,
 * @level: Bias level-> ON, PREPARE, STANDBY, OFF.
 *
 * Return: Return 0 on success.
 */
static int aic3262_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{

	switch (level) {
		/* full On */
	case SND_SOC_BIAS_ON:

		dev_dbg(codec->dev, "set_bias_on\n");
		break;

		/* partial On */
	case SND_SOC_BIAS_PREPARE:
		dev_dbg(codec->dev, "set_bias_prepare\n");
		break;

		/* Off, with power */
	case SND_SOC_BIAS_STANDBY:
		/*
		 * all power is driven by DAPM system,
		 * so output power is safe if bypass was set
		 */
		dev_dbg(codec->dev, "set_bias_stby\n");
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			snd_soc_update_bits(codec, AIC3262_POWER_CONF,
					    (AIC3262_AVDD_TO_DVDD_MASK |
					     AIC3262_EXT_ANALOG_SUPPLY_MASK),
					    0x0);
			snd_soc_update_bits(codec, AIC3262_REF_PWR_DLY,
					    AIC3262_CHIP_REF_PWR_ON_MASK,
					    AIC3262_CHIP_REF_PWR_ON);
			mdelay(40);
		}

		break;

		/* Off, without power */
	case SND_SOC_BIAS_OFF:
		dev_dbg(codec->dev, "set_bias_off\n");
		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY) {
			snd_soc_update_bits(codec, AIC3262_REF_PWR_DLY,
				AIC3262_CHIP_REF_PWR_ON_MASK, 0x0);
			snd_soc_update_bits(codec, AIC3262_POWER_CONF,
						(AIC3262_AVDD_TO_DVDD_MASK |
						AIC3262_EXT_ANALOG_SUPPLY_MASK),
						(AIC3262_AVDD_TO_DVDD |
						AIC3262_EXT_ANALOG_SUPPLY_OFF));
		}
		break;
	}

	codec->dapm.bias_level = level;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_suspend
 * Purpose  : This function is to suspend the AIC3262 driver.
 *
 *----------------------------------------------------------------------------
 */
static int aic3262_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	aic3262_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_resume
 * Purpose  : This function is to resume the AIC3262 driver
 *
 *----------------------------------------------------------------------------
 */
static int aic3262_resume(struct snd_soc_codec *codec)
{
	aic3262_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_probe
 * Purpose  : This is first driver function called by the SoC core driver.
 *
 *----------------------------------------------------------------------------
 */
static int aic3262_codec_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	int ret_btn = 0;
	struct aic3262 *control;
	struct aic3262_priv *aic3262;
	struct aic3262_jack_data *jack;

	if (codec == NULL)
		dev_err(codec->dev, "codec pointer is NULL.\n");

	codec->control_data = dev_get_drvdata(codec->dev->parent);
	control = codec->control_data;
	aic3262 = kzalloc(sizeof(struct aic3262_priv), GFP_KERNEL);
	if (aic3262 == NULL)
		return -ENOMEM;

	snd_soc_codec_set_drvdata(codec, aic3262);
	aic3262->pdata = dev_get_platdata(codec->dev->parent);
	aic3262->codec = codec;
	aic3262->cur_fw = NULL;
	aic3262->isdefault_fw = 0;
	aic3262->cfw_p = &(aic3262->cfw_ps);
	aic3xxx_cfw_init(aic3262->cfw_p, &aic3262_cfw_codec_ops, aic3262);
	aic3262->workqueue = create_singlethread_workqueue("aic3262-codec");
	if (!aic3262->workqueue) {
		ret = -ENOMEM;
		goto work_err;
	}
	ret = device_create_file(codec->dev, &dev_attr_debug_level);
	if (ret)
		dev_info(codec->dev, "Failed to add debug_level sysfs\n");
	INIT_DELAYED_WORK(&aic3262->delayed_work, aic3262_accessory_work);
	mutex_init(&aic3262->mutex);
	mutex_init(&codec->mutex);
	mutex_init(&aic3262->cfw_mutex);
	aic3262->dsp_runstate = 0;
	/* use switch-class based headset reporting if platform requires it */
	jack = &aic3262->hs_jack;
	aic3262->idev = input_allocate_device();
	if (aic3262->idev <= 0)
		printk(KERN_ERR, "Allocate failed\n");

	input_set_capability(aic3262->idev, EV_KEY, KEY_MEDIA);
	ret = input_register_device(aic3262->idev);
	if (ret < 0) {
		dev_err(codec->dev, "register input dev fail\n");
		goto input_dev_err;
	}

	if (control->irq) {
		ret = aic3262_request_irq(codec->control_data,
			AIC3262_IRQ_HEADSET_DETECT,
			aic3262_audio_handler, 0,
			"aic3262_irq_headset", codec);

		if (ret) {
			dev_err(codec->dev, "HEADSET detect irq request"
			"failed: %d\n", ret);
			goto irq_err;
		}

		ret = aic3262_request_irq(codec->control_data,
			AIC3262_IRQ_BUTTON_PRESS,
			aic3262_button_handler, 0, "aic3262_irq_button",
			codec);

		if (ret) {
			dev_err(codec->dev, "button press irq request"
			"failed: %d\n", ret);
			goto irq_err;
		}
	}
	/* Keep the reference voltage ON while in$
	   STANDBY mode for fast power up */

	snd_soc_update_bits(codec, AIC3262_REF_PWR_DLY,
			    AIC3262_CHIP_REF_PWR_ON_MASK,
			    AIC3262_CHIP_REF_PWR_ON);
	mdelay(40);
	aic3262_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	aic3262->mute_asi = 0;

	snd_soc_add_controls(codec, aic3262_snd_controls,
			ARRAY_SIZE(aic3262_snd_controls));

	aic3262_add_widgets(codec);

#ifdef AIC3262_TiLoad
	ret = aic3262_driver_init(codec);
	if (ret < 0)
		dev_err(codec->dev, "\nTiLoad Initialization failed\n");
#endif
	/* force loading the default firmware */
	aic3262_firmware_load(NULL, codec);
	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				"tlv320aic3262_fw_v1.bin", codec->dev,
				GFP_KERNEL, codec, aic3262_firmware_load);

	return 0;
irq_err:
	input_unregister_device(aic3262->idev);
	input_free_device(aic3262->idev);
input_dev_err:
reg_err:
work_err:
	kfree(aic3262);
	return 0;
}

/*
* aic3262_remove: Cleans up and Remove aic3262 soc device
* @codec: pointer variable to codec having informaton related to codec,
*
* Return: Return 0 on success.
*/
static int aic3262_codec_remove(struct snd_soc_codec *codec)
{
	/* power down chip */
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	struct aic3262 *control = codec->control_data;
	struct aic3262_jack_data *jack = &aic3262->hs_jack;

	aic3262_set_bias_level(codec, SND_SOC_BIAS_OFF);

	/* free_irq if any */
	switch (control->type) {
	case TLV320AIC3262:
		if (control->irq) {
			aic3262_free_irq(control, AIC3262_IRQ_HEADSET_DETECT,
				codec);
			aic3262_free_irq(control, AIC3262_IRQ_BUTTON_PRESS,
				codec);
		}
		break;
	}
	/* release firmware if any */
	if (aic3262->cur_fw != NULL)
		release_firmware(aic3262->cur_fw);
	/* destroy workqueue for jac dev */
	destroy_workqueue(aic3262->workqueue);
	input_unregister_device(aic3262->idev);
	input_free_device(aic3262->idev);

	kfree(aic3262);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_driver_aic326x = {
	.probe = aic3262_codec_probe,
	.remove = aic3262_codec_remove,
	.suspend = aic3262_suspend,
	.resume = aic3262_resume,
	.read = aic3262_codec_read,
	.write = aic3262_codec_write,
	.set_bias_level = aic3262_set_bias_level,
	.reg_cache_size = 0,
	.reg_word_size = sizeof(u8),
	.reg_cache_default = NULL,
};

static int aic326x_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_driver_aic326x,
				      aic326x_dai_driver,
				      ARRAY_SIZE(aic326x_dai_driver));

}

static int aic326x_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver aic326x_codec_driver = {
	.driver = {
		   .name = "tlv320aic3262-codec",
		   .owner = THIS_MODULE,
		   },
	.probe = aic326x_probe,
	.remove = __devexit_p(aic326x_remove),
};
/*
*----------------------------------------------------------------------------
* Function : tlv320aic3262_modinit
* Purpose  : module init function. First function to run.
*
*----------------------------------------------------------------------------
*/
static int __init tlv320aic3262_modinit(void)
{
	return platform_driver_register(&aic326x_codec_driver);
}

module_init(tlv320aic3262_modinit);

/*
*----------------------------------------------------------------------------
* Function : tlv320aic3262_exit
* Purpose  : module init function. First function to run.
*
*----------------------------------------------------------------------------
*/
static void __exit tlv320aic3262_exit(void)
{
	platform_driver_unregister(&aic326x_codec_driver);

}

module_exit(tlv320aic3262_exit);
MODULE_ALIAS("platform:tlv320aic3262-codec");
MODULE_DESCRIPTION("ASoC TLV320AIC3262 codec driver");
MODULE_AUTHOR("Y Preetam Sashank Reddy ");
MODULE_AUTHOR("Barani Prashanth ");
MODULE_AUTHOR("Mukund Navada K <navada@ti.com>");
MODULE_AUTHOR("Naren Vasanad <naren.vasanad@ti.com>");
MODULE_LICENSE("GPL");
