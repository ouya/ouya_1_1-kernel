#ifndef AIC3XXX_CFW_HOST_BLD
#   include <linux/module.h>
#   include <linux/delay.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/slab.h>
#include <sound/tlv.h>
#   define warn(fmt, ...) printk(fmt "\n", ##__VA_ARGS__)
#   define error(fmt, ...) printk(fmt "\n", ##__VA_ARGS__)

#else
#   define _GNU_SOURCE
#   include <stdlib.h>
#   include "utils.h"
#   include <string.h>
#   include <assert.h>
#   define EINVAL 1

#endif

#include "aic3xxx_cfw.h"
#include "aic3xxx_cfw_ops.h"
#ifndef AIC3XXX_CFW_HOST_BLD
static struct cfw_project *aic3xxx_cfw_unpickle(void *pcfw, int n);
#endif


/*
 * Firmware version numbers are used to make sure that the
 * host and target code stay in sync.  It is _not_ recommended
 * to provide this number from the outside (E.g., from a makefile)
 * Instead, a set of automated tools are relied upon to keep the numbers
 * in sync at the time of host testing.
 */
#define CFW_FW_VERSION 0x000100B3

static int aic3xxx_cfw_dlimage(struct cfw_state *ps, struct cfw_image *pim);
static int aic3xxx_cfw_dlcfg(struct cfw_state *ps, struct cfw_image *pim);
static int aic3xxx_cfw_dlctl(struct cfw_state *ps, struct cfw_block *pb,
			    u32 mute_flags);
static void aic3xxx_cfw_dlcmds(struct cfw_state *ps, struct cfw_block *pb);
static void aic3xxx_wait(struct cfw_state *ps, unsigned int reg, u8 mask,
			u8 data);
static int aic3xxx_cfw_set_mode_id(struct cfw_state *ps);
static int aic3xxx_cfw_mute(struct cfw_state *ps, int mute, u32 flags);
static int aic3xxx_cfw_setmode_cfg_u(struct cfw_state *ps, int mode, int cfg);
static int aic3xxx_cfw_setcfg_u(struct cfw_state *ps, int cfg);
static int aic3xxx_cfw_transition_u(struct cfw_state *ps, char *ttype);
static int aic3xxx_cfw_set_pll_u(struct cfw_state *ps, int asi);
static int aic3xxx_cfw_control_u(struct cfw_state *ps, char *cname, int param);


#if defined(AIC3XXX_CFW_HOST_BLD)

static int mutex_init(struct mutex *m)
{
	m->lock = 0;
	return 0;
}

static int mutex_lock(struct mutex *m)
{
	assert(m->lock == 0);
	m->lock = 1;
	return 0;
}

static int mutex_unlock(struct mutex *m)
{
	assert(m->lock == 1);
	m->lock = 0;
	return 0;
}
/*
static void mdelay(int val)
{
	int i;
	for (i = 0; i < (val * 10); i++);
}
*/
#endif

int aic3xxx_cfw_init(struct cfw_state *ps, struct aic3xxx_codec_ops const *ops,
		    void *ops_obj)
{
	ps->ops = ops;
	ps->ops_obj = ops_obj;
	ps->pjt = NULL;
	mutex_init(&ps->mutex);
	return 0;
}

int aic3xxx_cfw_lock(struct cfw_state *ps, int lock)
{
	if (lock)
		mutex_lock(&ps->mutex);
	else
		mutex_unlock(&ps->mutex);
	return 0;
}

int aic3xxx_cfw_reload(struct cfw_state *ps, void *pcfw, int n)
{
	ps->pjt = aic3xxx_cfw_unpickle(pcfw, n);
	ps->cur_mode_id =
	    ps->cur_mode = ps->cur_pfw = ps->cur_ovly = ps->cur_cfg = -1;
	if (ps->pjt == NULL)
		return -1;
	return 0;
}

int aic3xxx_cfw_setmode(struct cfw_state *ps, int mode)
{
	struct cfw_project *pjt;
	int ret;

	aic3xxx_cfw_lock(ps, 1);
	pjt = ps->pjt;
	if (pjt == NULL) {
		aic3xxx_cfw_lock(ps, 0);
		return -1;
	}
	ret = aic3xxx_cfw_setmode_cfg_u(ps, mode, pjt->mode[mode]->cfg);
	aic3xxx_cfw_lock(ps, 0);
	return ret;
}

int aic3xxx_cfw_setcfg(struct cfw_state *ps, int cfg)
{
	int ret;

	aic3xxx_cfw_lock(ps, 1);
	ret = aic3xxx_cfw_setcfg_u(ps, cfg);
	aic3xxx_cfw_lock(ps, 0);
	return ret;
}

static int aic3xxx_cfw_setcfg_u(struct cfw_state *ps, int cfg)
{
	struct cfw_project *pjt = ps->pjt;
	struct cfw_pfw *pfw;

	if (pjt == NULL)
		return -1;
	if (ps->cur_pfw < 0 || ps->cur_pfw >= pjt->npfw)
		return -1;
	if (ps->cur_cfg == cfg)
		return 0;
	pfw = pjt->pfw[ps->cur_pfw];
	if (pfw->ncfg == 0 && cfg != 0)
		return -1;
	if (cfg > 0 && cfg >= pfw->ncfg)
		return -1;
	ps->cur_cfg = cfg;
	aic3xxx_cfw_set_mode_id(ps);
	if (pfw->ncfg != 0)
		return aic3xxx_cfw_dlcfg(ps,
					 pfw->ovly_cfg[ps->cur_ovly][ps->
								     cur_cfg]);
	return 0;
}

int aic3xxx_cfw_setmode_cfg(struct cfw_state *ps, int mode, int cfg)
{
	int ret;

	aic3xxx_cfw_lock(ps, 1);
	ret = aic3xxx_cfw_setmode_cfg_u(ps, mode, cfg);
	aic3xxx_cfw_lock(ps, 0);
	return ret;
}

static int aic3xxx_cfw_setmode_cfg_u(struct cfw_state *ps, int mode, int cfg)
{
	struct cfw_project *pjt = ps->pjt;
	int which = 0;
	struct cfw_pfw *pfw;
	struct cfw_image *im;

	if (pjt == NULL)
		return -1;
	if ((mode < 0) || (mode >= pjt->nmode))
		return -1;
	if (cfg < 0)
		return -1;
	if (mode == ps->cur_mode)
		return aic3xxx_cfw_setcfg_u(ps, cfg);

	/* Apply exit sequence for previous mode if present */
	if (ps->cur_mode >= 0 && pjt->mode[ps->cur_mode]->exit)
		aic3xxx_cfw_dlcmds(ps, pjt->mode[ps->cur_mode]->exit);

	if (pjt->mode[mode]->pfw < pjt->npfw) {
		/* New mode uses miniDSP */
		pfw = pjt->pfw[pjt->mode[mode]->pfw];
		/* Make sure cfg is valid and supported in this mode */
		if (pfw->ncfg == 0 && cfg != 0)
			return -1;
		if (cfg > 0 && cfg >= pfw->ncfg)
			return -1;
		/*
		 * Decisions about which miniDSP to stop/restart are taken
		 * on the basis of sections present in the _base_ image
		 * This allows for correct sync mode operation even in cases
		 * where the base PFW uses both miniDSPs where a particular
		 * overlay applies only to one
		 */
		im = pfw->base;
		if (im->block[CFW_BLOCK_A_INST])
			which |= AIC3XX_COPS_MDSP_A;
		if (im->block[CFW_BLOCK_D_INST])
			which |= AIC3XX_COPS_MDSP_D;

		/* New mode requires different PFW */
		if (pjt->mode[mode]->pfw != ps->cur_pfw) {
			ps->cur_pfw = pjt->mode[mode]->pfw;
			ps->cur_ovly = 0;
			ps->cur_cfg = 0;

			which = ps->ops->stop(ps->ops_obj, which);
			aic3xxx_cfw_dlimage(ps, im);
			if (pjt->mode[mode]->ovly
				&& pjt->mode[mode]->ovly < pfw->novly) {
				/* New mode uses ovly */
				if (pfw->ovly_cfg[pjt->mode[mode]
					->ovly][cfg] != NULL)
					aic3xxx_cfw_dlimage(ps,
							pfw->ovly_cfg[pjt->
							mode[mode]->
							ovly][cfg]);
			} else if (pfw->ncfg > 0) {
				/* new mode needs only a cfg change */
				aic3xxx_cfw_dlimage(ps, pfw->ovly_cfg[0][cfg]);
			}
			ps->ops->restore(ps->ops_obj, which);

		} else if (pjt->mode[mode]->ovly != ps->cur_ovly) {
			/* New mode requires only an ovly change */
			which = ps->ops->stop(ps->ops_obj, which);
			aic3xxx_cfw_dlimage(ps,
					    pfw->ovly_cfg[pjt->mode[mode]->
							  ovly][cfg]);
			ps->ops->restore(ps->ops_obj, which);
		} else if (pfw->ncfg > 0 && cfg != ps->cur_cfg) {
				/* New mode requires only a cfg change */
			aic3xxx_cfw_dlcfg(ps,
					  pfw->ovly_cfg[pjt->mode[mode]->
							ovly][cfg]);
		}
		ps->cur_ovly = pjt->mode[mode]->ovly;
		ps->cur_cfg = cfg;

		ps->cur_mode = mode;
		aic3xxx_cfw_set_pll_u(ps, 0);

	} else if (pjt->mode[mode]->pfw != 0xFF) {
		warn("Bad pfw setting detected (%d).  Max pfw=%d",
		     pjt->mode[mode]->pfw, pjt->npfw);
	}
	ps->cur_mode = mode;
	aic3xxx_cfw_set_mode_id(ps);
	/* Transition to netural mode */
	aic3xxx_cfw_transition_u(ps, "NEUTRAL");
	/* Apply entry sequence if present */
	if (pjt->mode[mode]->entry)
		aic3xxx_cfw_dlcmds(ps, pjt->mode[mode]->entry);
	DBG("setmode_cfg: DONE (mode=%d pfw=%d ovly=%d cfg=%d)", ps->cur_mode,
	    ps->cur_pfw, ps->cur_ovly, ps->cur_cfg);
	return 0;
}

int aic3xxx_cfw_transition(struct cfw_state *ps, char *ttype)
{
	int ret;

	aic3xxx_cfw_lock(ps, 1);
	ret = aic3xxx_cfw_transition_u(ps, ttype);
	aic3xxx_cfw_lock(ps, 0);
	return ret;
}

static int aic3xxx_cfw_transition_u(struct cfw_state *ps, char *ttype)
{
	int i;

	if (ps->pjt == NULL)
		return -1;
	for (i = 0; i < CFW_TRN_N; ++i) {
		if (!strcasecmp(ttype, cfw_transition_id[i])) {
			DBG("Sending transition %s[%d]", ttype, i);
			if (ps->pjt->transition[i]) {
				aic3xxx_cfw_dlcmds(ps,
						   ps->pjt->transition[i]->
						   block);
			}
			return 0;
		}
	}
	warn("Transition %s not present or invalid", ttype);
	return 0;
}

int aic3xxx_cfw_set_pll(struct cfw_state *ps, int asi)
{
	int ret;

	aic3xxx_cfw_lock(ps, 1);
	ret = aic3xxx_cfw_set_pll_u(ps, asi);
	aic3xxx_cfw_lock(ps, 0);
	return ret;
}

static int aic3xxx_cfw_set_pll_u(struct cfw_state *ps, int asi)
{
	struct cfw_project *pjt = ps->pjt;
	struct cfw_pfw *pfw;

	if (pjt == NULL)
		return -1;
	if (ps->cur_mode < 0)
		return -EINVAL;
	pfw = pjt->pfw[pjt->mode[ps->cur_mode]->pfw];
	if (pfw->pll) {
		DBG("Configuring PLL for ASI%d using PFW%d", asi,
		    pjt->mode[ps->cur_mode]->pfw);
		aic3xxx_cfw_dlcmds(ps, pfw->pll);
	}
	return 0;
}

int aic3xxx_cfw_control(struct cfw_state *ps, char *cname, int param)
{
	int ret;

	aic3xxx_cfw_lock(ps, 1);
	ret = aic3xxx_cfw_control_u(ps, cname, param);
	aic3xxx_cfw_lock(ps, 0);
	return ret;
}

static int aic3xxx_cfw_control_u(struct cfw_state *ps, char *cname, int param)
{
	struct cfw_pfw *pfw;
	int i;

	if (ps->cur_pfw < 0 || ps->cur_pfw >= ps->pjt->npfw) {
		warn("Not in MiniDSP mode");
		return 0;
	}
	pfw = ps->pjt->pfw[ps->cur_pfw];
	for (i = 0; i < pfw->nctrl; ++i) {
		if (!strcasecmp(cname, pfw->ctrl[i]->name)) {
			struct cfw_control *pc = pfw->ctrl[i];
			if (param < 0 || param > pc->imax) {
				warn("Parameter out of range\n");
				return -EINVAL;
			}
			DBG("Sending control %s[%d]", cname, param);
			pc->icur = param;
			aic3xxx_cfw_dlctl(ps, pc->output[param],
					  pc->mute_flags);
			return 0;
		}
	}
	warn("Control named %s nort found in pfw %s", cname, pfw->name);

	return 0;
}

static void aic3xxx_cfw_dlcmds(struct cfw_state *ps, struct cfw_block *pb)
{
	int i = 0, lock = 0;

	while (i < pb->ncmds) {
		if (CFW_BLOCK_BURSTS(pb->type))
			ps->ops->bulk_write(ps->ops_obj,
					    pb->cmd[i].burst->reg.bpod,
					    pb->cmd[i].burst->length,
					    pb->cmd[i].burst->data);
		else {
			struct cfw_meta_delay d = pb->cmd[i].reg.meta.delay;
			struct cfw_meta_bitop b = pb->cmd[i].reg.meta.bitop;
			switch (pb->cmd[i].reg.meta.mcmd) {
			case CFW_META_DELAY:
				mdelay(d.delay);
				break;
			case CFW_META_UPDTBITS:
				ps->ops->set_bits(ps->ops_obj,
						  pb->cmd[i + 1].reg.bpod,
						  b.mask,
						  pb->cmd[i + 1].reg.data);
				i++;
				break;
			case CFW_META_WAITBITS:
				aic3xxx_wait(ps, pb->cmd[i + 1].reg.bpod,
					     b.mask, pb->cmd[i + 1].reg.data);
				i++;
				break;
			case CFW_META_LOCK:
				if (d.delay) {
					ps->ops->lock(ps->ops_obj);
					lock = 1;
				} else {
					if (!lock)
						error("already lock\n");
					ps->ops->unlock(ps->ops_obj);
					lock = 0;
				}
				break;
			default:
				ps->ops->reg_write(ps->ops_obj,
						   pb->cmd[i].reg.bpod,
						   pb->cmd[i].reg.data);
			}
		}
		++i;
	}
	if (lock)
		error("exiting blkcmds with lock ON");
}

static void aic3xxx_wait(struct cfw_state *ps, unsigned int reg, u8 mask,
			u8 data)
{
	while ((ps->ops->reg_read(ps->ops_obj, reg) & mask) != data)
		mdelay(2);
}

static const struct {
	u32 mdsp;
	int buf_a, buf_b;
	u32 swap;
} csecs[] = {
	{
		.mdsp = AIC3XX_COPS_MDSP_A,
		.swap = AIC3XX_ABUF_MDSP_A,
		.buf_a = CFW_BLOCK_A_A_COEF,
		.buf_b = CFW_BLOCK_A_B_COEF
	},
	{
		.mdsp = AIC3XX_COPS_MDSP_D,
		.swap = AIC3XX_ABUF_MDSP_D1,
		.buf_a = CFW_BLOCK_D_A1_COEF,
		.buf_b = CFW_BLOCK_D_B1_COEF
	},
	{
		.mdsp = AIC3XX_COPS_MDSP_D,
		.swap = AIC3XX_ABUF_MDSP_D2,
		.buf_a = CFW_BLOCK_D_A2_COEF,
		.buf_b = CFW_BLOCK_D_B2_COEF
	},
};

static int aic3xxx_cfw_dlctl(struct cfw_state *ps, struct cfw_block *pb,
				u32 mute_flags)
{
	int i, btype = CFW_BLOCK_TYPE(pb->type);
	int run_state = ps->ops->lock(ps->ops_obj);

	DBG("Download CTL");
	for (i = 0; i < sizeof(csecs) / sizeof(csecs[0]); ++i) {
		if (csecs[i].buf_a == btype || csecs[i].buf_b == btype) {
			DBG("\tDownload once to %d", btype);
			aic3xxx_cfw_dlcmds(ps, pb);
			if (run_state & csecs[i].mdsp) {
				DBG("Download again %d", btype);
				aic3xxx_cfw_mute(ps, 1, run_state & mute_flags);
				ps->ops->bswap(ps->ops_obj, csecs[i].swap);
				aic3xxx_cfw_mute(ps, 0, run_state & mute_flags);
				aic3xxx_cfw_dlcmds(ps, pb);
			}
			break;
		}
	}
	ps->ops->unlock(ps->ops_obj);
	return 0;
}

static int aic3xxx_cfw_dlcfg(struct cfw_state *ps, struct cfw_image *pim)
{
	int i, run_state, swap;

	DBG("Download CFG %s", pim->name);
	run_state = ps->ops->lock(ps->ops_obj);
	swap = 0;
	for (i = 0; i < sizeof(csecs) / sizeof(csecs[0]); ++i) {
		if (pim->block[csecs[i].buf_a]) {
			if (run_state & csecs[i].mdsp) {
				aic3xxx_cfw_dlcmds(ps,
						   pim->block[csecs[i].buf_a]);
				swap |= csecs[i].swap;
			} else {
				aic3xxx_cfw_dlcmds(ps,
						   pim->block[csecs[i].buf_a]);
				aic3xxx_cfw_dlcmds(ps,
						   pim->block[csecs[i].buf_b]);
			}
		}
	}
	if (swap) {
		aic3xxx_cfw_mute(ps, 1, run_state & pim->mute_flags);
		ps->ops->bswap(ps->ops_obj, swap);
		aic3xxx_cfw_mute(ps, 0, run_state & pim->mute_flags);
		for (i = 0; i < sizeof(csecs) / sizeof(csecs[0]); ++i) {
			if (pim->block[csecs[i].buf_a]) {
				if (run_state & csecs[i].mdsp)
					aic3xxx_cfw_dlcmds(ps,
							pim->block[csecs[i].
							buf_a]);
			}
		}
	}
	ps->ops->unlock(ps->ops_obj);
	return 0;
}

static int aic3xxx_cfw_dlimage(struct cfw_state *ps, struct cfw_image *pim)
{
	int i;

	DBG("Download IMAGE %s", pim->name);
	for (i = 0; i < CFW_BLOCK_N; ++i)
		if (pim->block[i])
			aic3xxx_cfw_dlcmds(ps, pim->block[i]);
	return 0;
}

static int aic3xxx_cfw_mute(struct cfw_state *ps, int mute, u32 flags)
{
	if ((flags & AIC3XX_COPS_MDSP_D) && (flags & AIC3XX_COPS_MDSP_A))
		aic3xxx_cfw_transition_u(ps, mute ? "AD_MUTE" : "AD_UNMUTE");
	else if (flags & AIC3XX_COPS_MDSP_D)
		aic3xxx_cfw_transition_u(ps, mute ? "D_MUTE" : "D_UNMUTE");
	else if (flags & AIC3XX_COPS_MDSP_A)
		aic3xxx_cfw_transition_u(ps, mute ? "A_MUTE" : "A_UNMUTE");
	return 0;
}

#define FW_NDX2PTR(x, b) do {                        \
x = (void *)((u8 *)(b) + ((int)(x)));           \
} while (0)

static void aic3xxx_cfw_unpickle_block(struct cfw_block *pb, void *p)
{
	int i;

	if (CFW_BLOCK_BURSTS(pb->type))
		for (i = 0; i < pb->ncmds; ++i)
			FW_NDX2PTR(pb->cmd[i].burst, p);
}

static void aic3xxx_cfw_unpickle_image(struct cfw_image *im, void *p)
{
	int i;
	for (i = 0; i < CFW_BLOCK_N; ++i)
		if (im->block[i]) {
			FW_NDX2PTR(im->block[i], p);
			aic3xxx_cfw_unpickle_block(im->block[i], p);
		}
}

static void aic3xxx_cfw_unpickle_control(struct cfw_control *ct, void *p)
{
	int i;
	FW_NDX2PTR(ct->output, p);
	for (i = 0; i <= ct->imax; ++i) {
		FW_NDX2PTR(ct->output[i], p);
		aic3xxx_cfw_unpickle_block(ct->output[i], p);
	}
}
#ifndef AIC3XXX_CFW_HOST_BLD
static
#endif
unsigned int crc32(unsigned int *pdata, int n)
{
	u32 crc = 0, i, crc_poly = 0x04C11DB7;	/* CRC - 32 */
	u32 msb;
	u32 residue_value;
	int bits;

	for (i = 0; i < (n >> 2); i++) {
		bits = 32;
		while (--bits >= 0) {
			msb = crc & 0x80000000;
			crc = (crc << 1) ^ ((*pdata >> bits) & 1);
			if (msb)
				crc = crc ^ crc_poly;
		}
		pdata++;
	}

	switch (n & 3) {
	case 0:
		break;
	case 1:
		residue_value = (*pdata & 0xFF);
		bits = 8;
		break;
	case 2:
		residue_value = (*pdata & 0xFFFF);
		bits = 16;
		break;
	case 3:
		residue_value = (*pdata & 0xFFFFFF);
		bits = 24;
		break;
	}

	if (n & 3) {
		while (--bits >= 0) {
			msb = crc & 0x80000000;
			crc = (crc << 1) ^ ((residue_value >> bits) & 1);
			if (msb)
				crc = crc ^ crc_poly;
		}
	}
	return crc;
}

static int crc_chk(void *p, int n)
{
	struct cfw_project *pjt = (void *)p;
	u32 crc = pjt->cksum, crc_comp;

	pjt->cksum = 0;
	DBG("Entering crc %d", n);
	crc_comp = crc32(p, n);
	if (crc_comp != crc) {
		DBG("CRC mismatch 0x%08X != 0x%08X", crc, crc_comp);
		return 0;
	}
	DBG("CRC pass");
	pjt->cksum = crc;
	return 1;
}
#ifndef AIC3XXX_CFW_HOST_BLD
static
#endif
struct cfw_project *aic3xxx_cfw_unpickle(void *p, int n)
{
	struct cfw_project *pjt = p;
	int i, j, k;

	if (pjt->magic != CFW_FW_MAGIC ||
		pjt->size != n || pjt->bmagic != CFW_FW_VERSION ||
		!crc_chk(p, n)) {
		error
		("magic:0x%08X!=0x%08X || size:%d!=%d ||version:0x%08X!=0x%08X",
		pjt->magic, CFW_FW_MAGIC, pjt->size, n, pjt->cksum,
		CFW_FW_VERSION);

		return NULL;
	}
	DBG("Loaded firmware inside unpickle\n");

	for (i = 0; i < CFW_MAX_TRANSITIONS; i++) {
		if (pjt->transition[i]) {
			FW_NDX2PTR(pjt->transition[i], p);
			FW_NDX2PTR(pjt->transition[i]->block, p);
			aic3xxx_cfw_unpickle_block(pjt->transition[i]->block,
						   p);
		}
	}

	for (i = 0; i < pjt->npfw; i++) {
		DBG("loading pfw %d\n", i);
		FW_NDX2PTR(pjt->pfw[i], p);
		if (pjt->pfw[i]->base) {
			FW_NDX2PTR(pjt->pfw[i]->base, p);
			aic3xxx_cfw_unpickle_image(pjt->pfw[i]->base, p);
		}
		if (pjt->pfw[i]->pll) {
			FW_NDX2PTR(pjt->pfw[i]->pll, p);
			aic3xxx_cfw_unpickle_block(pjt->pfw[i]->pll, p);
		}
		for (j = 0; j < pjt->pfw[i]->novly; ++j)
			for (k = 0; k < pjt->pfw[i]->ncfg; ++k) {
				FW_NDX2PTR(pjt->pfw[i]->ovly_cfg[j][k], p);
				aic3xxx_cfw_unpickle_image(pjt->pfw[i]->
							   ovly_cfg[j][k], p);
			}
		for (j = 0; j < pjt->pfw[i]->nctrl; ++j) {
			FW_NDX2PTR(pjt->pfw[i]->ctrl[j], p);
			aic3xxx_cfw_unpickle_control(pjt->pfw[i]->ctrl[j], p);
		}
	}

	DBG("loaded pfw's\n");
	for (i = 0; i < pjt->nmode; i++) {
		FW_NDX2PTR(pjt->mode[i], p);
		if (pjt->mode[i]->entry) {
			FW_NDX2PTR(pjt->mode[i]->entry, p);
			aic3xxx_cfw_unpickle_block(pjt->mode[i]->entry, p);
		}
		if (pjt->mode[i]->exit) {
			FW_NDX2PTR(pjt->mode[i]->exit, p);
			aic3xxx_cfw_unpickle_block(pjt->mode[i]->exit, p);
		}
	}
	if (pjt->asoc_toc)
		FW_NDX2PTR(pjt->asoc_toc, p);
	else {
		warn("asoc_toc not defined.  FW version mismatch?");
		return NULL;
	}
	DBG("loaded modes");
	return pjt;
}

#ifndef AIC3XXX_CFW_HOST_BLD
static int aic3xxx_get_control(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cfw_state *ps =
		(struct cfw_state *) kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cfw_pfw *pfw;
	int i;

	if (ps->cur_pfw >= ps->pjt->npfw) {
		DBG("Not in MiniDSP mode");
		return 0;
	}
	pfw = ps->pjt->pfw[ps->cur_pfw];
	for (i = 0; i < pfw->nctrl; ++i) {
		if (!strcasecmp(kcontrol->id.name, pfw->ctrl[i]->name)) {
			struct cfw_control *pc = pfw->ctrl[i];
			ucontrol->value.integer.value[0] = pc->icur;
			return 0;
		}
	}
	return 0;
}

static int aic3xxx_put_control(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct cfw_state *ps =
		(struct cfw_state *) kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);


	aic3xxx_cfw_control(ps, kcontrol->id.name,
				ucontrol->value.integer.value[0]);
	return 0;
}

static int aic3xxx_info_control(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *ucontrol)
{
	struct cfw_state *ps =
		(struct cfw_state *) kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cfw_pfw *pfw;
	int i;

	if (ps->cur_pfw >= ps->pjt->npfw) {
		DBG("Not in MiniDSP mode");
		return 0;
	}
	pfw = ps->pjt->pfw[ps->cur_pfw];
	for (i = 0; i < pfw->nctrl; ++i) {
		if (!strcasecmp(kcontrol->id.name, pfw->ctrl[i]->name)) {
			struct cfw_control *pc = pfw->ctrl[i];
			ucontrol->value.integer.min = 0;
			ucontrol->value.integer.max = pc->imax;
			if (pc->imax == 1)
				ucontrol->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
			else
				ucontrol->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		}
	}

	ucontrol->count = 1;
	return 0;
}
#endif
int aic3xxx_cfw_add_controls(struct snd_soc_codec *codec, struct cfw_state *ps)
{
	int i, j;
	struct cfw_pfw *pfw;

	for (j = 0; j < ps->pjt->npfw; ++j) {
		pfw = ps->pjt->pfw[j];

		for (i = 0; i < pfw->nctrl; ++i) {
			struct cfw_control *pc = pfw->ctrl[i];
#ifndef AIC3XXX_CFW_HOST_BLD
			struct snd_kcontrol_new *generic_control =
				kzalloc(sizeof(struct snd_kcontrol_new),
					GFP_KERNEL);
			unsigned int *tlv_array =
				kzalloc(4 * sizeof(unsigned int), GFP_KERNEL);

			if (generic_control == NULL)
				return -ENOMEM;
			generic_control->access =
				SNDRV_CTL_ELEM_ACCESS_TLV_READ |
				SNDRV_CTL_ELEM_ACCESS_READWRITE;
			tlv_array[0] = SNDRV_CTL_TLVT_DB_SCALE;
			tlv_array[1] = 2 * sizeof(unsigned int);
			tlv_array[2] = pc->min;
			tlv_array[3] = ((pc->step) & TLV_DB_SCALE_MASK);
			if (pc->step > 0)
				generic_control->tlv.p = tlv_array;
			generic_control->name = pc->name;
			generic_control->private_value = (unsigned long) ps;
			generic_control->get = aic3xxx_get_control;
			generic_control->put = aic3xxx_put_control;
			generic_control->info = aic3xxx_info_control;
			generic_control->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
#endif
			DBG("Adding control %s", pc->name);
#ifndef AIC3XXX_CFW_HOST_BLD
			snd_soc_add_controls(codec, generic_control, 1);
#endif
		}
	}
	return 0;

}

static int aic3xxx_cfw_set_mode_id(struct cfw_state *ps)
{
	struct cfw_asoc_toc *toc = ps->pjt->asoc_toc;
	int i;

	for (i = 0; i < toc->nentries; ++i) {
		if (toc->entry[i].cfg == ps->cur_cfg &&
		    toc->entry[i].mode == ps->cur_mode) {
			ps->cur_mode_id = i;
			return 0;
		}
	}
	DBG("Unknown mode, cfg combination [%d, %d]",
		ps->cur_mode, ps->cur_cfg);
	return -1;
}
#ifndef AIC3XXX_CFW_HOST_BLD
static int aic3xxx_get_mode(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct cfw_state *ps = (struct cfw_state *)e->mask;

	ucontrol->value.enumerated.item[0] = ps->cur_mode_id;

	return 0;
}
#endif
#ifndef AIC3XXX_CFW_HOST_BLD
static int aic3xxx_put_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct cfw_state *ps = (struct cfw_state *)e->mask;
	struct cfw_asoc_toc *toc;
	int index, ret;

	aic3xxx_cfw_lock(ps, 1);
	toc = ps->pjt->asoc_toc;

	index = ucontrol->value.enumerated.item[0];
	if (index < 0 || index >= toc->nentries) {
		aic3xxx_cfw_lock(ps, 0);
		return -EINVAL;
	}
	ret =
	    aic3xxx_cfw_setmode_cfg_u(ps, toc->entry[index].mode,
				      toc->entry[index].cfg);
	aic3xxx_cfw_lock(ps, 0);
	return ret;
}
#endif

int aic3xxx_cfw_add_modes(struct snd_soc_codec *codec, struct cfw_state *ps)
{
#ifndef AIC3XXX_CFW_HOST_BLD
	int j;
	struct cfw_asoc_toc *toc = ps->pjt->asoc_toc;
	struct soc_enum *mode_cfg_enum =
	    kzalloc(sizeof(struct soc_enum), GFP_KERNEL);
	struct snd_kcontrol_new *mode_cfg_control =
	    kzalloc(sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	char **enum_texts;

	if (mode_cfg_enum == NULL)
		goto mem_err;
	if (mode_cfg_control == NULL)
		goto mem_err;

	mode_cfg_enum->texts =
	    kzalloc(toc->nentries * sizeof(char *), GFP_KERNEL);
	if (mode_cfg_enum->texts == NULL)
		goto mem_err;
	/* Hack to overwrite the const * const pointer */
	enum_texts = (char **)mode_cfg_enum->texts;

	for (j = 0; j < toc->nentries; j++)
		enum_texts[j] = toc->entry[j].etext;
	mode_cfg_enum->reg = j;
	mode_cfg_enum->max = toc->nentries;
	mode_cfg_enum->mask = (unsigned int)ps;
	mode_cfg_control->name = "Codec Firmware Setmode";
	mode_cfg_control->get = aic3xxx_get_mode;
	mode_cfg_control->put = aic3xxx_put_mode;
	mode_cfg_control->info = snd_soc_info_enum_ext;
	mode_cfg_control->private_value = (unsigned long)mode_cfg_enum;
	mode_cfg_control->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	snd_soc_add_controls(codec, mode_cfg_control, 1);
	return 0;
mem_err:
	kfree(mode_cfg_control);
	kfree(mode_cfg_enum);
	kfree(mode_cfg_enum->texts);
	return -ENOMEM;
#else
	return 0;
#endif

}
