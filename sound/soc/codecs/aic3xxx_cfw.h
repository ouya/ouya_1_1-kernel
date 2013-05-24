/**
 * \file Codec Firmware Declarations
 */

#ifndef CFW_FIRMWARE_H_
#define CFW_FIRMWARE_H_
/** \defgroup bt Basic Types */
/* @{ */
#ifndef AIC3XXX_CFW_HOST_BLD
#include <asm-generic/int-ll64.h>
#else
typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned long  int u32;
#endif

#define CFW_FW_MAGIC 0xC0D1F1ED

/** defgroup pd Arbitrary Limitations */

#ifndef CFW_MAX_ID
#define CFW_MAX_ID	  (64)    /* Max length of string identifies */
#endif

#ifndef CFW_MAX_DESC
#define  CFW_MAX_DESC	(512)   /* Max length of description */
#endif
/* <Max number of overlays per PFW */
#ifndef CFW_MAX_NOVLY
#define CFW_MAX_NOVLY		(4)
#endif

#ifndef CFW_MAX_NCFG
#define CFW_MAX_NCFG	(16)    /* Max number of configurations per PFW */
#endif

#ifndef CFW_MAX_TRANSITIONS
#define CFW_MAX_TRANSITIONS (32)    /* max number of pre-defined transition */
#endif

#ifndef CFW_MAX_NPFW
#define CFW_MAX_NPFW	(16)    /* Max number fo process flows */
#endif

#ifndef CFW_MAX_MODES
#define CFW_MAX_MODES       (32)    /* Max number of modes */
#endif

#ifndef CFW_MAX_ASI
#define CFW_MAX_ASI	 (4)     /* Max number ASIs in a single device */
#endif


#ifndef CFW_MAX_CTRL
#define CFW_MAX_CTRL	(16)    /* Max number of control per pfw */
#endif

/** defgroup st Enums, Flags, Macros and Supporting Types */

/**
 * Sample rate bitmask
 *
 */
enum cfw_fs {
	CFW_FS_8KHZ     = 0x0001u,
	CFW_FS_11KHZ    = 0x0002u,
	CFW_FS_16KHZ    = 0x0004u,
	CFW_FS_22KHZ    = 0x0008u,
	CFW_FS_24KHZ    = 0x0010u,
	CFW_FS_32KHZ    = 0x0020u,
	CFW_FS_44KHZ    = 0x0040u,
	CFW_FS_48KHZ    = 0x0080u,
	CFW_FS_88KHZ    = 0x0100u,
	CFW_FS_96KHZ    = 0x0200u,
	CFW_FS_176KHZ   = 0x0400u,
	CFW_FS_192KHZ   = 0x0800u,
	CFW_FS_ANY      = 0x8000u,
	CFW_FS_ALL      = 0x0FFFu,
};

/**
 * Sample rate index
 *
 */
enum cfw_fsi {
	CFW_FSI_8KHZ,
	CFW_FSI_11KHZ,
	CFW_FSI_16KHZ,
	CFW_FSI_22KHZ,
	CFW_FSI_24KHZ,
	CFW_FSI_32KHZ,
	CFW_FSI_44KHZ,
	CFW_FSI_48KHZ,
	CFW_FSI_88KHZ,
	CFW_FSI_96KHZ,
	CFW_FSI_176KHZ,
	CFW_FSI_192KHZ,
	CFW_FSI_ANY = 15,
};

/**
 * Device Family Identifier
 *
 */
enum __attribute__ ((__packed__)) cfw_dfamily {
	CFW_DFM_TYPE_A,
	CFW_DFM_TYPE_B,
	CFW_DFM_TYPE_C
};

/**
 * Device Identifier
 *
 */
enum __attribute__ ((__packed__)) cfw_device {
	CFW_DEV_DAC3120,
	CFW_DEV_DAC3100,

	CFW_DEV_AIC3120,
	CFW_DEV_AIC3100,
	CFW_DEV_AIC3110,
	CFW_DEV_AIC3111,

	CFW_DEV_AIC36,

	CFW_DEV_AIC3206,
	CFW_DEV_AIC3204,
	CFW_DEV_AIC3254,
	CFW_DEV_AIC3256,
	CFW_DEV_AIC3253,

	CFW_DEV_AIC3212,
	CFW_DEV_AIC3262,
	CFW_DEV_AIC3017,
	CFW_DEV_AIC3008,

};

/**
 * Transition Sequence Identifier
 *
 */
enum cfw_transition_t {
	CFW_TRN_INIT,
	CFW_TRN_RESUME,
	CFW_TRN_NEUTRAL,
	CFW_TRN_A_MUTE,
	CFW_TRN_D_MUTE,
	CFW_TRN_AD_MUTE,
	CFW_TRN_A_UNMUTE,
	CFW_TRN_D_UNMUTE,
	CFW_TRN_AD_UNMUTE,
	CFW_TRN_SUSPEND,
	CFW_TRN_EXIT,
	CFW_TRN_N
};

static const char * const cfw_transition_id[] = {
	[CFW_TRN_INIT] "INIT",
	[CFW_TRN_RESUME] "RESUME",
	[CFW_TRN_NEUTRAL] "NEUTRAL",
	[CFW_TRN_A_MUTE] "A_MUTE",
	[CFW_TRN_D_MUTE] "D_MUTE",
	[CFW_TRN_AD_MUTE] "AD_MUTE",
	[CFW_TRN_A_UNMUTE] "A_UNMUTE",
	[CFW_TRN_D_UNMUTE] "D_UNMUTE",
	[CFW_TRN_AD_UNMUTE] "AD_UNMUTE",
	[CFW_TRN_SUSPEND] "SUSPEND",
	[CFW_TRN_EXIT] "EXIT",
};

/** defgroup ds Data Structures */

/**
* CFW Meta Command
* These commands do not appear in the register
* set of the device.
* Mainly delay, wait and set_bits.
*/
enum __attribute__ ((__packed__)) cfw_meta_cmd {
	CFW_META_DELAY = 0x80,
	CFW_META_UPDTBITS,
	CFW_META_WAITBITS,
	CFW_META_LOCK,
};

/**
* CFW Delay
* Used for the meta command delay
* Has one parameter of delay time in ms
*/
struct cfw_meta_delay {
	u16 delay;
	enum cfw_meta_cmd mcmd;
	u8	unused1;
};

/**
* CFW set_bits or wait
* Both these meta commands have same arguments
* mcmd will be used to specify which command it is
* has parameters of book, page, offset and mask
*/
struct cfw_meta_bitop {
	u16 unused1;
	enum cfw_meta_cmd mcmd;
	u8   mask;
};

/**
* CFW meta register
* Contains the data structures for the meta commands
*/
union cfw_meta_register {
	struct {
		u16 unused1;
		enum cfw_meta_cmd mcmd;
		u8 unused2;
	};
	struct cfw_meta_delay delay;
	struct cfw_meta_bitop bitop;
};

/**
 * CFW Register
 *
 * A single reg write
 *
 */
union cfw_register {
	struct {
		u8 book;
		u8 page;
		u8 offset;
		u8 data;
	};
	u32 bpod;
	union cfw_meta_register meta;
};

/**
 * CFW Burst
 *
 * A single I2C/SPI burst write sequence
 *
 */
struct cfw_burst {
	u32 length;
	union {
		union cfw_register reg;
		struct {
			u8 bpo[3];
			u8 data[1];
		};
	};
};

/**
 * CFW Command
 *
 * Can be a either a
 *      -# single register write,
 *      -# a burst write, or
 *      -# meta-command
 *
 */
union cfw_cmd {
	union cfw_register reg;
	struct cfw_burst    *burst;
};

/**
 * CFW Block Type
 *
 * Block identifier
 *
 */
enum __attribute__ ((__packed__)) cfw_block_t {
	CFW_BLOCK_SYSTEM_PRE,
	CFW_BLOCK_A_INST,
	CFW_BLOCK_A_A_COEF,
	CFW_BLOCK_A_B_COEF,
	CFW_BLOCK_A_F_COEF,
	CFW_BLOCK_D_INST,
	CFW_BLOCK_D_A1_COEF,
	CFW_BLOCK_D_B1_COEF,
	CFW_BLOCK_D_A2_COEF,
	CFW_BLOCK_D_B2_COEF,
	CFW_BLOCK_D_F_COEF,
	CFW_BLOCK_SYSTEM_POST,
	CFW_BLOCK_N,
	CFW_BLOCK_INVALID,
	CFW_BLOCK_BURSTS = 0x80
};
#define CFW_BLOCK_BURSTS(x) ((x)&CFW_BLOCK_BURSTS)
#define CFW_BLOCK_TYPE(x) ((x)&(~CFW_BLOCK_BURSTS))
#define CFW_BLOCK_D_A_COEF CFW_BLOCK_D_A1_COEF
#define CFW_BLOCK_D_B_COEF CFW_BLOCK_D_B1_COEF

/**
 * CFW Block
 *
 * A block of logically grouped sequences/commands/meta-commands
 *
 */
struct cfw_block {
	enum cfw_block_t type;
	int ncmds;
	union cfw_cmd cmd[];
};

/**
 * CFW Image
 *
 * A downloadable image
 */
struct cfw_image {
	char name[CFW_MAX_ID];	/* Name of the pfw/overlay/configuration */
	char desc[CFW_MAX_DESC];	/* User string */
	int  mute_flags;
	struct cfw_block *block[CFW_BLOCK_N];
};

struct cfw_control {
	char name[CFW_MAX_ID];  /* Control identifier */
	char desc[CFW_MAX_DESC];/* User string */
	int  mute_flags;

	int  min;	       /* Min value of control (*100) */
	int  max;	       /* Max  value of control (*100) */
	int  step;	      /* Control step size (*100) */

	int  imax;	      /* Max index into controls array */
	int  ireset;	    /* Reset control to defaults */
	int  icur;	      /* Last value set */
	struct cfw_block **output;     /* Array of sequences to send */
};

/**
 * Process flow
 *
 * Complete description of a process flow
 */
struct cfw_pfw {
	char name[CFW_MAX_ID];  /* Name of the process flow */
	char desc[CFW_MAX_DESC];    /* User string */
	u32 version;
	u8  prb_a;
	u8  prb_d;
	int novly;	      /* Number of overlays (1 or more) */
	int ncfg;	       /* Number of configurations (0 or more) */
	int nctrl;	      /* Number of run-time controls */
	struct cfw_block *pll;
	struct cfw_image *base;	/* Base sequence */
	/* Overlay and cfg */
	struct cfw_image *ovly_cfg[CFW_MAX_NOVLY][CFW_MAX_NCFG];
	/* Array of run-time controls */
	struct cfw_control *ctrl[CFW_MAX_CTRL];
};

/**
 * Process transition
 *
 * Sequence for specific state transisitions within the driver
 *
 */
struct cfw_transition {
	char name[CFW_MAX_ID];		/* Name of the transition */
	char desc[CFW_MAX_DESC];	/* User string */
	struct cfw_block *block;
};

/**
 * Device audio mode
 *
 * Structure linking various operating modes to process flows,
 * configurations and sequences
 *
 */
struct cfw_mode {
	char name[CFW_MAX_ID];
	char desc[CFW_MAX_DESC];    /* User string */
	u32 flags;
	u8  pfw;
	u8  ovly;
	u8  cfg;
	struct cfw_block *entry;
	struct cfw_block *exit;
};

struct cfw_asoc_toc_entry {
	char etext[CFW_MAX_ID];
	int mode;
	int cfg;
};

struct cfw_asoc_toc {
	int nentries;
	struct cfw_asoc_toc_entry entry[];
};

/**
 * CFW Project
 *
 * Top level structure describing the CFW project
 */
struct cfw_project {
	u32 magic;
	u32 bmagic;
	u32 size;
	u32 cksum;
	u32 version;
	u32 tstamp;
	char name[CFW_MAX_ID];      /* Project name */
	char desc[CFW_MAX_DESC];    /* User string */
	enum cfw_dfamily dfamily;
	enum cfw_device  device;
	u32  flags;
	struct cfw_transition *transition[CFW_MAX_TRANSITIONS];
	u16  npfw;		   /* Number of process flows */
	u16  nmode;		  /* Number of operating modes */
	struct cfw_pfw *pfw[CFW_MAX_NPFW]; /* Indices to PFW locations */
	struct cfw_mode *mode[CFW_MAX_MODES];
	struct cfw_asoc_toc *asoc_toc;
};

#endif /* CFW_FIRMWARE_H_ */
