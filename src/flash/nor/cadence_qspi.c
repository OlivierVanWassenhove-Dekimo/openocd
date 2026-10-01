// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Driver for SPI NOR flash attached to a Cadence Quad SPI flash controller,
 * as found e.g. in the Intel/Altera SoCFPGA families (Cyclone V, Arria 10,
 * Stratix 10, Agilex 5/7), TI K2G/AM654 and Xilinx Versal.
 *
 * The controller specific parts of this driver are a port of the Linux
 * kernel driver drivers/spi/spi-cadence-quadspi.c:
 *
 *   Copyright Altera Corporation (C) 2012-2014. All rights reserved.
 *   Copyright Intel Corporation (C) 2019-2020. All rights reserved.
 *   Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com
 *
 * Function names and program flow deliberately follow the Linux driver so
 * the two can be compared side by side. Differences with respect to Linux:
 *
 *  - Only the configuration used for "intel,socfpga-qspi" is implemented
 *    (quirks CQSPI_DISABLE_DAC_MODE, CQSPI_NO_SUPPORT_WR_COMPLETION,
 *    CQSPI_SLOW_SRAM and CQSPI_DISABLE_STIG_MODE), so data is always moved
 *    via indirect mode (SRAM FIFO accessed through the AHB trigger address),
 *    and small register reads with an address phase are not done via STIG.
 *  - There is no interrupt available to a debugger. Everything is done by
 *    polling, as the Linux driver does for controllers with the
 *    CQSPI_RD_NO_IRQ quirk: the interrupt mask is always kept at zero.
 *  - DTR and multi-IO (dual/quad/octal) operation are not used; all flash
 *    operations are issued in 1-1-1 mode. This is the one mode every SPI NOR
 *    flash is guaranteed to be in after boot.
 *  - The SPI clock divider and delay registers are only (re)programmed when
 *    the QSPI reference clock is supplied via "-ref-clk"; otherwise the
 *    timing set up by the boot firmware (SDM, U-Boot, ...) is left alone.
 *  - By default, data is programmed with multi-page indirect writes (see
 *    cqspi_write_stream()) instead of one page program per page as the
 *    Linux spi-nor layer does; the latter is available as "page" write
 *    mode. Polling sequences are batched into single DAP transactions where
 *    that yields the same register accesses; for a debugger, round trips
 *    rather than the flash or the controller determine performance.
 *
 * The (small) part of the Linux spi-nor core needed to erase, program and
 * read a flash is implemented on top of cqspi_mem_process(), mirroring the
 * spi-mem interface the Linux controller driver exports. Micron/ST
 * N25Q/MT25Q devices (e.g. MT25QU01GBBB) use the flag status register for
 * completion and error reporting, like Linux does for devices with the
 * USE_FSR flag.
 *
 * Accessing the FIFO requires repeated 32-bit accesses to the same AHB
 * address (ioread32_rep()/iowrite32_rep() in Linux). When the flash bank's
 * target is a "mem_ap" target (e.g. the HPS AXI-AP "hps.sys_bus" on Agilex 5)
 * this is done with non-incrementing MEM-AP bursts, and controller register
 * writes are queued in the DAP instead of being flushed one by one. For any
 * other target type every access is an individual target memory access,
 * which works but is considerably slower.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include "sfdp.h"
#include <helper/align.h>
#include <helper/bits.h>
#include <helper/time_support.h>
#include <target/arm_adi_v5.h>

/* Agilex 5 HPS defaults (hps2sdm_qspi_regs / hps2sdm_qspi_data) */
#define CQSPI_AGILEX5_CTRL_BASE			0x108D2000
#define CQSPI_AGILEX5_AHB_BASE			0x10900000

/* Quirks (subset of Linux, the ones used by "intel,socfpga-qspi") */
#define CQSPI_DISABLE_DAC_MODE			BIT(1)
#define CQSPI_NO_SUPPORT_WR_COMPLETION		BIT(3)
#define CQSPI_SLOW_SRAM				BIT(4)
#define CQSPI_DISABLE_STIG_MODE			BIT(9)

#define CQSPI_SOCFPGA_QUIRKS	(CQSPI_DISABLE_DAC_MODE | \
				 CQSPI_NO_SUPPORT_WR_COMPLETION | \
				 CQSPI_SLOW_SRAM | CQSPI_DISABLE_STIG_MODE)

/* Operation timeout value */
#define CQSPI_TIMEOUT_MS			500

#define CQSPI_DUMMY_CLKS_PER_BYTE		8
#define CQSPI_DUMMY_CLKS_MAX			31

#define CQSPI_STIG_DATA_LEN_MAX			8

/* Register map */
#define CQSPI_REG_CONFIG			0x00
#define CQSPI_REG_CONFIG_ENABLE_MASK		BIT(0)
#define CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL	BIT(7)
#define CQSPI_REG_CONFIG_DECODE_MASK		BIT(9)
#define CQSPI_REG_CONFIG_CHIPSELECT_LSB		10
#define CQSPI_REG_CONFIG_DMA_MASK		BIT(15)
#define CQSPI_REG_CONFIG_BAUD_LSB		19
#define CQSPI_REG_CONFIG_DTR_PROTO		BIT(24)
#define CQSPI_REG_CONFIG_DUAL_OPCODE		BIT(30)
#define CQSPI_REG_CONFIG_IDLE_LSB		31
#define CQSPI_REG_CONFIG_CHIPSELECT_MASK	0xF
#define CQSPI_REG_CONFIG_BAUD_MASK		0xF

#define CQSPI_REG_RD_INSTR			0x04
#define CQSPI_REG_RD_INSTR_OPCODE_LSB		0
#define CQSPI_REG_RD_INSTR_TYPE_INSTR_LSB	8
#define CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB	12
#define CQSPI_REG_RD_INSTR_TYPE_DATA_LSB	16
#define CQSPI_REG_RD_INSTR_MODE_EN_LSB		20
#define CQSPI_REG_RD_INSTR_DUMMY_LSB		24
#define CQSPI_REG_RD_INSTR_DUMMY_MASK		0x1F

#define CQSPI_REG_WR_INSTR			0x08
#define CQSPI_REG_WR_INSTR_OPCODE_LSB		0
#define CQSPI_REG_WR_INSTR_TYPE_ADDR_LSB	12
#define CQSPI_REG_WR_INSTR_TYPE_DATA_LSB	16

#define CQSPI_REG_DELAY				0x0C
#define CQSPI_REG_DELAY_TSLCH_LSB		0
#define CQSPI_REG_DELAY_TCHSH_LSB		8
#define CQSPI_REG_DELAY_TSD2D_LSB		16
#define CQSPI_REG_DELAY_TSHSL_LSB		24
#define CQSPI_REG_DELAY_TSLCH_MASK		0xFF
#define CQSPI_REG_DELAY_TCHSH_MASK		0xFF
#define CQSPI_REG_DELAY_TSD2D_MASK		0xFF
#define CQSPI_REG_DELAY_TSHSL_MASK		0xFF

#define CQSPI_REG_READCAPTURE			0x10
#define CQSPI_REG_READCAPTURE_BYPASS_LSB	0
#define CQSPI_REG_READCAPTURE_DELAY_LSB		1
#define CQSPI_REG_READCAPTURE_DELAY_MASK	0xF

#define CQSPI_REG_SIZE				0x14
#define CQSPI_REG_SIZE_ADDRESS_LSB		0
#define CQSPI_REG_SIZE_ADDRESS_MASK		0xF
#define CQSPI_REG_SIZE_PAGE_LSB			4
#define CQSPI_REG_SIZE_PAGE_MASK		0xFFF

#define CQSPI_REG_SRAMPARTITION			0x18
#define CQSPI_REG_INDIRECTTRIGGER		0x1C

#define CQSPI_REG_REMAP				0x24

#define CQSPI_REG_SDRAMLEVEL			0x2C
#define CQSPI_REG_SDRAMLEVEL_RD_LSB		0
#define CQSPI_REG_SDRAMLEVEL_WR_LSB		16
#define CQSPI_REG_SDRAMLEVEL_RD_MASK		0xFFFF
#define CQSPI_REG_SDRAMLEVEL_WR_MASK		0xFFFF

#define CQSPI_REG_IRQSTATUS			0x40
#define CQSPI_REG_IRQMASK			0x44

#define CQSPI_REG_INDIRECTRD			0x60
#define CQSPI_REG_INDIRECTRD_START_MASK		BIT(0)
#define CQSPI_REG_INDIRECTRD_CANCEL_MASK	BIT(1)
#define CQSPI_REG_INDIRECTRD_DONE_MASK		BIT(5)

#define CQSPI_REG_INDIRECTRDWATERMARK		0x64
#define CQSPI_REG_INDIRECTRDSTARTADDR		0x68
#define CQSPI_REG_INDIRECTRDBYTES		0x6C

#define CQSPI_REG_CMDCTRL			0x90
#define CQSPI_REG_CMDCTRL_EXECUTE_MASK		BIT(0)
#define CQSPI_REG_CMDCTRL_INPROGRESS_MASK	BIT(1)
#define CQSPI_REG_CMDCTRL_DUMMY_LSB		7
#define CQSPI_REG_CMDCTRL_WR_BYTES_LSB		12
#define CQSPI_REG_CMDCTRL_WR_EN_LSB		15
#define CQSPI_REG_CMDCTRL_ADD_BYTES_LSB		16
#define CQSPI_REG_CMDCTRL_ADDR_EN_LSB		19
#define CQSPI_REG_CMDCTRL_RD_BYTES_LSB		20
#define CQSPI_REG_CMDCTRL_RD_EN_LSB		23
#define CQSPI_REG_CMDCTRL_OPCODE_LSB		24
#define CQSPI_REG_CMDCTRL_WR_BYTES_MASK		0x7
#define CQSPI_REG_CMDCTRL_ADD_BYTES_MASK	0x3
#define CQSPI_REG_CMDCTRL_RD_BYTES_MASK		0x7
#define CQSPI_REG_CMDCTRL_DUMMY_MASK		0x1F

#define CQSPI_REG_INDIRECTWR			0x70
#define CQSPI_REG_INDIRECTWR_START_MASK		BIT(0)
#define CQSPI_REG_INDIRECTWR_CANCEL_MASK	BIT(1)
#define CQSPI_REG_INDIRECTWR_DONE_MASK		BIT(5)

#define CQSPI_REG_INDIRECTWRWATERMARK		0x74
#define CQSPI_REG_INDIRECTWRSTARTADDR		0x78
#define CQSPI_REG_INDIRECTWRBYTES		0x7C

#define CQSPI_REG_CMDADDRESS			0x94
#define CQSPI_REG_CMDREADDATALOWER		0xA0
#define CQSPI_REG_CMDREADDATAUPPER		0xA4
#define CQSPI_REG_CMDWRITEDATALOWER		0xA8
#define CQSPI_REG_CMDWRITEDATAUPPER		0xAC

#define CQSPI_IRQ_STATUS_MASK			0x1FFFF

/* SPI NOR opcodes and status bits not provided by spi.h */
#define SPINOR_OP_WRDI				0x04	/* Write disable */
#define SPINOR_OP_READ_4B			0x13	/* Read data bytes (4-byte address) */
#define SPINOR_OP_READ_FAST_4B			0x0C	/* Fast read (4-byte address) */
#define SPINOR_OP_RDFSR				0x70	/* Read flag status register */
#define SPINOR_OP_CLFSR				0x50	/* Clear flag status register */
#define SPINOR_OP_MT_RD_VCR			0x85	/* Micron: read volatile config register */

#define FSR_READY				BIT(7)	/* Device status, 0 = Busy, 1 = Ready */
#define FSR_E_ERR				BIT(5)	/* Erase operation status */
#define FSR_P_ERR				BIT(4)	/* Program operation status */
#define FSR_PT_ERR				BIT(1)	/* Protection error bit */

#define SPI_NOR_MFR_MICRON			0x20
/* Linux: DEFAULT_READY_WAIT_JIFFIES */
#define SPI_NOR_READY_WAIT_MS			40000
/* Default number of dummy clocks of FAST_READ in 1-1-1 mode */
#define SPI_NOR_FAST_READ_DUMMY_CLKS		8

/* Maximum size of one multi-page ("streaming") write, see cqspi_flash_write() */
#define CQSPI_STREAM_CHUNK			0x10000

enum cqspi_write_mode {
	CQSPI_WRITE_STREAM,	/* multi-page indirect writes (default) */
	CQSPI_WRITE_PAGE,	/* one page program per page, like Linux spi-nor */
};

static const char * const cqspi_write_mode_names[] = {
	[CQSPI_WRITE_STREAM] = "stream",
	[CQSPI_WRITE_PAGE] = "page",
};

/* Minimal equivalent of the Linux 'struct spi_mem_op' */
enum cqspi_data_dir {
	CQSPI_DATA_NONE,
	CQSPI_DATA_IN,
	CQSPI_DATA_OUT,
};

struct cqspi_mem_op {
	struct {
		uint8_t nbytes;
		uint8_t buswidth;
		uint16_t opcode;
	} cmd;
	struct {
		uint8_t nbytes;
		uint8_t buswidth;
		uint32_t val;
	} addr;
	struct {
		uint8_t nbytes;
		uint8_t buswidth;
	} dummy;
	struct {
		uint8_t buswidth;
		enum cqspi_data_dir dir;
		uint32_t nbytes;
		uint8_t *in;
		const uint8_t *out;
	} data;
};

/* Equivalent of the Linux 'struct cqspi_flash_pdata' */
struct cqspi_flash_pdata {
	uint32_t clk_rate;
	uint32_t read_delay;
	uint32_t tshsl_ns;
	uint32_t tsd2d_ns;
	uint32_t tchsh_ns;
	uint32_t tslch_ns;
	uint8_t cs;
};

/* Equivalent of the Linux 'struct cqspi_st' plus the spi-nor state */
struct cqspi_flash_bank {
	bool probed;

	/* Controller */
	target_addr_t iobase;
	target_addr_t ahb_base;
	uint32_t trigger_address;
	uint32_t fifo_depth;
	uint32_t fifo_width;
	uint32_t quirks;
	uint32_t master_ref_clk_hz;
	unsigned int sclk;
	int current_cs;
	bool is_decoded_cs;
	bool rclk_en;
	bool use_direct_mode;
	bool disable_stig_mode;
	struct cqspi_flash_pdata f_pdata;

	/* SPI NOR flash */
	struct flash_device dev;
	uint32_t jedec_id;
	uint8_t addr_nbytes;
	uint8_t read_opcode;
	uint8_t read_dummy_nbytes;
	bool use_fsr;
	enum cqspi_write_mode write_mode;

	/* MEM-AP of a "mem_ap" target, held for the duration of one operation */
	struct adiv5_ap *ap;
	/* landing zone for queued reads whose value is ignored */
	uint32_t discard;

	/*
	 * Controller state known for the duration of one operation (reset by
	 * cqspi_op_begin()); nothing but this driver changes these registers
	 * meanwhile, so they need not be read back for every flash command.
	 */
	bool dtr_off_known;	/* CONFIG has DTR and dual opcode disabled */
	bool size_reg_valid;	/* size_reg holds the value of CQSPI_REG_SIZE */
	uint32_t size_reg;
};

/* ------------------------------------------------------------------------ */
/* Low level target access                                                  */
/* ------------------------------------------------------------------------ */

/* Returns the MEM-AP behind a "mem_ap" target, or NULL. Release with dap_put_ap(). */
static struct adiv5_ap *cqspi_get_mem_ap(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (strcmp(target_type_name(target), "mem_ap") != 0 || !target_was_examined(target))
		return NULL;

	struct adiv5_private_config *pc = target->private_config;
	if (!pc || !pc->dap)
		return NULL;

	return dap_get_ap(pc->dap, pc->ap_num);
}

/*
 * Every top level flash operation is bracketed by cqspi_op_begin() and
 * cqspi_op_end(). For a "mem_ap" target, register writes are only queued
 * in the DAP (in order) and get flushed by the next register read, FIFO
 * burst or cqspi_op_end(). This keeps the Linux driver's access sequence
 * while saving a debugger round trip for every register write.
 */
static void cqspi_op_begin(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;

	cqspi->ap = cqspi_get_mem_ap(bank);
	cqspi->dtr_off_known = false;
	cqspi->size_reg_valid = false;
}

static int cqspi_op_end(struct flash_bank *bank, int retval)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;

	if (cqspi->ap) {
		int retval2 = dap_run(cqspi->ap->dap);

		dap_put_ap(cqspi->ap);
		cqspi->ap = NULL;
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

static int cqspi_readl(struct flash_bank *bank, uint32_t reg, uint32_t *val)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	int retval;

	if (cqspi->ap)
		retval = mem_ap_read_atomic_u32(cqspi->ap, cqspi->iobase + reg, val);
	else
		retval = target_read_u32(bank->target, cqspi->iobase + reg, val);

	if (retval != ERROR_OK)
		LOG_ERROR("cqspi: failed to read register 0x%02" PRIx32 " at " TARGET_ADDR_FMT,
			reg, cqspi->iobase + reg);
	return retval;
}

static int cqspi_writel(struct flash_bank *bank, uint32_t val, uint32_t reg)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	int retval;

	if (cqspi->ap)
		retval = mem_ap_write_u32(cqspi->ap, cqspi->iobase + reg, val);
	else
		retval = target_write_u32(bank->target, cqspi->iobase + reg, val);

	if (retval != ERROR_OK)
		LOG_ERROR("cqspi: failed to write register 0x%02" PRIx32 " at " TARGET_ADDR_FMT,
			reg, cqspi->iobase + reg);
	return retval;
}

/*
 * Equivalent of the "Flush posted write" readl() calls in Linux: the read is
 * issued on the bus in order, but for a mem_ap target only queued, since
 * its value is not needed.
 */
static int cqspi_readl_flush(struct flash_bank *bank, uint32_t reg)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;

	if (cqspi->ap)
		return mem_ap_read_u32(cqspi->ap, cqspi->iobase + reg, &cqspi->discard);

	return cqspi_readl(bank, reg, &cqspi->discard);
}

/*
 * Read a list of registers back to back. For a mem_ap target this is done
 * in a single DAP transaction, i.e. a single debugger round trip.
 */
static int cqspi_readl_multi(struct flash_bank *bank, const uint32_t *regs, uint32_t *vals,
		unsigned int count)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	int retval = ERROR_OK;

	if (!cqspi->ap) {
		for (unsigned int i = 0; i < count && retval == ERROR_OK; i++)
			retval = cqspi_readl(bank, regs[i], &vals[i]);
		return retval;
	}

	for (unsigned int i = 0; i < count && retval == ERROR_OK; i++)
		retval = mem_ap_read_u32(cqspi->ap, cqspi->iobase + regs[i], &vals[i]);
	if (retval == ERROR_OK)
		retval = dap_run(cqspi->ap->dap);

	if (retval != ERROR_OK)
		LOG_ERROR("cqspi: failed to read registers at " TARGET_ADDR_FMT, cqspi->iobase);
	return retval;
}

/* Read a register 'count' times back to back (single DAP transaction). */
static int cqspi_readl_rep(struct flash_bank *bank, uint32_t reg, uint32_t *vals,
		unsigned int count)
{
	uint32_t regs[8];

	assert(count <= ARRAY_SIZE(regs));
	for (unsigned int i = 0; i < count; i++)
		regs[i] = reg;
	return cqspi_readl_multi(bank, regs, vals, count);
}

/* Equivalent of ioread32_rep(ahb_base, buf, words) */
static int cqspi_ahb_read_rep(struct flash_bank *bank, uint8_t *buf, uint32_t words)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	int retval = ERROR_OK;

	if (cqspi->ap) {
		retval = mem_ap_read_buf_noincr(cqspi->ap, buf, 4, words, cqspi->ahb_base);
	} else {
		for (uint32_t i = 0; i < words && retval == ERROR_OK; i++)
			retval = target_read_memory(bank->target, cqspi->ahb_base, 4, 1, buf + 4 * i);
	}

	if (retval != ERROR_OK)
		LOG_ERROR("cqspi: failed to read SRAM FIFO at " TARGET_ADDR_FMT, cqspi->ahb_base);
	return retval;
}

/* Equivalent of iowrite32_rep(ahb_base, buf, words) */
static int cqspi_ahb_write_rep(struct flash_bank *bank, const uint8_t *buf, uint32_t words)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	int retval = ERROR_OK;

	if (cqspi->ap) {
		retval = mem_ap_write_buf_noincr(cqspi->ap, buf, 4, words, cqspi->ahb_base);
	} else {
		for (uint32_t i = 0; i < words && retval == ERROR_OK; i++)
			retval = target_write_memory(bank->target, cqspi->ahb_base, 4, 1, buf + 4 * i);
	}

	if (retval != ERROR_OK)
		LOG_ERROR("cqspi: failed to write SRAM FIFO at " TARGET_ADDR_FMT, cqspi->ahb_base);
	return retval;
}

/* ------------------------------------------------------------------------ */
/* Cadence QSPI controller (port of drivers/spi/spi-cadence-quadspi.c)      */
/* ------------------------------------------------------------------------ */

static unsigned int cqspi_ilog2(unsigned int v)
{
	unsigned int r = 0;

	while (v >>= 1)
		r++;
	return r;
}

/* Linux: CQSPI_OP_WIDTH(part) */
static unsigned int cqspi_op_width(uint32_t nbytes, uint8_t buswidth)
{
	return nbytes ? cqspi_ilog2(buswidth) : 0;
}

static int cqspi_wait_for_bit(struct flash_bank *bank, uint32_t reg,
		const uint32_t mask, bool clr)
{
	int64_t timeout = timeval_ms() + CQSPI_TIMEOUT_MS;
	uint32_t val;
	int retval;

	while (1) {
		retval = cqspi_readl(bank, reg, &val);
		if (retval != ERROR_OK)
			return retval;

		if (((clr ? ~val : val) & mask) == mask)
			return ERROR_OK;

		if (timeval_ms() > timeout)
			return ERROR_TIMEOUT_REACHED;

		keep_alive();
	}
}

static int cqspi_get_rd_sram_level(struct flash_bank *bank, uint32_t *level)
{
	uint32_t reg = 0;
	int retval = cqspi_readl(bank, CQSPI_REG_SDRAMLEVEL, &reg);

	reg >>= CQSPI_REG_SDRAMLEVEL_RD_LSB;
	*level = reg & CQSPI_REG_SDRAMLEVEL_RD_MASK;
	return retval;
}

static unsigned int cqspi_calc_rdreg(const struct cqspi_mem_op *op)
{
	uint32_t rdreg = 0;

	rdreg |= cqspi_op_width(op->cmd.nbytes, op->cmd.buswidth) << CQSPI_REG_RD_INSTR_TYPE_INSTR_LSB;
	rdreg |= cqspi_op_width(op->addr.nbytes, op->addr.buswidth) << CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB;
	rdreg |= cqspi_op_width(op->data.nbytes, op->data.buswidth) << CQSPI_REG_RD_INSTR_TYPE_DATA_LSB;

	return rdreg;
}

static unsigned int cqspi_calc_dummy(const struct cqspi_mem_op *op)
{
	if (!op->dummy.nbytes)
		return 0;

	/* DTR is not supported by this driver, so no halving here */
	return op->dummy.nbytes * (8 / op->dummy.buswidth);
}

static int cqspi_wait_idle(struct flash_bank *bank)
{
	const unsigned int poll_idle_retry = 3;
	unsigned int count = 0;
	int64_t timeout = timeval_ms() + CQSPI_TIMEOUT_MS;
	uint32_t cfg[3];
	int retval;

	while (1) {
		/*
		 * Read few times in succession to ensure the controller
		 * is indeed idle, that is, the bit does not transition
		 * low again.
		 */
		retval = cqspi_readl_rep(bank, CQSPI_REG_CONFIG, cfg, ARRAY_SIZE(cfg));
		if (retval != ERROR_OK)
			return retval;

		for (unsigned int i = 0; i < ARRAY_SIZE(cfg); i++) {
			if (cfg[i] & BIT(CQSPI_REG_CONFIG_IDLE_LSB))
				count++;
			else
				count = 0;
		}

		if (count >= poll_idle_retry)
			return ERROR_OK;

		if (timeval_ms() > timeout) {
			/* Timeout, in busy mode. */
			LOG_ERROR("cqspi: QSPI is still busy after %dms timeout.",
				CQSPI_TIMEOUT_MS);
			return ERROR_TIMEOUT_REACHED;
		}

		keep_alive();
	}
}

static bool cqspi_all_idle(const uint32_t *cfg, unsigned int count)
{
	for (unsigned int i = 0; i < count; i++)
		if (!(cfg[i] & BIT(CQSPI_REG_CONFIG_IDLE_LSB)))
			return false;
	return true;
}

/*
 * Linux: cqspi_exec_flash_cmd(). Additionally reads the first 'n_rd'
 * (0..2) STIG read data registers into rd_data once the command completed,
 * as cqspi_command_read() does afterwards in Linux.
 */
static int cqspi_exec_flash_cmd(struct flash_bank *bank, uint32_t reg,
		uint32_t *rd_data, unsigned int n_rd)
{
	static const uint32_t status_regs[] = {
		CQSPI_REG_CMDCTRL,
		CQSPI_REG_CONFIG, CQSPI_REG_CONFIG, CQSPI_REG_CONFIG,
		CQSPI_REG_CMDREADDATALOWER, CQSPI_REG_CMDREADDATAUPPER,
	};
	uint32_t vals[ARRAY_SIZE(status_regs)];
	int retval;

	/* Write the CMDCTRL without start execution. */
	retval = cqspi_writel(bank, reg, CQSPI_REG_CMDCTRL);
	if (retval != ERROR_OK)
		return retval;
	/* Start execute */
	reg |= CQSPI_REG_CMDCTRL_EXECUTE_MASK;
	retval = cqspi_writel(bank, reg, CQSPI_REG_CMDCTRL);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * Fast path: a STIG command takes a few microseconds, while every
	 * debugger round trip takes much longer. So read what Linux reads next
	 * in one batch: the completion status, the idle status three times in
	 * succession (cqspi_wait_idle()) and the read data. If all reads show
	 * completion, this is exactly what the polling below would have read;
	 * otherwise fall back to polling.
	 */
	retval = cqspi_readl_multi(bank, status_regs, vals, 4 + n_rd);
	if (retval != ERROR_OK)
		return retval;
	if (!(vals[0] & CQSPI_REG_CMDCTRL_INPROGRESS_MASK) && cqspi_all_idle(&vals[1], 3)) {
		for (unsigned int i = 0; i < n_rd; i++)
			rd_data[i] = vals[4 + i];
		return ERROR_OK;
	}

	/* Polling for completion. */
	retval = cqspi_wait_for_bit(bank, CQSPI_REG_CMDCTRL,
			CQSPI_REG_CMDCTRL_INPROGRESS_MASK, true);
	if (retval != ERROR_OK) {
		LOG_ERROR("cqspi: Flash command execution timed out.");
		return retval;
	}

	/* Polling QSPI idle status. */
	retval = cqspi_wait_idle(bank);
	if (retval != ERROR_OK || !n_rd)
		return retval;

	return cqspi_readl_multi(bank, &status_regs[4], rd_data, n_rd);
}

/*
 * DTR operations are never issued by this driver, so only the "disable" leg
 * of the Linux cqspi_enable_dtr() is needed. It still matters, since boot
 * firmware may have left the controller in DTR mode. Once DTR is known to
 * be off, it stays off until the end of the current operation.
 */
static int cqspi_enable_dtr(struct flash_bank *bank, const struct cqspi_mem_op *op)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	const uint32_t mask = CQSPI_REG_CONFIG_DTR_PROTO | CQSPI_REG_CONFIG_DUAL_OPCODE;
	uint32_t reg;
	int retval;

	if (cqspi->dtr_off_known)
		return ERROR_OK;

	retval = cqspi_readl(bank, CQSPI_REG_CONFIG, &reg);
	if (retval != ERROR_OK)
		return retval;

	/* Shortcut if DTR is already disabled. */
	if ((reg & mask) == 0) {
		cqspi->dtr_off_known = true;
		return ERROR_OK;
	}
	reg &= ~mask;

	retval = cqspi_writel(bank, reg, CQSPI_REG_CONFIG);
	if (retval != ERROR_OK)
		return retval;

	retval = cqspi_wait_idle(bank);
	if (retval == ERROR_OK)
		cqspi->dtr_off_known = true;
	return retval;
}

/*
 * Address width setup of Linux' cqspi_read_setup() / cqspi_write_setup():
 * read-modify-write of CQSPI_REG_SIZE, followed by a read to flush the
 * posted write. The register value is remembered for the rest of the
 * operation, so it only has to be read from the controller once.
 */
static int cqspi_set_addr_width(struct flash_bank *bank, unsigned int addr_nbytes)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	uint32_t reg;
	int ret;

	if (!cqspi->size_reg_valid) {
		ret = cqspi_readl(bank, CQSPI_REG_SIZE, &cqspi->size_reg);
		if (ret != ERROR_OK)
			return ret;
		cqspi->size_reg_valid = true;
	}

	reg = cqspi->size_reg;
	reg &= ~CQSPI_REG_SIZE_ADDRESS_MASK;
	reg |= (addr_nbytes - 1);
	ret = cqspi_writel(bank, reg, CQSPI_REG_SIZE);
	if (ret != ERROR_OK) {
		cqspi->size_reg_valid = false;
		return ret;
	}
	cqspi->size_reg = reg;
	return cqspi_readl_flush(bank, CQSPI_REG_SIZE); /* Flush posted write. */
}

/*
 * Streaming writes: set the device page size in CQSPI_REG_SIZE, which the
 * controller uses to split indirect writes at page boundaries. Must follow
 * cqspi_set_addr_width() (which fetches the register value).
 */
static int cqspi_set_page_size(struct flash_bank *bank, uint32_t page_size)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	uint32_t reg;
	int ret;

	if (!cqspi->size_reg_valid)
		return ERROR_FAIL;

	reg = cqspi->size_reg;
	reg &= ~(CQSPI_REG_SIZE_PAGE_MASK << CQSPI_REG_SIZE_PAGE_LSB);
	reg |= (page_size & CQSPI_REG_SIZE_PAGE_MASK) << CQSPI_REG_SIZE_PAGE_LSB;
	if (reg == cqspi->size_reg)
		return ERROR_OK;

	ret = cqspi_writel(bank, reg, CQSPI_REG_SIZE);
	if (ret != ERROR_OK) {
		cqspi->size_reg_valid = false;
		return ret;
	}
	cqspi->size_reg = reg;
	return cqspi_readl_flush(bank, CQSPI_REG_SIZE); /* Flush posted write. */
}

static int cqspi_command_read(struct flash_bank *bank, const struct cqspi_mem_op *op)
{
	uint8_t *rxbuf = op->data.in;
	uint8_t opcode;
	size_t n_rx = op->data.nbytes;
	uint32_t rdreg;
	uint32_t reg;
	unsigned int dummy_clk;
	size_t read_len;
	int status;

	status = cqspi_enable_dtr(bank, op);
	if (status != ERROR_OK)
		return status;

	if (!n_rx || n_rx > CQSPI_STIG_DATA_LEN_MAX || !rxbuf) {
		LOG_ERROR("cqspi: Invalid input argument, len %zu rxbuf %p", n_rx, rxbuf);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;

	rdreg = cqspi_calc_rdreg(op);
	status = cqspi_writel(bank, rdreg, CQSPI_REG_RD_INSTR);
	if (status != ERROR_OK)
		return status;

	dummy_clk = cqspi_calc_dummy(op);
	if (dummy_clk > CQSPI_DUMMY_CLKS_MAX)
		return ERROR_NOT_IMPLEMENTED;

	if (dummy_clk)
		reg |= (dummy_clk & CQSPI_REG_CMDCTRL_DUMMY_MASK)
			<< CQSPI_REG_CMDCTRL_DUMMY_LSB;

	reg |= BIT(CQSPI_REG_CMDCTRL_RD_EN_LSB);

	/* 0 means 1 byte. */
	reg |= (((n_rx - 1) & CQSPI_REG_CMDCTRL_RD_BYTES_MASK)
		<< CQSPI_REG_CMDCTRL_RD_BYTES_LSB);

	/* setup ADDR BIT field */
	if (op->addr.nbytes) {
		reg |= BIT(CQSPI_REG_CMDCTRL_ADDR_EN_LSB);
		reg |= ((op->addr.nbytes - 1) &
			CQSPI_REG_CMDCTRL_ADD_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_ADD_BYTES_LSB;

		status = cqspi_writel(bank, op->addr.val, CQSPI_REG_CMDADDRESS);
		if (status != ERROR_OK)
			return status;
	}

	uint32_t rd_data[2];

	status = cqspi_exec_flash_cmd(bank, reg, rd_data, n_rx > 4 ? 2 : 1);
	if (status != ERROR_OK)
		return status;

	/* Put the read value into rx_buf */
	uint8_t tmp[4];
	h_u32_to_le(tmp, rd_data[0]);
	read_len = (n_rx > 4) ? 4 : n_rx;
	memcpy(rxbuf, tmp, read_len);
	rxbuf += read_len;

	if (n_rx > 4) {
		h_u32_to_le(tmp, rd_data[1]);
		read_len = n_rx - read_len;
		memcpy(rxbuf, tmp, read_len);
	}

	/* Reset CMD_CTRL Reg once command read completes */
	return cqspi_writel(bank, 0, CQSPI_REG_CMDCTRL);
}

static int cqspi_command_write(struct flash_bank *bank, const struct cqspi_mem_op *op)
{
	uint8_t opcode;
	const uint8_t *txbuf = op->data.out;
	size_t n_tx = op->data.nbytes;
	uint32_t reg;
	uint8_t data[4];
	size_t write_len;
	int ret;

	ret = cqspi_enable_dtr(bank, op);
	if (ret != ERROR_OK)
		return ret;

	if (n_tx > CQSPI_STIG_DATA_LEN_MAX || (n_tx && !txbuf)) {
		LOG_ERROR("cqspi: Invalid input argument, cmdlen %zu txbuf %p", n_tx, txbuf);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	reg = cqspi_calc_rdreg(op);
	ret = cqspi_writel(bank, reg, CQSPI_REG_RD_INSTR);
	if (ret != ERROR_OK)
		return ret;

	opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;

	if (op->addr.nbytes) {
		reg |= BIT(CQSPI_REG_CMDCTRL_ADDR_EN_LSB);
		reg |= ((op->addr.nbytes - 1) &
			CQSPI_REG_CMDCTRL_ADD_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_ADD_BYTES_LSB;

		ret = cqspi_writel(bank, op->addr.val, CQSPI_REG_CMDADDRESS);
		if (ret != ERROR_OK)
			return ret;
	}

	if (n_tx) {
		reg |= BIT(CQSPI_REG_CMDCTRL_WR_EN_LSB);
		reg |= ((n_tx - 1) & CQSPI_REG_CMDCTRL_WR_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_WR_BYTES_LSB;
		memset(data, 0, sizeof(data));
		write_len = (n_tx > 4) ? 4 : n_tx;
		memcpy(data, txbuf, write_len);
		txbuf += write_len;
		ret = cqspi_writel(bank, le_to_h_u32(data), CQSPI_REG_CMDWRITEDATALOWER);
		if (ret != ERROR_OK)
			return ret;

		if (n_tx > 4) {
			memset(data, 0, sizeof(data));
			write_len = n_tx - 4;
			memcpy(data, txbuf, write_len);
			ret = cqspi_writel(bank, le_to_h_u32(data), CQSPI_REG_CMDWRITEDATAUPPER);
			if (ret != ERROR_OK)
				return ret;
		}
	}

	ret = cqspi_exec_flash_cmd(bank, reg, NULL, 0);

	/* Reset CMD_CTRL Reg once command write completes */
	int ret2 = cqspi_writel(bank, 0, CQSPI_REG_CMDCTRL);

	return ret != ERROR_OK ? ret : ret2;
}

static int cqspi_read_setup(struct flash_bank *bank, const struct cqspi_mem_op *op)
{
	unsigned int dummy_clk = 0;
	uint32_t reg;
	int ret;
	uint8_t opcode;

	ret = cqspi_enable_dtr(bank, op);
	if (ret != ERROR_OK)
		return ret;

	opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_RD_INSTR_OPCODE_LSB;
	reg |= cqspi_calc_rdreg(op);

	/* Setup dummy clock cycles */
	dummy_clk = cqspi_calc_dummy(op);

	if (dummy_clk > CQSPI_DUMMY_CLKS_MAX)
		return ERROR_NOT_IMPLEMENTED;

	if (dummy_clk)
		reg |= (dummy_clk & CQSPI_REG_RD_INSTR_DUMMY_MASK)
			<< CQSPI_REG_RD_INSTR_DUMMY_LSB;

	ret = cqspi_writel(bank, reg, CQSPI_REG_RD_INSTR);
	if (ret != ERROR_OK)
		return ret;

	/* Set address width */
	return cqspi_set_addr_width(bank, op->addr.nbytes);
}

static int cqspi_indirect_read_execute(struct flash_bank *bank, uint8_t *rxbuf,
		uint32_t from_addr, const uint32_t n_rx)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	uint32_t remaining = n_rx;
	uint32_t mod_bytes = n_rx % 4;
	uint32_t bytes_to_read = 0;
	uint8_t *rxbuf_end = rxbuf + n_rx;
	int64_t timeout;
	int ret;

	ret = cqspi_writel(bank, from_addr, CQSPI_REG_INDIRECTRDSTARTADDR);
	if (ret != ERROR_OK)
		return ret;
	ret = cqspi_writel(bank, remaining, CQSPI_REG_INDIRECTRDBYTES);
	if (ret != ERROR_OK)
		return ret;

	/* Clear all interrupts. */
	ret = cqspi_writel(bank, CQSPI_IRQ_STATUS_MASK, CQSPI_REG_IRQSTATUS);
	if (ret != ERROR_OK)
		return ret;

	/*
	 * A debugger has no interrupt line, so this follows the Linux
	 * code path for controllers with the CQSPI_RD_NO_IRQ quirk.
	 */
	ret = cqspi_writel(bank, 0, CQSPI_REG_IRQMASK);
	if (ret != ERROR_OK)
		return ret;

	ret = cqspi_writel(bank, CQSPI_REG_INDIRECTRD_START_MASK, CQSPI_REG_INDIRECTRD);
	if (ret != ERROR_OK)
		return ret;
	ret = cqspi_readl_flush(bank, CQSPI_REG_INDIRECTRD); /* Flush posted write. */
	if (ret != ERROR_OK)
		goto failrd;

	timeout = timeval_ms() + CQSPI_TIMEOUT_MS;
	while (remaining > 0) {
		ret = cqspi_get_rd_sram_level(bank, &bytes_to_read);
		if (ret != ERROR_OK)
			goto failrd;

		/*
		 * Linux relies on the interrupt timeout here; without
		 * interrupts, time out if no data arrives for too long.
		 */
		if (bytes_to_read == 0) {
			if (timeval_ms() > timeout) {
				LOG_ERROR("cqspi: Indirect read timeout, no bytes");
				ret = ERROR_TIMEOUT_REACHED;
				goto failrd;
			}
			keep_alive();
			continue;
		}

		while (bytes_to_read != 0 && remaining > 0) {
			uint32_t word_remain = ALIGN_DOWN(remaining, 4);

			bytes_to_read *= cqspi->fifo_width;
			bytes_to_read = bytes_to_read > remaining ?
					remaining : bytes_to_read;
			bytes_to_read = ALIGN_DOWN(bytes_to_read, 4);
			/* Read 4 byte word chunks then single bytes */
			if (bytes_to_read) {
				ret = cqspi_ahb_read_rep(bank, rxbuf, bytes_to_read / 4);
				if (ret != ERROR_OK)
					goto failrd;
			} else if (!word_remain && mod_bytes) {
				uint8_t temp[4];

				ret = cqspi_ahb_read_rep(bank, temp, 1);
				if (ret != ERROR_OK)
					goto failrd;

				bytes_to_read = mod_bytes;
				memcpy(rxbuf, temp, MIN((uint32_t)(rxbuf_end - rxbuf),
							bytes_to_read));
			}
			rxbuf += bytes_to_read;
			remaining -= bytes_to_read;
			ret = cqspi_get_rd_sram_level(bank, &bytes_to_read);
			if (ret != ERROR_OK)
				goto failrd;
		}

		timeout = timeval_ms() + CQSPI_TIMEOUT_MS;
		keep_alive();
	}

	/* Check indirect done status */
	ret = cqspi_wait_for_bit(bank, CQSPI_REG_INDIRECTRD,
			CQSPI_REG_INDIRECTRD_DONE_MASK, false);
	if (ret != ERROR_OK) {
		LOG_ERROR("cqspi: Indirect read completion error (%i)", ret);
		goto failrd;
	}

	/* Disable interrupt */
	ret = cqspi_writel(bank, 0, CQSPI_REG_IRQMASK);
	if (ret != ERROR_OK)
		return ret;

	/* Clear indirect completion status */
	return cqspi_writel(bank, CQSPI_REG_INDIRECTRD_DONE_MASK, CQSPI_REG_INDIRECTRD);

failrd:
	/* Disable interrupt */
	cqspi_writel(bank, 0, CQSPI_REG_IRQMASK);

	/* Cancel the indirect read */
	cqspi_writel(bank, CQSPI_REG_INDIRECTRD_CANCEL_MASK, CQSPI_REG_INDIRECTRD);
	return ret;
}

static int cqspi_controller_enable(struct flash_bank *bank, bool enable)
{
	uint32_t reg;
	int retval;

	retval = cqspi_readl(bank, CQSPI_REG_CONFIG, &reg);
	if (retval != ERROR_OK)
		return retval;

	if (enable)
		reg |= CQSPI_REG_CONFIG_ENABLE_MASK;
	else
		reg &= ~CQSPI_REG_CONFIG_ENABLE_MASK;

	return cqspi_writel(bank, reg, CQSPI_REG_CONFIG);
}

static int cqspi_write_setup(struct flash_bank *bank, const struct cqspi_mem_op *op)
{
	uint32_t reg;
	int ret;
	uint8_t opcode;

	ret = cqspi_enable_dtr(bank, op);
	if (ret != ERROR_OK)
		return ret;

	opcode = op->cmd.opcode;

	/* Set opcode. */
	reg = opcode << CQSPI_REG_WR_INSTR_OPCODE_LSB;
	reg |= cqspi_op_width(op->data.nbytes, op->data.buswidth) << CQSPI_REG_WR_INSTR_TYPE_DATA_LSB;
	reg |= cqspi_op_width(op->addr.nbytes, op->addr.buswidth) << CQSPI_REG_WR_INSTR_TYPE_ADDR_LSB;
	ret = cqspi_writel(bank, reg, CQSPI_REG_WR_INSTR);
	if (ret != ERROR_OK)
		return ret;
	reg = cqspi_calc_rdreg(op);
	ret = cqspi_writel(bank, reg, CQSPI_REG_RD_INSTR);
	if (ret != ERROR_OK)
		return ret;

	/*
	 * Linux disables the controller's write completion auto polling here
	 * when supported. SoCFPGA has CQSPI_NO_SUPPORT_WR_COMPLETION, so there
	 * is nothing to do: the spi-nor layer polls the flash status itself.
	 */

	return cqspi_set_addr_width(bank, op->addr.nbytes);
}

/*
 * Streaming writes (not in Linux, see cqspi_write_stream()): wait until the
 * SRAM write partition has room, return the free space in bytes.
 */
static int cqspi_wait_wr_sram_space(struct flash_bank *bank, uint32_t *space_bytes)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	const uint32_t partition_words = cqspi->fifo_depth - cqspi->fifo_depth / 2;
	int64_t timeout = timeval_ms() + CQSPI_TIMEOUT_MS;
	uint32_t reg, fill;
	int ret;

	while (1) {
		ret = cqspi_readl(bank, CQSPI_REG_SDRAMLEVEL, &reg);
		if (ret != ERROR_OK)
			return ret;

		fill = (reg >> CQSPI_REG_SDRAMLEVEL_WR_LSB) & CQSPI_REG_SDRAMLEVEL_WR_MASK;
		if (fill < partition_words) {
			*space_bytes = (partition_words - fill) * cqspi->fifo_width;
			return ERROR_OK;
		}

		if (timeval_ms() > timeout) {
			LOG_ERROR("cqspi: Indirect write timeout, SRAM stays full");
			return ERROR_TIMEOUT_REACHED;
		}
		keep_alive();
	}
}

/*
 * Linux: cqspi_indirect_write_execute(). With 'stream' set, n_tx may exceed
 * the SRAM write partition: data is then written whenever there is room
 * (see cqspi_write_stream()); otherwise this is the Linux code path.
 */
static int cqspi_indirect_write_execute(struct flash_bank *bank, uint32_t to_addr,
		const uint8_t *txbuf, const uint32_t n_tx, bool stream)
{
	uint32_t remaining = n_tx;
	uint32_t write_bytes;
	int ret;

	ret = cqspi_writel(bank, to_addr, CQSPI_REG_INDIRECTWRSTARTADDR);
	if (ret != ERROR_OK)
		return ret;
	ret = cqspi_writel(bank, remaining, CQSPI_REG_INDIRECTWRBYTES);
	if (ret != ERROR_OK)
		return ret;

	/* Clear all interrupts. */
	ret = cqspi_writel(bank, CQSPI_IRQ_STATUS_MASK, CQSPI_REG_IRQSTATUS);
	if (ret != ERROR_OK)
		return ret;

	/* No interrupts for a debugger; completion is polled below. */
	ret = cqspi_writel(bank, 0, CQSPI_REG_IRQMASK);
	if (ret != ERROR_OK)
		return ret;

	ret = cqspi_writel(bank, CQSPI_REG_INDIRECTWR_START_MASK, CQSPI_REG_INDIRECTWR);
	if (ret != ERROR_OK)
		return ret;
	ret = cqspi_readl_flush(bank, CQSPI_REG_INDIRECTWR); /* Flush posted write. */
	if (ret != ERROR_OK)
		goto failwr;

	/*
	 * No CQSPI_NEEDS_WR_DELAY and no CQSPI_NEEDS_APB_AHB_HAZARD_WAR on
	 * SoCFPGA; besides, every debugger access takes way longer than the
	 * few QSPI_REF_CLK cycles those are about.
	 */

	while (remaining > 0) {
		uint32_t write_words, mod_bytes;

		write_bytes = remaining;
		if (stream) {
			uint32_t space;

			ret = cqspi_wait_wr_sram_space(bank, &space);
			if (ret != ERROR_OK)
				goto failwr;
			write_bytes = MIN(remaining, space);
		}
		write_words = write_bytes / 4;
		mod_bytes = write_bytes % 4;
		/* Write 4 bytes at a time then single bytes. */
		if (write_words) {
			ret = cqspi_ahb_write_rep(bank, txbuf, write_words);
			if (ret != ERROR_OK)
				goto failwr;
			txbuf += (write_words * 4);
		}
		if (mod_bytes) {
			uint8_t temp[4] = { 0xFF, 0xFF, 0xFF, 0xFF };

			memcpy(temp, txbuf, mod_bytes);
			ret = cqspi_ahb_write_rep(bank, temp, 1);
			if (ret != ERROR_OK)
				goto failwr;
			txbuf += mod_bytes;
		}

		/*
		 * Linux waits for an interrupt here; the indirect done
		 * status is polled below instead.
		 */
		remaining -= write_bytes;
		if (stream)
			keep_alive();
	}

	/*
	 * Fast path: by the time the data has made it through the debugger,
	 * the controller has usually long finished writing it to the flash.
	 * So read the indirect done status and the idle status (three times,
	 * see cqspi_wait_idle()) in one batch. Linux checks idle only after
	 * clearing the done status; register writes don't make the controller
	 * busy, so if all of these show completion, the outcome is the same.
	 */
	static const uint32_t done_regs[] = {
		CQSPI_REG_INDIRECTWR,
		CQSPI_REG_CONFIG, CQSPI_REG_CONFIG, CQSPI_REG_CONFIG,
	};
	uint32_t vals[ARRAY_SIZE(done_regs)];

	ret = cqspi_readl_multi(bank, done_regs, vals, ARRAY_SIZE(done_regs));
	if (ret != ERROR_OK)
		goto failwr;
	bool fast_done = (vals[0] & CQSPI_REG_INDIRECTWR_DONE_MASK) &&
		cqspi_all_idle(&vals[1], 3);

	/* Check indirect done status */
	if (!fast_done) {
		ret = cqspi_wait_for_bit(bank, CQSPI_REG_INDIRECTWR,
				CQSPI_REG_INDIRECTWR_DONE_MASK, false);
		if (ret != ERROR_OK) {
			LOG_ERROR("cqspi: Indirect write completion error (%i)", ret);
			goto failwr;
		}
	}

	/* Disable interrupt. */
	ret = cqspi_writel(bank, 0, CQSPI_REG_IRQMASK);
	if (ret != ERROR_OK)
		return ret;

	/* Clear indirect completion status */
	ret = cqspi_writel(bank, CQSPI_REG_INDIRECTWR_DONE_MASK, CQSPI_REG_INDIRECTWR);
	if (ret != ERROR_OK)
		return ret;

	if (fast_done)
		return ERROR_OK;

	return cqspi_wait_idle(bank);

failwr:
	/* Disable interrupt. */
	cqspi_writel(bank, 0, CQSPI_REG_IRQMASK);

	/* Cancel the indirect write */
	cqspi_writel(bank, CQSPI_REG_INDIRECTWR_CANCEL_MASK, CQSPI_REG_INDIRECTWR);
	return ret;
}

static int cqspi_chipselect(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	unsigned int chip_select = cqspi->f_pdata.cs;
	uint32_t reg;
	int retval;

	retval = cqspi_readl(bank, CQSPI_REG_CONFIG, &reg);
	if (retval != ERROR_OK)
		return retval;

	if (cqspi->is_decoded_cs) {
		reg |= CQSPI_REG_CONFIG_DECODE_MASK;
	} else {
		reg &= ~CQSPI_REG_CONFIG_DECODE_MASK;

		/* Convert CS if without decoder.
		 * CS0 to 4b'1110
		 * CS1 to 4b'1101
		 * CS2 to 4b'1011
		 * CS3 to 4b'0111
		 */
		chip_select = 0xF & ~BIT(chip_select);
	}

	reg &= ~(CQSPI_REG_CONFIG_CHIPSELECT_MASK
		 << CQSPI_REG_CONFIG_CHIPSELECT_LSB);
	reg |= (chip_select & CQSPI_REG_CONFIG_CHIPSELECT_MASK)
		<< CQSPI_REG_CONFIG_CHIPSELECT_LSB;
	return cqspi_writel(bank, reg, CQSPI_REG_CONFIG);
}

static unsigned int calculate_ticks_for_ns(const unsigned int ref_clk_hz,
		const unsigned int ns_val)
{
	unsigned int ticks;

	ticks = ref_clk_hz / 1000;	/* kHz */
	ticks = DIV_ROUND_UP(ticks * ns_val, 1000000);

	return ticks;
}

static int cqspi_delay(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	const struct cqspi_flash_pdata *f_pdata = &cqspi->f_pdata;
	const unsigned int ref_clk_hz = cqspi->master_ref_clk_hz;
	unsigned int tshsl, tchsh, tslch, tsd2d;
	uint32_t reg;
	unsigned int tsclk;

	/* calculate the number of ref ticks for one sclk tick */
	tsclk = DIV_ROUND_UP(ref_clk_hz, cqspi->sclk);

	tshsl = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tshsl_ns);
	/* this particular value must be at least one sclk */
	if (tshsl < tsclk)
		tshsl = tsclk;

	tchsh = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tchsh_ns);
	tslch = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tslch_ns);
	tsd2d = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tsd2d_ns);

	reg = (tshsl & CQSPI_REG_DELAY_TSHSL_MASK)
		<< CQSPI_REG_DELAY_TSHSL_LSB;
	reg |= (tchsh & CQSPI_REG_DELAY_TCHSH_MASK)
		<< CQSPI_REG_DELAY_TCHSH_LSB;
	reg |= (tslch & CQSPI_REG_DELAY_TSLCH_MASK)
		<< CQSPI_REG_DELAY_TSLCH_LSB;
	reg |= (tsd2d & CQSPI_REG_DELAY_TSD2D_MASK)
		<< CQSPI_REG_DELAY_TSD2D_LSB;
	return cqspi_writel(bank, reg, CQSPI_REG_DELAY);
}

static int cqspi_config_baudrate_div(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	const unsigned int ref_clk_hz = cqspi->master_ref_clk_hz;
	uint32_t reg, div;
	int retval;

	/* Recalculate the baudrate divisor based on QSPI specification. */
	div = DIV_ROUND_UP(ref_clk_hz, 2 * cqspi->sclk) - 1;

	/* Maximum baud divisor */
	if (div > CQSPI_REG_CONFIG_BAUD_MASK) {
		div = CQSPI_REG_CONFIG_BAUD_MASK;
		LOG_WARNING("cqspi: Unable to adjust clock <= %u hz. Reduced to %u hz",
			cqspi->sclk, ref_clk_hz / ((div + 1) * 2));
	}

	retval = cqspi_readl(bank, CQSPI_REG_CONFIG, &reg);
	if (retval != ERROR_OK)
		return retval;
	reg &= ~(CQSPI_REG_CONFIG_BAUD_MASK << CQSPI_REG_CONFIG_BAUD_LSB);
	reg |= div << CQSPI_REG_CONFIG_BAUD_LSB;
	return cqspi_writel(bank, reg, CQSPI_REG_CONFIG);
}

static int cqspi_readdata_capture(struct flash_bank *bank, const bool bypass,
		const unsigned int delay)
{
	uint32_t reg;
	int retval;

	retval = cqspi_readl(bank, CQSPI_REG_READCAPTURE, &reg);
	if (retval != ERROR_OK)
		return retval;

	if (bypass)
		reg |= BIT(CQSPI_REG_READCAPTURE_BYPASS_LSB);
	else
		reg &= ~BIT(CQSPI_REG_READCAPTURE_BYPASS_LSB);

	reg &= ~(CQSPI_REG_READCAPTURE_DELAY_MASK
		 << CQSPI_REG_READCAPTURE_DELAY_LSB);

	reg |= (delay & CQSPI_REG_READCAPTURE_DELAY_MASK)
		<< CQSPI_REG_READCAPTURE_DELAY_LSB;

	return cqspi_writel(bank, reg, CQSPI_REG_READCAPTURE);
}

static int cqspi_configure(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	const struct cqspi_flash_pdata *f_pdata = &cqspi->f_pdata;
	unsigned int sclk = f_pdata->clk_rate;
	bool switch_cs = (cqspi->current_cs != f_pdata->cs);
	/*
	 * Only touch the baud rate / delay configuration when the reference
	 * clock is known; otherwise keep what the boot firmware set up.
	 */
	bool switch_ck = cqspi->master_ref_clk_hz && sclk && (cqspi->sclk != sclk);
	int retval;

	if (switch_cs || switch_ck) {
		retval = cqspi_controller_enable(bank, false);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Switch chip select. */
	if (switch_cs) {
		cqspi->current_cs = f_pdata->cs;
		retval = cqspi_chipselect(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Setup baudrate divisor and delays */
	if (switch_ck) {
		cqspi->sclk = sclk;
		retval = cqspi_config_baudrate_div(bank);
		if (retval == ERROR_OK)
			retval = cqspi_delay(bank);
		if (retval == ERROR_OK)
			retval = cqspi_readdata_capture(bank, !cqspi->rclk_en,
					f_pdata->read_delay);
		if (retval != ERROR_OK)
			return retval;
	}

	if (switch_cs || switch_ck)
		return cqspi_controller_enable(bank, true);

	return ERROR_OK;
}

static int cqspi_write(struct flash_bank *bank, const struct cqspi_mem_op *op)
{
	int ret;

	ret = cqspi_write_setup(bank, op);
	if (ret != ERROR_OK)
		return ret;

	/* Direct (DAC) mode is disabled on SoCFPGA (CQSPI_DISABLE_DAC_MODE) */
	return cqspi_indirect_write_execute(bank, op->addr.val, op->data.out, op->data.nbytes,
			false);
}

/*
 * Multi-page ("streaming") write; not in Linux, where spi-nor issues one
 * page program per page. The controller can program any number of pages
 * with a single indirect write: it splits the data at page boundaries
 * (using the page size in CQSPI_REG_SIZE), sends WRITE ENABLE before each
 * page program (unless disabled in CQSPI_REG_WR_INSTR) and polls the flash
 * status after each one (write completion polling, which cannot even be
 * turned off on SoCFPGA, see CQSPI_NO_SUPPORT_WR_COMPLETION). For a
 * debugger, where every round trip is expensive, this avoids the per-page
 * WRITE ENABLE, completion and status polling round trips: the data is
 * simply streamed into the SRAM FIFO whenever it has room.
 *
 * The caller checks the flash status afterwards; the flag status register
 * error bits are sticky, so a failure of any page in the transfer is seen.
 */
static int cqspi_write_stream(struct flash_bank *bank, const struct cqspi_mem_op *op,
		uint32_t page_size)
{
	int ret;

	ret = cqspi_configure(bank);
	if (ret != ERROR_OK)
		return ret;

	ret = cqspi_write_setup(bank, op);
	if (ret != ERROR_OK)
		return ret;

	ret = cqspi_set_page_size(bank, page_size);
	if (ret != ERROR_OK)
		return ret;

	return cqspi_indirect_write_execute(bank, op->addr.val, op->data.out, op->data.nbytes,
			true);
}

static int cqspi_read(struct flash_bank *bank, const struct cqspi_mem_op *op)
{
	int ret;

	ret = cqspi_read_setup(bank, op);
	if (ret != ERROR_OK)
		return ret;

	/* Direct (DAC) mode is disabled on SoCFPGA and there is no DMA */
	return cqspi_indirect_read_execute(bank, op->data.in, op->addr.val, op->data.nbytes);
}

static int cqspi_mem_process(struct flash_bank *bank, const struct cqspi_mem_op *op)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	int retval;

	retval = cqspi_configure(bank);
	if (retval != ERROR_OK)
		return retval;

	if (op->data.dir == CQSPI_DATA_IN && op->data.in) {
		/*
		 * Performing reads in DAC mode forces to read minimum 4 bytes
		 * which is unsupported on some flash devices during register
		 * reads, prefer STIG mode for such small reads.
		 */
		if (!op->addr.nbytes ||
				(op->data.nbytes <= CQSPI_STIG_DATA_LEN_MAX &&
				!cqspi->disable_stig_mode))
			return cqspi_command_read(bank, op);

		return cqspi_read(bank, op);
	}

	if (!op->addr.nbytes || !op->data.out)
		return cqspi_command_write(bank, op);

	return cqspi_write(bank, op);
}

static int cqspi_controller_init(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	uint32_t reg;
	int retval;

	/* Configure the remap address register, no remap */
	retval = cqspi_writel(bank, 0, CQSPI_REG_REMAP);
	if (retval != ERROR_OK)
		return retval;

	/* Disable all interrupts. */
	retval = cqspi_writel(bank, 0, CQSPI_REG_IRQMASK);
	if (retval != ERROR_OK)
		return retval;

	/* Configure the SRAM split to 1:1 . */
	retval = cqspi_writel(bank, cqspi->fifo_depth / 2, CQSPI_REG_SRAMPARTITION);
	if (retval != ERROR_OK)
		return retval;
	/* Load indirect trigger address. */
	retval = cqspi_writel(bank, cqspi->trigger_address, CQSPI_REG_INDIRECTTRIGGER);
	if (retval != ERROR_OK)
		return retval;

	/* Program read watermark -- 1/2 of the FIFO. */
	retval = cqspi_writel(bank, cqspi->fifo_depth * cqspi->fifo_width / 2,
			CQSPI_REG_INDIRECTRDWATERMARK);
	if (retval != ERROR_OK)
		return retval;
	/* Program write watermark -- 1/8 of the FIFO. */
	retval = cqspi_writel(bank, cqspi->fifo_depth * cqspi->fifo_width / 8,
			CQSPI_REG_INDIRECTWRWATERMARK);
	if (retval != ERROR_OK)
		return retval;

	/* Disable direct access controller */
	if (!cqspi->use_direct_mode) {
		retval = cqspi_readl(bank, CQSPI_REG_CONFIG, &reg);
		if (retval != ERROR_OK)
			return retval;
		reg &= ~CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL;
		retval = cqspi_writel(bank, reg, CQSPI_REG_CONFIG);
		if (retval != ERROR_OK)
			return retval;
	}

	/* No DMA read support, but make sure the DMA interface is off */
	retval = cqspi_readl(bank, CQSPI_REG_CONFIG, &reg);
	if (retval != ERROR_OK)
		return retval;
	if (reg & CQSPI_REG_CONFIG_DMA_MASK) {
		reg &= ~CQSPI_REG_CONFIG_DMA_MASK;
		retval = cqspi_writel(bank, reg, CQSPI_REG_CONFIG);
	}
	return retval;
}

static int cqspi_controller_detect_fifo_depth(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	uint32_t reg, fifo_depth;
	int retval;

	/*
	 * Bits N-1:0 are writable while bits 31:N are read as zero, with 2^N
	 * the FIFO depth.
	 */
	retval = cqspi_writel(bank, UINT32_MAX, CQSPI_REG_SRAMPARTITION);
	if (retval != ERROR_OK)
		return retval;
	retval = cqspi_readl(bank, CQSPI_REG_SRAMPARTITION, &reg);
	if (retval != ERROR_OK)
		return retval;
	fifo_depth = reg + 1;

	/* FIFO depth of zero means no value was provided by the user. */
	if (cqspi->fifo_depth == 0) {
		if (fifo_depth < 2 || fifo_depth > 0x10000 || (fifo_depth & (fifo_depth - 1))) {
			LOG_ERROR("cqspi: implausible FIFO depth detected (SRAM partition "
				"readback 0x%08" PRIx32 "), please use -fifo-depth", reg);
			return ERROR_FAIL;
		}
		cqspi->fifo_depth = fifo_depth;
		LOG_DEBUG("cqspi: using FIFO depth of %" PRIu32, fifo_depth);
	} else if (fifo_depth != cqspi->fifo_depth) {
		LOG_WARNING("cqspi: detected FIFO depth (%" PRIu32 ") different from config (%" PRIu32 ")",
			fifo_depth, cqspi->fifo_depth);
	}

	return ERROR_OK;
}

/* Equivalent of the controller setup part of cqspi_probe() */
static int cqspi_controller_probe(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	uint32_t id;
	int retval;

	retval = cqspi_readl(bank, 0xFC, &id);	/* module ID */
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("cqspi: controller at " TARGET_ADDR_FMT ", module ID 0x%08" PRIx32,
		cqspi->iobase, id);

	retval = cqspi_wait_idle(bank);
	if (retval != ERROR_OK)
		return retval;
	retval = cqspi_controller_enable(bank, false);
	if (retval != ERROR_OK)
		return retval;
	retval = cqspi_controller_detect_fifo_depth(bank);
	if (retval == ERROR_OK)
		retval = cqspi_controller_init(bank);
	int retval2 = cqspi_controller_enable(bank, true);
	if (retval != ERROR_OK)
		return retval;
	if (retval2 != ERROR_OK)
		return retval2;

	cqspi->current_cs = -1;
	cqspi->sclk = 0;

	return ERROR_OK;
}

/* ------------------------------------------------------------------------ */
/* SPI NOR layer (subset of Linux drivers/mtd/spi-nor)                      */
/* ------------------------------------------------------------------------ */

static void spi_nor_op_init(struct cqspi_mem_op *op, uint8_t opcode)
{
	memset(op, 0, sizeof(*op));
	op->cmd.nbytes = 1;
	op->cmd.buswidth = 1;
	op->cmd.opcode = opcode;
}

static void spi_nor_op_addr(struct cqspi_mem_op *op, uint8_t nbytes, uint32_t val)
{
	op->addr.nbytes = nbytes;
	op->addr.buswidth = 1;
	op->addr.val = val;
}

static void spi_nor_op_dummy(struct cqspi_mem_op *op, uint8_t nbytes)
{
	op->dummy.nbytes = nbytes;
	op->dummy.buswidth = 1;
}

static void spi_nor_op_data_in(struct cqspi_mem_op *op, uint8_t *buf, uint32_t nbytes)
{
	op->data.buswidth = 1;
	op->data.dir = CQSPI_DATA_IN;
	op->data.nbytes = nbytes;
	op->data.in = buf;
}

static void spi_nor_op_data_out(struct cqspi_mem_op *op, const uint8_t *buf, uint32_t nbytes)
{
	op->data.buswidth = 1;
	op->data.dir = CQSPI_DATA_OUT;
	op->data.nbytes = nbytes;
	op->data.out = buf;
}

/* Simple register read command without address, e.g. RDSR, RDFSR, RDID */
static int spi_nor_read_reg(struct flash_bank *bank, uint8_t opcode, uint8_t *buf, uint32_t len)
{
	struct cqspi_mem_op op;

	spi_nor_op_init(&op, opcode);
	spi_nor_op_data_in(&op, buf, len);
	return cqspi_mem_process(bank, &op);
}

/* Simple command without address or data, e.g. WREN, WRDI, CLFSR */
static int spi_nor_write_cmd(struct flash_bank *bank, uint8_t opcode)
{
	struct cqspi_mem_op op;

	spi_nor_op_init(&op, opcode);
	return cqspi_mem_process(bank, &op);
}

static int spi_nor_read_id(struct flash_bank *bank, uint32_t *id)
{
	uint8_t buf[3];
	int retval = spi_nor_read_reg(bank, SPIFLASH_READ_ID, buf, sizeof(buf));

	/* same byte order as flash_devices[].device_id */
	*id = buf[0] | (buf[1] << 8) | (buf[2] << 16);
	return retval;
}

/* Equivalent of micron_st_nor_ready() / spi_nor_sr_ready() */
static int spi_nor_ready(struct flash_bank *bank, bool *ready)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	uint8_t sr, fsr;
	int retval;

	retval = spi_nor_read_reg(bank, SPIFLASH_READ_STATUS, &sr, 1);
	if (retval != ERROR_OK)
		return retval;

	*ready = !(sr & SPIFLASH_BSY_BIT);

	if (!cqspi->use_fsr)
		return ERROR_OK;

	retval = spi_nor_read_reg(bank, SPINOR_OP_RDFSR, &fsr, 1);
	if (retval != ERROR_OK)
		return retval;

	if (fsr & (FSR_E_ERR | FSR_P_ERR)) {
		if (fsr & FSR_E_ERR)
			LOG_ERROR("cqspi: Erase operation failed.");
		else
			LOG_ERROR("cqspi: Program operation failed.");

		if (fsr & FSR_PT_ERR)
			LOG_ERROR("cqspi: Attempted to modify a protected sector.");

		spi_nor_write_cmd(bank, SPINOR_OP_CLFSR);

		/*
		 * WEL bit remains set to one when an erase or page program
		 * error occurs. Issue a Write Disable command to protect
		 * against inadvertent writes that can possibly corrupt the
		 * contents of the memory.
		 */
		spi_nor_write_cmd(bank, SPINOR_OP_WRDI);

		return ERROR_FLASH_OPERATION_FAILED;
	}

	*ready = *ready && (fsr & FSR_READY);
	return ERROR_OK;
}

static int spi_nor_wait_till_ready(struct flash_bank *bank, unsigned int timeout_ms)
{
	int64_t timeout = timeval_ms() + timeout_ms;
	bool ready;
	int retval;

	while (1) {
		retval = spi_nor_ready(bank, &ready);
		if (retval != ERROR_OK)
			return retval;
		if (ready)
			return ERROR_OK;
		if (timeval_ms() > timeout) {
			LOG_ERROR("cqspi: flash operation timed out");
			return ERROR_TIMEOUT_REACHED;
		}
		keep_alive();
	}
}

static int spi_nor_erase_sector(struct flash_bank *bank, uint32_t addr)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	struct cqspi_mem_op op;
	int retval;

	retval = spi_nor_write_cmd(bank, SPIFLASH_WRITE_ENABLE);
	if (retval != ERROR_OK)
		return retval;

	spi_nor_op_init(&op, cqspi->dev.erase_cmd);
	spi_nor_op_addr(&op, cqspi->addr_nbytes, addr);
	retval = cqspi_mem_process(bank, &op);
	if (retval != ERROR_OK)
		return retval;

	return spi_nor_wait_till_ready(bank, SPI_NOR_READY_WAIT_MS);
}

static int spi_nor_page_program(struct flash_bank *bank, uint32_t addr,
		const uint8_t *buf, uint32_t len)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	struct cqspi_mem_op op;
	int retval;

	retval = spi_nor_write_cmd(bank, SPIFLASH_WRITE_ENABLE);
	if (retval != ERROR_OK)
		return retval;

	spi_nor_op_init(&op, cqspi->dev.pprog_cmd);
	spi_nor_op_addr(&op, cqspi->addr_nbytes, addr);
	spi_nor_op_data_out(&op, buf, len);
	retval = cqspi_mem_process(bank, &op);
	if (retval != ERROR_OK)
		return retval;

	return spi_nor_wait_till_ready(bank, SPI_NOR_READY_WAIT_MS);
}

/*
 * Program 'len' bytes, possibly spanning many pages, with a single
 * multi-page indirect write (see cqspi_write_stream()); the controller
 * issues WRITE ENABLE and waits for completion for every page. Then check
 * the flash status: this also reports errors of any page of the transfer,
 * since the flag status register error bits are sticky.
 */
static int spi_nor_write_stream(struct flash_bank *bank, uint32_t addr,
		const uint8_t *buf, uint32_t len)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	struct cqspi_mem_op op;
	int retval;

	spi_nor_op_init(&op, cqspi->dev.pprog_cmd);
	spi_nor_op_addr(&op, cqspi->addr_nbytes, addr);
	spi_nor_op_data_out(&op, buf, len);
	retval = cqspi_write_stream(bank, &op, cqspi->dev.pagesize);
	if (retval != ERROR_OK)
		return retval;

	return spi_nor_wait_till_ready(bank, SPI_NOR_READY_WAIT_MS);
}

static int spi_nor_read_data(struct flash_bank *bank, uint32_t addr, uint8_t *buf, uint32_t len)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	struct cqspi_mem_op op;

	spi_nor_op_init(&op, cqspi->read_opcode);
	spi_nor_op_addr(&op, cqspi->addr_nbytes, addr);
	if (cqspi->read_dummy_nbytes)
		spi_nor_op_dummy(&op, cqspi->read_dummy_nbytes);
	spi_nor_op_data_in(&op, buf, len);
	return cqspi_mem_process(bank, &op);
}

static int cqspi_read_sfdp_block(struct flash_bank *bank, uint32_t addr,
		unsigned int words, uint32_t *buffer)
{
	struct cqspi_mem_op op;
	uint8_t *buf = malloc(4 * words);
	int retval;

	if (!buf)
		return ERROR_FAIL;

	/* JESD216: 3 address bytes and 8 dummy clocks */
	spi_nor_op_init(&op, SPIFLASH_READ_SFDP);
	spi_nor_op_addr(&op, 3, addr);
	spi_nor_op_dummy(&op, 1);
	spi_nor_op_data_in(&op, buf, 4 * words);
	retval = cqspi_mem_process(bank, &op);

	for (unsigned int i = 0; i < words; i++)
		buffer[i] = le_to_h_u32(&buf[4 * i]);

	free(buf);
	return retval;
}

/*
 * Pick the read command: FAST_READ (1-1-1, 8 dummy clocks) like Linux does
 * by default, unless the Micron volatile configuration register tells us the
 * dummy cycle count has been changed (e.g. by boot firmware), in which case
 * fall back to the plain READ command which has no dummy cycles at all.
 */
static int spi_nor_setup_read(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	uint8_t fast_opcode;

	if (cqspi->dev.read_cmd == SPINOR_OP_READ_4B)
		fast_opcode = SPINOR_OP_READ_FAST_4B;
	else if (cqspi->dev.read_cmd == SPIFLASH_READ)
		fast_opcode = SPIFLASH_FAST_READ;
	else
		fast_opcode = 0;

	cqspi->read_opcode = cqspi->dev.read_cmd;
	cqspi->read_dummy_nbytes = 0;

	if (!fast_opcode)
		return ERROR_OK;

	if (cqspi->use_fsr) {
		uint8_t vcr;
		int retval = spi_nor_read_reg(bank, SPINOR_OP_MT_RD_VCR, &vcr, 1);
		if (retval != ERROR_OK)
			return retval;

		unsigned int dummy = vcr >> 4;
		LOG_DEBUG("cqspi: Micron VCR 0x%02" PRIx8, vcr);
		if (dummy != 0 && dummy != 0xF && dummy != SPI_NOR_FAST_READ_DUMMY_CLKS) {
			LOG_INFO("cqspi: flash configured for %u dummy cycles, "
				"using READ instead of FAST_READ", dummy);
			return ERROR_OK;
		}
	}

	cqspi->read_opcode = fast_opcode;
	cqspi->read_dummy_nbytes = SPI_NOR_FAST_READ_DUMMY_CLKS / CQSPI_DUMMY_CLKS_PER_BYTE;
	return ERROR_OK;
}

/* ------------------------------------------------------------------------ */
/* OpenOCD flash driver interface                                           */
/* ------------------------------------------------------------------------ */

/*
 * flash bank <name> cadence_qspi <base> <size> <chip_width> <bus_width> <target>
 *	[-ctrl-base <addr>] [-ahb-base <addr>] [-trigger-address <addr>]
 *	[-fifo-depth <words>] [-fifo-width <bytes>] [-cs <n>] [-decoded-cs]
 *	[-ref-clk <hz>] [-sclk <hz>] [-read-delay <n>] [-rclk-en]
 *	[-tshsl-ns <ns>] [-tsd2d-ns <ns>] [-tchsh-ns <ns>] [-tslch-ns <ns>]
 */
FLASH_BANK_COMMAND_HANDLER(cqspi_flash_bank_command)
{
	struct cqspi_flash_bank *cqspi;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	cqspi = calloc(1, sizeof(*cqspi));
	if (!cqspi) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	/* Agilex 5 HPS defaults, see socfpga_agilex5.dtsi / socfpga_agilex5_socdk.dts */
	cqspi->iobase = CQSPI_AGILEX5_CTRL_BASE;
	cqspi->ahb_base = CQSPI_AGILEX5_AHB_BASE;
	cqspi->trigger_address = 0;
	cqspi->fifo_depth = 0;		/* auto detect */
	cqspi->fifo_width = 4;
	cqspi->quirks = CQSPI_SOCFPGA_QUIRKS;
	cqspi->f_pdata.cs = 0;
	cqspi->f_pdata.clk_rate = 100000000;
	cqspi->f_pdata.read_delay = 2;
	cqspi->f_pdata.tshsl_ns = 50;
	cqspi->f_pdata.tsd2d_ns = 50;
	cqspi->f_pdata.tchsh_ns = 4;
	cqspi->f_pdata.tslch_ns = 4;

	for (unsigned int i = 6; i < CMD_ARGC; i++) {
		const char *opt = CMD_ARGV[i];
		bool has_value = i + 1 < CMD_ARGC;
		int retval = ERROR_OK;

		if (strcmp(opt, "-decoded-cs") == 0) {
			cqspi->is_decoded_cs = true;
			continue;
		} else if (strcmp(opt, "-rclk-en") == 0) {
			cqspi->rclk_en = true;
			continue;
		}

		if (!has_value) {
			LOG_ERROR("cadence_qspi: missing or unknown argument '%s'", opt);
			retval = ERROR_COMMAND_SYNTAX_ERROR;
		} else if (strcmp(opt, "-ctrl-base") == 0) {
			COMMAND_PARSE_ADDRESS(CMD_ARGV[++i], cqspi->iobase);
		} else if (strcmp(opt, "-ahb-base") == 0) {
			COMMAND_PARSE_ADDRESS(CMD_ARGV[++i], cqspi->ahb_base);
		} else if (strcmp(opt, "-trigger-address") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[++i], cqspi->trigger_address);
		} else if (strcmp(opt, "-fifo-depth") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[++i], cqspi->fifo_depth);
		} else if (strcmp(opt, "-fifo-width") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[++i], cqspi->fifo_width);
		} else if (strcmp(opt, "-cs") == 0) {
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[++i], cqspi->f_pdata.cs);
		} else if (strcmp(opt, "-ref-clk") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[++i], cqspi->master_ref_clk_hz);
		} else if (strcmp(opt, "-sclk") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[++i], cqspi->f_pdata.clk_rate);
		} else if (strcmp(opt, "-read-delay") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[++i], cqspi->f_pdata.read_delay);
		} else if (strcmp(opt, "-tshsl-ns") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[++i], cqspi->f_pdata.tshsl_ns);
		} else if (strcmp(opt, "-tsd2d-ns") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[++i], cqspi->f_pdata.tsd2d_ns);
		} else if (strcmp(opt, "-tchsh-ns") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[++i], cqspi->f_pdata.tchsh_ns);
		} else if (strcmp(opt, "-tslch-ns") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[++i], cqspi->f_pdata.tslch_ns);
		} else if (strcmp(opt, "-write-mode") == 0) {
			const char *mode = CMD_ARGV[++i];

			if (strcmp(mode, "stream") == 0) {
				cqspi->write_mode = CQSPI_WRITE_STREAM;
			} else if (strcmp(mode, "page") == 0) {
				cqspi->write_mode = CQSPI_WRITE_PAGE;
			} else {
				LOG_ERROR("cadence_qspi: unknown write mode '%s'", mode);
				retval = ERROR_COMMAND_SYNTAX_ERROR;
			}
		} else {
			LOG_ERROR("cadence_qspi: unknown argument '%s'", opt);
			retval = ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (retval != ERROR_OK) {
			free(cqspi);
			return retval;
		}
	}

	if (cqspi->f_pdata.cs > 3) {
		LOG_ERROR("cadence_qspi: chip select %u out of range", cqspi->f_pdata.cs);
		free(cqspi);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	if (cqspi->fifo_width != 4) {
		LOG_ERROR("cadence_qspi: only a FIFO width of 4 bytes is supported");
		free(cqspi);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	if (cqspi->master_ref_clk_hz && !cqspi->f_pdata.clk_rate) {
		LOG_ERROR("cadence_qspi: -sclk must not be 0");
		free(cqspi);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	cqspi->use_direct_mode = !(cqspi->quirks & CQSPI_DISABLE_DAC_MODE);
	cqspi->disable_stig_mode = cqspi->quirks & CQSPI_DISABLE_STIG_MODE;
	cqspi->current_cs = -1;

	bank->driver_priv = cqspi;
	return ERROR_OK;
}

static int cqspi_do_probe(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	uint32_t id;
	int retval;

	retval = cqspi_controller_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = spi_nor_read_id(bank, &id);
	if (retval != ERROR_OK)
		return retval;

	if (id == 0 || id == 0xFFFFFF) {
		LOG_ERROR("cqspi: no flash detected (ID 0x%06" PRIx32 ")", id);
		return ERROR_FAIL;
	}
	cqspi->jedec_id = id;

	const struct flash_device *p;
	for (p = flash_devices; p->name; p++)
		if (p->device_id == id)
			break;

	if (p->name) {
		cqspi->dev = *p;
	} else {
		LOG_INFO("cqspi: unknown flash ID 0x%06" PRIx32 ", trying SFDP", id);
		retval = spi_sfdp(bank, &cqspi->dev, &cqspi_read_sfdp_block);
		if (retval != ERROR_OK) {
			LOG_ERROR("cqspi: unknown flash device (ID 0x%06" PRIx32 ")", id);
			return retval;
		}
		cqspi->dev.device_id = id;
	}

	if (!cqspi->dev.size_in_bytes || !cqspi->dev.pagesize || !cqspi->dev.sectorsize ||
			!cqspi->dev.erase_cmd) {
		LOG_ERROR("cqspi: device '%s' is not a supported SPI NOR flash", cqspi->dev.name);
		return ERROR_FAIL;
	}

	/* N25Q / MT25Q (Micron & ST): flag status register (Linux USE_FSR) */
	uint8_t mfr = id & 0xFF;
	uint8_t type = (id >> 8) & 0xFF;
	cqspi->use_fsr = mfr == SPI_NOR_MFR_MICRON && (type == 0xBA || type == 0xBB);

	cqspi->addr_nbytes = cqspi->dev.size_in_bytes > (1UL << 24) ? 4 : 3;
	if (cqspi->addr_nbytes == 4 && cqspi->dev.read_cmd == SPIFLASH_READ)
		LOG_WARNING("cqspi: flash > 16 MiB without 4-byte address opcodes");

	retval = spi_nor_setup_read(bank);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("cqspi: found flash device '%s' (ID 0x%06" PRIx32 "), %" PRIu32 " KiB",
		cqspi->dev.name, id, cqspi->dev.size_in_bytes / 1024);

	if (bank->size && bank->size != cqspi->dev.size_in_bytes)
		LOG_WARNING("cqspi: bank size 0x%" PRIx32 " does not match detected flash size "
			"0x%" PRIx32 ", using the latter", bank->size, cqspi->dev.size_in_bytes);
	bank->size = cqspi->dev.size_in_bytes;

	bank->num_sectors = cqspi->dev.size_in_bytes / cqspi->dev.sectorsize;
	bank->sectors = alloc_block_array(0, cqspi->dev.sectorsize, bank->num_sectors);
	if (!bank->sectors) {
		bank->num_sectors = 0;
		return ERROR_FAIL;
	}

	/*
	 * alloc_block_array() marks the protection state as unknown (-1).
	 * Protection is only handled in software by this driver (see
	 * cqspi_protect()), so all sectors start out unprotected.
	 */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++)
		bank->sectors[sector].is_protected = 0;

	cqspi->probed = true;
	return ERROR_OK;
}

static int cqspi_probe(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;

	cqspi->probed = false;
	free(bank->sectors);
	bank->sectors = NULL;
	bank->num_sectors = 0;

	cqspi_op_begin(bank);
	if (!cqspi->ap)
		LOG_WARNING("cqspi: target '%s' is not a mem_ap target, falling back to "
			"(slow) single word accesses", target_name(bank->target));

	return cqspi_op_end(bank, cqspi_do_probe(bank));
}

static int cqspi_auto_probe(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;

	if (cqspi->probed)
		return ERROR_OK;
	return cqspi_probe(bank);
}

static int cqspi_check_range(struct flash_bank *bank, uint32_t offset, uint32_t count)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;

	if (!cqspi->probed) {
		LOG_ERROR("cqspi: flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}
	if (offset > bank->size || count > bank->size - offset) {
		LOG_ERROR("cqspi: access beyond end of flash");
		return ERROR_FLASH_DST_OUT_OF_BANK;
	}
	return ERROR_OK;
}

static int cqspi_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	int retval;

	if (!cqspi->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (last < first || last >= bank->num_sectors) {
		LOG_ERROR("cqspi: flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("cqspi: flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	cqspi_op_begin(bank);
	retval = ERROR_OK;
	for (unsigned int sector = first; sector <= last; sector++) {
		retval = spi_nor_erase_sector(bank, bank->sectors[sector].offset);
		if (retval != ERROR_OK) {
			LOG_ERROR("cqspi: erasing sector %u failed", sector);
			break;
		}
		bank->sectors[sector].is_erased = 1;
		keep_alive();
	}

	return cqspi_op_end(bank, retval);
}

static int cqspi_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	/* Protection is only handled in software, like other SPI flash drivers */
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int cqspi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int cqspi_flash_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	int retval;

	retval = cqspi_check_range(bank, offset, count);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		const struct flash_sector *s = &bank->sectors[sector];
		if (count && offset < s->offset + s->size &&
				offset + count - 1 >= s->offset && s->is_protected) {
			LOG_ERROR("cqspi: flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	const uint32_t page_size = cqspi->dev.pagesize;
	cqspi_op_begin(bank);
	while (count > 0) {
		uint32_t len;

		if (cqspi->write_mode == CQSPI_WRITE_PAGE) {
			/* Page program, never crossing a page boundary (spi-nor write loop) */
			uint32_t page_remain = page_size - (offset % page_size);

			len = MIN(count, page_remain);
			retval = spi_nor_page_program(bank, offset, buffer, len);
		} else {
			/*
			 * Multi-page writes of up to CQSPI_STREAM_CHUNK bytes, so
			 * that a failure is reported for a limited address range.
			 */
			len = MIN(count, CQSPI_STREAM_CHUNK - (offset % CQSPI_STREAM_CHUNK));
			retval = spi_nor_write_stream(bank, offset, buffer, len);
		}
		if (retval != ERROR_OK) {
			LOG_ERROR("cqspi: programming 0x%08" PRIx32 "..0x%08" PRIx32 " failed",
				offset, offset + len - 1);
			break;
		}

		offset += len;
		buffer += len;
		count -= len;
		keep_alive();
	}

	return cqspi_op_end(bank, retval);
}

static int cqspi_flash_read(struct flash_bank *bank, uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	int retval = cqspi_check_range(bank, offset, count);
	if (retval != ERROR_OK)
		return retval;

	if (!count)
		return ERROR_OK;

	cqspi_op_begin(bank);
	retval = spi_nor_read_data(bank, offset, buffer, count);
	return cqspi_op_end(bank, retval);
}

static int cqspi_flash_verify(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	uint8_t *data;
	int retval;

	if (!count)
		return ERROR_OK;

	data = malloc(count);
	if (!data) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	retval = cqspi_flash_read(bank, data, offset, count);
	if (retval == ERROR_OK && memcmp(data, buffer, count) != 0) {
		LOG_ERROR("cqspi: verify failed");
		retval = ERROR_FAIL;
	}

	free(data);
	return retval;
}

static int cqspi_erase_check(struct flash_bank *bank)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;
	uint8_t *data;
	int retval = ERROR_OK;

	if (!cqspi->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	data = malloc(cqspi->dev.sectorsize);
	if (!data) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		struct flash_sector *s = &bank->sectors[sector];

		retval = cqspi_flash_read(bank, data, s->offset, s->size);
		if (retval != ERROR_OK)
			break;

		s->is_erased = 1;
		for (uint32_t i = 0; i < s->size; i++) {
			if (data[i] != bank->erased_value) {
				s->is_erased = 0;
				break;
			}
		}
		keep_alive();
	}

	free(data);
	return retval;
}

static int cqspi_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct cqspi_flash_bank *cqspi = bank->driver_priv;

	if (!cqspi->probed) {
		command_print_sameline(cmd, "\nCadence QSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	command_print_sameline(cmd, "\nCadence QSPI controller at " TARGET_ADDR_FMT
			", FIFO at " TARGET_ADDR_FMT " (depth %" PRIu32 " words), CS%u\n",
			cqspi->iobase, cqspi->ahb_base, cqspi->fifo_depth, cqspi->f_pdata.cs);
	command_print_sameline(cmd, "  Device '%s' (ID 0x%06" PRIx32 "), "
			"%" PRIu32 " bytes, page 0x%" PRIx32 ", sector 0x%" PRIx32 "\n",
			cqspi->dev.name, cqspi->jedec_id, cqspi->dev.size_in_bytes,
			cqspi->dev.pagesize, cqspi->dev.sectorsize);
	command_print_sameline(cmd, "  Commands: read 0x%02" PRIx8 " (%u dummy clocks), "
			"program 0x%02" PRIx8 ", erase 0x%02" PRIx8 ", %u address bytes%s\n",
			cqspi->read_opcode, cqspi->read_dummy_nbytes * CQSPI_DUMMY_CLKS_PER_BYTE,
			cqspi->dev.pprog_cmd, cqspi->dev.erase_cmd, cqspi->addr_nbytes,
			cqspi->use_fsr ? ", flag status register" : "");
	if (cqspi->master_ref_clk_hz)
		command_print_sameline(cmd, "  SCLK %u Hz requested, reference clock %" PRIu32 " Hz\n",
				cqspi->f_pdata.clk_rate, cqspi->master_ref_clk_hz);
	else
		command_print_sameline(cmd, "  SCLK: left as configured by boot firmware\n");
	command_print_sameline(cmd, "  Write mode: %s\n", cqspi_write_mode_names[cqspi->write_mode]);

	return ERROR_OK;
}

COMMAND_HANDLER(cqspi_handle_write_mode_command)
{
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank_probe_optional, 0, &bank, false);
	if (retval != ERROR_OK)
		return retval;

	if (bank->driver != &cadence_qspi_flash) {
		command_print(CMD, "flash bank '%s' is not a cadence_qspi bank", bank->name);
		return ERROR_FAIL;
	}

	struct cqspi_flash_bank *cqspi = bank->driver_priv;

	if (CMD_ARGC == 2) {
		if (strcmp(CMD_ARGV[1], "stream") == 0)
			cqspi->write_mode = CQSPI_WRITE_STREAM;
		else if (strcmp(CMD_ARGV[1], "page") == 0)
			cqspi->write_mode = CQSPI_WRITE_PAGE;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "%s", cqspi_write_mode_names[cqspi->write_mode]);
	return ERROR_OK;
}

static const struct command_registration cqspi_subcommand_handlers[] = {
	{
		.name = "write_mode",
		.handler = cqspi_handle_write_mode_command,
		.mode = COMMAND_ANY,
		.usage = "bank_id ['stream'|'page']",
		.help = "Show or select how data is programmed: 'stream' (default) "
			"uses multi-page writes, 'page' programs page by page "
			"like the Linux spi-nor layer.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration cqspi_command_handlers[] = {
	{
		.name = "cadence_qspi",
		.mode = COMMAND_ANY,
		.help = "Cadence QSPI flash command group",
		.usage = "",
		.chain = cqspi_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver cadence_qspi_flash = {
	.name = "cadence_qspi",
	.usage = "flash bank <name> cadence_qspi <base> <size> 0 0 <target> "
		"[-ctrl-base <addr>] [-ahb-base <addr>] [-trigger-address <addr>] "
		"[-fifo-depth <words>] [-fifo-width <bytes>] [-cs <n>] [-decoded-cs] "
		"[-ref-clk <hz>] [-sclk <hz>] [-read-delay <n>] [-rclk-en] "
		"[-tshsl-ns <ns>] [-tsd2d-ns <ns>] [-tchsh-ns <ns>] [-tslch-ns <ns>] "
		"[-write-mode stream|page]",
	.commands = cqspi_command_handlers,
	.flash_bank_command = cqspi_flash_bank_command,
	.erase = cqspi_erase,
	.protect = cqspi_protect,
	.write = cqspi_flash_write,
	.read = cqspi_flash_read,
	.verify = cqspi_flash_verify,
	.probe = cqspi_probe,
	.auto_probe = cqspi_auto_probe,
	.erase_check = cqspi_erase_check,
	.protect_check = cqspi_protect_check,
	.info = cqspi_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
