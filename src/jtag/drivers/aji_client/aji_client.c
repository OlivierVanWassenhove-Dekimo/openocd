/***************************************************************************
 *   Copyright (C) 2007-2008 by Øyvind Harboe                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef INC_CONFIG_H
#include "config.h"
#endif

#include <time.h>

#include <jtag/jtag.h>
#include <target/embeddedice.h>
#include <jtag/interface.h>
#include <helper/jep106.h>

#include "jtag/commands.h"
#include "jtag/jtagcore_overwrite.h"
#include "server/server.h"

#include "aji/aji.h"
#include "aji/c_jtag_client_gnuaji.h"
#include "jtagservice.h"

extern void jtag_tap_add(struct jtag_tap *t);

//=================================
// Helpers
//=================================

/*
 * Access functions to lowlevel driver, agnostic of libftdi/libftdxx
 */
static char *hexdump(const uint8_t *buf, const unsigned int size)
{
	unsigned int i;
	char *str = calloc(size * 2 + 1, 1);

	for (i = 0; i < size; i++)
		sprintf(str + 2 * i, "%02x", buf[i]);
	return str;
}

//========================================
// Init/Free
//========================================

/**
 * Initialize
 *
 * Returns ERROR_OK if JTAG Server found.
 * TODO: Write a TCL Command that allows me to specify the jtagserver config file
 */
static int aji_client_init(void)
{
	char *quartus_jtag_client_config = getenv("QUARTUS_JTAG_CLIENT_CONFIG");
	if (quartus_jtag_client_config != NULL)
	{
		LOG_INFO("Configuration file, set via QUARTUS_JTAG_CLIENT_CONFIG, is '%s'",
				 quartus_jtag_client_config);
	}
	else
	{
		LOG_DEBUG("You can choose to pass a jtag client config file QUARTUS_JTAG_CLIENT_CONFIG");
	}

	AJI_ERROR status = AJI_NO_ERROR;
	status = jtagservice_init(JTAGSERVICE_TIMEOUT_MS);
	if (AJI_NO_ERROR != status)
	{
		LOG_ERROR("Cannot initialize AJI JTAG services Return status is %d (%s)",
				  status, c_aji_error_decode(status));
		return ERROR_JTAG_INIT_FAILED;
	}

	status = jtagservice_scan_for_cables();
	if (AJI_NO_ERROR != status)
	{
		LOG_ERROR("Cannot select JTAG Cable. Return status is %d (%s)",
				  status, c_aji_error_decode(status));
		jtagservice_free(JTAGSERVICE_TIMEOUT_MS);
		return ERROR_JTAG_INIT_FAILED;
	}

	status = jtagservice_scan_for_taps();
	if (AJI_NO_ERROR != status)
	{
		LOG_ERROR("Cannot select TAP devices. Return status is %d (%s)",
				  status, c_aji_error_decode(status));
		jtagservice_free(JTAGSERVICE_TIMEOUT_MS);
		return ERROR_JTAG_INIT_FAILED;
	}

	// This driver automanage the tap_state. Hence, this is just
	//  to tell the targets which state we think our TAP interface is at.
	tap_set_state(TAP_RESET);

	// overwrites JTAGCORE
	struct jtagcore_overwrite *record = jtagcore_get_overwrite_record();
	record->jtag_examine_chain = jtagservice_jtag_examine_chain;
	record->jtag_validate_ircapture = jtagservice_jtag_validate_ircapture;

	return ERROR_OK;
}

static int aji_client_quit(void)
{
	struct jtagcore_overwrite *record = jtagcore_get_overwrite_record();
	record->jtag_examine_chain = NULL;
	record->jtag_validate_ircapture = NULL;

	jtagservice_free(JTAGSERVICE_TIMEOUT_MS);

	c_jtag_client_gnuaji_free();

	return ERROR_OK;
}

//-----------------
// jtag operations
//-----------------

/**
 * Go to Run-Test-Idle State. Stay there for \c cycles,
 * then go to \c state
 *
 * \pre #jtagservice_lock() locked the JTAG scan chain by locking
 *      one of the TAP in the JTAG scan chain.

 * \param cycles The number of cyles to stay at \c TAP_IDLE
 *					state
 * \param state The final state the JTAG state machine is expected
 *					to go to. Currently must be set to
 *					\c TAP_IDLE or a warning will be produced.
 * 					This request to move to \c state is actually
 *					ignored ignored because	\c jtagserv
 *					automatically managed the JTAG state
 *					machine
 */
static int aji_client_runtest(int cycles, enum tap_state state)
{
	LOG_DEBUG_IO("%s(cycles=%i, end_state=%d(%s))", __func__,
				 cycles, state, tap_state_name(state));

	AJI_ERROR status = AJI_NO_ERROR;

	AJI_OPEN_ID open_id = jtagservice_get_in_use_open_id();
	status = c_aji_run_test_idle(open_id, cycles);
	if (AJI_NO_ERROR != status)
	{
		LOG_ERROR("Unexpected error setting TAPs to RUN/IDLE state."
				  " Return status is %d (%s)",
				  status, c_aji_error_decode(status));
	}
	tap_set_state(TAP_IDLE);

	if (state != TAP_IDLE)
	{
		LOG_WARNING("Not yet support interface_jtag_add_runtest() "
					" to finish in non TAP_IDLE state. "
					"State %s (%d) requested",
					tap_state_name(state), state);
	}

	return status == AJI_NO_ERROR ? ERROR_OK : ERROR_FAIL;
}

/**
 * Go to TLR (Test Logic Reset) State
 *
 * \pre #jtagservice_lock() locked the JTAG scan chain by locking
 *      one of the TAP in the JTAG scan chain.
 */
static int aji_client_goto_tlr(void)
{

	LOG_DEBUG_IO("(from %s to %s)", tap_state_name(tap_get_state()),
				 tap_state_name(TAP_RESET));

	/*
	 The code below are working, but disabled because
	 (1) jtagserv.exe automatically managers the internal JTAG State.
		 One of the thing it does is to cycle through the TLR state
		 automatically. Making this function redundant
	 (2) In testing, it is found that when another AJI Client is connected
		 to the same JTAG Scan Chain, we will consistently take the
		 "Ignore call to set TAPs to TLR state ..." path meaning
		 we do not move the JTAG state machine to TLR. To have multiple
		 AJI Clients connecting to the same JTAG Scan Chain is the norm
		 for our user.
	 */
	LOG_DEBUG("No need to perform TLR request. The server manages JTAG states");
	tap_set_state(TAP_RESET); // Lie to OpenOCD that TAP_RESET state was reached.
	return ERROR_OK;

	/*
	 The code below are working, See previous comment on why it is redundant
	 */
	/*
	AJI_ERROR status = AJI_NO_ERROR;

	AJI_OPEN_ID open_id =jtagservice_get_in_use_tap_open_id();
	assert(open_id); // i.e., not NULL
	status = c_aji_test_logic_reset(open_id);
	if (AJI_NO_ERROR == status) {
		tap_set_state(TAP_RESET);
	} else if (AJI_CHAIN_IN_USE == status) {
		LOG_WARNING("Ignoring call to set TAPs to TLR state after IR scan."
		" Someone else is using the scan chain."

		);
		tap_set_state(TAP_RESET); //Lie to OpenOCD that TAP_RESET state was reached.
		status = AJI_NO_ERROR;
	} else {
		LOG_ERROR("Unexpected error setting TAPs to TLR state. Return status is %d (%s)",
			status, c_aji_error_decode(status)
		);
	}

	return status == AJI_NO_ERROR ? ERROR_OK : ERROR_FAIL;
	*/
}

/**
 * Build I/O buffers from \c cmd
 *
 * \param cmd The \c scan_command to extract data from
 * \param bit_count On output, the number of data bits required by \c cmd.
 * \param write_buffer On Output, a buffer of at least \c bit_count bits
 *             containing the data to be written to the TAP pointed to by \c cmd.
 *             If NULL, no data write  is needed.
 *             caller to free the memory after use.
 * \param read_buffer On Output, a buffer of at least \c bit_count bits
 *             filled with zero to receive data from the TAP pointed to by \c cmd
 *             If NULL, no data read is needed.
 *             caller to free the memory after use.
 *
 * \return #ERROR_OK success
 * \return #ERROR_FAIL if there is a problem
 */
static int aji_client_build_buffers(
	const struct scan_command *const cmd,
	DWORD *bit_count,
	BYTE **write_buffer,
	BYTE **read_buffer)
{
	struct scan_command mock_cmd = {
		.ir_scan = cmd->ir_scan,
		.fields = cmd->tap_fields,
		.num_fields = cmd->num_tap_fields};
	*bit_count = jtag_scan_size(&mock_cmd);

	*write_buffer = NULL;
	int req = jtag_build_buffer(&mock_cmd, write_buffer);
	if (req)
	{
		if (*write_buffer == NULL)
		{
			LOG_ERROR("Insufficient memory for write buffer");
			return ERROR_FAIL;
		}
	}
	else
	{
		free(*write_buffer);
		*write_buffer = NULL;
	}

	*read_buffer = NULL;
	req = 0;
	for (int i = 0; i < cmd->num_tap_fields; i++)
	{
		if (cmd->tap_fields[i].in_value)
		{
			req += cmd->tap_fields[i].num_bits;
		}
	}
	if (req)
	{
		*read_buffer = calloc(1, DIV_ROUND_UP(*bit_count, 8));
		if (*read_buffer == NULL)
		{
			LOG_ERROR("Insufficient memory for read buffer");
			return ERROR_FAIL;
		}
	}
	else
	{
		*read_buffer = NULL;
	}

	return ERROR_OK;
}

static int aji_client_read_buffer(
	BYTE *read_buffer,
	const struct scan_command *cmd)
{
	struct scan_command mock_cmd = {
		.ir_scan = cmd->ir_scan,
		.fields = cmd->tap_fields,
		.num_fields = cmd->num_tap_fields};
	return jtag_read_buffer(read_buffer, &mock_cmd);
}

/**
 * A scan whose captured data has to be handed back to OpenOCD.
 *
 * With deferred (packed) AJI commands the captured data is only valid after
 * the AJI command queue has been flushed, so the buffers are kept until
 * the end of #aji_client_execute_queue().
 */
struct aji_client_pending_read
{
	const struct scan_command *cmd;
	BYTE *read_buffer;
	DWORD bit_count;
	DWORD ir_capture; // IR scans capture into a DWORD
};

/**
 * Handles IR and DR scans
 *
 * \pre #jtagservice_lock() locked the required TAP
 *
 * \param cmd Contains the detail about the scan operation
 * \param pending On output, \c pending->cmd is set if the scan captures
 *             data, which has to be completed with
 *             #aji_client_complete_read() after the AJI commands have been
 *             flushed. The caller must free \c pending->read_buffer.
 *             Must not move in memory until then.
 */
static int aji_client_scan(struct scan_command *const cmd,
						   struct aji_client_pending_read *pending)
{
	DWORD bit_count = 0;
	BYTE *write_buffer = NULL,
		 *read_buffer = NULL;
	char *log_buf = NULL;

	pending->cmd = NULL;
	pending->read_buffer = NULL;

	if (aji_client_build_buffers(cmd, &bit_count, &write_buffer, &read_buffer) != ERROR_OK)
	{
		free(write_buffer);
		free(read_buffer);
		return ERROR_FAIL;
	}

	if (write_buffer && LOG_LEVEL_IS(LOG_LVL_DEBUG_IO))
	{
		log_buf = hexdump(write_buffer, DIV_ROUND_UP(bit_count, 8));
		LOG_DEBUG_IO("%s(scan=%s%s, type=OUT, bits=%lu, buf=[%s], end_state=%d)", __func__,
					 cmd->tap_is_sld ? "Virtual " : "",
					 cmd->ir_scan ? "IRSCAN" : "DRSCAN",
					 (long unsigned)bit_count, log_buf, cmd->end_state);
		free(log_buf);
	}
	else if (!write_buffer)
	{
		LOG_DEBUG_IO("%s(scan=%s%s, type=OUT, no write,  end_state=%d)", __func__,
					 cmd->tap_is_sld ? "Virtual " : "",
					 cmd->ir_scan ? "IRSCAN" : "DRSCAN",
					 cmd->end_state);
	}

	AJI_ERROR status = AJI_NO_ERROR;
	AJI_OPEN_ID open_id = jtagservice_get_in_use_open_id();

	if (cmd->ir_scan)
	{
		// LIMITATION: Max IR length is 32 bit has it has to fit into a DWORD
		//   since c_aji_access_ir, a.k.a. aji_access_ir(DWORD)
		//   Cannot be relaxed even if c_aji_access_ir_a(), a.k.a.
		//   aji_access_ir(BYTE) version, is used because that function also
		//   translate back to c_aji_access_ir()
		//
		// TODO: use c_aji_access_ir_a(), i.e. aji_acess_ir(BYTE) to avoid
		//   having to translate  write_buffer -> instruction and
		//   capture -> read_buffer explicitly

		DWORD instruction = 0;
		for (DWORD i = 0; i < (bit_count + 7) / 8; i++)
		{
			instruction |= write_buffer[i] << (i * 8);
		}

		// The capture may be filled in later, when the AJI commands are
		// flushed, so it must live in *pending.
		pending->ir_capture = 0;
		if (cmd->tap_is_sld)
		{
			status = c_aji_access_overlay(open_id, instruction,
										  read_buffer ? &pending->ir_capture : NULL);
		}
		else
		{
			status = c_aji_access_ir(
				open_id, instruction, read_buffer ? &pending->ir_capture : NULL, 0);
		}
	}
	else
	{
		// When deferred, libaji_client copies write_buffer but fills
		// read_buffer only once the commands are flushed.
		status = c_aji_access_dr(
			open_id, bit_count, AJI_DR_UNUSED_X,
			0, write_buffer ? bit_count : 0, write_buffer,
			0, read_buffer ? bit_count : 0, read_buffer);
	} // end else-if (cmd->ir_scan)

	free(write_buffer);

	if (status != AJI_NO_ERROR)
	{
		LOG_ERROR("Failure to access %s%s register. Return Status is %d (%s)",
				  cmd->tap_is_sld ? "Virtual " : "",
				  cmd->ir_scan ? "IRSCAN" : "DRSCAN",
				  status, c_aji_error_decode(status));
		free(read_buffer);
		return ERROR_FAIL;
	}

	if (read_buffer)
	{
		pending->cmd = cmd;
		pending->read_buffer = read_buffer;
		pending->bit_count = bit_count;
	}

	if (TAP_IDLE != cmd->end_state)
	{
		LOG_WARNING("%s%s not yet handle transition to state other than TAP_IDLE(%d)."
					" Requested state is %s(%d)",
					cmd->tap_is_sld ? "Virtual " : "",
					cmd->ir_scan ? "IRSCAN" : "DRSCAN",
					TAP_IDLE, tap_state_name(cmd->end_state), cmd->end_state);
		assert(0); // deliberately assert() to be able to see where the error is, if it occurs
	}

	//// Let jtagserv manages the JTAG state instead of setting it manually
	/* status = c_aji_run_test_idle(open_id, 2);
	 if (AJI_NO_ERROR != status) {
		LOG_WARNING("Failed to go to TAP_IDLE after IR register write");
	}
	*/

	tap_set_state(TAP_IDLE); // Faking move to TAP_IDLE
	return ERROR_OK;
}

/**
 * Hand the data captured by a scan back to OpenOCD.
 *
 * \pre The AJI commands have been flushed, i.e. the captured data is valid.
 */
static void aji_client_complete_read(struct aji_client_pending_read *pending)
{
	const struct scan_command *cmd = pending->cmd;
	BYTE *read_buffer = pending->read_buffer;
	DWORD bit_count = pending->bit_count;

	if (cmd->ir_scan)
	{
		for (DWORD i = 0; i < (bit_count + 7) / 8; i++)
		{
			read_buffer[i] = (BYTE)(pending->ir_capture >> (i * 8));
		}
	}

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO))
	{
		char *log_buf = hexdump(read_buffer, DIV_ROUND_UP(bit_count, 8));
		LOG_DEBUG_IO("%s(scan=%s%s, type=IN, bits=%lu, buf=[%s], end_state=%d)", __func__,
					 cmd->tap_is_sld ? "Virtual " : "",
					 cmd->ir_scan ? "IRSCAN" : "DRSCAN",
					 (long unsigned)bit_count, log_buf, cmd->end_state);
		free(log_buf);
	}

	aji_client_read_buffer(read_buffer, cmd);
}

/**
 * Find the next tap used in a jtag_command sequence
 *
 * \param cmds The list of commands to search
 * \param next_tap On output,the next tap if found.
 *                 if not, no modification to the input value,
 */
static void find_next_active_tap(
	const struct jtag_command *cmds,
	struct jtag_tap **next_tap)
{
	for (const struct jtag_command *candidate = cmds;
		 candidate != NULL;
		 candidate = candidate->next)
	{
		if (candidate->type == JTAG_SCAN)
		{
			(*next_tap) = candidate->cmd.scan->tap;
			return;
		}
	}
}

int aji_client_execute_queue(struct jtag_command *cmd_queue)
{
	int ret = ERROR_OK;

	if (cmd_queue == NULL)
	{
		return ERROR_OK;
	}

	// Captured scan data is only valid once the AJI commands have been
	// flushed (see jtagservice_tap_pack_style), so keep track of all scans
	// with captures and complete them at the end. The array is sized up front
	// because libaji_client may hold pointers into it.
	unsigned int scan_count = 0;
	for (struct jtag_command *cmd = cmd_queue; cmd != NULL; cmd = cmd->next)
	{
		if (cmd->type == JTAG_SCAN)
			scan_count++;
	}

	struct aji_client_pending_read *pending = NULL;
	unsigned int pending_count = 0;
	if (scan_count)
	{
		pending = calloc(scan_count, sizeof(*pending));
		if (pending == NULL)
		{
			LOG_ERROR("Insufficient memory for JTAG queue");
			return ERROR_FAIL;
		}
	}

	struct jtag_tap *tap = NULL;
	for (struct jtag_command *cmd = cmd_queue; ret == ERROR_OK && cmd != NULL;
		 cmd = cmd->next)
	{

		keep_alive();

		find_next_active_tap(cmd, &tap);
		jtagservice_lock(tap);

		switch (cmd->type)
		{
		case JTAG_RESET:
			LOG_DEBUG("Ignoring command JTAG_RESET(trst=%d, srst=%d)",
					  cmd->cmd.reset->trst,
					  cmd->cmd.reset->srst);
			// aji_client_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			break;
		case JTAG_RUNTEST:
			aji_client_runtest(
				cmd->cmd.runtest->num_cycles,
				cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			LOG_ERROR("Not yet coded JTAG_STABLECLOCKS(num_cycles=%d)",
					  cmd->cmd.stableclocks->num_cycles);
			// aji_client_stableclocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			aji_client_goto_tlr();
			break;
		case JTAG_PATHMOVE:
			LOG_ERROR(
				"Ignoring JTAG_PATHMOVE(numstate=%d, first_state=0x%x, end_state=0x%x)",
				cmd->cmd.pathmove->num_states,
				cmd->cmd.pathmove->path[0],
				cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);
			// aji_client_path_move(cmd->cmd.pathmove);
			assert(0); // deliberately assert() to be able to see where the error is, if it occurs
			break;
		case JTAG_TMS:
			LOG_ERROR("Not yet coded JTAG_TMS(num_bits=%d)",
					  cmd->cmd.tms->num_bits);
			// aji_client_tms(cmd->cmd.tms);
			assert(0); // deliberately assert() to be able to see where the error is, if it occurs
			break;
		case JTAG_SLEEP:
			LOG_ERROR("Not yet coded JTAG_SLEEP(time=%d us)", cmd->cmd.sleep->us);
			// aji_client_usleep(cmd->cmd.sleep->us);
			assert(0); // deliberately assert() to be able to see where the error is, if it occurs
			break;
		case JTAG_SCAN:
			ret = aji_client_scan(cmd->cmd.scan, &pending[pending_count]);
			if (pending[pending_count].cmd)
				pending_count++;
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%X",
					  cmd->type);
			ret = ERROR_FAIL;
			break;
		}
	}

	// Unlocking flushes all deferred AJI commands, which makes the captured
	// data valid and reports errors of deferred commands.
	AJI_ERROR status = jtagservice_unlock();
	if (status != AJI_NO_ERROR)
	{
		LOG_ERROR("JTAG queue execution failed. Return status is %d (%s)",
				  status, c_aji_error_decode(status));
		ret = ERROR_FAIL;
	}

	for (unsigned int i = 0; i < pending_count; i++)
	{
		if (ret == ERROR_OK)
			aji_client_complete_read(&pending[i]);
		free(pending[i].read_buffer);
	}
	free(pending);

	return ret;
}

static const struct command_registration vjtag_subcommand_handlers[] = {
	{
		.name = "create",
		.mode = COMMAND_CONFIG,
		.handler = vjtag_create,
		.help = "Create a new virtual TAP instance on tapname, "
				"and appends it to the scan chain.",
		.usage = "name '-chain-position' tapname '-expected_id' idcode "
				 "['-ignore-version'] ",
	},
	COMMAND_REGISTRATION_DONE}; // end vjtag_subcommand_handlers

COMMAND_HANDLER(aji_client_handle_pack_style_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 1)
	{
		if (strcmp(CMD_ARGV[0], "manual") == 0)
			jtagservice_set_tap_pack_style(AJI_PACK_MANUAL);
		else if (strcmp(CMD_ARGV[0], "auto") == 0)
			jtagservice_set_tap_pack_style(AJI_PACK_AUTO);
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "%s",
				  jtagservice_get_tap_pack_style() == AJI_PACK_MANUAL ? "manual" : "auto");
	return ERROR_OK;
}

static const struct command_registration aji_client_subcommand_handlers[] = {
	{
		.name = "pack_style",
		.handler = &aji_client_handle_pack_style_command,
		.mode = COMMAND_ANY,
		.help = "Select how JTAG scans are sent to the JTAG server: "
				"'manual' (default) packs all scans of a JTAG queue, "
				"'auto' sends every scan capturing data synchronously "
				"(slow, previous behaviour)",
		.usage = "[manual|auto]",
	},
	{
		/*
		 * This is a duplication of "hardware" top level command
		 * Prefer the "hardware" top level command over this one,
		 * i.e., "aji_client hardware ..."
		 * The reason this exists is because there are implementation that
		 * relies on "aji_client hardware ..."
		 */
		.name = "hardware",
		.handler = &jtag_newhardware,
		.mode = COMMAND_CONFIG,
		.help = "select the hardware",
		.usage = "<name> <type>[<port>]",
	},
	COMMAND_REGISTRATION_DONE};

static const struct command_registration aji_client_command_handlers[] = {
	{
		.name = "aji_client",
		.mode = COMMAND_ANY,
		.help = "Perform aji_client management",
		.usage = "",
		.chain = aji_client_subcommand_handlers,
	},
	{
		.name = "hardware",
		.handler = &jtag_newhardware,
		.mode = COMMAND_CONFIG,
		.help = "select the hardware",
		.usage = "<name> <type>[<port>]",
	},
	{
		.name = "vjtag",
		.mode = COMMAND_ANY,
		.help = "perform jtag tap actions",
		.usage = "",

		.chain = vjtag_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE};

static struct jtag_interface aji_client_interface = {
	.execute_queue = aji_client_execute_queue,
};

struct adapter_driver aji_client_adapter_driver = {
	.name = "aji_client",
	.transport_ids = TRANSPORT_JTAG,
	.transport_preferred_id = TRANSPORT_JTAG,
	.commands = aji_client_command_handlers,

	.init = aji_client_init,
	.quit = aji_client_quit,
	.speed = NULL,
	.khz = NULL,
	.speed_div = NULL,
	.power_dropout = NULL,
	.srst_asserted = NULL,

	.jtag_ops = &aji_client_interface,
};
