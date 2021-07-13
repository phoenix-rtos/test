/*
 * Phoenix-RTOS
 *
 * psu - sdp script loader
 *
 * Copyright 2001, 2004 Pawel Pisarczyk
 * Copyright 2012, 2018-2020 Phoenix Systems
 *
 * Author: Pawel Pisarczyk, Jacek Popko, Hubert Buczynski, Aleksander Kaminski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <termios.h>
#include <stdlib.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <ctype.h>

#include "../common/hid.h"
#include "../common/script.h"

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

/* SDP protocol section */
#define SET_CMD_TYPE(b, v) (b)[0] = (b)[1] = (v)
#define SET_ADDR(b, v) *((uint32_t *)((b) + 2)) = htonl(v)
#define SET_COUNT(b, v) *((uint32_t *)((b) + 7)) = htonl(v);
#define SET_DATA(b, v) *((uint32_t *)((b) + 11)) = htonl(v);
#define SET_FORMAT(b, v) (b)[6] = (v);

/* MCUBoot protocol */
#define FRAME_CMD_OUT 1
#define FRAME_DATA 2
#define FRAME_CMD_IN 3
#define MCU_CMD_SIZE 32
#define MCU_GET_PROPERTY 0x07
#define MCU_GET_PROPERTY_RESPONSE 0xa7
#define MCU_MAX_PAYLOAD 1016

#define CMD_SIZE 17
#define BUF_SIZE 1025
#define INTERRUPT_SIZE 65


int usage(char *progname)
{
	printf("Usage: %s script_path\n", progname);

	return -1;
}


static inline void set_write_file_cmd(unsigned char *b, uint32_t addr, uint32_t size)
{
	SET_CMD_TYPE(b, 0x04);
	SET_ADDR(b, addr);
	SET_COUNT(b, size);
	SET_FORMAT(b, 0x20);
}


static inline void set_dcd_write_cmd(unsigned char *b, uint32_t addr, uint32_t size)
{
	SET_CMD_TYPE(b, 0x0a);
	SET_ADDR(b, addr);
	SET_COUNT(b, size);
}


static inline void set_jmp_cmd(unsigned char *b, uint32_t addr)
{
	SET_CMD_TYPE(b, 0x0b);
	SET_ADDR(b, addr);
	SET_FORMAT(b, 0x20);
}


static inline void set_status_cmd(unsigned char *b)
{
	SET_CMD_TYPE(b, 0x05);
}


static inline void set_write_reg_cmd(unsigned char *b, uint32_t addr, uint8_t format, uint32_t data)
{
	SET_CMD_TYPE(b, 0x02);
	SET_ADDR(b, addr);
	SET_FORMAT(b, format);
	SET_COUNT(b, format / 8);
	SET_DATA(b, data);
}


static int sdp_writeRegister(hid_device *dev, uint32_t addr, uint8_t format, uint32_t data)
{
	int rc;
	unsigned char b[BUF_SIZE] = { 0 };
	const uint32_t pattern = 0x128a8a12;

	fprintf(stderr, " - Writing value: %#x, to the address: %#x\n", data, addr);
	/* Send write command */
	b[0] = 1;
	set_write_reg_cmd(b + 1, addr, format, data);

	if ((rc = hid_write(dev, b, CMD_SIZE)) < 0) {
		fprintf(stderr, "Failed to send write_register command (rc=%d)\n", rc);
		return SCRIPT_ERROR;
	}

	/* Receive report 3 */
	if ((rc = hid_read(dev, b, BUF_SIZE)) < 5) {
		fprintf(stderr, "Failed to receive HAB mode (rc=%d)\n", rc);
		return SCRIPT_ERROR;
	}

	if ((rc = hid_read(dev, b, BUF_SIZE)) < 0 || memcmp(b + 1, &pattern, 4)) {
		fprintf(stderr, "Failed to receive complete status (rc=%d, status=%02x%02x%02x%02x)\n", rc, b[1], b[2], b[3], b[4]);
		return SCRIPT_ERROR;
	}

	return SCRIPT_OK;
}


static int sdp_writeFile(hid_device *dev, uint32_t addr, void *data, size_t size)
{
	int n, rc;
	ssize_t offset = 0;
	unsigned char b[BUF_SIZE] = { 0 };
	const uint32_t pattern = 0x88888888;

	/* Send write command */
	b[0] = 1;
	set_write_file_cmd(b + 1, addr, size);

	if ((rc = hid_write(dev, b, CMD_SIZE)) < 0) {
		fprintf(stderr, "Failed to send write_file command (%d)\n", rc);
		return SCRIPT_ERROR;
	}

	/* Send contents */
	b[0] = 2;
	while (offset < size) {
		n = (BUF_SIZE - 1 > size - offset) ? (size - offset) : (BUF_SIZE - 1);
		memcpy(b + 1, data + offset, n);
		offset += n;

		/* print progress */
		fprintf(stderr, "\r - Sent (%lu/%lu) %3.0f%% ", offset, size, ((float)offset / (float)size) * 100.0f);

		if ((rc = hid_write(dev, b, BUF_SIZE)) < 0) {
			fprintf(stderr, "\nFailed to send image contents (rc=%d)\n", rc);
			return SCRIPT_ERROR;
		}
	}
	fprintf(stderr, "\n");

	/* Receive report 3 */
	if ((rc = hid_read(dev, b, BUF_SIZE)) < 5) {
		fprintf(stderr, "Failed to receive HAB mode (rc=%d)\n", rc);
		return SCRIPT_ERROR;
	}

	if ((rc = hid_read(dev, b, BUF_SIZE)) < 0 || memcmp(b + 1, &pattern, 4)) {
		fprintf(stderr, "Failed to receive complete status (rc=%d, status=%02x%02x%02x%02x)\n", rc, b[1], b[2], b[3], b[4]);
		return SCRIPT_ERROR;
	}

	fprintf(stderr, " - File has been written correctly.\n");

	return SCRIPT_OK;
}


static int sdp_jmpAddr(hid_device *dev, uint32_t addr)
{
	int rc;
	unsigned char b[BUF_SIZE] = { 0 };

	fprintf(stderr, " - To the address: %#x\n", addr);

	/* Send write command */
	b[0] = 1;
	set_jmp_cmd(b + 1, addr);

	if ((rc = hid_write(dev, b, CMD_SIZE)) < 0) {
		fprintf(stderr, "Failed to send jump_address command (rc=%d)\n", rc);
		return rc;
	}

	/* Receive report 3 */
	if ((rc = hid_read(dev, b, BUF_SIZE)) < 5) {
		fprintf(stderr, "Failed to receive HAB mode (rc=%d)\n", rc);
		return -1;
	}

	return SCRIPT_OK;
}


static int sdp_errStatus(hid_device *dev)
{
	unsigned char b[INTERRUPT_SIZE] = { 0 };
	int rc;

	b[0] = 1;
	set_status_cmd(b + 1);

	if ((rc = hid_write(dev, b, CMD_SIZE)) < 0) {
		fprintf(stderr, "Failed to send status command (rc=%d)\n", rc);
		return rc;
	}

	if ((rc = hid_read(dev, b, INTERRUPT_SIZE)) < 5) {
		fprintf(stderr, "Failed to receive HAB mode (rc=%d)\n", rc);
		return rc;
	}

	if ((rc = hid_read(dev, b, INTERRUPT_SIZE)) < 0) {
		fprintf(stderr, "Failed to receive status (rc=%d)\n", rc);
		return rc;
	}

	return SCRIPT_OK;
}


typedef struct {
	unsigned char reportID;
	unsigned char padding;
	unsigned short size;
	unsigned char payload[];
} __attribute__((packed)) mcuboot_frame_t;


typedef struct {
	unsigned char tag;
	unsigned char flags;
	unsigned char reserved;
	unsigned char paramcnt;
	int params[];
} __attribute__((packed)) mcuboot_cmd_t;


static unsigned short size2LE(unsigned short size)
{
	if (size == htons(size))
		return (unsigned short)((size & (unsigned short)0xff) << 8) | (size >> 8);

	return size;
}


static unsigned int paramByteSwap(unsigned int param)
{
	/* If host is BE, swap bytes. Works both way */
	if (param == htonl(param))
		return ((param & 0xff) << 24) | ((param & 0xff00) << 8) |
				((param & 0xff0000) >> 8) | ((param & 0xff000000) >> 24);

	return param;
}


static int mcuboot_getProperty(hid_device *dev, int which)
{
	unsigned char b[sizeof(mcuboot_frame_t) + sizeof(mcuboot_cmd_t) + 2 * 4] = { 0 };
	mcuboot_frame_t *frame = (mcuboot_frame_t *)b;
	mcuboot_cmd_t *cmd = (mcuboot_cmd_t *)(b + sizeof(mcuboot_frame_t));
	int rc;

	frame->reportID = FRAME_CMD_OUT;
	frame->size = size2LE(sizeof(b) - sizeof(mcuboot_frame_t));

	cmd->tag = MCU_GET_PROPERTY;
	cmd->paramcnt = 2;
	cmd->params[0] = paramByteSwap(which);
	cmd->params[1] = 0;

	if ((rc = hid_write(dev, b, sizeof(b))) < 0) {
		fprintf(stderr, "Failed to send get_property command (rc=%d)\n", rc);
		return rc;
	}

	if ((rc = hid_read(dev, b, sizeof(b))) < 0) {
		fprintf(stderr, "Failed to receive GetProperty Response (rc=%d)\n", rc);
		return rc;
	}

	if (cmd->params[0] != 0) {
		fprintf(stderr, "GetPropertyResponse status != 0 (%d)\n", paramByteSwap(cmd->params[0]));
		return -1;
	}

	fprintf(stderr, "Status: %d, Property: 0x%08x\n", paramByteSwap(cmd->params[0]), paramByteSwap(cmd->params[1]));

	return SCRIPT_OK;
}


static int mcuboot_loadImage(hid_device *dev, void *data, size_t size)
{
	unsigned char b[sizeof(mcuboot_frame_t) + MCU_MAX_PAYLOAD] = { 0 };
	mcuboot_frame_t *frame = (mcuboot_frame_t *)b;
	int rc;
	size_t offset = 0, chunk;

	while (offset < size) {
		chunk = size - offset;
		if (chunk > MCU_MAX_PAYLOAD)
			chunk = MCU_MAX_PAYLOAD;

		frame->reportID = FRAME_DATA;
		frame->size = size2LE(chunk);

		memcpy(frame->payload, (char *)data + offset, chunk);

		if ((rc = hid_write(dev, b, sizeof(mcuboot_frame_t) + chunk)) < 0) {
			fprintf(stderr, "Failed to send data (rc=%d)\n", rc);
			return rc;
		}

		offset += chunk;

		/* print progress */
		fprintf(stderr, "\r - Sent (%lu/%lu) %3.0f%% ", offset, size, ((float)offset / (float)size) * 100.0f);
	}

	fprintf(stderr, " - File has been written correctly.\n");

	return SCRIPT_OK;
}


static inline int8_t char_to_hex(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	else if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	else if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}

	return SCRIPT_ERROR;
}


static int parse_byte_string(script_blob_t str, script_blob_t *blob)
{
	void *ptr = blob->ptr;
	int8_t bh, bl;

	if (!(blob->ptr = realloc(ptr, str.end - str.ptr + 1 ))) {
		*blob = SCRIPT_BLOB_EMPTY;
		free(ptr);

		fprintf(stderr, "Unable to allocate memory.\n");

		return SCRIPT_ERROR;
	}

	for (blob->end = blob->ptr; str.ptr < str.end; str.ptr++) {
		if (*str.ptr != '\\') {
			*blob->end++ = *str.ptr;
			continue;
		}

		str.ptr++;

		if (*str.ptr == '\\') {
			*blob->end++ = *str.ptr;
			continue;
		}
		else if (*str.ptr == 'x' || *str.ptr == 'X') {
			if (((bh = char_to_hex(*(++str.ptr))) != SCRIPT_ERROR) && ((bl = char_to_hex(*(++str.ptr))) != SCRIPT_ERROR)) {
				*blob->end    = bh << 4;
				*blob->end++ |= bl;
				continue;
			}
		}

		free(blob->ptr);
		blob->ptr = NULL;

		fprintf(stderr, "Malformed byte string passed.\n");
		return SCRIPT_ERROR;
	}

	return SCRIPT_OK;
}


static int close_buffer(int type, script_blob_t *blob)
{
	if (type == 'F') {
		return munmap(blob->ptr, blob->end - blob->ptr);
	}
	else if (type == 'S') {
		free(blob->ptr);
	}

	*blob = SCRIPT_BLOB_EMPTY;

	return SCRIPT_OK;
}


static int get_buffer(script_t *s, int type, script_blob_t str, script_blob_t *blob)
{
	int fd;
	struct stat statbuf;

	if (type == 'F') {
		char *name = strndup(str.ptr, str.end - str.ptr);

		if ((fd = open(name, O_RDONLY)) < 0)
			s->errstr = "File not found.";

		free(name);

		if (!s->errstr) {
			fstat(fd, &statbuf);
			if ((blob->ptr = mmap(NULL, statbuf.st_size, PROT_READ, MAP_PRIVATE, fd, 0)) != MAP_FAILED) {
				blob->end = blob->ptr + statbuf.st_size;
			}
			else {
				s->errstr = "Unable to mmap file.";
			}

			close(fd);
		}
	}
	else if (type == 'S') {
		if (parse_byte_string(str, blob) < 0)
			s->errstr = "Error while parsing byte string.";
	}

	if (s->errstr) {
		s->next.str = str;
		return SCRIPT_ERROR;
	}

	if (s->flags & SCRIPT_F_DRYRUN) {
		close_buffer(type, blob);
	}
	else {
		fprintf(stderr, " - Sending to the device: %.*s\n", (int)(str.end - str.ptr), str.ptr);
	}

	return SCRIPT_OK;
}


static int wait_cmd(script_t *s)
{
	int retries;
	long int vid, pid;
	hid_device **dev = (hid_device **)s->arg;

	if (*dev != NULL)
		hid_close(*dev);

	if (script_expect(s, script_tok_integer, "VID number was expected") != SCRIPT_OK)
		return SCRIPT_ERROR;

	vid = s->token.num & 0xffff;

	if (script_expect(s, script_tok_integer, "PID number was expected") != SCRIPT_OK)
		return SCRIPT_ERROR;

	pid = s->token.num & 0xffff;

	if (s->flags & SCRIPT_F_DRYRUN)
		return SCRIPT_OK;

	for (retries = 10; ; retries--) {
		fprintf(stderr, "Waiting (%02d sec) for USB hid device %04x:%04x.\r", retries, (int)vid, (int)pid);

		sleep(1);

		if ((*dev = open_device(vid, pid)) != NULL)
			break;

		if (retries > 0)
			continue;

		s->errstr = "Timeout";

		return SCRIPT_ERROR;
	}

	return SCRIPT_OK;
}


static int write_reg_cmd(script_t *s)
{
	long int addr, data, format;
	hid_device *dev = *(hid_device **)s->arg;

	if (script_expect(s, script_tok_integer, "Address value was expected") != SCRIPT_OK)
		return SCRIPT_ERROR;

	addr = s->token.num;

	if (script_expect(s, script_tok_integer, "Data value was expected") != SCRIPT_OK)
		return SCRIPT_ERROR;

	data = s->token.num;

	if (script_expect(s, script_tok_integer, "Format value was expected") != SCRIPT_OK)
		return SCRIPT_ERROR;

	format = s->token.num;

	if (s->flags & SCRIPT_F_DRYRUN)
		return SCRIPT_OK;

	if (!dev) {
		s->errstr = "Device not available";
		return SCRIPT_ERROR;
	}

	if (sdp_writeRegister(dev, addr, format, data) == SCRIPT_OK)
		return SCRIPT_OK;

	s->errstr = "Command failed";

	return SCRIPT_ERROR;
}


static int jump_addr_cmd(script_t *s)
{
	long int addr;
	hid_device *dev = *(hid_device **)s->arg;

	if (script_expect(s, script_tok_integer, "Address value was expected") != SCRIPT_OK)
		return SCRIPT_ERROR;

	addr = s->token.num;

	if (s->flags & SCRIPT_F_DRYRUN)
		return SCRIPT_OK;

	if (!dev) {
		s->errstr = "Device not available";
		return SCRIPT_ERROR;
	}

	if (sdp_jmpAddr(dev, addr) == SCRIPT_OK)
		return SCRIPT_OK;

	s->errstr = "Command failed";

	return SCRIPT_ERROR;
}


static int err_status_cmd(script_t *s)
{
	hid_device *dev = *(hid_device **)s->arg;

	if (s->flags & SCRIPT_F_DRYRUN)
		return SCRIPT_OK;

	if (!dev) {
		s->errstr = "Device not available";
		return SCRIPT_ERROR;
	}

	if (sdp_errStatus(dev) == SCRIPT_OK)
		return SCRIPT_OK;

	s->errstr = "Command failed";

	return SCRIPT_ERROR;
}


static int write_file_cmd(script_t *s)
{
	int type, res;
	script_blob_t str;
	script_blob_t blob = SCRIPT_BLOB_EMPTY;
	long int offset = 0, addr = 0, size = 0;
	hid_device *dev = *( hid_device **)s->arg;


	if (!(s->next.str.end - s->next.str.ptr == 1 && (*s->next.str.ptr == 'F' || *s->next.str.ptr == 'S'))) {
		s->errstr = "Type F or S expected";
		return SCRIPT_ERROR;
	}

	if (script_expect(s, script_tok_identifier, "Literal F or S expected") != SCRIPT_OK)
		return SCRIPT_ERROR;

	type = *s->token.str.ptr;

	if (script_expect(s, script_tok_string, "String in quotes was expected") != SCRIPT_OK)
		return SCRIPT_ERROR;

	str = s->token.str;

	if (script_expect_opt(s, script_tok_integer, "Optional <offset> value was expected") == SCRIPT_OK)
		offset = s->token.num;

	if (s->errstr)
		return SCRIPT_ERROR;

	if (script_expect_opt(s, script_tok_integer, "Optional <address> value was expected") == SCRIPT_OK)
		addr = s->token.num;

	if (s->errstr)
		return SCRIPT_ERROR;

	if (script_expect_opt(s, script_tok_integer, "Optional <size> value was expected") == SCRIPT_OK)
		size = s->token.num;

	if (s->errstr)
		return SCRIPT_ERROR;

	if (get_buffer(s, type, str, &blob) < 0)
		return SCRIPT_ERROR;

	if (s->flags & SCRIPT_F_DRYRUN)
		return SCRIPT_OK;

	if (size) {
		size = MIN(size, blob.end - blob.ptr);
	}
	else {
		size = blob.end - blob.ptr;
	}

	res = SCRIPT_ERROR;

	if (dev)
		res = sdp_writeFile(dev, addr, blob.ptr + offset, size);

	close_buffer(type, &blob);

	if (res == SCRIPT_OK)
		return SCRIPT_OK;

	s->errstr = "Device not available";

	return SCRIPT_ERROR;
}


static int not_implemented_cmd(script_t *s)
{
	s->errstr = "This function is not yet implemented.";

	return SCRIPT_ERROR;
}


static int load_image_cmd(script_t *s)
{
	int res;
	script_blob_t str;
	script_blob_t blob = SCRIPT_BLOB_EMPTY;
	hid_device *dev = *( hid_device **)s->arg;

	if (script_expect(s, script_tok_string, "String in quotes was expected") != SCRIPT_OK)
		return SCRIPT_ERROR;

	str = s->token.str;

	if (get_buffer(s, 'F', str, &blob) < 0)
		return SCRIPT_ERROR;

	if (s->flags & SCRIPT_F_DRYRUN)
		return SCRIPT_OK;

	res = SCRIPT_ERROR;

	if (dev)
		res = mcuboot_loadImage(dev, blob.ptr, blob.end - blob.ptr);

	close_buffer('F', &blob);

	if (res == SCRIPT_OK)
		return SCRIPT_OK;

	s->errstr = "Device not available";

	return SCRIPT_ERROR;
}


static int get_property_cmd(script_t *s)
{
	hid_device *dev = *(hid_device **)s->arg;

	if (s->flags & SCRIPT_F_DRYRUN)
		return SCRIPT_OK;

	if (!dev) {
		s->errstr = "Device not available";
		return SCRIPT_ERROR;
	}

	if (mcuboot_getProperty(dev, 1) == SCRIPT_OK)
		return SCRIPT_OK;

	s->errstr = "Command failed";

	return SCRIPT_ERROR;
}


/*
 * NOTE: because binary search is used, function names
 * must be sorted in lexical order, use upper-case,
 * the list must be terminated with a NULL element.
 */
static const script_funct_t funcs[] = {
	{ "DCD_WRITE", not_implemented_cmd },
	{ "ERROR_STATUS", err_status_cmd },
	{ "GET_PROPERTY", get_property_cmd },
	{ "JUMP_ADDRESS", jump_addr_cmd },
	{ "LOAD_IMAGE", load_image_cmd },
	{ "PROMPT", not_implemented_cmd },
	{ "REBOOT", not_implemented_cmd },
	{ "WAIT", wait_cmd },
	{ "WRITE_FILE", write_file_cmd },
	{ "WRITE_REGISTER", write_reg_cmd },
	{ NULL, NULL }
};


int main(int argc, char *argv[])
{
	int res = -1;
	script_t script;
	hid_device *dev = NULL;

	if (argc < 2)
		return usage(argv[0]);

	if (script_load(&script, argv[1]) != SCRIPT_OK)
		return -1;

	script_set_funcs(&script, funcs, &dev);

	/* First interpret script in dry-run mode to check syntax
	 * and if files specified in the script exist and check
	 * if they are readable
	 */
	if (script_parse(&script, SCRIPT_F_DRYRUN) != SCRIPT_OK) {
		script_close(&script);
		fprintf(stderr, "Exiting due to error in script file.\n");
		return -1;
	}

	if (!hid_init()) {
		/* Interpret script, now things like memalloc, hid device comm. may fail */
		res = script_parse(&script, SCRIPT_F_SHOWLINES);
		hid_close(dev);
		hid_exit();
	}

	script_close(&script);

	return res;
}
