/*
 * Phoenix-RTOS
 * 
 * Phoenix server
 *
 * BSP2 message dispatcher
 *
 * Copyright 2004 Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "errors.h"
#include "serial.h"
#include "msg.h"
#include "dispatch.h"
#include "phfs.h"
#include "msg_udp.h"

int (*msg_send)(int fd, msg_t *msg);
int (*msg_recv)(int fd, msg_t *msg, int *state);

static char *concat(char *s1, char *s2)
{
	char *result = malloc(strlen(s1) + strlen(s2) + 1);
	strcpy(result, s1);
	strcat(result, s2);
	return result;

}

static int connect_pipes(const char *dev_in, const char *dev_out, int *fd_in, int *fd_out) 
{
		if ((*fd_in = open(dev_in, O_RDONLY)) < 0) {
			fprintf(stderr, "[%d] dispatch: Can't open pipe '%s'\n", getpid(), dev_in);	
			return ERR_DISPATCH_IO;
		}

		if ((*fd_out = open(dev_out, O_WRONLY)) < 0) {
			fprintf(stderr, "[%d] dispatch: Can't open pipe '%s'\n", getpid(), dev_out);		
			return ERR_DISPATCH_IO;
		}
	return 0;
}

/* Function reads and dispatches messages */
int dispatch(char *dev_addr, dmode_t mode, unsigned int speed_port, char *sysdir)
{
	int fd = -1;
	int fd_out = -1;
	int retries = 128;
	msg_t msg;
	int state, err;
	char *dev_in = 0; 
	char *dev_out = 0;
	
	printf("[%d] dispatch: Starting message dispatcher on %s\n", getpid(), dev_addr);
	if (mode == SERIAL) {
		if ((fd = serial_open(dev_addr, speed_port)) < 0) {
			fprintf(stderr, "[%d] dispatch: Can't open serial port '%s'\n", getpid(), dev_addr);
			return ERR_DISPATCH_IO;
		}
		msg_send = msg_serial_send;
		msg_recv = msg_serial_recv;
	}
	else if (mode == UDP) {
		if ((fd = udp_open(dev_addr, speed_port)) < 0) {
			fprintf(stderr, "[%d] dispatch: Can't open connection at '%s:%hu'\n", getpid(), dev_addr, speed_port);
			return ERR_DISPATCH_IO;
		}
		msg_send = msg_udp_send;
		msg_recv = msg_udp_recv;
	}
	else if (mode == PIPE) {
		dev_in = concat(dev_addr, ".out"); // because output from quemu is our input
		dev_out = concat(dev_addr, ".in"); // same logic

		if (connect_pipes(dev_in, dev_out, &fd, &fd_out)) { 
			free(dev_in);
			free(dev_out);
			return ERR_DISPATCH_IO;
		}
		msg_send = msg_serial_send;
		msg_recv = msg_serial_recv;
	}

	for (state = MSGRECV_DESYN;;) {
		if (msg_recv(fd, &msg, &state) < 0) {
			fprintf(stderr, "[%d] dispatch: Message receiving error on %s, state=%d!\n", getpid(), dev_addr, state);
			// if this is pipe - try to reconnect - it's because qemu closes pipe 
			if (mode == PIPE && --retries) {
				usleep(10000);
				(void) connect_pipes(dev_in, dev_out, &fd, &fd_out);
			}
			continue;
		}
		
		if ((err = phfs_handlemsg((mode == PIPE ? fd_out : fd), &msg, sysdir)))
			continue;

		switch (msg_gettype(&msg)) {
		case MSG_ERR:
			msg_settype(&msg, MSG_ERR);
			msg_setlen(&msg, MSG_MAXLEN);
			msg_send((mode == PIPE ? fd_out : fd), &msg);
			break;
		}
			
	}
	
	if (mode == PIPE) {
		free(dev_in);
		free(dev_out);
	}
	
	return 0;
}
