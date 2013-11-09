/*
 * Copyright (c) 2009, Siemens AG.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the company nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COMPANY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COMPANY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         TFTP Server
 * \author Gidon Ernst <gidon.ernst@googlemail.com>
 * \author Thomas Mair <mair.thomas@gmx.net>
 */

#ifndef __TFTPD_H__
#define __TFTPD_H__

#ifndef PORT
#define PORT 69
#endif

#ifndef MAX_CLIENTS
#define MAX_CLIENTS 4
#endif

#define TFTP_MAX_BLKSIZE (UIP_APPDATA_SIZE-4<512 ? UIP_APPDATA_SIZE-4 : 512)
#define TFTP_TIMEOUT 10
#define TFTP_MAXTRIES 3

PROCESS_NAME(tftpd_process);

enum {
    TFTP_NONE,
    TFTP_RRQ,
    TFTP_WRQ,
    TFTP_DATA,
    TFTP_ACK,
    TFTP_ERROR,
    TFTP_OACK,
};

#define EUNDEF    0  /* not defined */
#define ENOTFOUND 1  /* file not found */
#define EACCESS   2  /* access violation */
#define ENOSPACE  3  /* disk full or allocation exceeded */
#define EBADOP    4  /* illegal TFTP operation */
#define EBADID    5  /* unknown transfer ID */
#define EEXISTS   6  /* file already exists */
#define ENOUSER   7  /* no such user */
#define EOPTNEG   8  /* option negotiation failed */

/* flags to specify, which options should be acknowlegded */
#define OACK_BLKSIZE 0x01
#define OACK_TIMEOUT 0x02
#define OACK_EXEC    0x10
#define OACK_ERROR   0x80

struct tftp_header {
    u16_t op;
    union {
        u16_t block_nr;
        u16_t error_code;
        char filename[1];
        char options[1];
    };
    union {
        char data[1];
        char error_msg[1];
    };
} __attribute__((packed));
 /* packed: do not put empty space in the structure for compiler optimization */

struct tftp_config {
    u16_t blksize;
    u16_t timeout;
    u8_t exec;
    u8_t to_ack;
    char filename[16];
};

typedef struct tftp_header tftp_header;

#endif /* __TFTPD_H__ */
