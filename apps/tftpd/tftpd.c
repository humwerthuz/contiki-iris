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

#include "contiki.h"
#include "contiki-net.h"

#include "cfs.h"
#include "cfs-coffee.h"

#if WITH_EXEC
#include "loader/elfloader.h"
#endif

#include <string.h>
#include <stdlib.h>

#include "util.h"
#include "tftpd.h"

PROCESS(tftpd_process, "tftpd");

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static struct uip_udp_conn *server_conn, *client_conn;
static struct tftp_config config;

static void init_config() {
    config.blksize = TFTP_MAX_BLKSIZE;
    config.timeout = TFTP_TIMEOUT;
    config.exec = 1;
    config.to_ack = 0;
    memset(config.filename, 0, sizeof(config.filename));
}

static void send_packet(void *buf, int len) {
    uip_udp_packet_send(client_conn, buf, len);
}

#define RECV_PACKET_TIMEOUT(h,t) \
    do { \
        etimer_set(&t, CLOCK_CONF_SECOND*config.timeout); \
        PROCESS_WAIT_EVENT_UNTIL((ev == tcpip_event && uip_newdata()) || \
                                 (ev == PROCESS_EVENT_TIMER && etimer_expired(&t))); \
        etimer_stop(&t); \
        h = (tftp_header *)uip_appdata; \
    } while(0);


#define RECV_PACKET(h) \
    do { \
        PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event && uip_newdata()); \
        h = (tftp_header *)uip_appdata; \
    } while(0);

static char *to_str(u16_t i) {
    static char x[6];
    fmt_uint(x, i);
    return x;
}

static int parse_opt(char *src, char **opt, char **val) {
    int len = 0;
    *opt = src+len;  len += strlen(*opt)+1;
    *val = src+len;  len += strlen(*val)+1;
    return len;
}

static int format_opt(char *dest, char *opt, char *val) {
    int len = 0;
    strcpy(dest+len, opt);  len += strlen(opt)+1;
    strcpy(dest+len, val);  len += strlen(val)+1;
    return len;
}

static void parse_opts(char *src, int len) {
    char *opt, *val;
    int n;
    config.to_ack = 0;
    while(len>0 && (n = parse_opt(src, &opt, &val))>2) {
        if(strcmp(opt, "blksize")==0) {
            config.blksize = min(atoi(val), TFTP_MAX_BLKSIZE);
            config.to_ack |= OACK_BLKSIZE;
        } else if(strcmp(opt, "timeout")==0) {
            config.timeout = atoi(val);
            config.to_ack |= OACK_TIMEOUT;
        } else if(strcmp(opt, "exec")==0 && val[0]=='1') {
            config.exec = 1;
            config.to_ack |= OACK_EXEC;
        }
        len -= n;
        src += n;
    }

    PRINTF("blksize: %d\ntimeout: %d\n", config.blksize, config.timeout);
}

static int format_opts(char *dest) {
    int len = 0;
    if(config.to_ack & OACK_BLKSIZE)
        len += format_opt(dest+len, "blksize", to_str(config.blksize));
    if(config.to_ack & OACK_TIMEOUT)
        len += format_opt(dest+len, "timeout", to_str(config.timeout));
    if(config.to_ack & OACK_EXEC)
        len += format_opt(dest+len, "exec", "1");
    return len;
}

static void send_ack(int block) {
    tftp_header *h = (tftp_header *)uip_appdata;
    h->op = uip_htons(TFTP_ACK);
    h->block_nr = uip_htons(block);
    send_packet(h, 4);
}

static char send_oack() {
    tftp_header *h = (tftp_header *)uip_appdata;
    if(config.to_ack) {
        h->op = uip_htons(TFTP_OACK);
        send_packet(h, 2+format_opts(h->options));
        return 1;
    }
    return 0;
}

static int send_data(int fd, int block){
    int len;
    tftp_header *h = (tftp_header *)uip_appdata;
    h->op = uip_htons(TFTP_DATA);
    h->block_nr = uip_htons(block);
    cfs_seek(fd, (block-1)*config.blksize, CFS_SEEK_SET);
    len = cfs_read(fd, h->data, config.blksize);
    if(len>=0) send_packet(h, 4+len);
    PRINTF("> block %d (len %d)\n", block, len);
    return len;
}

static void send_error(int code, const char *msg) {
    tftp_header *h = (tftp_header *)uip_appdata;
    h->op = uip_htons(TFTP_ERROR);
    h->error_code = uip_htons(code);
    strcpy(h->error_msg, msg);
    send_packet(h, 4+strlen(h->error_msg)+1);
    PRINTF("> error %d\n", code);
}

static int recv_data(int fd, int block) {
    int len;
    tftp_header *h = (tftp_header *)uip_appdata;
    len = cfs_write(fd, h->data, uip_datalen()-4);
    PRINTF("< block %d (len %d)\n", block, len);
    return len;
}

static void connect_back() {
    client_conn = udp_new(UDP_REMOTE_ADDR, UDP_REMOTE_PORT, NULL);
    PRINTF("connection from " IP_FORMAT ": %d\n", IP_PARAMS(UDP_REMOTE_ADDR), uip_ntohs(UDP_REMOTE_PORT));
}

static void setup_server() {
    IP_LOCAL_ADDRESS(a);
    PRINTF("ip address: " IP_FORMAT "\n", IP_PARAMS(a));

    server_conn = udp_new(NULL, uip_htons(0), NULL);
    udp_bind(server_conn, uip_htons(PORT));
    PRINTF("listening on port %d. max blksize: %d\n", PORT, TFTP_MAX_BLKSIZE);
}

#if WITH_EXEC
static int exec_file(char *name, char **error)
{
  int fd;
  int ret;
  *error = 0;

  /* Kill any old processes. */
  if(elfloader_autostart_processes != NULL) {
    autostart_exit(elfloader_autostart_processes);
  }

  fd = cfs_open(name, CFS_READ | CFS_WRITE);
  if(fd < 0) {
#if DEBUG
      error = "Could not open file";
#endif
  } else {
    ret = elfloader_load(fd);
    cfs_close(fd);

#if DEBUG
    switch(ret) {
    case ELFLOADER_OK:
      *error = "OK";
      break;
    case ELFLOADER_BAD_ELF_HEADER:
      *error = "Bad ELF header";
      break;
    case ELFLOADER_NO_SYMTAB:
      *error = "No symbol table";
      break;
    case ELFLOADER_NO_STRTAB:
      *error = "No string table";
      break;
    case ELFLOADER_NO_TEXT:
      *error = "No text segment";
      break;
    case ELFLOADER_SYMBOL_NOT_FOUND:
      *error = "Symbol not found";
      // symbol = elfloader_unknown;
      PRINTF("Symbol not found: %s\n", elfloader_unknown);
      break;
    case ELFLOADER_SEGMENT_NOT_FOUND:
      *error = "Segment not found";
      // symbol = elfloader_unknown;
      PRINTF("Segment not found: %s\n", elfloader_unknown);
      break;
    case ELFLOADER_NO_STARTPOINT:
      *error = "No starting point";
      break;
    default:
      *error = "Unknown return code from the ELF loader (internal bug)";
      break;
    }
#endif

    if(ret == ELFLOADER_OK) {
      autostart_start(elfloader_autostart_processes);
      return 0;
    }
  }

  return 1;
}
#endif

PROCESS_THREAD(tftpd_process, ev, data)
{
    static struct etimer t;
    static tftp_header *h;
    static int len, block, ack;
    static int tries;
    static int fd = -1;
#if WITH_EXEC
    static char *elf_err;
#endif

    PROCESS_BEGIN();

    etimer_set(&t, CLOCK_CONF_SECOND*3);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

    setup_server();
#if WITH_EXEC
    elfloader_init();
#endif

	print_local_addresses();

    while(1) {
        /* connection from client */
        RECV_PACKET(h);
        len = 0;
        init_config();

        if(h->op == uip_htons(TFTP_RRQ)) {
            connect_back();
            PRINTF("< rrq for %s\n", h->filename);
            len += strlen(h->filename)+1;

            if(strcmp("octet", h->filename+len)) {
                send_error(EUNDEF, "only octet mode supported");
                goto close_connection;
            }
            len += strlen(h->filename+len)+1; /* skip mode */

            parse_opts(h->options+len, uip_datalen()-len-2);

            if(config.to_ack & OACK_ERROR) {
                send_error(EOPTNEG, "");
                goto close_connection;
            } 

            fd = cfs_open(h->filename, CFS_READ);
            if(fd<0) {
                send_error(ENOTFOUND, "");
                goto close_connection;
            }

            block = 0; ack = 0;
            tries = TFTP_MAXTRIES;

            PRINTF("starting transfer...\n");

            for(;;) {
                if(send_oack())
                    len = config.blksize; /* XXX hack to prevent loop exit*/
                else
                    len = send_data(fd, block+1);

                if(len<0) {
                    send_error(EUNDEF, "read failed");
                    goto close_file;
                }

                RECV_PACKET_TIMEOUT(h,t);

                if(ev == PROCESS_EVENT_TIMER) {
                    PRINTF("ack timed out, tries left: %d\n", tries);
                    if(--tries<=0) goto close_file;
                    continue;
                }

                if(h->op != uip_htons(TFTP_ACK)) {
                    send_error(EBADOP, "");
                    goto close_file;
                }

                config.to_ack = 0;
                tries = TFTP_MAXTRIES;
                ack = uip_htons(h->block_nr);
                if(ack == block+1) block++;

                if(len < config.blksize && ack == block) goto done;
            }
        }
        
        else if(h->op == uip_htons(TFTP_WRQ)) {
            connect_back();
            PRINTF("< wrq for %s\n", h->filename);
            len += strlen(h->filename)+1;
            strncpy(config.filename, h->filename, sizeof(config.filename)-1);

            if(strcmp("octet", h->filename+strlen(h->filename)+1)) {
                send_error(EUNDEF, "only octet mode supported");
                goto close_connection;
            }
            len += strlen(h->filename+len)+1; /* skip mode */

            parse_opts(h->options+len, uip_datalen()-len-2);

            if(config.to_ack & OACK_ERROR) {
                send_error(EOPTNEG, "");
                goto close_connection;
            } 

            cfs_remove(h->filename);
            fd = cfs_open(h->filename, CFS_WRITE);
            if(fd<0) {
                send_error(EACCESS, "");
                goto close_connection;
            }

            block = 0; ack = 0;
            tries = TFTP_MAXTRIES;

            PRINTF("starting transfer...\n");

            if(!send_oack())
                send_ack(block);

            for(;;) {
                RECV_PACKET_TIMEOUT(h,t);

                if(ev == PROCESS_EVENT_TIMER) {
                    PRINTF("data timed out, tries left: %d\n", tries);
                    if(--tries<=0) goto close_file;
                    len = config.blksize; /* XXX hack to prevent loop exit*/
                    goto resend_ack;
                }

                if(h->op != uip_htons(TFTP_DATA)) {
                    send_error(EBADOP, "");
                    goto close_file;
                }

                config.to_ack = 0;
                tries = TFTP_MAXTRIES;
                ack = uip_htons(h->block_nr);
                if(ack != block+1) continue;
                /* else */ block++;

                len = recv_data(fd, block);
                if(len<0) {
                    send_error(EUNDEF, "write failed");
                    goto close_file;
                }

#if WITH_EXEC
                if(len < config.blksize) {
                    if(config.exec) {
                        if(exec_file(config.filename, &elf_err) != 0) {
                            send_error(EUNDEF, elf_err);
                            goto close_file;
                        }
                    }
                }
#endif

resend_ack:

                if(!send_oack())
                    send_ack(block);

                if(len < config.blksize) goto done;
            }
        }

done:
            PRINTF("done.\n");

close_file:
        if(fd>=0)
            cfs_close(fd);
        fd = -1;

close_connection:
        if(client_conn)
            uip_udp_remove(client_conn);
        client_conn = 0;

        PRINTF("connection closed.\n");
    } 
    PROCESS_END();
}


