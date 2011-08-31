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
 *         Utility Functions
 * \author Gidon Ernst <gidon.ernst@googlemail.com>
 * \author Thomas Mair <mair.thomas@gmx.net>
 */

#ifndef __UTIL_IPV6_H__
#define __UTIL_IPV6_H__

#include <stdint.h>
#include <stdint.h>

#define ip4addr_format "%d.%d.%d.%d"
#define ip4addr_to_4dec(addr) \
    ((u8_t *)addr)[0],  \
    ((u8_t *)addr)[1],  \
    ((u8_t *)addr)[2],  \
    ((u8_t *)addr)[3]

#define ip6addr_format "%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x"
#define ip6addr_to_16hex(addr)\
    ((u8_t *)addr)[0],  \
    ((u8_t *)addr)[1],  \
    ((u8_t *)addr)[2],  \
    ((u8_t *)addr)[3],  \
    ((u8_t *)addr)[4],  \
    ((u8_t *)addr)[5],  \
    ((u8_t *)addr)[6],  \
    ((u8_t *)addr)[7],  \
    ((u8_t *)addr)[8],  \
    ((u8_t *)addr)[9],  \
    ((u8_t *)addr)[10], \
    ((u8_t *)addr)[11], \
    ((u8_t *)addr)[12], \
    ((u8_t *)addr)[13], \
    ((u8_t *)addr)[14], \
    ((u8_t *)addr)[15]

#if UIP_CONF_IPV6
#define IP_FORMAT ip6addr_format
#define IP_PARAMS(a) ip6addr_to_16hex(a)
#define IP_LOCAL_ADDRESS(name) uip_ds6_addr_t *(name) = uip_ds6_get_link_local(-1)
#else
#define IP_FORMAT ip4addr_format
#define IP_PARAMS(a) ip4addr_to_4dec(a)
#define IP_LOCAL_ADDRESS(name) uip_ipaddr_t *(name) = &uip_hostaddr
#endif

#ifndef min
#define min(a,b) ((a)<(b) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) ((a)>(b) ? (a) : (b))
#endif

#define UDP_IP_BUF ((struct uip_udpip_hdr *)&uip_buf[UIP_LLH_LEN])
#define TCP_IP_BUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_REMOTE_ADDR &UDP_IP_BUF->srcipaddr
#define UDP_REMOTE_PORT  UDP_IP_BUF->srcport

#define TCP_REMOTE_ADDR &TCP_IP_BUF->srcipaddr
#define TCP_REMOTE_PORT  TCP_IP_BUF->srcport

#if UIP_CONF_IPV6
void util_print_ip6addresses();
void util_set_global_address(u16_t );
#endif

u16_t fmt_uint(char *dest, u16_t i);
u16_t fmt_int(char *dest, int i);
u16_t fmt_str(char *dest, const char *src);
u16_t fmt_xuint(char *dest, u16_t i);

#endif /*__UTIL_IPV6_H__*/ 
