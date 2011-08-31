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

#include "contiki.h"
#include "contiki-net.h"
#include "util.h"

#include <stdio.h>

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"
//#define PRINTF(...) 

#if UIP_CONF_IPV6
void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
#if UIP_CONF_ROUTER
void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
}
#endif /* UIP_CONF_ROUTER */
//void util_print_ip6addresses() {
//    int i;
//    uip_netif_state state;
//
//    for(i = 0; i < UIP_CONF_NETIF_MAX_ADDRESSES; i++) {
//        state = uip_netif_physical_if.addresses[i].state;
//            PRINTF(IP_FORMAT, IP_PARAMS(&uip_netif_physical_if.addresses[i].ipaddr));
//            PRINTF("\n");
//    }
//
//}

//void util_set_global_address(u16_t prefix) {
//  uip_ipaddr_t ipaddr;
//
//  uip_ip6addr(&ipaddr, prefix, 0, 0, 0, 0, 0, 0, 0);
//  uip_netif_addr_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0, AUTOCONF);
//}
#endif

char fmt_tohex(char c) {
    return c>=10?c-10+'a':c+'0';
}

u16_t fmt_uint(char *dest, u16_t i) {
    u16_t len,tmp,len2;
    /* first count the number of bytes needed */
    for (len=1, tmp=i; tmp>9; ++len) tmp/=10;
    if (dest)
        for (tmp=i, dest+=len, len2=len+1; --len2; tmp/=10)
            *--dest = (tmp%10)+'0';
    return len;
}

u16_t fmt_int(char *dest, int i) {
    u8_t neg = i<0 ? 1 : 0;
    if(neg) {
        dest[0] = '-';
        i = -i;
    }
    return fmt_uint(dest+neg, i)+neg;
}

u16_t fmt_xuint(char *dest, u16_t i) {
    u16_t len,tmp;
    /* first count the number of bytes needed */
    for (len=1, tmp=i; tmp>15; ++len) tmp>>=4;
    if (dest)
        for (tmp=i, dest+=len; ; ) {
            *--dest = fmt_tohex(tmp&15);
            if (!(tmp>>=4)) break;
        }
    return len;
}


u16_t fmt_str(char *dest, const char *src) {
    char *res = dest;
    while ((*dest++ = *src++));
    return dest-res-1;
}
