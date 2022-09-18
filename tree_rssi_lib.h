/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
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


#ifndef TREE_LIB_H
#define TREE_LIB_H

////////////////////////////
////////  LIBRARIES  ///////
////////////////////////////
#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include <stdio.h>
////////////////////////////
////////  DEFINE  //////////
////////////////////////////

////////////////////////////
////////  STRUCT  //////////
////////////////////////////

struct beacon
{
  linkaddr_t id;   // Node id
  uint16_t rssi_p; // Accumulated parent RSSI
};

struct node
{
  linkaddr_t preferred_parent;   // Node id
  uint16_t rssi_p; // Accumulated parent RSSI
};


struct preferred_parent
{
  struct preferred_parent *next;
  linkaddr_t id;   // Node id
  uint16_t rssi_a; // Accumulated RSSI
  //struct ctimer ctimer;
};

struct list_unicast_msg
{
  struct list *next;
  linkaddr_t id;   // Node id
};

////////////////////////////
////////  DEF STRUCT  //////
////////////////////////////


////////////////////////////
////////  FUNCION //////////
////////////////////////////
void fill_beacon(struct beacon *b, linkaddr_t id, uint16_t rssi_p);

#endif /* TREE_LIB_H */t id, signed int rssi_c);
