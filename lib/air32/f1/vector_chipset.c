/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/scb.h>

static void pre_main(void)
{
  /* // bullshit secret registers knock
  *(volatile uint32_t *)(0x400210F0) = 1;//开启sys_cfg门控
  *(volatile uint32_t *)(0x40016C00) = 0xa7d93a86;//解一、二、三级锁
  *(volatile uint32_t *)(0x40016C00) = 0xab12dfcd;
  *(volatile uint32_t *)(0x40016C00) = 0xcded3526;
  *(volatile uint32_t *)(0x4002228C) = 0xa5a5a5a5;//QSPI解锁
  *(volatile uint32_t *)(0x40022214) |= 1;
  *(volatile uint32_t *)(0x40016C18) = 0x200183FF;
  */	
}
