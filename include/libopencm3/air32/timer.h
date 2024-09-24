/* This provides unification of code over STM32 subfamilies */

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Piotr Esden-Tempski <piotr@esden.net>
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

#include <libopencm3/cm3/common.h>
#include <libopencm3/air32/memorymap.h>

#if defined(AIR32F0)
#       include <libopencm3/air32/f0/timer.h>
#elif defined(AIR32F1)
#       include <libopencm3/air32/f1/timer.h>
#elif defined(AIR32F2)
#       include <libopencm3/air32/f2/timer.h>
#elif defined(AIR32F3)
#       include <libopencm3/air32/f3/timer.h>
#elif defined(AIR32F4)
#       include <libopencm3/air32/f4/timer.h>
#elif defined(AIR32F7)
#       include <libopencm3/air32/f7/timer.h>
#elif defined(AIR32L0)
#       include <libopencm3/air32/l0/timer.h>
#elif defined(AIR32L1)
#       include <libopencm3/air32/l1/timer.h>
#elif defined(AIR32L4)
#       include <libopencm3/air32/l4/timer.h>
#elif defined(AIR32G0)
#       include <libopencm3/air32/g0/timer.h>
#elif defined(AIR32G4)
#       include <libopencm3/air32/g4/timer.h>
#elif defined(AIR32H7)
#       include <libopencm3/air32/h7/timer.h>
#else
#       error "air32 family not defined."
#endif
