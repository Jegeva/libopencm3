/* This provides unification of code over STM32 subfamilies */

/*
 * This file is part of the libopencm3 project.
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
#       include <libopencm3/air32/f0/crs.h>
#elif defined(AIR32L0)
#       include <libopencm3/air32/l0/crs.h>
#elif defined(AIR32L4)
#       include <libopencm3/air32/l4/crs.h>
#elif defined(AIR32G4)
#       include <libopencm3/air32/g4/crs.h>
#else
#       error "air32 family not defined or not supported for this peripheral"
#endif
