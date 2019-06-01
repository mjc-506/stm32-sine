/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define CST_DIGITS 15
#include "my_fp.h"
#include "my_math.h"
#include "foc.h"
#include "sine_core.h"

#define SQRT3           FP_FROMFLT(1.732050807568877293527446315059)

static const s32fp sqrt3inv1 = FP_FROMFLT(0.57735026919); //1/sqrt(3)
static const s32fp sqrt3inv2 = 2*sqrt3inv1; //2/sqrt(2)
static const s32fp sqrt3ov2 = (SQRT3 / 2);
static const s32fp zeroOffset = FP_FROMINT(1);
static int32_t minPulse = 1000;
static int32_t maxPulse = FP_FROMFLT(2) - 1000;

s32fp FOC::id;
s32fp FOC::iq;
int32_t FOC::DutyCycles[3];

/** @brief Transform current to rotor system using Clarke and Park transformation
  * @post flux producing (id) and torque producing (iq) current are written
  *       to FOC::id and FOC::iq
  */
void FOC::ParkClarke(s32fp il1, s32fp il2, uint16_t angle)
{
   s32fp sin = SineCore::Sine(angle);
   s32fp cos = SineCore::Cosine(angle);
   //Clarke transformation
   s32fp ia = il1;
   s32fp ib = FP_MUL(sqrt3inv1, il1) + FP_MUL(sqrt3inv2, il2);
   //Park transformation
   s32fp idl = FP_MUL(cos, ia) + FP_MUL(sin, ib);
   s32fp iql = FP_MUL(cos, ib) - FP_MUL(sin, ia);

   id = IIRFILTER(id, idl, 2);
   iq = IIRFILTER(iq, iql, 2);
}

void FOC::InvParkClarke(s32fp id, s32fp iq, uint16_t angle)
{
   s32fp sin = SineCore::Sine(angle);
   s32fp cos = SineCore::Cosine(angle);

   //Inverse Park transformation
   s32fp ia = FP_MUL(cos, id) - FP_MUL(sin, iq);
   s32fp ib = FP_MUL(cos, iq) + FP_MUL(sin, id);
   //Inverse Clarke transformation
   DutyCycles[0] = ia;
   DutyCycles[1] = FP_MUL(-FP_FROMFLT(0.5), ia) + FP_MUL(sqrt3ov2, ib);
   DutyCycles[2] = FP_MUL(-FP_FROMFLT(0.5), ia) - FP_MUL(sqrt3ov2, ib);

   int32_t offset = SineCore::CalcSVPWMOffset(DutyCycles[0], DutyCycles[1], DutyCycles[2]);

   for (int i = 0; i < 3; i++)
   {
      /* 4. subtract it from all 3 phases -> no difference in phase-to-phase voltage */
      DutyCycles[i] -= offset;
      /* Shift above 0 */
      DutyCycles[i] += zeroOffset;
      /* Short pulse supression */
      if (DutyCycles[i] < minPulse)
      {
         DutyCycles[i] = 0U;
      }
      else if (DutyCycles[i] > maxPulse)
      {
         DutyCycles[i] = FP_FROMINT(1);
      }
   }
}
