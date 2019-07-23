/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2015 Johannes Huebner <dev@johanneshuebner.com>
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
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include "pwmgeneration.h"
#include "hwdefs.h"
#include "params.h"
#include "inc_encoder.h"
#include "sine_core.h"
#include "fu.h"
#include "errormessage.h"
#include "digio.h"
#include "anain.h"
#include "my_math.h"
#include "foc.h"

#define SHIFT_180DEG (uint16_t)32768
#define SHIFT_90DEG  (uint16_t)16384
#define FRQ_TO_ANGLE(frq) FP_TOINT((frq << SineCore::BITS) / pwmfrq)
#define DIGIT_TO_DEGREE(a) FP_FROMINT(angle) / (65536 / 360)


static uint8_t  pwmdigits;
static uint16_t pwmfrq;
static volatile uint16_t angle;
static s32fp ampnom;
static uint16_t slipIncr;
static s32fp fslip;
static s32fp frq;
static uint8_t shiftForTimer;
static int opmode;
static bool tripped;
static s32fp ilofs[2];
static uint16_t execTicks = 0;
static s32fp idref = 0, iqref = 0;
static int curdkp, curqkp, curdki, curqki;
static s32fp sumq = 0, sumd = 0;
static int initwait = 0;

int32_t PwmGeneration::PiController(s32fp refVal, s32fp curVal, s32fp& sum, int kp, int ki)
{
   s32fp err = refVal - curVal;

   sum += err;
   sum = MAX(FP_FROMINT(-1000000), sum);
   sum = MIN(FP_FROMINT(1000000), sum);

   return FP_TOINT(err * kp + (sum / pwmfrq) * ki);
}

void PwmGeneration::Run()
{
   if (opmode == MOD_MANUAL || opmode == MOD_RUN || opmode == MOD_SINE)
   {
      int dir = Param::GetInt(Param::dir);
      uint16_t dc[3];
      s32fp id, iq;

      Encoder::UpdateRotorAngle(dir);

      if (opmode == MOD_SINE)
         CalcNextAngleConstant(dir);
      else if (Encoder::IsSyncMode())
         CalcNextAngleSync(dir);
      else
         CalcNextAngleAsync(dir);

      ProcessCurrents(id, iq);

      s32fp ud = PiController(idref, id, sumd, curdkp, curdki);
      s32fp uq = PiController(iqref, iq, sumq, curqkp, curqki);
      FOC::LimitVoltages(ud, uq);
      FOC::InvParkClarke(ud, uq, angle);

      Param::SetFlt(Param::fstat, frq);
      Param::SetFlt(Param::angle, DIGIT_TO_DEGREE(angle));
      Param::SetInt(Param::ud, ud);
      Param::SetInt(Param::uq, uq);

      /* Shut down PWM on zero voltage request */
      if (0 == iqref || 0 == dir || initwait > 0)
      {
         timer_disable_break_main_output(PWM_TIMER);
         sumd = 0;
         sumq = 0;
      }
      else
      {
         timer_enable_break_main_output(PWM_TIMER);
      }

      for (int i = 0; i < 3; i++)
      {
         dc[i] = FOC::DutyCycles[i] >> shiftForTimer;
         Param::SetInt((Param::PARAM_NUM)(Param::dc1+i), dc[i]);
      }

      if ((Param::GetInt(Param::pinswap) & SWAP_PWM) > 0)
      {
         timer_set_oc_value(PWM_TIMER, TIM_OC1, dc[2]);
         timer_set_oc_value(PWM_TIMER, TIM_OC2, dc[1]);
         timer_set_oc_value(PWM_TIMER, TIM_OC3, dc[0]);
      }
      else
      {
         timer_set_oc_value(PWM_TIMER, TIM_OC1, dc[0]);
         timer_set_oc_value(PWM_TIMER, TIM_OC2, dc[1]);
         timer_set_oc_value(PWM_TIMER, TIM_OC3, dc[2]);
      }
   }
   else if (opmode == MOD_BOOST || opmode == MOD_BUCK)
   {
      s32fp id, iq;
      initwait = 0;
      ProcessCurrents(id, iq);
      Charge();
   }
   else if (opmode == MOD_ACHEAT)
   {
      initwait = 0;
      AcHeat();
   }
}

uint16_t PwmGeneration::GetAngle()
{
   return angle;
}

bool PwmGeneration::Tripped()
{
   return tripped;
}

void PwmGeneration::SetAmpnom(s32fp amp)
{
   ampnom = amp;
}

void PwmGeneration::SetCurrents(s32fp id, s32fp iq)
{
   idref = id;
   iqref = iq;
}

void PwmGeneration::SetControllerGains(int dkp, int dki, int qkp, int qki)
{
   curdkp = dkp;
   curdki = dki;
   curqkp = qkp;
   curqki = qki;
}

void PwmGeneration::SetFslip(s32fp _fslip)
{
   slipIncr = FRQ_TO_ANGLE(_fslip);
   fslip = _fslip;
}

void PwmGeneration::SetCurrentOffset(int offset1, int offset2)
{
   ilofs[0] = FP_FROMINT(offset1);
   ilofs[1] = FP_FROMINT(offset2);

   if (CHK_BIPOLAR_OFS(offset1))
   {
      ErrorMessage::Post(ERR_HICUROFS1);
   }
   if (CHK_BIPOLAR_OFS(offset2))
   {
      ErrorMessage::Post(ERR_HICUROFS2);
   }

   SetCurrentLimitThreshold(Param::Get(Param::ocurlim));
}

int PwmGeneration::GetCpuLoad()
{
   //PWM period 2x counter because of center aligned mode
   return (1000 * execTicks) / (2 << pwmdigits);
}

void PwmGeneration::SetOpmode(int _opmode)
{
   opmode = _opmode;

   if (opmode != MOD_OFF)
   {
      sumd = 0;
      sumq = 0;
      FOC::id = 0;
      FOC::iq = 0;
      PwmInit();
   }

   switch (opmode)
   {
      default:
      case MOD_OFF:
         DisableOutput();
         execTicks = 0;
         break;
      case MOD_ACHEAT:
         DisableOutput();
         timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
         timer_enable_oc_output(PWM_TIMER, TIM_OC2);
         break;
      case MOD_BOOST:
         DisableOutput();
         timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
         break;
      case MOD_BUCK:
         DisableOutput();
         timer_enable_oc_output(PWM_TIMER, TIM_OC2);
         break;
      case MOD_MANUAL:
      case MOD_RUN:
      case MOD_SINE:
         EnableOutput();
         break;
   }
}

extern "C" void tim1_brk_isr(void)
{
   if (!DigIo::Get(Pin::desat_in) && hwRev != HW_REV1)
      ErrorMessage::Post(ERR_DESAT);
   else if (!DigIo::Get(Pin::emcystop_in) && hwRev != HW_REV3)
      ErrorMessage::Post(ERR_EMCYSTOP);
   else if (!DigIo::Get(Pin::mprot_in))
      ErrorMessage::Post(ERR_MPROT);
   else //if (ocur || hwRev == HW_REV1)
      ErrorMessage::Post(ERR_OVERCURRENT);

   timer_disable_irq(PWM_TIMER, TIM_DIER_BIE);
   Param::SetInt(Param::opmode, MOD_OFF);
   DigIo::Set(Pin::err_out);
   tripped = true;
}

extern "C" void pwm_timer_isr(void)
{
   gpio_set(GPIOB, GPIO13);
   int start = timer_get_counter(PWM_TIMER);
   /* Clear interrupt pending flag */
   timer_clear_flag(PWM_TIMER, TIM_SR_UIF);

   PwmGeneration::Run();
   int time = timer_get_counter(PWM_TIMER) - start;

   if (TIM_CR1(PWM_TIMER) & TIM_CR1_DIR_DOWN)
      time = (2 << pwmdigits) - timer_get_counter(PWM_TIMER) - start;

   execTicks = ABS(time);
   gpio_clear(GPIOB, GPIO13);
}

/**
* Enable timer PWM output
*/
void PwmGeneration::EnableOutput()
{
   timer_enable_oc_output(PWM_TIMER, TIM_OC1);
   timer_enable_oc_output(PWM_TIMER, TIM_OC2);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3);
   timer_enable_oc_output(PWM_TIMER, TIM_OC1N);
   timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3N);
}

/**
* Disable timer PWM output
*/
void PwmGeneration::DisableOutput()
{
   timer_disable_oc_output(PWM_TIMER, TIM_OC1);
   timer_disable_oc_output(PWM_TIMER, TIM_OC2);
   timer_disable_oc_output(PWM_TIMER, TIM_OC3);
   timer_disable_oc_output(PWM_TIMER, TIM_OC1N);
   timer_disable_oc_output(PWM_TIMER, TIM_OC2N);
   timer_disable_oc_output(PWM_TIMER, TIM_OC3N);
}

void PwmGeneration::SetCurrentLimitThreshold(s32fp ocurlim)
{
   //We use the average offset and gain values because we only
   //have one reference channel per polarity
   s32fp iofs = (ilofs[0] + ilofs[1]) / 2;
   s32fp igain= (Param::Get(Param::il1gain) + Param::Get(Param::il2gain)) / 2;

   ocurlim = FP_MUL(igain, ocurlim);
   int limNeg = FP_TOINT(iofs-ocurlim);
   int limPos = FP_TOINT(iofs+ocurlim);
   limNeg = MAX(0, limNeg);
   limPos = MIN(OCURMAX, limPos);

   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC2, limNeg);
   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC3, limPos);
}


/*----- Private methods ----------------------------------------- */
void PwmGeneration::CalcNextAngleSync(int dir)
{
   if (Encoder::SeenNorthSignal())
   {
      uint16_t polePairs = Param::GetInt(Param::polepairs) / Param::GetInt(Param::respolepairs);
      uint16_t syncOfs = Param::GetInt(Param::syncofs);
      uint16_t rotorAngle = Encoder::GetRotorAngle();
      int16_t syncAdv = FP_TOINT(FP_MUL(Param::Get(Param::syncadv), frq));

      syncOfs += syncAdv;

      angle = polePairs * rotorAngle + syncOfs;
      frq = polePairs * Encoder::GetRotorFrequency();
   }
   else
   {
      frq = fslip;
      angle += dir * FRQ_TO_ANGLE(fslip);
   }
}

void PwmGeneration::CalcNextAngleAsync(int dir)
{
   static uint16_t slipAngle = 0;
   uint32_t polePairs = Param::GetInt(Param::polepairs);
   uint16_t rotorAngle = Encoder::GetRotorAngle();

   frq = polePairs * Encoder::GetRotorFrequency() + fslip;
   slipAngle += dir * slipIncr;

   if (frq < 0) frq = 0;

   angle = polePairs * rotorAngle + slipAngle;
}

void PwmGeneration::CalcNextAngleConstant(int dir)
{
   frq = fslip;
   angle += dir * slipIncr;

   if (frq < 0) frq = 0;
}

void PwmGeneration::Charge()
{
   int dc = ampnom * (1 << pwmdigits);
   dc = FP_TOINT(dc) / 100;

   if (dc > ((1 << pwmdigits) - 100))
      dc = (1 << pwmdigits) - 100;
   if (dc < 0)
      dc = 0;

   Param::SetInt(Param::amp, dc);

   timer_set_oc_value(PWM_TIMER, TIM_OC2, dc);
}

void PwmGeneration::AcHeat()
{
   //We need to make sure the negative output is NEVER permanently on.
   if (ampnom < FP_FROMFLT(20))
   {
      timer_disable_break_main_output(PWM_TIMER);
   }
   else
   {
      timer_enable_break_main_output(PWM_TIMER);
      int dc = FP_TOINT((ampnom * 30000) / 100);
      Param::SetInt(Param::amp, dc);
      timer_set_period(PWM_TIMER, dc);
      timer_set_oc_value(PWM_TIMER, TIM_OC2, dc / 2);
   }
}

void PwmGeneration::PwmInit()
{
   pwmdigits = MIN_PWM_DIGITS + Param::GetInt(Param::pwmfrq);
   pwmfrq = TimerSetup(Param::GetInt(Param::deadtime), Param::GetInt(Param::pwmpol));
   slipIncr = FRQ_TO_ANGLE(fslip);
   shiftForTimer = SineCore::BITS - pwmdigits;
   tripped = false;
   Encoder::SetPwmFrequency(pwmfrq);
   initwait = 10000;

   if (opmode == MOD_ACHEAT)
      AcHeatTimerSetup();
}

s32fp PwmGeneration::GetCurrent(AnaIn::AnaIns input, s32fp offset, s32fp gain)
{
   s32fp il = FP_FROMINT(AnaIn::Get(input));
   il -= offset;
   return FP_DIV(il, gain);
}

s32fp PwmGeneration::ProcessCurrents(s32fp& id, s32fp& iq)
{
   if (initwait > 0)
   {
      initwait--;
      SetCurrentOffset(AnaIn::Get(AnaIn::il1), AnaIn::Get(AnaIn::il2));
   }
   else
   {
      s32fp il1 = GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain));
      s32fp il2 = GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain));

      if ((Param::GetInt(Param::pinswap) & SWAP_CURRENTS) > 0)
         FOC::ParkClarke(il2, il1, angle);
      else
         FOC::ParkClarke(il1, il2, angle);
      id = FOC::id;
      iq = FOC::iq;

      Param::SetFlt(Param::id, FOC::id);
      Param::SetFlt(Param::iq, FOC::iq);
      Param::SetFlt(Param::il1, il1);
      Param::SetFlt(Param::il2, il2);
      //Param::SetFlt(Param::ilmax, ilMax);
   }

   return 0;//ilMax;
}

/**
* Setup main PWM timer
*
* @param[in] deadtime Deadtime between bottom and top (coded value, consult STM32 manual)
* @param[in] pwmpol Output Polarity. 0=Active High, 1=Active Low
* @return PWM frequency
*/
uint16_t PwmGeneration::TimerSetup(uint16_t deadtime, int pwmpol)
{
   const uint16_t pwmmax = 1U << pwmdigits;
   uint8_t outputMode;

   rcc_periph_reset_pulse(PWM_TIMRST);
   /* disable timer */
   timer_disable_counter(PWM_TIMER);
   /* Center aligned PWM */
   timer_set_alignment(PWM_TIMER, TIM_CR1_CMS_CENTER_1);
   timer_enable_preload(PWM_TIMER);
   /* PWM mode 1 and preload enable */
   TIM_CCMR1(PWM_TIMER) = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE |
                          TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
   TIM_CCMR2(PWM_TIMER) = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;

   if (pwmpol)
   {
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC1);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC2);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC3);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC1N);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC2N);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC3N);
      outputMode = GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN;
   }
   else
   {
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC1);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC2);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC3);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC1N);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC2N);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC3N);
      outputMode = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL;
   }

   timer_disable_break_automatic_output(PWM_TIMER);
   timer_enable_break_main_output(PWM_TIMER);
   timer_set_break_polarity_high(PWM_TIMER);
   timer_enable_break(PWM_TIMER);
   timer_set_enabled_off_state_in_run_mode(PWM_TIMER);
   timer_set_enabled_off_state_in_idle_mode(PWM_TIMER);

   timer_set_deadtime(PWM_TIMER, deadtime);

   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC1);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC2);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC3);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC1N);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC2N);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC3N);

   timer_clear_flag(PWM_TIMER, TIM_SR_BIF);
   timer_enable_irq(PWM_TIMER, TIM_DIER_UIE);
   timer_enable_irq(PWM_TIMER, TIM_DIER_BIE);

   timer_set_prescaler(PWM_TIMER, 0);
   /* PWM frequency */
   timer_set_period(PWM_TIMER, pwmmax);
   timer_set_repetition_counter(PWM_TIMER, 1);

   timer_set_oc_value(PWM_TIMER, TIM_OC1, 0);
   timer_set_oc_value(PWM_TIMER, TIM_OC2, 0);
   timer_set_oc_value(PWM_TIMER, TIM_OC3, 0);
   timer_generate_event(PWM_TIMER, TIM_EGR_UG);

   timer_enable_counter(PWM_TIMER);

   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, outputMode, GPIO8 | GPIO9 | GPIO10);
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, outputMode, GPIO13 | GPIO14 | GPIO15);

   return PERIPH_CLK / (uint32_t)pwmmax;
}

void PwmGeneration::AcHeatTimerSetup()
{
   timer_disable_counter(PWM_TIMER);
   timer_set_clock_division(PWM_TIMER, TIM_CR1_CKD_CK_INT_MUL_4);
   timer_set_deadtime(PWM_TIMER, 255);
   timer_set_period(PWM_TIMER, 8000);
   timer_set_oc_value(PWM_TIMER, TIM_OC2, 0);
   timer_generate_event(PWM_TIMER, TIM_EGR_UG);
   timer_enable_counter(PWM_TIMER);
}
