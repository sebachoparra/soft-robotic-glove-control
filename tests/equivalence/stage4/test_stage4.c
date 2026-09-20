// Copyright 2026 Sebastian Parra
#include "soft_glove_core/soft_glove_core_sched.h"
#define main fw_main
#define gpio_put observed_gpio_put
#define add_alarm_in_ms observed_add_alarm_in_ms
#define cancel_alarm observed_cancel_alarm
#include "main.c"
#undef main
#undef gpio_put
#undef add_alarm_in_ms
#undef cancel_alarm
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static unsigned compared[13], mismatched[13];
static int group;
static int ids[13]={1,2,3,4,5,6,7,8,13,9,10,11,12};
static void eqf(const char *name, float a, float b)
{
    uint32_t aa, bb;
    memcpy(&aa, &a, 4); memcpy(&bb, &b, 4);
    ++compared[group];
    if (aa != bb) { ++mismatched[group]; if (mismatched[group] < 8) fprintf(stderr, "T4.%d %s: %08x != %08x\n", ids[group], name, aa, bb); }
}
static void eqi(const char *name, int a, int b)
{
    ++compared[group];
    if (a != b) { ++mismatched[group]; if (mismatched[group] < 8) fprintf(stderr, "T4.%d %s: %d != %d\n", ids[group], name, a, b); }
}
#define F(x) eqf(#x, ext->x, x)
#define I(x) eqi(#x, ext->x, x)
#define RF(x) eqf(#x, a->x, b->x)
#define RI(x) eqi(#x, a->x, b->x)
static void pcmp(const SgcPositionPIResult *a, const PositionPIResult *b)
{
    RF(error); RF(previous_error); RF(p_increment_kpa); RF(i_increment_kpa); RF(delta_pref_kpa);
    RF(pressure_ref_previous_kpa); RF(pressure_ref_raw_kpa); RF(pressure_ref_rate_limited_kpa); RF(pressure_ref_kpa);
    RI(rate_limited); RI(saturated);
}
static void lcmp(const SgcLADRCResult *a, const LADRCResult *b)
{
    RF(position_error_pct); RF(equivalent_pressure_kpa); RF(feedback_correction_kpa);
    RF(feedforward_step_kpa); RF(feedforward_gamma); RF(feedforward_applied_kpa);
    RF(pressure_ref_previous_kpa); RF(pressure_ref_raw_kpa); RF(pressure_ref_rate_limited_kpa); RF(pressure_ref_kpa);
    RI(rate_limited); RI(saturated);
}
static void rcmp(const SgcPressurePIResult *a, const PressurePIResult *b)
{ RF(error); RF(p_term); RF(i_term); RF(output_unsat); RF(output); }
static void ctrlcmp(const SgcCtrlState *ext)
{
    I(active_gain_region_id);
    const SgcPlantSchedule *a=&ext->scheduled_plant; const PlantSchedule *b=&scheduled_plant;
    RF(q); RF(eta); RF(k); RF(tau); RF(kp); RF(ki); RF(b0); RI(clamped);
    I(schedule_input_valid); F(active_kp_position); F(active_ki_position); F(previous_position_error);
    pcmp(&ext->last_position_pi,&last_position_pi);
    F(eso_z1_pct); F(eso_z2_pct_s); F(active_b0); F(target_b0);
    F(adrc_model_k); F(adrc_model_tau_s); F(adrc_b0_identified); I(adrc_model_valid);
    F(step_ff_kpa); F(ff_gamma_state); lcmp(&ext->last_ladrc,&last_ladrc);
    F(pressure_integral); rcmp(&ext->last_pressure_pi,&last_pressure_pi); F(pressure_ref_kpa);
}

static void ukfcmp(const SgcUkfState *s)
{
  for(int i=0;i<4;i++) eqf("ukf.x",s->x[i],ukfs_x[i]);
  for(int i=0;i<4;i++) for(int j=0;j<4;j++) eqf("ukf.P",s->P[i][j],ukfs_P[i][j]);
#define Q(a,b) eqf("ukf.q",s->a.b,ukfs_##a.b)
  Q(qrel0,w); Q(qrel0,x); Q(qrel0,y); Q(qrel0,z);
  Q(qdelta_prev,w); Q(qdelta_prev,x); Q(qdelta_prev,y); Q(qdelta_prev,z);
#undef Q
  eqf("ukf.half_prev",s->half_prev,ukfs_half_prev);
  eqf("ukf.half_accum",s->half_accum,ukfs_half_accum);
  eqi("ukf.reference_valid",s->reference_valid,ukfs_reference_valid);
#define UF(f) eqf("ukf." #f,s->telem.f,ukfs_telem.f)
#define UI(f) eqi("ukf." #f,s->telem.f,ukfs_telem.f)
  UI(valid); UI(theta_valid); UI(reference_valid);
  UF(theta_deg); UF(s_hat); UF(v_hat); UF(eta_hat); UF(b_f_hat);
  UF(innovation_flex); UF(innovation_theta); UF(sigma_flex_model);
  UF(sigma_theta_model); UF(sigma_s); UF(sigma_v); UF(sigma_eta);
  UF(sigma_b_f); UF(nis); UF(rho_eta_b_f);
#undef UF
#undef UI
}
static void supcmp(const SgcSupState *s)
{
#define SI(f) eqi("sup." #f,s->f,f)
#define SF(f) eqf("sup." #f,s->f,f)
  SI(pneumatic_state); SI(pulse_active); SI(pulse_end_ms); SI(last_pulse_ms);
  SI(last_pulse_direction); SI(pending_reversal_direction); SI(reversal_block_until_ms);
  SI(controller_mode); SI(control_enabled); SI(logging_enabled); SI(command_abort);
  SI(outer_updated_since_log); SI(eso_updated_since_log); SI(pressure_updated_since_log);
  SF(position_ref_pct); SF(previous_q_ref_pct);
#undef SI
#undef SF
}
static void read_ctrl(SgcCtrlState *s)
{
  sgc_ctrl_state_init(s);
#define C(f) s->f=f
  C(active_gain_region_id); memcpy(&s->scheduled_plant,&scheduled_plant,sizeof(scheduled_plant));
  C(schedule_input_valid); C(active_kp_position); C(active_ki_position);
  C(previous_position_error); memcpy(&s->last_position_pi,&last_position_pi,sizeof(last_position_pi));
  C(eso_z1_pct); C(eso_z2_pct_s); C(active_b0); C(target_b0); C(adrc_model_k);
  C(adrc_model_tau_s); C(adrc_b0_identified); C(adrc_model_valid);
  C(step_ff_kpa); C(ff_gamma_state); memcpy(&s->last_ladrc,&last_ladrc,sizeof(last_ladrc));
  C(pressure_integral); memcpy(&s->last_pressure_pi,&last_pressure_pi,sizeof(last_pressure_pi));
  C(pressure_ref_kpa);
#undef C
}
static void read_ukf(SgcUkfState *s)
{
  memset(s,0,sizeof(*s));
  memcpy(s->x,ukfs_x,sizeof(s->x)); memcpy(s->P,ukfs_P,sizeof(s->P));
#define QC(a,b) s->a.b=ukfs_##a.b
  QC(qrel0,w); QC(qrel0,x); QC(qrel0,y); QC(qrel0,z);
  QC(qdelta_prev,w); QC(qdelta_prev,x); QC(qdelta_prev,y); QC(qdelta_prev,z);
#undef QC
  s->half_prev=ukfs_half_prev; s->half_accum=ukfs_half_accum;
  s->reference_valid=ukfs_reference_valid;
#define U(f) s->telem.f=ukfs_telem.f
  U(valid); U(theta_valid); U(reference_valid); U(theta_deg);
  U(s_hat); U(v_hat); U(eta_hat); U(b_f_hat); U(innovation_flex);
  U(innovation_theta); U(sigma_flex_model); U(sigma_theta_model);
  U(sigma_s); U(sigma_v); U(sigma_eta); U(sigma_b_f); U(nis); U(rho_eta_b_f);
#undef U
}
static void read_sup(SgcSupState *s)
{
  sgc_sup_init(s);
#define S(f) s->f=f
  s->pneumatic_state=(SgcPneumaticState)pneumatic_state; S(pulse_active); S(pulse_end_ms); S(last_pulse_ms);
  s->last_pulse_direction=(SgcPneumaticState)last_pulse_direction; s->pending_reversal_direction=(SgcPneumaticState)pending_reversal_direction; S(reversal_block_until_ms);
  s->controller_mode=(SgcControllerMode)controller_mode; S(control_enabled); S(logging_enabled); S(command_abort);
  S(outer_updated_since_log); S(eso_updated_since_log); S(pressure_updated_since_log);
  S(position_ref_pct); S(previous_q_ref_pct);
#undef S
}
enum { MAX_TICKS=800 };
static uint32_t times[MAX_TICKS];
static SgcTickInput events[MAX_TICKS];
static bool resets[MAX_TICKS];
static int count, index_tick, previous_tick, off_calls;
static bool loop_ready, in_event, exhausted, terminal_exit;
static uint32_t current_ms;
static SgcSchedState sched;
static SgcSupState sup;
static SgcCtrlState ctrl;
static SgcUkfState ukf;
static int observed_gpio_count, observed_arm_count, observed_cancel_count;
extern uint16_t stage4_adc_raw;
extern unsigned stage4_adc_calls;
static uint16_t raw_adc[MAX_TICKS];
static int observed_time_us32_count;
static long log_start_offset;
static FILE *captured_read;
static bool observed_csv_row;
static PositionPIResult prior_position_pi;
static bool pulse_open_before;
static unsigned pulse_early_count, direct_fill_count, direct_vent_count, direct_hold_count, direct_lockout_count;
static int t44_case=-1, t44_n[3];
static uint32_t t44_ticks[3][16];
static unsigned sample_blocked_count, sample_blocked_due_count, ukf_clamp_boundary_count;
static uint32_t blocked_due_example_ms, blocked_due_example_ukf;
static int exact_adc_raw=-1;

static uint32_t observed_arm_ms;
static bool observed_pump_off, observed_valves, observed_outputs_off;
static SgcPneumaticState observed_valve;
static int v1_seen, v2_seen;
void observed_gpio_put(uint32_t pin,bool value)
{
  observed_gpio_count++;
  if(pin==PUMP_PIN && !value) observed_pump_off=true;
  if(pin==V1_PIN) v1_seen=value?2:1;
  if(pin==V2_PIN) v2_seen=value?2:1;
  if(v1_seen && v2_seen) {
    observed_valves=true; observed_valve=(SgcPneumaticState)pneumatic_state;
    if(observed_pump_off && v1_seen==1 && v2_seen==1 )
      observed_outputs_off=true;
    v1_seen=v2_seen=0;
  }
}
alarm_id_t observed_add_alarm_in_ms(uint32_t ms,alarm_callback_t cb,void *ud,bool fire_if_past)
{ (void)cb;(void)ud;(void)fire_if_past;observed_arm_count++;observed_arm_ms=ms;return 1; }
bool observed_cancel_alarm(alarm_id_t id)
{ (void)id;observed_cancel_count++;return true; }
static void zero_observed(void)
{ observed_gpio_count=observed_arm_count=observed_cancel_count=0; observed_arm_ms=0;
  observed_pump_off=observed_valves=observed_outputs_off=false;v1_seen=v2_seen=0; }
static void reset_witnesses(void)
{
  stage4_adc_calls=0;
  observed_time_us32_count=0;
  last_pressure_pi.error=NAN;
  fflush(stdout);
  log_start_offset=ftell(stdout);
}
static void read_csv_witness(void)
{
  fflush(stdout);
  long end=ftell(stdout);
  observed_csv_row=false;
  fseek(captured_read,log_start_offset,SEEK_SET);
  char row[8192];
  while(ftell(captured_read)<end && fgets(row,sizeof(row),captured_read)) {
    if(row[0]!='#' && row[0]!='\n') observed_csv_row=true;
  }
}
static void compare_actions(const SgcActions *a, int golden_return)
{
  eqi("action.sample",a->ran_sample,stage4_adc_calls>0);
  eqi("action.ukf",a->ran_ukf,observed_time_us32_count>0);
  eqi("action.pressure",a->ran_pressure,!isnan(last_pressure_pi.error));
  eqi("action.log",a->ran_log,observed_csv_row);
  eqi("action.valves",a->set_valves,observed_valves);
  if(a->set_valves && observed_valves) eqi("action.valve",a->valve_cmd,observed_valve);
  eqi("action.arm",a->arm_pulse,observed_arm_count>0);
  if(a->arm_pulse && observed_arm_count) eqi("action.arm_ms",a->arm_pulse_ms,observed_arm_ms);
  eqi("action.cancel",a->cancel_pulse,observed_cancel_count>0);
  eqi("action.pump_off",a->pump_off,observed_pump_off);
  eqi("action.outputs_off",a->outputs_off,observed_outputs_off);
  eqi("action.shutdown",a->shutdown,golden_return!=0);
  eqi("action.exit_code",a->exit_code,golden_return);
}
static int finish_previous(int golden_return)
{
  if(group==3 && t44_case>=0 &&
     memcmp(&prior_position_pi,&last_position_pi,sizeof(last_position_pi))!=0 &&
     t44_n[t44_case]<16) t44_ticks[t44_case][t44_n[t44_case]++]=times[previous_tick];
  if(group==9 && pulse_open_before && pulse_active && last_pulse_ms==0 &&
     !isnan(last_pressure_pi.error)) ++pulse_early_count;
  SgcTickInput in=events[previous_tick]; in.now_ms=times[previous_tick];
  SgcSchedState before=sched;
  bool golden_sample_blocked=stage4_adc_calls==0;
  bool ukf_nominally_due=(int32_t)(in.now_ms-before.next_ukf)>=0;
  bool eso_nominally_due=sup.control_enabled &&
    (sup.controller_mode==SGC_CTRL_ADRC || sup.controller_mode==SGC_CTRL_ADRC_UKF) &&
    (int32_t)(in.now_ms-before.next_eso)>=0;
  bool outer_nominally_due=sup.control_enabled &&
    (sup.controller_mode==SGC_CTRL_PI || sup.controller_mode==SGC_CTRL_PI_UKF ||
     sup.controller_mode==SGC_CTRL_ADRC || sup.controller_mode==SGC_CTRL_ADRC_UKF) &&
    (int32_t)(in.now_ms-before.next_outer)>=0;
  if(group==2 && !golden_sample_blocked && ukf_nominally_due &&
     (int32_t)(in.now_ms-(before.next_ukf+SGC_UKF_SHADOW_LOOP_MS))==
       (int32_t)SGC_UKF_SHADOW_LOOP_MS) ++ukf_clamp_boundary_count;
  in.pressure_kpa=latest_pressure_kpa;in.pressure_filtered=pressure_filtered;
  in.flex_filtered_raw=flex_filtered_raw;in.latest_position_pct=latest_position_pct;
  ctrl.last_pressure_pi.error=NAN;
  SgcActions out;
  int disposition=sgc_sched_tick(&sched,&sup,&ukf,&ctrl,&in,&out);
  eqi("disposition",disposition,golden_return);
  read_csv_witness();
  compare_actions(&out,golden_return);
  if(group==2 && golden_sample_blocked && golden_return==0) {
    ++sample_blocked_count;
    if(ukf_nominally_due || eso_nominally_due || outer_nominally_due) {
      ++sample_blocked_due_count;
      blocked_due_example_ms=in.now_ms;
      blocked_due_example_ukf=before.next_ukf;
    }
    eqi("blocked.sample",out.ran_sample,0);
    eqi("blocked.ukf",out.ran_ukf,0);
    eqi("blocked.eso",out.ran_eso,0);
    eqi("blocked.outer",out.ran_outer,0);
    eqi("blocked.pressure",out.ran_pressure,0);
    eqi("blocked.log",out.ran_log,0);
    eqi("blocked.next_sample",sched.next_sample,before.next_sample);
    eqi("blocked.next_ukf",sched.next_ukf,before.next_ukf);
    eqi("blocked.next_eso",sched.next_eso,before.next_eso);
    eqi("blocked.next_outer",sched.next_outer,before.next_outer);
    eqi("blocked.next_pressure",sched.next_pressure,before.next_pressure);
    eqi("blocked.next_log",sched.next_log,before.next_log);
  }
  supcmp(&sup); ctrlcmp(&ctrl); ukfcmp(&ukf);
  return disposition;
}
static void begin_tick(void)
{
  current_ms=times[index_tick];
  zero_observed();
  SgcTickInput *e=&events[index_tick];
  in_event=true;
  if(e->set_q) apply_reference_change(e->q_value);
  if(e->set_mode) select_controller_mode((ControllerMode)e->mode_value);
  if(resets[index_tick]) reset_controller_state_to_safe_none();
  if(e->set_logging) logging_enabled=e->logging_value;
  if(e->cmd_abort) command_abort=true;
  in_event=false;
  terminal_exit=e->cmd_abort || raw_adc[index_tick]==3213u ||
    (exact_adc_raw>=0 && raw_adc[index_tick]==(uint16_t)exact_adc_raw);
  stage4_adc_raw=raw_adc[index_tick];
  reset_witnesses();
  memcpy(&prior_position_pi,&last_position_pi,sizeof(last_position_pi));
  pulse_open_before=pulse_active && (int32_t)(pulse_end_ms-current_ms)>0;
  previous_tick=index_tick;
  index_tick++;
}
absolute_time_t get_absolute_time(void)
{
  if(in_event || exhausted || terminal_exit) return (absolute_time_t)current_ms*1000ull;
  if(!loop_ready) {
    if(pneumatic_state==STATE_OFF) {
      off_calls++;
      if(off_calls==3) {
        loop_ready=true;
        sgc_sched_init(&sched,0);
        read_sup(&sup);read_ctrl(&ctrl);read_ukf(&ukf);
      }
    }
    if(!loop_ready) return 0ull;
  } else if(index_tick>0) {
    finish_previous(0);
  }
  if(index_tick>=count) {exhausted=true;command_abort=true;return (absolute_time_t)current_ms*1000ull;}
  begin_tick();
  return (absolute_time_t)current_ms*1000ull;
}
uint32_t to_ms_since_boot(absolute_time_t t){return (uint32_t)(t/1000ull);}
uint32_t time_us_32(void){observed_time_us32_count++;return current_ms*1000u;}
uint64_t time_us_64(void){return (uint64_t)current_ms*1000ull;}
void sleep_ms(uint32_t ms){(void)ms;}
void sleep_us(uint64_t us){(void)us;}
static void setup(void)
{
  memset(events,0,sizeof(events));memset(resets,0,sizeof(resets));memset(raw_adc,0,sizeof(raw_adc));stage4_adc_raw=0;stage4_adc_calls=0;
  count=index_tick=previous_tick=off_calls=0;loop_ready=in_event=exhausted=terminal_exit=false;
  current_ms=0;pneumatic_state=STATE_UNKNOWN;command_abort=false;
}
static void add(uint32_t now){times[count++]=now;}
static void run(void)
{
  int rc=fw_main();
  if(terminal_exit) { int extracted_rc=finish_previous(rc); fprintf(stderr,"T4.%d terminal golden_rc=%d extracted_rc=%d\n",ids[group],rc,extracted_rc); }
  eqi("script count",index_tick,count);
}

static void direct_actions(const SgcActions *a)
{
  eqi("direct.valves",a->set_valves,observed_valves);
  if(a->set_valves && observed_valves) eqi("direct.valve",a->valve_cmd,observed_valve);
  eqi("direct.arm",a->arm_pulse,observed_arm_count>0);
  if(a->arm_pulse && observed_arm_count) eqi("direct.arm_ms",a->arm_pulse_ms,observed_arm_ms);
  eqi("direct.cancel",a->cancel_pulse,observed_cancel_count>0);
  eqi("direct.pump_off",a->pump_off,observed_pump_off);
  eqi("direct.outputs_off",a->outputs_off,observed_outputs_off);
}
static void direct_pressure(float ref,float pressure,uint32_t now)
{
  SgcSupState direct_sup;SgcCtrlState direct_ctrl;SgcActions a={0};
  read_sup(&direct_sup);read_ctrl(&direct_ctrl);zero_observed();
  execute_pressure_control(ref,pressure,now);
  if(pneumatic_state==STATE_FILL) ++direct_fill_count;
  if(pneumatic_state==STATE_VENT) ++direct_vent_count;
  if(pneumatic_state==STATE_HOLD) ++direct_hold_count;
  if(pending_reversal_direction!=STATE_HOLD) ++direct_lockout_count;
  sgc_sup_execute_pressure_control(&direct_sup,&direct_ctrl,ref,pressure,now,&a);
  direct_actions(&a);supcmp(&direct_sup);ctrlcmp(&direct_ctrl);
}
static void direct_expire(uint32_t now)
{
  SgcSupState direct_sup;SgcActions a={0};
  read_sup(&direct_sup);zero_observed();
  update_active_pulse(now);
  sgc_sup_update_active_pulse(&direct_sup,now,&a);
  direct_actions(&a);supcmp(&direct_sup);
}
static void direct_reset(void)
{
  SgcSupState direct_sup;SgcCtrlState direct_ctrl;SgcTickInput in={0};
  read_sup(&direct_sup);read_ctrl(&direct_ctrl);
  in.latest_position_pct=latest_position_pct;
  in.pressure_filtered=pressure_filtered;
  reset_controller_state_to_safe_none();
  sgc_sup_reset_to_safe_none(&direct_sup,&direct_ctrl,&in);
  supcmp(&direct_sup);ctrlcmp(&direct_ctrl);
}
static int find_exact_overpressure_raw(void)
{
  uint32_t boundary_bits;
  memcpy(&boundary_bits,&(float){HARD_PRESSURE_KPA},sizeof(boundary_bits));
  for(unsigned raw=0;raw<=4095u;raw++) {
    stage4_adc_raw=(uint16_t)raw;
    float averaged=read_adc_average(PRESSURE_ADC,ADC_AVERAGE_SAMPLES);
    float converted=pressure_raw_to_kpa(averaged);
    uint32_t converted_bits;
    memcpy(&converted_bits,&converted,sizeof(converted_bits));
    if(converted_bits==boundary_bits) return (int)raw;
  }
  return -1;
}
int main(void)
{
  stdout=freopen("/tmp/stage4-build/golden_stdout.csv","w+",stdout);
  captured_read=fopen("/tmp/stage4-build/golden_stdout.csv","r");
  group=0;for(int c=0;c<4;c++){setup();for(uint32_t t=0;t<=1000;t+=10)add(t);
    if(c==1 || c==2){events[0].set_mode=true;events[0].mode_value=c==1?SGC_CTRL_PI:SGC_CTRL_ADRC;}
    if(c==3){events[0].set_logging=true;events[0].logging_value=true;}
    run();}
  group=1;setup();for(uint32_t t=0;t<=1000;t+=10)add(t);
  events[0].set_mode=true;events[0].mode_value=SGC_CTRL_ADRC;
  events[0].set_logging=true;events[0].logging_value=true;run();
  group=2;setup();add(0);add(10);add(30);add(80);add(250);add(880);add(890);add(900);run();
  setup();for(uint32_t t=0;t<400;t+=3)add(t);
  events[0].set_mode=true;events[0].mode_value=SGC_CTRL_ADRC;run();
  setup();add(0);add(98);add(100);add(103);add(106);add(108);add(118);
  events[0].set_mode=true;events[0].mode_value=SGC_CTRL_ADRC;run();
  setup();add(0);add(150);add(160);add(170);
  events[0].set_mode=true;events[0].mode_value=SGC_CTRL_ADRC;run();
  fprintf(stderr,"T4.3 sample blocked=%u blocked with due downstream=%u example now=%u next_ukf=%u UKF clamp equality=%u\n",
    sample_blocked_count,sample_blocked_due_count,blocked_due_example_ms,
    blocked_due_example_ukf,ukf_clamp_boundary_count);
  if(!sample_blocked_count || !sample_blocked_due_count || !ukf_clamp_boundary_count) {
    ++mismatched[group];fprintf(stderr,"T4.3 boundary coverage incomplete\n");
  }
  group=3;for(int c=0;c<3;c++) {t44_case=c;setup();for(uint32_t t=0;t<=(c==2?2000u:1500u);t+=10)add(t);
    uint32_t reentry=c==0?980:c==1?990:1000;events[reentry/10].set_mode=true;
    events[reentry/10].mode_value=SGC_CTRL_PI;
    events[reentry/10].set_q=true;events[reentry/10].q_value=30.0f;run();}
  t44_case=-1;
  const uint32_t t44_expected[3][4]={{980,990,1000,1500},{990,1000,1010,1500},{1000,1500,2000,0}};
  const int t44_expected_n[3]={4,4,3};
  for(int c=0;c<3;c++){fprintf(stderr,"T4.4 case %d golden outer ticks:",c);
    for(int j=0;j<t44_n[c];j++) { fprintf(stderr," %u",t44_ticks[c][j]); }
    fputc('\n',stderr);
    eqi("T4.4 tick count",t44_n[c],t44_expected_n[c]);
    for(int j=0;j<t44_n[c] && j<t44_expected_n[c];j++)
      eqi("T4.4 tick",t44_ticks[c][j],t44_expected[c][j]);
  }
  group=4;for(int m=0;m<6;m++){setup();for(uint32_t t=0;t<=600;t+=10)add(t);
    events[0].set_mode=true;events[0].mode_value=(SgcControllerMode)m;run();}
  group=5;setup();for(uint32_t t=0;t<=1100;t+=10)add(t);
  events[0].set_mode=true;events[0].mode_value=SGC_CTRL_PI_UKF;
  events[55].set_mode=true;events[55].mode_value=SGC_CTRL_ADRC_UKF;run();
  group=6;setup();for(uint32_t t=0;t<=1900;t+=10)add(t);
  events[20].set_mode=true;events[20].mode_value=SGC_CTRL_PI;
  events[40].set_mode=true;events[40].mode_value=SGC_CTRL_NONE;
  events[150].set_mode=true;events[150].mode_value=SGC_CTRL_PI;run();
  group=7;setup();for(uint32_t t=0;t<=1000;t+=10)add(t);
  events[0].set_mode=true;events[0].mode_value=SGC_CTRL_PI;
  events[30].set_mode=true;events[30].mode_value=SGC_CTRL_ADRC;
  events[60].set_mode=true;events[60].mode_value=SGC_CTRL_PRESSURE;
  run();direct_reset();
  for(int m=SGC_CTRL_PI;m<=SGC_CTRL_ADRC;m+=2) { setup(); for(uint32_t t=0;t<=600;t+=10)add(t); events[0].set_q=true;events[0].q_value=45.0f;events[0].set_mode=true;events[0].mode_value=(SgcControllerMode)m;run();direct_reset(); }
  group=8;setup();for(uint32_t t=0;t<=2000;t+=10)add(t);
  events[0].set_q=true;events[0].q_value=30;
  events[30].set_mode=true;events[30].mode_value=SGC_CTRL_PI;
  events[60].set_q=true;events[60].q_value=120;
  events[90].set_q=true;events[90].q_value=100.0005f;
  events[120].set_mode=true;events[120].mode_value=SGC_CTRL_ADRC;
  events[150].set_mode=true;events[150].mode_value=SGC_CTRL_PRESSURE;
  run();direct_reset();
  group=9;setup();for(uint32_t t=0;t<=400;t+=10)add(t);
  events[19].set_mode=true;events[19].mode_value=SGC_CTRL_PI;
  events[19].set_q=true;events[19].q_value=100.0f;run();
  fprintf(stderr,"T4.9 pulse_active early-return witnesses=%u\n",pulse_early_count);
  if(!pulse_early_count) { ++mismatched[group]; fprintf(stderr,"T4.9 pulse-active coverage empty\n"); }
  pulse_active=false;pulse_alarm_armed=false;
  pending_reversal_direction=STATE_HOLD;last_pulse_direction=STATE_HOLD;
  direct_pressure(0.0f,0.0f,500u);
  direct_pressure(100.0f,0.0f,510u);
  direct_expire(pulse_end_ms);
  last_pulse_direction=STATE_HOLD;
  direct_pressure(0.0f,100.0f,600u);
  group=10;setup();for(uint32_t t=0;t<=700;t+=10)add(t);
  events[0].set_mode=true;events[0].mode_value=SGC_CTRL_PRESSURE;run();
  pulse_active=false;pulse_alarm_armed=false;
  pending_reversal_direction=STATE_HOLD;last_pulse_direction=STATE_HOLD;
  direct_pressure(100.0f,0.0f,800u);
  direct_expire(pulse_end_ms);
  direct_pressure(0.0f,100.0f,900u);
  direct_pressure(100.0f,0.0f,910u);
  direct_pressure(0.0f,100.0f,920u);
  direct_pressure(0.0f,100.0f,930u);
  direct_pressure(0.0f,100.0f,reversal_block_until_ms);
  fprintf(stderr,"T4.9/T4.10 golden direct outcomes fill=%u vent=%u hold=%u lockout=%u\n",
    direct_fill_count,direct_vent_count,direct_hold_count,direct_lockout_count);
  if(!direct_fill_count || !direct_vent_count || !direct_hold_count || !direct_lockout_count)
    { ++mismatched[group]; fprintf(stderr,"T4.10 direct supervisor coverage incomplete\n"); }
  group=11;setup();for(uint32_t t=0;t<=1000;t+=10)add(t);run();
  exact_adc_raw=find_exact_overpressure_raw();
  if(exact_adc_raw>=0) {
    fprintf(stderr,"T4.11 exact overpressure raw=%d\n",exact_adc_raw);
    setup();add(0);add(10);raw_adc[1]=(uint16_t)exact_adc_raw;run();
  } else {
    fprintf(stderr,"T4.11 exact overpressure raw: none among integers 0..4095 through golden conversion\n");
  }
  setup();add(0);add(10);raw_adc[1]=3213u;run();
  setup();add(0);add(10);events[1].cmd_abort=true;run();
  group=12;setup();for(uint32_t t=0;t<=1000;t+=10)add(t);
  events[90].set_logging=true;events[90].logging_value=true;run();
  unsigned tc=0,tm=0;
  for(int i=0;i<13;i++){fprintf(stderr,"T4.%d compared=%u mismatched=%u\n",ids[i],compared[i],mismatched[i]);
    tc+=compared[i];tm+=mismatched[i];if(!compared[i])tm++;}
  fprintf(stderr,"TOTAL compared=%u mismatched=%u\n",tc,tm);
  return tm?1:0;
}
