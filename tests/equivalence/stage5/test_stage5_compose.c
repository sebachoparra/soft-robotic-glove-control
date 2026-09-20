// Copyright 2026 Sebastian Parra
#include "soft_glove_core/soft_glove_core_ukf_compose.h"
#define main fw_main
#include "main.c"
#undef main

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t virtual_now_us;
static unsigned compared[5], mismatched[5];
static int group;

uint32_t time_us_32(void) {return virtual_now_us;}
uint64_t time_us_64(void) {return virtual_now_us;}
absolute_time_t get_absolute_time(void) {return virtual_now_us;}
uint32_t to_ms_since_boot(absolute_time_t t) {return (uint32_t)(t / 1000ull);}
void sleep_ms(uint32_t ms) {(void)ms;}
void sleep_us(uint64_t us) {(void)us;}

static void eqi(const char *name, int a, int b)
{
  ++compared[group];
  if(a!=b) {
    ++mismatched[group];
    if(mismatched[group]<8) fprintf(stderr,"T5.1%c %s %d != %d\n",'a'+group,name,a,b);
  }
}
static void equ(const char *name, uint32_t a, uint32_t b)
{
  ++compared[group];
  if(a!=b) {
    ++mismatched[group];
    if(mismatched[group]<8) fprintf(stderr,"T5.1%c %s %u != %u\n",'a'+group,name,a,b);
  }
}
static void eqf(const char *name, float a, float b)
{
  uint32_t aa,bb;memcpy(&aa,&a,4);memcpy(&bb,&b,4);
  ++compared[group];
  if(aa!=bb) {
    ++mismatched[group];
    if(mismatched[group]<8) fprintf(stderr,"T5.1%c %s %08x != %08x\n",'a'+group,name,aa,bb);
  }
}
static void quatcmp(const char *name,SgcQuat a,UKFSQuat b)
{
  eqf(name,a.w,b.w);eqf(name,a.x,b.x);eqf(name,a.y,b.y);eqf(name,a.z,b.z);
}
static void seed_from_golden(SgcUkfCompose *c)
{
  sgc_ukf_compose_init(c);
  memcpy(c->ukf.x,ukfs_x,sizeof(c->ukf.x));
  memcpy(c->ukf.P,ukfs_P,sizeof(c->ukf.P));
  c->ukf.qrel0=(SgcQuat){ukfs_qrel0.w,ukfs_qrel0.x,ukfs_qrel0.y,ukfs_qrel0.z};
  c->ukf.qdelta_prev=(SgcQuat){ukfs_qdelta_prev.w,ukfs_qdelta_prev.x,ukfs_qdelta_prev.y,ukfs_qdelta_prev.z};
  c->ukf.half_prev=ukfs_half_prev;
  c->ukf.half_accum=ukfs_half_accum;
  c->ukf.reference_valid=ukfs_reference_valid;
  c->ukf.telem.valid=ukfs_telem.valid;
  c->ukf.telem.theta_valid=ukfs_telem.theta_valid;
  c->ukf.telem.reference_valid=ukfs_telem.reference_valid;
  c->ukf.telem.theta_deg=ukfs_telem.theta_deg;
  c->ukf.telem.s_hat=ukfs_telem.s_hat;c->ukf.telem.v_hat=ukfs_telem.v_hat;
  c->ukf.telem.eta_hat=ukfs_telem.eta_hat;c->ukf.telem.b_f_hat=ukfs_telem.b_f_hat;
  c->ukf.telem.innovation_flex=ukfs_telem.innovation_flex;
  c->ukf.telem.innovation_theta=ukfs_telem.innovation_theta;
  c->ukf.telem.sigma_flex_model=ukfs_telem.sigma_flex_model;
  c->ukf.telem.sigma_theta_model=ukfs_telem.sigma_theta_model;
  c->ukf.telem.sigma_s=ukfs_telem.sigma_s;c->ukf.telem.sigma_v=ukfs_telem.sigma_v;
  c->ukf.telem.sigma_eta=ukfs_telem.sigma_eta;c->ukf.telem.sigma_b_f=ukfs_telem.sigma_b_f;
  c->ukf.telem.nis=ukfs_telem.nis;c->ukf.telem.rho_eta_b_f=ukfs_telem.rho_eta_b_f;
  c->last_update_us=ukfs_last_update_us;
  c->bno1_ok=ukfs_telem.bno1_ok;c->bno2_ok=ukfs_telem.bno2_ok;
  c->bno1_q=(SgcQuat){ukfs_telem.bno1_qw,ukfs_telem.bno1_qx,ukfs_telem.bno1_qy,ukfs_telem.bno1_qz};
  c->bno2_q=(SgcQuat){ukfs_telem.bno2_qw,ukfs_telem.bno2_qx,ukfs_telem.bno2_qy,ukfs_telem.bno2_qz};
  c->flex_aligned=ukfs_telem.flex_aligned;
}
static void compare_state(const SgcUkfCompose *c)
{
  for(int i=0;i<4;i++)eqf("x",c->ukf.x[i],ukfs_x[i]);
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)eqf("P",c->ukf.P[i][j],ukfs_P[i][j]);
  quatcmp("qrel0",c->ukf.qrel0,ukfs_qrel0);
  quatcmp("qdelta_prev",c->ukf.qdelta_prev,ukfs_qdelta_prev);
  eqf("half_prev",c->ukf.half_prev,ukfs_half_prev);
  eqf("half_accum",c->ukf.half_accum,ukfs_half_accum);
  eqi("reference_valid",c->ukf.reference_valid,ukfs_reference_valid);
#define TB(f) eqi(#f,c->ukf.telem.f,ukfs_telem.f)
#define TF(f) eqf(#f,c->ukf.telem.f,ukfs_telem.f)
  TB(valid);TB(theta_valid);TB(reference_valid);
  TF(theta_deg);TF(s_hat);TF(v_hat);TF(eta_hat);TF(b_f_hat);
  TF(innovation_flex);TF(innovation_theta);
  TF(sigma_flex_model);TF(sigma_theta_model);
  TF(sigma_s);TF(sigma_v);TF(sigma_eta);TF(sigma_b_f);
  TF(nis);TF(rho_eta_b_f);
#undef TB
#undef TF
  equ("last_update_us",c->last_update_us,ukfs_last_update_us);
  eqi("bno1_ok",c->bno1_ok,ukfs_telem.bno1_ok);
  eqi("bno2_ok",c->bno2_ok,ukfs_telem.bno2_ok);
  eqf("bno1_qw",c->bno1_q.w,ukfs_telem.bno1_qw);
  eqf("bno1_qx",c->bno1_q.x,ukfs_telem.bno1_qx);
  eqf("bno1_qy",c->bno1_q.y,ukfs_telem.bno1_qy);
  eqf("bno1_qz",c->bno1_q.z,ukfs_telem.bno1_qz);
  eqf("bno2_qw",c->bno2_q.w,ukfs_telem.bno2_qw);
  eqf("bno2_qx",c->bno2_q.x,ukfs_telem.bno2_qx);
  eqf("bno2_qy",c->bno2_q.y,ukfs_telem.bno2_qy);
  eqf("bno2_qz",c->bno2_q.z,ukfs_telem.bno2_qz);
  eqf("flex_aligned",c->flex_aligned,ukfs_telem.flex_aligned);
}
static void reset_fixture(SgcUkfCompose *c,bool reference_valid)
{
  ukf_shadow_reset_filter();
  memset(&ukfs_telem,0,sizeof(ukfs_telem));
  ukfs_last_update_us=0u;
  ukfs_reference_valid=reference_valid;
  seed_from_golden(c);
}
static void step(SgcUkfCompose *c,float flex,uint32_t now_us)
{
  virtual_now_us=now_us;
  ukf_shadow_update(flex);
  SgcQuat unused={1.0f,0.0f,0.0f,0.0f};
  sgc_ukf_compose_step(c,flex,false,unused,false,unused,now_us);
  compare_state(c);
}
static void direct_theta(SgcUkfCompose *c)
{
  UKFSQuat g1={1.0f,0.0f,0.0f,0.0f};
  UKFSQuat g2={0.9238795f,0.3826834f,0.0f,0.0f};
  SgcQuat s1={g1.w,g1.x,g1.y,g1.z},s2={g2.w,g2.x,g2.y,g2.z};
  float gt=NAN,st=NAN;
  bool gb=ukfs_update_theta(g1,g2,&gt);
  bool sb=sgc_ukf_update_theta(&c->ukf,s1,s2,&st);
  eqi("direct_theta_ok",sb,gb);
  eqf("direct_theta",st,gt);
  compare_state(c);
}
int main(void)
{
  SgcUkfCompose c;
  group=0;reset_fixture(&c,false);
  step(&c,10.0f,0u);eqf("dt_first",c.dt_used,0.05f);
  step(&c,11.0f,50000u);eqf("dt_first_zero",c.dt_used,0.05f);
  step(&c,12.0f,100000u);eqf("dt_normal",c.dt_used,0.05f);
  step(&c,13.0f,100000u);eqf("dt_zero",c.dt_used,0.05f);
  step(&c,14.0f,400001u);eqf("dt_long",c.dt_used,0.05f);
  ukfs_last_update_us=0xfffffff0u;c.last_update_us=0xfffffff0u;
  step(&c,15.0f,0x20u);
  eqf("dt_wrap",c.dt_used,(float)(uint32_t)(0x20u-0xfffffff0u)*1.0e-6f);
  group=1;reset_fixture(&c,false);
  ukfs_telem.bno1_qw=0.5f;ukfs_telem.bno2_qx=-0.25f;ukfs_telem.theta_deg=7.0f;
  seed_from_golden(&c);
  step(&c,20.0f,1000u);
  eqf("stale_theta",c.ukf.telem.theta_deg,7.0f);
  direct_theta(&c);
  group=2;reset_fixture(&c,false);step(&c,21.0f,50000u);
  ukfs_reference_valid=true;c.ukf.reference_valid=true;
  step(&c,22.0f,100000u);
  group=3;reset_fixture(&c,false);
  ukfs_P[2][2]=0.0f;ukfs_P[3][3]=0.0f;ukfs_P[2][3]=10.0f;seed_from_golden(&c);
  step(&c,23.0f,1000u);eqf("rho_plus",c.ukf.telem.rho_eta_b_f,1.0f);
  ukfs_P[2][3]=-10.0f;c.ukf.P[2][3]=-10.0f;
  step(&c,24.0f,2000u);eqf("rho_minus",c.ukf.telem.rho_eta_b_f,-1.0f);
  group=4;sgc_ukf_compose_init(&c);
  SgcRefSample samples[4]={
    {true,{1.0f,0.0f,0.0f,0.0f},true,{-0.9238795f,-0.3826834f,0.0f,0.0f}},
    {true,{1.0f,0.0f,0.0f,0.0f},true,{0.9238795f,0.3826834f,0.0f,0.0f}},
    {false,{0.0f,0.0f,0.0f,0.0f},false,{0.0f,0.0f,0.0f,0.0f}},
    {false,{0.0f,0.0f,0.0f,0.0f},false,{0.0f,0.0f,0.0f,0.0f}}
  };
  SgcUkfState before=c.ukf;
  eqi("reject",sgc_ukf_accumulate_reference(&c,samples,4u),true);
  eqi("reference_valid",c.ukf.reference_valid,true);
  SgcUkfState expected=before;
  SgcQuat rel=sgc_quat_relative(samples[0].q1,samples[0].q2);
  sgc_ukf_set_reference(&expected,rel);
  eqf("reference_w",c.ukf.qrel0.w,expected.qrel0.w);
  eqf("reference_x",c.ukf.qrel0.x,expected.qrel0.x);
  eqf("reference_y",c.ukf.qrel0.y,expected.qrel0.y);
  eqf("reference_z",c.ukf.qrel0.z,expected.qrel0.z);
  sgc_ukf_compose_init(&c);
  samples[1].ok1=false;
  eqi("reject_low_good",sgc_ukf_accumulate_reference(&c,samples,4u),false);
  eqi("reject_keeps_reference",c.ukf.reference_valid,false);
  samples[1].ok1=true;
  samples[2].ok1=true;samples[2].ok2=true;samples[2].q1=samples[0].q1;
  samples[2].q2=(SgcQuat){1.0f,0.0f,0.0f,0.0f};
  eqi("average_success",sgc_ukf_accumulate_reference(&c,samples,4u),true);
  eqi("average_reference_valid",c.ukf.reference_valid,true);
  eqi("average_changes_reference",memcmp(&c.ukf.qrel0,&expected.qrel0,sizeof(SgcQuat))!=0,true);
  eqi("average_keeps_first_sign",sgc_quat_dot(c.ukf.qrel0,rel)>0.0f,true);
  unsigned total=0,bad=0;
  for(int i=0;i<5;i++){
    fprintf(stderr,"T5.1%c compared=%u mismatched=%u\n",'a'+i,compared[i],mismatched[i]);
    total+=compared[i];bad+=mismatched[i];if(!compared[i])bad++;
  }
  fprintf(stderr,"TOTAL compared=%u mismatched=%u\n",total,bad);
  return bad?1:0;
}
