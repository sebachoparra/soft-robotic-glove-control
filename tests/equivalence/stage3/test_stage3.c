#include "soft_glove_core/soft_glove_core_ctrl.h"
#include "main.c"
#undef main
#include <stdint.h>

static unsigned compared[11], mismatched[11];
static int group;

static void eqf(const char *name, float a, float b)
{
    uint32_t aa, bb;
    memcpy(&aa, &a, 4); memcpy(&bb, &b, 4);
    ++compared[group];
    if (aa != bb) { ++mismatched[group]; if (mismatched[group] < 8) fprintf(stderr, "T3.%d %s: %08x != %08x\n", group+1, name, aa, bb); }
}
static void eqi(const char *name, int a, int b)
{
    ++compared[group];
    if (a != b) { ++mismatched[group]; if (mismatched[group] < 8) fprintf(stderr, "T3.%d %s: %d != %d\n", group+1, name, a, b); }
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
static void sync_both(const SgcCtrlState *src, SgcCtrlState *ext)
{
    *ext=*src;
    active_gain_region_id=src->active_gain_region_id;
    memcpy(&scheduled_plant,&src->scheduled_plant,sizeof(scheduled_plant));
    schedule_input_valid=src->schedule_input_valid;
    active_kp_position=src->active_kp_position; active_ki_position=src->active_ki_position;
    previous_position_error=src->previous_position_error;
    memcpy(&last_position_pi,&src->last_position_pi,sizeof(last_position_pi));
    eso_z1_pct=src->eso_z1_pct; eso_z2_pct_s=src->eso_z2_pct_s;
    active_b0=src->active_b0; target_b0=src->target_b0;
    adrc_model_k=src->adrc_model_k; adrc_model_tau_s=src->adrc_model_tau_s;
    adrc_b0_identified=src->adrc_b0_identified; adrc_model_valid=src->adrc_model_valid;
    step_ff_kpa=src->step_ff_kpa; ff_gamma_state=src->ff_gamma_state;
    memcpy(&last_ladrc,&src->last_ladrc,sizeof(last_ladrc));
    pressure_integral=src->pressure_integral;
    memcpy(&last_pressure_pi,&src->last_pressure_pi,sizeof(last_pressure_pi));
    pressure_ref_kpa=src->pressure_ref_kpa;
}
static void statecmp(const SgcCtrlState *ext)
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
static SgcCtrlState dirty(int i)
{
    SgcCtrlState s; sgc_ctrl_state_init(&s);
    s.pressure_ref_kpa=40.0f+i; s.previous_position_error=(float)i-3.0f;
    s.eso_z1_pct=30.0f+i; s.eso_z2_pct_s=-12.0f+i;
    s.step_ff_kpa=5.0f; s.ff_gamma_state=1.0f;
    s.pressure_integral=0.1f*i; s.target_b0=2.5f;
    s.last_position_pi=(SgcPositionPIResult){.error=1,.previous_error=2,.p_increment_kpa=3,.i_increment_kpa=4,.delta_pref_kpa=5,.pressure_ref_previous_kpa=6,.pressure_ref_raw_kpa=7,.pressure_ref_rate_limited_kpa=8,.pressure_ref_kpa=9,.rate_limited=true,.saturated=true};
    s.last_ladrc=(SgcLADRCResult){.position_error_pct=1,.equivalent_pressure_kpa=2,.feedback_correction_kpa=3,.feedforward_step_kpa=4,.feedforward_gamma=5,.feedforward_applied_kpa=6,.pressure_ref_previous_kpa=7,.pressure_ref_raw_kpa=8,.pressure_ref_rate_limited_kpa=9,.pressure_ref_kpa=10,.rate_limited=true,.saturated=true};
    s.last_pressure_pi=(SgcPressurePIResult){.error=1,.p_term=2,.i_term=3,.output_unsat=4,.output=5}; return s;
}
static void telemetry(bool v,bool r,bool t,float s,float eta)
{ ukfs_telem.valid=v; ukfs_telem.reference_valid=r; ukfs_telem.theta_valid=t; ukfs_telem.s_hat=s; ukfs_telem.eta_hat=eta; }
static void schedule_case(SgcCtrlState src,bool avail,float s,float eta)
{
    SgcCtrlState ext; sync_both(&src,&ext); telemetry(avail,avail,avail,s,eta);
    sgc_ctrl_update_plant_schedule(&ext,avail,s,eta); update_plant_schedule(); statecmp(&ext);
}
int main(void)
{
    SgcCtrlState src, ext; sgc_ctrl_state_init(&src);
    group=0;
    for(int i=0;i<9;i++) { bool v=i!=1,r=i!=2,t=i!=3; float s=i==4?NAN:i==5?INFINITY:i==6?-INFINITY:45.0f;
        telemetry(v,r,t,s,0.3f); sync_both(&src,&ext);
        SgcUkfTelemetry input={0}; input.valid=v; input.reference_valid=r; input.theta_valid=t; input.s_hat=s;
        eqi("available",sgc_ctrl_ukf_feedback_available(&input),ukf4_feedback_available()); statecmp(&ext); }
    group=1;
    schedule_case(dirty(0),false,45,0.3f); schedule_case(dirty(1),true,45,NAN);
    float qs[]={0,30,45,50,70,100}, es[]={0,0.25f,0.5f,0.75f,1};
    for(unsigned i=0;i<6;i++) for(unsigned j=0;j<5;j++) schedule_case(dirty(i),true,qs[i],es[j]);
    sync_both(&src,&ext); telemetry(true,true,true,45,0.3f);
    sgc_ctrl_update_plant_schedule(&ext,true,45,0.3f); update_plant_schedule(); statecmp(&ext);
    telemetry(false,false,false,45,0.3f); sgc_ctrl_update_plant_schedule(&ext,false,45,0.3f); update_plant_schedule(); statecmp(&ext);
    telemetry(true,true,true,70,0.5f); sgc_ctrl_update_plant_schedule(&ext,true,70,0.5f); update_plant_schedule(); statecmp(&ext);
    group=2;
    float ks[]={1,0,1e-6f,-1e-6f,1e-5f,-1e-5f,NAN,INFINITY,-INFINITY,0.3f,-0.3f};
    float ds[]={0,1,-1,8,-5,26.666666f,-16.666666f,100,-100};
    for(unsigned i=0;i<11;i++) for(unsigned j=0;j<9;j++) for(int valid=0;valid<2;valid++) {
        sync_both(&src,&ext); eqf("feedforward",sgc_ctrl_compute_adrc_feedforward(20,20+ds[j],ks[i],valid),compute_adrc_feedforward(20,20+ds[j],ks[i],valid)); statecmp(&ext); }
    group=3;
    float targets[]={2.5f,2.2f,2.200001f,-100,NAN,0.0f,4};
    for(unsigned i=0;i<7;i++) for(int sign=-1;sign<=1;sign++) {
        src=dirty(i);src.target_b0=targets[i];src.pressure_ref_kpa=sign*40.0f;
        sync_both(&src,&ext);sgc_ctrl_update_active_b0(&ext);update_active_b0();statecmp(&ext); }
    src=dirty(0);sync_both(&src,&ext);
    for(int i=0;i<60;i++){sgc_ctrl_update_active_b0(&ext);update_active_b0();statecmp(&ext);}
    group=4;
    float refs[]={0,100,50,50,100,-100,200};
    for(unsigned i=0;i<7;i++){src=dirty(i);src.pressure_ref_kpa=i&1?0:170;sync_both(&src,&ext);
        SgcPositionPIResult a=sgc_ctrl_position_pi_update(&ext,refs[i],50);PositionPIResult b=position_pi_update(refs[i],50);pcmp(&a,&b);statecmp(&ext);}
    src=dirty(0);sync_both(&src,&ext);
    for(int i=0;i<120;i++){float q=(float)((i*17)%101);SgcPositionPIResult a=sgc_ctrl_position_pi_update(&ext,60,q);PositionPIResult b=position_pi_update(60,q);pcmp(&a,&b);statecmp(&ext);}
    { unsigned masks=0;
      for(int mode=0;mode<4;mode++){
        src=dirty(0); src.pressure_ref_kpa=mode==2?40.0f:0.0f;
        src.previous_position_error=0.0f; src.active_ki_position=0.0f;
        src.active_kp_position=mode==0?0.0f:mode==1?0.1f:mode==2?10.0f:100.0f;
        float ref=mode==0?0.0f:mode==2?1.0f:-1.0f;
        sync_both(&src,&ext);
        SgcPositionPIResult a=sgc_ctrl_position_pi_update(&ext,ref,0.0f);
        PositionPIResult b=position_pi_update(ref,0.0f);
        pcmp(&a,&b);statecmp(&ext);
        masks |= 1u << ((unsigned)b.rate_limited*2u+(unsigned)b.saturated);
      }
      if(masks!=15u){fprintf(stderr,"T3.5 missing flag combination: %u\n",masks);mismatched[group]++;}
    }
    group=5;
    for(int i=0;i<6;i++){src=dirty(i);src.active_b0=i&1?SGC_B0_FALLBACK:SGC_B0_NOMINAL;sync_both(&src,&ext);
        sgc_ctrl_leso_update(&ext,i*10.0f, i&1?0:100);leso_update(i*10.0f,i&1?0:100);statecmp(&ext);}
    src=dirty(0);sync_both(&src,&ext);
    for(int i=0;i<220;i++){float p=(float)((i*7)%70),q=(float)((i*13)%100);sgc_ctrl_leso_update(&ext,p,q);leso_update(p,q);statecmp(&ext);}
    group=6;
    float errors[]={15,5,1,5,15,-5,0,30,-30};src=dirty(0);src.eso_z1_pct=50;sync_both(&src,&ext);
    for(unsigned i=0;i<9;i++){SgcLADRCResult a=sgc_ctrl_ladrc_update(&ext,50+errors[i]);LADRCResult b=ladrc_update(50+errors[i]);lcmp(&a,&b);statecmp(&ext);}
    src=dirty(1);src.step_ff_kpa=0;sync_both(&src,&ext);
    {SgcLADRCResult a=sgc_ctrl_ladrc_update(&ext,60);LADRCResult b=ladrc_update(60);lcmp(&a,&b);statecmp(&ext);}
    src=dirty(1);src.step_ff_kpa=-5;sync_both(&src,&ext);
    for(int i=0;i<120;i++){float q=(float)((i*19)%101);SgcLADRCResult a=sgc_ctrl_ladrc_update(&ext,q);LADRCResult b=ladrc_update(q);lcmp(&a,&b);statecmp(&ext);}
    group=7;
    float errs[]={-40,-20,-1,-0.5f,0,0.5f,1,2,10,20,40};
    float pressures[]={-60,-25,-1,1,20,60,100};
    for(unsigned i=0;i<11;i++) for(unsigned j=0;j<7;j++) {
        float pressure=pressures[j]; float reference=pressure+errs[i];
        src=dirty(i);src.pressure_integral=i&1?-1:1;sync_both(&src,&ext);
        SgcPressurePIResult a=sgc_ctrl_pressure_pi_update(&ext,reference,pressure);
        PressurePIResult b=pressure_pi_update(reference,pressure);rcmp(&a,&b);statecmp(&ext);}
    src=dirty(0);src.pressure_integral=0;sync_both(&src,&ext);
    { float pressure=35.0f; float reference=pressure+16.0f;
      SgcPressurePIResult a=sgc_ctrl_pressure_pi_update(&ext,reference,pressure);
      PressurePIResult b=pressure_pi_update(reference,pressure);rcmp(&a,&b);statecmp(&ext); }
    for(int i=0;i<120;i++){
        float pressure=pressures[i%7]; float e=(float)((i%9)-4); float reference=pressure+e;
        SgcPressurePIResult a=sgc_ctrl_pressure_pi_update(&ext,reference,pressure);
        PressurePIResult b=pressure_pi_update(reference,pressure);rcmp(&a,&b);statecmp(&ext);}
    group=8;
    float change[]={10,0.0005f,0.001f,10,0.00001f};
    for(int i=0;i<5;i++){src=dirty(i);src.schedule_input_valid=i!=3;src.step_ff_kpa=2;src.ff_gamma_state=0.4f;sync_both(&src,&ext);
        bool avail=i!=3;float sh=45,eta=0.3f;telemetry(avail,avail,avail,sh,eta);
        float old_ref=i==2?0.0f:50.0f;
        sgc_ctrl_update_plant_schedule(&ext,avail,sh,eta);sgc_ctrl_latch_step_feedforward(&ext,old_ref,old_ref+change[i]);
        position_ref_pct=old_ref;apply_reference_change(old_ref+change[i]);statecmp(&ext);}
    group=9;
    for(int i=0;i<8;i++){src=dirty(i);src.target_b0=i==6?NAN:i==7?-1:2.5f;sync_both(&src,&ext);
        position_ref_pct=20+i;latest_position_pct=30+i;controller_mode=CTRL_PI;
        sgc_ctrl_initialize_pi_bumpless(&ext,position_ref_pct,latest_position_pct);initialize_pi_bumpless();statecmp(&ext);
        sgc_ctrl_initialize_adrc_bumpless(&ext,position_ref_pct,latest_position_pct);initialize_adrc_bumpless();statecmp(&ext);}
    group=10;
    for(int i=0;i<8;i++){src=dirty(i);sync_both(&src,&ext);
        sgc_ctrl_reset_outer_diagnostics(&ext);reset_outer_diagnostics();statecmp(&ext);
        sgc_ctrl_reset_pressure_pi(&ext);reset_pressure_actuation_state();statecmp(&ext);}
    unsigned tc=0,tm=0;
    for(int i=0;i<11;i++){printf("T3.%d compared=%u mismatched=%u\n",i+1,compared[i],mismatched[i]);tc+=compared[i];tm+=mismatched[i];if(!compared[i])tm++;}
    printf("TOTAL compared=%u mismatched=%u\n",tc,tm);return tm?1:0;
}
