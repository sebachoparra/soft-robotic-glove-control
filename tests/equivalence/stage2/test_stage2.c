// Copyright 2026 Sebastian Parra
#include "soft_glove_core/soft_glove_core_ukf.h"
#include "main.c"
#undef main
#include <stdio.h>
#include <string.h>
#include <math.h>

static unsigned compared[11], mismatched[11];
static int group;
static void bits(const void *a,const void *b,size_t n){const unsigned char *x=a,*y=b; for(size_t i=0;i<n;i+=4){size_t z=n-i<4?n-i:4;compared[group]++;if(memcmp(x+i,y+i,z))mismatched[group]++;}}
#define CMP(a,b) bits(&(a),&(b),sizeof(a))
static UKFSQuat uq(SgcQuat q){return (UKFSQuat){q.w,q.x,q.y,q.z};}
static void qcmp(SgcQuat a,UKFSQuat b){CMP(a.w,b.w);CMP(a.x,b.x);CMP(a.y,b.y);CMP(a.z,b.z);}
static void sync_both(const SgcUkfState *src,SgcUkfState *ext){*ext=*src;memcpy(ukfs_x,src->x,sizeof ukfs_x);memcpy(ukfs_P,src->P,sizeof ukfs_P);ukfs_qrel0=uq(src->qrel0);ukfs_qdelta_prev=uq(src->qdelta_prev);ukfs_half_prev=src->half_prev;ukfs_half_accum=src->half_accum;ukfs_reference_valid=src->reference_valid;ukfs_telem.valid=src->telem.valid;ukfs_telem.theta_valid=src->telem.theta_valid;ukfs_telem.reference_valid=src->telem.reference_valid;ukfs_telem.theta_deg=src->telem.theta_deg;ukfs_telem.s_hat=src->telem.s_hat;ukfs_telem.v_hat=src->telem.v_hat;ukfs_telem.eta_hat=src->telem.eta_hat;ukfs_telem.b_f_hat=src->telem.b_f_hat;ukfs_telem.innovation_flex=src->telem.innovation_flex;ukfs_telem.innovation_theta=src->telem.innovation_theta;ukfs_telem.sigma_flex_model=src->telem.sigma_flex_model;ukfs_telem.sigma_theta_model=src->telem.sigma_theta_model;ukfs_telem.sigma_s=src->telem.sigma_s;ukfs_telem.sigma_v=src->telem.sigma_v;ukfs_telem.sigma_eta=src->telem.sigma_eta;ukfs_telem.sigma_b_f=src->telem.sigma_b_f;ukfs_telem.nis=src->telem.nis;ukfs_telem.rho_eta_b_f=src->telem.rho_eta_b_f;}
static void statecmp(SgcUkfState *s){bits(s->x,ukfs_x,sizeof s->x);bits(s->P,ukfs_P,sizeof s->P);qcmp(s->qrel0,ukfs_qrel0);qcmp(s->qdelta_prev,ukfs_qdelta_prev);CMP(s->half_prev,ukfs_half_prev);CMP(s->half_accum,ukfs_half_accum);CMP(s->reference_valid,ukfs_reference_valid);CMP(s->telem.valid,ukfs_telem.valid);CMP(s->telem.theta_valid,ukfs_telem.theta_valid);CMP(s->telem.reference_valid,ukfs_telem.reference_valid);
#define T(f) CMP(s->telem.f,ukfs_telem.f)
T(theta_deg);T(s_hat);T(v_hat);T(eta_hat);T(b_f_hat);T(innovation_flex);T(innovation_theta);T(sigma_flex_model);T(sigma_theta_model);T(sigma_s);T(sigma_v);T(sigma_eta);T(sigma_b_f);T(nis);T(rho_eta_b_f);
#undef T
}
int main(void){
 group=0;SgcQuat qs[]={{1,0,0,0},{0,0,0,0},{-0.5f,0.2f,0.3f,0.4f},{1e-7f,0,0,0},{0.9238795f,0,0.3826834f,0},{0.3f,-0.5f,0.7f,0.11f},{-0.21f,0.63f,-0.42f,0.55f},{0.13f,0.29f,-0.77f,0.31f}};for(size_t i=0;i<sizeof qs/sizeof qs[0];i++){UKFSQuat a=uq(qs[i]);qcmp(sgc_quat_normalize(qs[i]),ukfs_quat_normalize(a));qcmp(sgc_quat_conj(qs[i]),ukfs_quat_conj(a));qcmp(sgc_quat_neg(qs[i]),ukfs_quat_neg(a));for(size_t j=0;j<sizeof qs/sizeof qs[0];j++){UKFSQuat b=uq(qs[j]);qcmp(sgc_quat_mul(qs[i],qs[j]),ukfs_quat_mul(a,b));float x=sgc_quat_dot(qs[i],qs[j]),y=ukfs_quat_dot(a,b);CMP(x,y);qcmp(sgc_quat_relative(qs[i],qs[j]),ukfs_relative_quat(a,b));}}
 group=1;for(int j=0;j<9;j++){float x=sgc_ukf_weight_m(j),y=ukfs_weight_m(j);CMP(x,y);x=sgc_ukf_weight_c(j);y=ukfs_weight_c(j);CMP(x,y);}float A[4][4],B[4][4];sgc_ukf_zero4(A);ukfs_zero4(B);bits(A,B,sizeof A);sgc_ukf_identity4(A);ukfs_identity4(B);bits(A,B,sizeof A);
 group=2;for(int c=0;c<5;c++){float P[4][4]={{16,0,0,0},{0,225,0,0},{0,0,0.0625f,0},{0,0,0,225}},L[4][4],M[4][4];if(c==1)P[0][0]=1.1e-7f;if(c==2)P[0][0]=-1;if(c==3)P[1][1]=NAN;if(c==4)P[1][0]=NAN;bool x=sgc_ukf_cholesky4(P,L),y=ukfs_cholesky4(P,M);CMP(x,y);bits(L,M,sizeof L);}
 group=3;for(int c=0;c<6;c++){float P[4][4]={{16,0,0,0},{0,225,0,0},{0,0,0.0625f,0},{0,0,0,225}};if(c==1)P[0][1]=3;if(c==2)P[0][0]=NAN;if(c==3)P[0][1]=P[1][0]=1000;if(c==4){for(int i=0;i<4;i++)for(int j=0;j<4;j++)P[i][j]=i==j?1:0.9f;}if(c==5){for(int i=0;i<4;i++)for(int j=0;j<4;j++)P[i][j]=i==j?1e30f:-0.99e30f;}memcpy(B,P,sizeof B);sgc_ukf_repair_cov4(P);ukfs_repair_cov4(B);bits(P,B,sizeof P);}
 group=4;for(int i=-10;i<=110;i+=5)for(int e=-1;e<=2;e++){float x[4]={(float)i,0,(float)e*0.5f,(float)(i-50)},f,t,g,u;sgc_ukf_measurement_model(x,&f,&t);ukfs_measurement_model(x,&g,&u);CMP(f,g);CMP(t,u);sgc_ukf_measurement_sigma(x,&f,&t);ukfs_measurement_sigma(x,&g,&u);CMP(f,g);CMP(t,u);}
 group=5;for(int i=0;i<8;i++){float x[4]={(i&1)?101:-1,(i&2)?300:-300,(i&4)?2:-1,(i&1)?100:-100},y[4];memcpy(y,x,sizeof y);sgc_ukf_constrain_state(x);ukfs_constrain_state(y);bits(x,y,sizeof x);}
 group=6;for(int d=0;d<3;d++)for(int v=-1;v<=1;v++){float dt=(float[]){.01f,.05f,.2f}[d],x[4]={v==0?0:100,(float)v*250,v==0?2:-1,20},y[4],z[4],Q[4][4],R[4][4];sgc_ukf_process_model(x,dt,y);ukfs_process_model(x,dt,z);bits(y,z,sizeof y);sgc_ukf_process_noise(dt,Q);ukfs_process_noise(dt,R);bits(Q,R,sizeof Q);}
 group=7;for(int c=0;c<2;c++){float x[4]={30,2,.4f,3},P[4][4]={{16,0,0,0},{0,225,0,0},{0,0,.0625f,0},{0,0,0,225}},R[4][4],X[9][4],Y[9][4];if(c)P[0][0]=-2;memcpy(R,P,sizeof R);sgc_ukf_generate_sigma_points(x,P,X);ukfs_generate_sigma_points(x,R,Y);bits(P,R,sizeof P);bits(X,Y,sizeof X);}
 unsigned off_clamp=0, unwrap_pos=0, unwrap_neg=0;
 group=8;SgcUkfState s={0},t={0};s.qrel0=(SgcQuat){1,0,0,0};s.qdelta_prev=s.qrel0;sync_both(&s,&t);float th=123,gh=123;bool ok=sgc_ukf_update_theta(&t,s.qrel0,s.qrel0,&th),gok=ukfs_update_theta(uq(s.qrel0),uq(s.qrel0),&gh);CMP(ok,gok);CMP(th,gh);statecmp(&t);s.reference_valid=true;sync_both(&s,&t);for(int i=0;i<480;i++){float a=(i<240?(float)i:-(float)(i-240))*0.09f;SgcQuat q={cosf(a),sgc_ukf_n_axis[0]*sinf(a),sgc_ukf_n_axis[1]*sinf(a),sgc_ukf_n_axis[2]*sinf(a)};if(i%3==0)q=sgc_quat_neg(q);float old_half=t.half_prev;ok=sgc_ukf_update_theta(&t,(SgcQuat){1,0,0,0},q,&th);if(t.half_prev-old_half>SGC_UKFS_PI)unwrap_pos++;if(t.half_prev-old_half< -SGC_UKFS_PI)unwrap_neg++;gok=ukfs_update_theta((UKFSQuat){1,0,0,0},uq(q),&gh);CMP(ok,gok);CMP(th,gh);statecmp(&t);}
 group=9;memset(&s,0,sizeof s);s.qrel0=(SgcQuat){1,0,0,0};s.reference_valid=true;sgc_ukf_reset(&s);sync_both(&s,&t);for(int i=0;i<250;i++){float dt=.05f, flex=sgc_lut(sgc_lut_f_up,(float)(i%80+10)),theta=sgc_lut(sgc_lut_theta_up_deg,(float)(i%80+10));bool valid=true;if(i%23==3)valid=false;if(i%23==4)theta=NAN;if(i%23==5)flex=NAN;sgc_ukf_update_filter(&t,flex,valid,theta,dt);ukfs_update_filter(flex,valid,theta,dt);statecmp(&t);if(t.x[0]>0.0f&&t.x[0]<100.0f&&t.x[2]>0.0f&&t.x[2]<1.0f)off_clamp++;}
 group=10;for(int i=0;i<4;i++){memset(&s,0x55,sizeof s);s.qrel0=(SgcQuat){.5f,.5f,.5f,.5f};s.reference_valid=i&1;sync_both(&s,&t);sgc_ukf_reset(&t);ukf_shadow_reset_filter();statecmp(&t);}
 printf("coverage: off_clamp=%u unwrap_pos=%u unwrap_neg=%u\n",off_clamp,unwrap_pos,unwrap_neg);
 int fail=0;for(int i=0;i<11;i++){printf("T2.%d: %u / %u\n",i+1,compared[i],mismatched[i]);if(!compared[i]||mismatched[i])fail=1;}return fail;
}
