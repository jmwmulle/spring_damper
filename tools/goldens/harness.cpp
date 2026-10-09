// Ported from orangeduck/Spring-It-On at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under MIT; full text in THIRD_PARTY_NOTICES.md.
// Changes: extracted math functions; omitted rendering and demo drivers.
#include "reference.h"
#include <cstdio>

int main(){
 std::printf("{\n");
 {
 std::printf("\"fast_negexp\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g",(double)((frame*0.5f)));
float result=fast_negexp((frame*0.5f));
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"damper_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
 float state=0.13f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g",(double)(state),(double)(goal),(double)(0.3f),(double)(dt),(double)(1e-5f));
float result=damper_exact(state,goal,0.3f,dt,1e-5f);
 state=result;
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"damper_decay_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
 float state=0.13f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g",(double)(state),(double)(0.3f),(double)(dt),(double)(1e-5f));
float result=damper_decay_exact(state,0.3f,dt,1e-5f);
 state=result;
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"spring_damper_exact_stiffness_damping\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.4f),(double)(75.0f),(double)(8.0f),(double)(dt),(double)(1e-5f));
spring_damper_exact_stiffness_damping(x,v,goal,0.4f,75.0f,8.0f,dt,1e-5f);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 std::printf(",\"halflife_to_damping\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g",(double)(0.3f),(double)(1e-5f));
float result=halflife_to_damping(0.3f,1e-5f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"damping_to_halflife\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g",(double)(8.0f),(double)(1e-5f));
float result=damping_to_halflife(8.0f,1e-5f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"frequency_to_stiffness\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g",(double)(2.0f));
float result=frequency_to_stiffness(2.0f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"stiffness_to_frequency\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g",(double)(75.0f));
float result=stiffness_to_frequency(75.0f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"critical_halflife\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g",(double)(2.0f));
float result=critical_halflife(2.0f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"critical_frequency\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g",(double)(0.3f));
float result=critical_frequency(0.3f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"spring_damper_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.4f),(double)(2.0f),(double)(0.3f),(double)(dt),(double)(1e-5f));
spring_damper_exact(x,v,goal,0.4f,2.0f,0.3f,dt,1e-5f);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 std::printf(",\"damping_ratio_to_stiffness\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g",(double)(0.65f),(double)(8.0f));
float result=damping_ratio_to_stiffness(0.65f,8.0f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"damping_ratio_to_damping\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g",(double)(0.65f),(double)(75.0f));
float result=damping_ratio_to_damping(0.65f,75.0f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"spring_damper_exact_ratio\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.4f),(double)(0.65f),(double)(0.3f),(double)(dt),(double)(1e-5f));
spring_damper_exact_ratio(x,v,goal,0.4f,0.65f,0.3f,dt,1e-5f);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"critical_spring_damper_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.4f),(double)(0.3f),(double)(dt));
critical_spring_damper_exact(x,v,goal,0.4f,0.3f,dt);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"simple_spring_damper_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.3f),(double)(dt));
simple_spring_damper_exact(x,v,goal,0.3f,dt);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"decay_spring_damper_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(0.3f),(double)(dt));
decay_spring_damper_exact(x,v,0.3f,dt);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 std::printf(",\"halflife_to_lag\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g",(double)(0.3f));
float result=halflife_to_lag(0.3f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"lag_to_halflife\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g",(double)(0.2f));
float result=lag_to_halflife(0.2f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 float xi=0.39f;
 float vi=0.52f;
 std::printf(",\"double_spring_damper_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
xi=0.39f;
vi=0.52f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(xi),(double)(vi),(double)(goal),(double)(0.3f),(double)(dt));
double_spring_damper_exact(x,v,xi,vi,goal,0.3f,dt);
 std::printf("],[%.9g,%.9g,%.9g,%.9g]]",(double)(x),(double)(v),(double)(xi),(double)(vi));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 float a=0.39f;
 std::printf(",\"spring_character_update\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
a=0.39f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(a),(double)(0.4f),(double)(0.3f),(double)(dt));
spring_character_update(x,v,a,0.4f,0.3f,dt);
 std::printf("],[%.9g,%.9g,%.9g]]",(double)(x),(double)(v),(double)(a));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"tracking_spring_update\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.4f),(double)(-0.3f),(double)(0.07f),(double)(0.15f),(double)(0.2f),(double)(dt));
tracking_spring_update(x,v,goal,0.4f,-0.3f,0.07f,0.15f,0.2f,dt);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"tracking_spring_update_no_acceleration\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.4f),(double)(0.07f),(double)(0.15f),(double)(dt));
tracking_spring_update_no_acceleration(x,v,goal,0.4f,0.07f,0.15f,dt);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"tracking_spring_update_no_velocity_acceleration\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.07f),(double)(dt));
tracking_spring_update_no_velocity_acceleration(x,v,goal,0.07f,dt);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"tracking_spring_update_improved\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.4f),(double)(-0.3f),(double)(0.3f),(double)(0.5f),(double)(0.4f),(double)(dt));
tracking_spring_update_improved(x,v,goal,0.4f,-0.3f,0.3f,0.5f,0.4f,dt);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"tracking_spring_update_no_acceleration_improved\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.4f),(double)(0.3f),(double)(0.5f),(double)(dt));
tracking_spring_update_no_acceleration_improved(x,v,goal,0.4f,0.3f,0.5f,dt);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"tracking_spring_update_no_velocity_acceleration_improved\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.3f),(double)(dt));
tracking_spring_update_no_velocity_acceleration_improved(x,v,goal,0.3f,dt);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"tracking_spring_update_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.4f),(double)(-0.3f),(double)(0.07f),(double)(0.15f),(double)(0.2f),(double)(dt),(double)((1.0f/60.0f)));
tracking_spring_update_exact(x,v,goal,0.4f,-0.3f,0.07f,0.15f,0.2f,dt,(1.0f/60.0f));
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"tracking_spring_update_no_acceleration_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.4f),(double)(0.07f),(double)(0.15f),(double)(dt),(double)((1.0f/60.0f)));
tracking_spring_update_no_acceleration_exact(x,v,goal,0.4f,0.07f,0.15f,dt,(1.0f/60.0f));
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"tracking_spring_update_no_velocity_acceleration_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(goal),(double)(0.07f),(double)(dt),(double)((1.0f/60.0f)));
tracking_spring_update_no_velocity_acceleration_exact(x,v,goal,0.07f,dt,(1.0f/60.0f));
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 std::printf(",\"tracking_target_acceleration\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g",(double)(0.4f),(double)(0.2f),(double)(0.1f),(double)(dt));
float result=tracking_target_acceleration(0.4f,0.2f,0.1f,dt);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"tracking_target_velocity\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g",(double)(0.4f),(double)(0.2f),(double)(dt));
float result=tracking_target_velocity(0.4f,0.2f,dt);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 float ext_x=0.13f;
 float ext_v=0.26f;
 float ext_t=0.39f;
 std::printf(",\"dead_blending_transition\":[");
 for(int scenario=0;scenario<2;scenario++) {
ext_x=0.13f;
ext_v=0.26f;
ext_t=0.39f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g",(double)(ext_x),(double)(ext_v),(double)(ext_t),(double)(goal),(double)(0.4f));
dead_blending_transition(ext_x,ext_v,ext_t,goal,0.4f);
 std::printf("],[%.9g,%.9g,%.9g]]",(double)(ext_x),(double)(ext_v),(double)(ext_t));
 } } std::printf("]");
 }
 {
 std::printf(",\"smoothstep\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g",(double)((frame/300.0f)),(double)(0.4f));
float result=smoothstep((frame/300.0f),0.4f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 float out_x=0.13f;
 float out_v=0.26f;
 float ext_x=0.39f;
 float ext_v=0.52f;
 float ext_t=0.65f;
 std::printf(",\"dead_blending_update\":[");
 for(int scenario=0;scenario<2;scenario++) {
out_x=0.13f;
out_v=0.26f;
ext_x=0.39f;
ext_v=0.52f;
ext_t=0.65f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(out_x),(double)(out_v),(double)(ext_x),(double)(ext_v),(double)(ext_t),(double)(goal),(double)(0.4f),(double)(0.7f),(double)(dt),(double)(1e-5f));
dead_blending_update(out_x,out_v,ext_x,ext_v,ext_t,goal,0.4f,0.7f,dt,1e-5f);
 std::printf("],[%.9g,%.9g,%.9g,%.9g,%.9g]]",(double)(out_x),(double)(out_v),(double)(ext_x),(double)(ext_v),(double)(ext_t));
 } } std::printf("]");
 }
 {
 float out_x=0.13f;
 float out_v=0.26f;
 float ext_x=0.39f;
 float ext_v=0.52f;
 float ext_t=0.65f;
 std::printf(",\"dead_blending_update_decay\":[");
 for(int scenario=0;scenario<2;scenario++) {
out_x=0.13f;
out_v=0.26f;
ext_x=0.39f;
ext_v=0.52f;
ext_t=0.65f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(out_x),(double)(out_v),(double)(ext_x),(double)(ext_v),(double)(ext_t),(double)(goal),(double)(0.4f),(double)(0.7f),(double)(0.4f),(double)(dt),(double)(1e-5f));
dead_blending_update_decay(out_x,out_v,ext_x,ext_v,ext_t,goal,0.4f,0.7f,0.4f,dt,1e-5f);
 std::printf("],[%.9g,%.9g,%.9g,%.9g,%.9g]]",(double)(out_x),(double)(out_v),(double)(ext_x),(double)(ext_v),(double)(ext_t));
 } } std::printf("]");
 }
 {
 std::printf(",\"smoothstep_dt\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g",(double)((frame/300.0f)),(double)(0.4f));
float result=smoothstep_dt((frame/300.0f),0.4f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 float t=0.13f;
 float s=0.26f;
 std::printf(",\"smoothstep_solve\":[");
 for(int scenario=0;scenario<2;scenario++) {
t=0.13f;
s=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(t),(double)(s),(double)(goal),(double)(0.4f),(double)(0.1f),(double)(1e-5f));
smoothstep_solve(t,s,goal,0.4f,0.1f,1e-5f);
 std::printf("],[%.9g,%.9g]]",(double)(t),(double)(s));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"extrapolate\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(dt),(double)(0.3f),(double)(1e-5f));
extrapolate(x,v,dt,0.3f,1e-5f);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 float xi=0.39f;
 std::printf(",\"velocity_spring_damper_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
xi=0.39f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(xi),(double)(goal),(double)(0.4f),(double)(0.3f),(double)(dt),(double)(1e-5f));
velocity_spring_damper_exact(x,v,xi,goal,0.4f,0.3f,dt,1e-5f);
 std::printf("],[%.9g,%.9g,%.9g]]",(double)(x),(double)(v),(double)(xi));
 } } std::printf("]");
 }
 {
 float off_x=0.13f;
 float off_v=0.26f;
 std::printf(",\"inertialize_transition\":[");
 for(int scenario=0;scenario<2;scenario++) {
off_x=0.13f;
off_v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(off_x),(double)(off_v),(double)(goal),(double)(0.4f),(double)(goal),(double)(0.4f));
inertialize_transition(off_x,off_v,goal,0.4f,goal,0.4f);
 std::printf("],[%.9g,%.9g]]",(double)(off_x),(double)(off_v));
 } } std::printf("]");
 }
 {
 float out_x=0.13f;
 float out_v=0.26f;
 float off_x=0.39f;
 float off_v=0.52f;
 std::printf(",\"inertialize_update\":[");
 for(int scenario=0;scenario<2;scenario++) {
out_x=0.13f;
out_v=0.26f;
off_x=0.39f;
off_v=0.52f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(out_x),(double)(out_v),(double)(off_x),(double)(off_v),(double)(goal),(double)(0.4f),(double)(0.3f),(double)(dt));
inertialize_update(out_x,out_v,off_x,off_v,goal,0.4f,0.3f,dt);
 std::printf("],[%.9g,%.9g,%.9g,%.9g]]",(double)(out_x),(double)(out_v),(double)(off_x),(double)(off_v));
 } } std::printf("]");
 }
 {
 std::printf(",\"cubic\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g",(double)((frame/300.0f)),(double)(0.4f),(double)(goal));
float result=cubic((frame/300.0f),0.4f,goal);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"cubic_dt\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g",(double)((frame/300.0f)),(double)(0.4f),(double)(goal));
float result=cubic_dt((frame/300.0f),0.4f,goal);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 float xi=0.39f;
 std::printf(",\"timed_spring_damper_exact\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
xi=0.39f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)(xi),(double)(goal),(double)(0.8f),(double)(0.3f),(double)(dt));
timed_spring_damper_exact(x,v,xi,goal,0.8f,0.3f,dt);
 std::printf("],[%.9g,%.9g,%.9g]]",(double)(x),(double)(v),(double)(xi));
 } } std::printf("]");
 }
 {
 float x=0.13f;
 float v=0.26f;
 std::printf(",\"piecewise_interpolation\":[");
 for(int scenario=0;scenario<2;scenario++) {
x=0.13f;
v=0.26f;
 for(int frame=0;frame<300;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g",(double)(x),(double)(v),(double)((frame/300.0f)));
piecewise_interpolation(x,v,(frame/300.0f),pnts,4);
 std::printf("],[%.9g,%.9g]]",(double)(x),(double)(v));
 } } std::printf("]");
 }
 {
 std::printf(",\"spring_energy\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",(double)(goal),(double)(0.4f),(double)(2.0f),(double)(0.1f),(double)(0.2f),(double)(1.4f));
float result=spring_energy(goal,0.4f,2.0f,0.1f,0.2f,1.4f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 {
 std::printf(",\"resonant_frequency\":[");
 for(int scenario=0;scenario<2;scenario++) {
 for(int frame=0;frame<8;frame++){
 float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
 float goal=frame<100?1.2f:frame<200?-0.7f:0.3f;
 float pnts[]={0.1f,0.8f,-0.4f,1.0f};
 std::printf((frame==0 && scenario==0)?"[[":" ,[[");
 std::printf("%.9g,%.9g",(double)(4.0f),(double)(0.3f));
float result=resonant_frequency(4.0f,0.3f);
 std::printf("],[%.9g]]",(double)(result));
 } } std::printf("]");
 }
 std::printf("}\n");
}
