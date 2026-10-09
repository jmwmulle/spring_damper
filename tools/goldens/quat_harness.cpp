// Ported from orangeduck/Motion-Matching spring.h, quat.h at commit 57b7250e0d34a4e456a34d47e24c2f05fdcc711e.
// Copyright (c) 2021 Daniel Holden
// Licensed under MIT; full text in THIRD_PARTY_NOTICES.md.
// Changes: deterministic inputs and JSON output; original functions included unchanged.
#include <cmath>
#include <cstdio>
#include "upstream/Motion-Matching/spring.h"
void printq(quat q) { std::printf("[%.9g,%.9g,%.9g,%.9g]",q.x,q.y,q.z,q.w); }
void printv(vec3 v) { std::printf("[%.9g,%.9g,%.9g]",v.x,v.y,v.z); }
int main() {
 std::printf("[");
 for(int family=0;family<7;family++) {
  for(int scenario=0;scenario<3;scenario++) {
   quat x=quat_from_scaled_angle_axis(vec3(0.2f,-0.3f,0.5f)); vec3 v(0.1f,-0.2f,0.3f);
   for(int frame=0;frame<(scenario==2?1:300);frame++) {
    if(family==4) { x=quat_from_scaled_angle_axis(vec3(frame*0.002f,-0.3f,0.5f)); v=vec3(0.1f,-0.2f,0.3f); }
    float dt=scenario==0 ? 1.0f/60.0f : (float[]){1.0f/30.0f,1.0f/144.0f,1.0f/60.0f,1.0f/240.0f}[frame%4];
    quat g=scenario==2 ? -x : quat_from_scaled_angle_axis(frame<100 ? vec3(0.8f,0.2f,-0.6f) : frame<200 ? vec3(-0.4f,1.1f,0.2f) : vec3(0.2f,-0.7f,0.5f));
    vec3 gv(0.3f,-0.1f,0.2f);
    if(family||scenario||frame) std::printf(",");
    std::printf("[%d,%.9g,",family,dt);printq(x);std::printf(",");printv(v);std::printf(",");printq(g);std::printf(",");
    quat out=x;vec3 ov=v;
    if(family==0) {out=damper_exact(x,g,0.3f,dt);x=out;}
    if(family==1) {out=damp_adjustment_exact(g,0.3f,dt);}
    if(family==2) {simple_spring_damper_exact(x,v,g,0.3f,dt);out=x;ov=v;}
    if(family==3) {decay_spring_damper_exact(x,v,0.3f,dt);out=x;ov=v;}
    if(family==4) {inertialize_transition(x,v,g,gv,quat(),vec3());out=x;ov=v;}
    if(family==5) {inertialize_update(out,ov,x,v,g,gv,0.3f,dt);}
    if(family==6) {out=quat_from_scaled_angle_axis(quat_to_scaled_angle_axis(g));ov=quat_to_scaled_angle_axis(g);}
    printq(out);std::printf(",");printv(ov);std::printf(",");printq(x);std::printf(",");printv(v);std::printf("]");
   }
  }
 }
 std::printf("]\n");
}
