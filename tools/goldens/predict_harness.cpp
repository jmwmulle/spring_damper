// Ported from orangeduck/Spring-It-On controller.c at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under the MIT licence; full text in THIRD_PARTY_NOTICES.md.
// Changes: deterministic sampling and JSON output.
#include "reference.h"
#include <cstdio>
int main(){float x[30],v[30],a[30];spring_character_predict(x,v,a,30,0.4f,0.2f,0.1f,1.0f,0.3f,1.0f/60.0f);std::printf("[");for(int i=0;i<30;i++){std::printf(i?",[%.9g,%.9g,%.9g]":"[%.9g,%.9g,%.9g]",x[i],v[i],a[i]);}std::printf("]\n");}
