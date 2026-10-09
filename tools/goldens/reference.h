// Ported from orangeduck/Spring-It-On at commit 4d97b497a78e40c7b47d49e8a9da27e4aa5616d6.
// Copyright (c) 2021 Daniel Holden
// Licensed under MIT; full text in THIRD_PARTY_NOTICES.md.
// Changes: extracted math functions; omitted rendering and demo drivers.
#include <cmath>
#include <cfloat>
float lerp(float x, float y, float a)
{
    return (1.0f - a) * x + a * y;
}
float clamp(float x, float minimum, float maximum)
{
    return x > maximum ? maximum : x < minimum ? minimum : x;
}
float max(float x, float y)
{
    return x > y ? x : y;
}
float min(float x, float y)
{
    return x < y ? x : y;
}
float sign(float x)
{
    return x > 0.0f ? 1.0f : x < 0.0f ? -1.0f : 0.0f;
}
float fast_negexp(float x)
{
    return 1.0f / (1.0f + x + 0.48f*x*x + 0.235f*x*x*x);
}
float damper_exact(float x, float g, float halflife, float dt, float eps=1e-5f)
{
    return lerp(x, g, 1.0f - fast_negexp((0.69314718056f * dt) / (halflife + eps)));
}
float damper_decay_exact(float x, float halflife, float dt, float eps=1e-5f)
{
    return x * fast_negexp((0.69314718056f * dt) / (halflife + eps));
}
float fast_atan(float x)
{
    float z = fabs(x);
    float w = z > 1.0f ? 1.0f / z : z;
    float y = (M_PI / 4.0f)*w - w*(w - 1)*(0.2447f + 0.0663f*w);
    return copysign(z > 1.0f ? M_PI / 2.0 - y : y, x);
}
float squaref(float x)
{
    return x*x;
}
void spring_damper_exact_stiffness_damping(
    float& x,
    float& v,
    float x_goal,
    float v_goal,
    float stiffness,
    float damping,
    float dt,
    float eps = 1e-5f)
{
    float g = x_goal;
    float q = v_goal;
    float s = stiffness;
    float d = damping;
    float c = g + (d*q) / (s + eps);
    float y = d / 2.0f;

    if (fabs(s - (d*d) / 4.0f) < eps)
    {
        float j0 = x - c;
        float j1 = v + j0*y;

        float eydt = fast_negexp(y*dt);

        x =  j0*eydt + dt*j1*eydt + c;
        v = -y*j0*eydt - y*dt*j1*eydt + j1*eydt;
    }
    else if (s - (d*d) / 4.0f > 0.0)
    {
        float w = sqrtf(s - (d*d)/4.0f);
        float j = sqrtf(squaref(v + y*(x - c)) / (w*w + eps) + squaref(x - c));
        float p = fast_atan((v + (x - c) * y) / (-(x - c)*w + eps));

        j = (x - c) > 0.0f ? j : -j;

        float eydt = fast_negexp(y*dt);

        x = j*eydt*cosf(w*dt + p) + c;
        v = -y*j*eydt*cosf(w*dt + p) - w*j*eydt*sinf(w*dt + p);
    }
    else if (s - (d*d) / 4.0f < 0.0)
    {
        float y0 = (d + sqrtf(d*d - 4*s)) / 2.0f;
        float y1 = (d - sqrtf(d*d - 4*s)) / 2.0f;
        float j1 = (c*y0 - x*y0 - v) / (y1 - y0);
        float j0 = x - j1 - c;

        float ey0dt = fast_negexp(y0*dt);
        float ey1dt = fast_negexp(y1*dt);

        x =  j0*ey0dt + j1*ey1dt + c;
        v = -y0*j0*ey0dt - y1*j1*ey1dt;
    }
}
float halflife_to_damping(float halflife, float eps = 1e-5f)
{
    return (4.0f * 0.69314718056f) / (halflife + eps);
}
float damping_to_halflife(float damping, float eps = 1e-5f)
{
    return (4.0f * 0.69314718056f) / (damping + eps);
}
float frequency_to_stiffness(float frequency)
{
   return squaref(2.0f * M_PI * frequency);
}
float stiffness_to_frequency(float stiffness)
{
    return sqrtf(stiffness) / (2.0f * M_PI);
}
float critical_halflife(float frequency)
{
    return damping_to_halflife(sqrtf(frequency_to_stiffness(frequency) * 4.0f));
}
float critical_frequency(float halflife)
{
    return stiffness_to_frequency(squaref(halflife_to_damping(halflife)) / 4.0f);
}
void spring_damper_exact(
    float& x,
    float& v,
    float x_goal,
    float v_goal,
    float frequency,
    float halflife,
    float dt,
    float eps = 1e-5f)
{
    float g = x_goal;
    float q = v_goal;
    float s = frequency_to_stiffness(frequency);
    float d = halflife_to_damping(halflife);
    float c = g + (d*q) / (s + eps);
    float y = d / 2.0f;

    if (fabs(s - (d*d) / 4.0f) < eps)
    {
        float j0 = x - c;
        float j1 = v + j0*y;

        float eydt = fast_negexp(y*dt);

        x = j0*eydt + dt*j1*eydt + c;
        v = -y*j0*eydt - y*dt*j1*eydt + j1*eydt;
    }
    else if (s - (d*d) / 4.0f > 0.0)
    {
        float w = sqrtf(s - (d*d)/4.0f);
        float j = sqrtf(squaref(v + y*(x - c)) / (w*w + eps) + squaref(x - c));
        float p = fast_atan((v + (x - c) * y) / (-(x - c)*w + eps));

        j = (x - c) > 0.0f ? j : -j;

        float eydt = fast_negexp(y*dt);

        x = j*eydt*cosf(w*dt + p) + c;
        v = -y*j*eydt*cosf(w*dt + p) - w*j*eydt*sinf(w*dt + p);
    }
    else if (s - (d*d) / 4.0f < 0.0)
    {
        float y0 = (d + sqrtf(d*d - 4*s)) / 2.0f;
        float y1 = (d - sqrtf(d*d - 4*s)) / 2.0f;
        float j1 = (c*y0 - x*y0 - v) / (y1 - y0);
        float j0 = x - j1 - c;

        float ey0dt = fast_negexp(y0*dt);
        float ey1dt = fast_negexp(y1*dt);

        x =  j0*ey0dt + j1*ey1dt + c;
        v = -y0*j0*ey0dt - y1*j1*ey1dt;
    }
}
float damping_ratio_to_stiffness(float ratio, float damping)
{
    return squaref(damping / (ratio * 2.0f));
}
float damping_ratio_to_damping(float ratio, float stiffness)
{
    return ratio * 2.0f * sqrtf(stiffness);
}
void spring_damper_exact_ratio(
    float& x,
    float& v,
    float x_goal,
    float v_goal,
    float damping_ratio,
    float halflife,
    float dt,
    float eps = 1e-5f)
{
    float g = x_goal;
    float q = v_goal;
    float d = halflife_to_damping(halflife);
    float s = damping_ratio_to_stiffness(damping_ratio, d);
    float c = g + (d*q) / (s + eps);
    float y = d / 2.0f;

    if (fabs(s - (d*d) / 4.0f) < eps)
    {
        float j0 = x - c;
        float j1 = v + j0*y;

        float eydt = fast_negexp(y*dt);

        x = j0*eydt + dt*j1*eydt + c;
        v = -y*j0*eydt - y*dt*j1*eydt + j1*eydt;
    }
    else if (s - (d*d) / 4.0f > 0.0)
    {
        float w = sqrtf(s - (d*d)/4.0f);
        float j = sqrtf(squaref(v + y*(x - c)) / (w*w + eps) + squaref(x - c));
        float p = fast_atan((v + (x - c) * y) / (-(x - c)*w + eps));

        j = (x - c) > 0.0f ? j : -j;

        float eydt = fast_negexp(y*dt);

        x = j*eydt*cosf(w*dt + p) + c;
        v = -y*j*eydt*cosf(w*dt + p) - w*j*eydt*sinf(w*dt + p);
    }
    else if (s - (d*d) / 4.0f < 0.0)
    {
        float y0 = (d + sqrtf(d*d - 4*s)) / 2.0f;
        float y1 = (d - sqrtf(d*d - 4*s)) / 2.0f;
        float j1 = (c*y0 - x*y0 - v) / (y1 - y0);
        float j0 = x - j1 - c;

        float ey0dt = fast_negexp(y0*dt);
        float ey1dt = fast_negexp(y1*dt);

        x =  j0*ey0dt + j1*ey1dt + c;
        v = -y0*j0*ey0dt - y1*j1*ey1dt;
    }
}
void critical_spring_damper_exact(
    float& x,
    float& v,
    float x_goal,
    float v_goal,
    float halflife,
    float dt)
{
    float g = x_goal;
    float q = v_goal;
    float d = halflife_to_damping(halflife);
    float c = g + (d*q) / ((d*d) / 4.0f);
    float y = d / 2.0f;
    float j0 = x - c;
    float j1 = v + j0*y;
    float eydt = fast_negexp(y*dt);

    x = eydt*(j0 + j1*dt) + c;
    v = eydt*(v - j1*y*dt);
}
void simple_spring_damper_exact(
    float& x,
    float& v,
    float x_goal,
    float halflife,
    float dt)
{
    float y = halflife_to_damping(halflife) / 2.0f;
    float j0 = x - x_goal;
    float j1 = v + j0*y;
    float eydt = fast_negexp(y*dt);

    x = eydt*(j0 + j1*dt) + x_goal;
    v = eydt*(v - j1*y*dt);
}
void decay_spring_damper_exact(
    float& x,
    float& v,
    float halflife,
    float dt)
{
    float y = halflife_to_damping(halflife) / 2.0f;
    float j1 = v + x*y;
    float eydt = fast_negexp(y*dt);

    x = eydt*(x + j1*dt);
    v = eydt*(v - j1*y*dt);
}
float halflife_to_lag(float halflife)
{
    return halflife / 0.69314718056f;
}
float lag_to_halflife(float lag)
{
    return lag * 0.69314718056f;
}
static inline float smoothstep(float x)
{
    x = clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}
void double_spring_damper_exact(
    float& x,
    float& v,
    float& xi,
    float& vi,
    float x_goal,
    float halflife,
    float dt)
{
    simple_spring_damper_exact(xi, vi, x_goal, 0.5f * halflife, dt);
    simple_spring_damper_exact(x, v, xi, 0.5f * halflife, dt);
}
void spring_character_update(
    float& x,
    float& v,
    float& a,
    float v_goal,
    float halflife,
    float dt)
{
    float y = halflife_to_damping(halflife) / 2.0f;
    float j0 = v - v_goal;
    float j1 = a + j0*y;
    float eydt = fast_negexp(y*dt);

    x = eydt*(((-j1)/(y*y)) + ((-j0 - j1*dt)/y)) +
        (j1/(y*y)) + j0/y + v_goal * dt + x;
    v = eydt*(j0 + j1*dt) + v_goal;
    a = eydt*(a - j1*y*dt);
}
void spring_character_predict(
    float px[],
    float pv[],
    float pa[],
    int count,
    float x,
    float v,
    float a,
    float v_goal,
    float halflife,
    float dt)
{
    for (int i = 0; i < count; i++)
    {
        px[i] = x;
        pv[i] = v;
        pa[i] = a;
    }

    for (int i = 0; i < count; i++)
    {
        spring_character_update(px[i], pv[i], pa[i], v_goal, halflife, i * dt);
    }
}
void tracking_spring_update(
    float& x,
    float& v,
    float x_goal,
    float v_goal,
    float a_goal,
    float x_gain,
    float v_gain,
    float a_gain,
    float dt)
{
    v = lerp(v, v + a_goal * dt, a_gain);
    v = lerp(v, v_goal, v_gain);
    v = lerp(v, (x_goal - x) / dt, x_gain);
    x = x + dt * v;
}
void tracking_spring_update_no_acceleration(
    float& x,
    float& v,
    float x_goal,
    float v_goal,
    float x_gain,
    float v_gain,
    float dt)
{
    v = lerp(v, v_goal, v_gain);
    v = lerp(v, (x_goal - x) / dt, x_gain);
    x = x + dt * v;
}
void tracking_spring_update_no_velocity_acceleration(
    float& x,
    float& v,
    float x_goal,
    float x_gain,
    float dt)
{
    v = lerp(v, (x_goal - x) / dt, x_gain);
    x = x + dt * v;
}
void tracking_spring_update_improved(
    float& x,
    float& v,
    float x_goal,
    float v_goal,
    float a_goal,
    float x_halflife,
    float v_halflife,
    float a_halflife,
    float dt)
{
    v = damper_exact(v, v + a_goal * dt, a_halflife, dt);
    v = damper_exact(v, v_goal, v_halflife, dt);
    v = damper_exact(v, (x_goal - x) / dt, x_halflife, dt);
    x = x + dt * v;
}
void tracking_spring_update_no_acceleration_improved(
    float& x,
    float& v,
    float x_goal,
    float v_goal,
    float x_halflife,
    float v_halflife,
    float dt)
{
    v = damper_exact(v, v_goal, v_halflife, dt);
    v = damper_exact(v, (x_goal - x) / dt, x_halflife, dt);
    x = x + dt * v;
}
void tracking_spring_update_no_velocity_acceleration_improved(
    float& x,
    float& v,
    float x_goal,
    float x_halflife,
    float dt)
{
    v = damper_exact(v, (x_goal - x) / dt, x_halflife, dt);
    x = x + dt * v;
}
void tracking_spring_update_exact(
    float& x,
    float& v,
    float x_goal,
    float v_goal,
    float a_goal,
    float x_gain,
    float v_gain,
    float a_gain,
    float dt,
    float gain_dt)
{
    float t0 = (1.0f - v_gain) * (1.0f - x_gain);
    float t1 = a_gain * (1.0f - v_gain) * (1.0f - x_gain);
    float t2 = (v_gain * (1.0f - x_gain)) / gain_dt;
    float t3 = x_gain / (gain_dt*gain_dt);

    float stiffness = t3;
    float damping = (1.0f - t0) / gain_dt;
    float spring_x_goal = x_goal;
    float spring_v_goal = (t2*v_goal + t1*a_goal) / ((1.0f - t0) / gain_dt);

    spring_damper_exact_stiffness_damping(
      x,
      v,
      spring_x_goal,
      spring_v_goal,
      stiffness,
      damping,
      dt);
}
void tracking_spring_update_no_acceleration_exact(
    float& x,
    float& v,
    float x_goal,
    float v_goal,
    float x_gain,
    float v_gain,
    float dt,
    float gain_dt)
{
    float t0 = (1.0f - v_gain) * (1.0f - x_gain);
    float t2 = (v_gain * (1.0f - x_gain)) / gain_dt;
    float t3 = x_gain / (gain_dt*gain_dt);

    float stiffness = t3;
    float damping = (1.0f - t0) / gain_dt;
    float spring_x_goal = x_goal;
    float spring_v_goal = t2*v_goal / ((1.0f - t0) / gain_dt);

    spring_damper_exact_stiffness_damping(
      x,
      v,
      spring_x_goal,
      spring_v_goal,
      stiffness,
      damping,
      dt);
}
void tracking_spring_update_no_velocity_acceleration_exact(
    float& x,
    float& v,
    float x_goal,
    float x_gain,
    float dt,
    float gain_dt)
{
    float t0 = 1.0f - x_gain;
    float t3 = x_gain / (gain_dt*gain_dt);

    float stiffness = t3;
    float damping = (1.0f - t0) / gain_dt;
    float spring_x_goal = x_goal;
    float spring_v_goal = 0.0f;

    spring_damper_exact_stiffness_damping(
      x,
      v,
      spring_x_goal,
      spring_v_goal,
      stiffness,
      damping,
      dt);
}
float tracking_target_acceleration(
    float x_next,
    float x_curr,
    float x_prev,
    float dt)
{
    return (((x_next - x_curr) / dt) - ((x_curr - x_prev) / dt)) / dt;
}
float tracking_target_velocity(
    float x_next,
    float x_curr,
    float dt)
{
    return (x_next - x_curr) / dt;
}
void dead_blending_transition(
    float& ext_x,
    float& ext_v,
    float& ext_t,
    float src_x,
    float src_v)
{
    ext_x = src_x;
    ext_v = src_v;
    ext_t = 0.0f;
}
float smoothstep(float t, float s)
{
    return s * (t > 1.0f ? 1.0f : 3*t*t - 2*t*t*t);
}
void dead_blending_update(
    float& out_x,
    float& out_v,
    float& ext_x,
    float& ext_v,
    float& ext_t,
    float in_x,
    float in_v,
    float blendtime,
    float dt,
    float eps=1e-8f)
{
    if (ext_t < blendtime)
    {
        ext_x += ext_v * dt;
        ext_t += dt;

        float alpha = smoothstep(ext_t / max(blendtime, eps));
        out_x = lerp(ext_x, in_x, alpha);
        out_v = lerp(ext_v, in_v, alpha);
    }
    else
    {
        out_x = in_x;
        out_v = in_v;
        ext_t = FLT_MAX;
    }
}
void dead_blending_update_decay(
    float& out_x,
    float& out_v,
    float& ext_x,
    float& ext_v,
    float& ext_t,
    float in_x,
    float in_v,
    float blendtime,
    float decay_halflife,
    float dt,
    float eps=1e-8f)
{
    if (ext_t < blendtime)
    {
        ext_v = damper_decay_exact(ext_v, decay_halflife, dt);
        ext_x += ext_v * dt;
        ext_t += dt;

        float alpha = smoothstep(ext_t / max(blendtime, eps));
        out_x = lerp(ext_x, in_x, alpha);
        out_v = lerp(ext_v, in_v, alpha);
    }
    else
    {
        out_x = in_x;
        out_v = in_v;
        ext_t = FLT_MAX;
    }
}
float smoothstep_dt(float t, float s)
{
    return s * (t > 1.0f ? 0.0f : 6*t - 6*t*t);
}
void smoothstep_solve(
    float& t, float& s, float x, float v, float overshoot = 0.05f, float eps=1e-8f)
{

    if (fabsf(v) < 1e-8f)
    {
        t = 0.0f;
        s = x;
        return;
    }


    if (x < 0.0)
    {
        smoothstep_solve(t, s, -x, -v);
        s = -s;
        return;
    }


    float t0 = (v - 6*x + sqrtf(max((6*x - v)*(6*x - v) + 8*v*v, eps))) / (4*v);
    float t1 = (v - 6*x - sqrtf(max((6*x - v)*(6*x - v) + 8*v*v, eps))) / (4*v);


    t = -0.5f > t1 && t1 > -(0.5f + overshoot) ? t1 : t0;


    float vt = smoothstep_dt(t, 1.0f);


    s = fabsf(vt) < eps ? 0.0f : v / vt;
}
void extrapolate(
    float& x,
    float& v,
    float dt,
    float halflife,
    float eps = 1e-5f)
{
    float y = 0.69314718056f / (halflife + eps);
    x = x + (v / (y + eps)) * (1.0f - fast_negexp(y * dt));
    v = v * fast_negexp(y * dt);
}
void velocity_spring_damper_exact(
    float& x,
    float& v,
    float& xi,
    float x_goal,
    float v_goal,
    float halflife,
    float dt,
    float eps = 1e-5f)
{
    float x_diff = ((x_goal - xi) > 0.0f ? 1.0f : -1.0f) * v_goal;

    float t_goal_future = halflife_to_lag(halflife);
    float x_goal_future = fabs(x_goal - xi) > t_goal_future * v_goal ?
        xi + x_diff * t_goal_future : x_goal;

    simple_spring_damper_exact(x, v, x_goal_future, halflife, dt);

    xi = fabs(x_goal - xi) > dt * v_goal ? xi + x_diff * dt : x_goal;
}
void inertialize_transition(
    float& off_x, float& off_v,
    float src_x, float src_v,
    float dst_x, float dst_v)
{
    off_x = (src_x + off_x) - dst_x;
    off_v = (src_v + off_v) - dst_v;
}
void inertialize_update(
    float& out_x, float& out_v,
    float& off_x, float& off_v,
    float in_x, float in_v,
    float halflife,
    float dt)
{
    decay_spring_damper_exact(off_x, off_v, halflife, dt);
    out_x = in_x + off_x;
    out_v = in_v + off_v;
}
float cubic(float t, float v, float g)
{
    if (t > 1.0f)
    {
        return g;
    }
    else
    {
        float w1 = 3*t*t - 2*t*t*t;
        float w2 = t*t*t - 2*t*t + t;
        return w1*g + w2*v;
    }
}
float cubic_dt(float t, float v, float g)
{
    if (t > 1.0f)
    {
        return 0.0f;
    }
    else
    {
        float q1 = 6*t - 6*t*t;
        float q2 = 3*t*t - 4*t + 1;
        return q1*g + q2*v;
    }
}
void timed_spring_damper_exact(
    float& x,
    float& v,
    float& xi,
    float x_goal,
    float t_goal,
    float halflife,
    float dt)
{
    float min_time = t_goal > dt ? t_goal : dt;

    float v_goal = (x_goal - xi) / min_time;

    float t_goal_future = halflife_to_lag(halflife);
    float x_goal_future = t_goal_future < t_goal ?
        xi + v_goal * t_goal_future : x_goal;

    simple_spring_damper_exact(x, v, x_goal_future, halflife, dt);

    xi += v_goal * dt;
}
void piecewise_interpolation(
    float& x,
    float& v,
    float t,
    float pnts[],
    int npnts)
{
    t = t * (npnts - 1);
    int i0 = floorf(t);
    int i1 = i0 + 1;
    i0 = i0 > npnts - 1 ? npnts - 1 : i0;
    i1 = i1 > npnts - 1 ? npnts - 1 : i1;
    float alpha = fmod(t, 1.0f);

    x = lerp(pnts[i0], pnts[i1], alpha);
    v = (pnts[i0] - pnts[i1]) / npnts;
}
float spring_energy(
    float x,
    float v,
    float frequency,
    float x_rest = 0.0f,
    float v_rest = 0.0f,
    float scale = 1.0f)
{
    float s = frequency_to_stiffness(frequency);

    return (
        squaref(scale * (v - v_rest)) + s *
        squaref(scale * (x - x_rest))) / 2.0f;
}
float resonant_frequency(float goal_frequency, float halflife)
{
    float d = halflife_to_damping(halflife);
    float goal_stiffness = frequency_to_stiffness(goal_frequency);
    float resonant_stiffness = goal_stiffness - (d*d)/4.0f;
    return stiffness_to_frequency(resonant_stiffness);
}
