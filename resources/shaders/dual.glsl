#ifndef DUAL_GLSL
#define DUAL_GLSL

struct Dual {
    float f;
    float s;
};

Dual dual_const(float v) {
    return Dual(v, 0.0);
}

Dual dual_var(float v) {
    return Dual(v, 1.0);
}

Dual dual_add(Dual a, Dual b) {
    return Dual(a.f + b.f, a.s + b.s);
}

Dual dual_add_scalar(Dual a, float b) {
    return Dual(a.f + b, a.s);
}

Dual dual_sub(Dual a, Dual b) {
    return Dual(a.f - b.f, a.s - b.s);
}

Dual dual_sub_scalar(Dual a, float b) {
    return Dual(a.f - b, a.s);
}

Dual dual_scalar_sub(float a, Dual b) {
    return Dual(a - b.f, -b.s);
}

Dual dual_mul(Dual a, Dual b) {
    return Dual(a.f * b.f, a.s * b.f + a.f * b.s);
}

Dual dual_mul_scalar(Dual a, float b) {
    return Dual(a.f * b, a.s * b);
}

Dual dual_div(Dual a, Dual b) {
    return Dual(a.f / b.f, (a.s * b.f - a.f * b.s) / (b.f * b.f));
}

Dual dual_div_scalar(Dual a, float b) {
    return Dual(a.f / b, a.s / b);
}

Dual dual_scalar_div(float a, Dual b) {
    return dual_div(dual_const(a), b);
}

Dual dual_neg(Dual a) {
    return Dual(-a.f, -a.s);
}

Dual dual_pow(Dual a, float power) {
    return Dual(pow(a.f, power), a.s * power * pow(a.f, power - 1.0));
}

Dual dual_sin(Dual a) {
    return Dual(sin(a.f), a.s * cos(a.f));
}

Dual dual_cos(Dual a) {
    return Dual(cos(a.f), -a.s * sin(a.f));
}

Dual dual_tan(Dual a) {
    return dual_div(dual_sin(a), dual_cos(a));
}

Dual dual_sqrt(Dual a) {
    float sq = sqrt(a.f);
    return Dual(sq, a.s / (2.0 * sq));
}

Dual dual_exp(Dual a) {
    float ex = exp(a.f);
    return Dual(ex, a.s * ex);
}

Dual dual_log(Dual a) {
    return Dual(log(a.f), a.s / a.f);
}

#endif
