#include "dual.h"

Dual dual_create(double first, double second) {
    Dual d;
    d.f = first;
    d.s = second;
    return d;
}

Dual dual_constant(double value) {
    return dual_create(value, 0.0);
}

Dual dual_variable(double value) {
    return dual_create(value, 1.0);
}

Dual dual_add(Dual a, Dual b) {
    return dual_create(a.f + b.f, a.s + b.s);
}

Dual dual_add_scalar(Dual a, double b) {
    return dual_create(a.f + b, a.s);
}

Dual dual_sub(Dual a, Dual b) {
    return dual_create(a.f - b.f, a.s - b.s);
}

Dual dual_sub_scalar(Dual a, double b) {
    return dual_create(a.f - b, a.s);
}

Dual dual_scalar_sub(double a, Dual b) {
    return dual_create(a - b.f, -b.s);
}

Dual dual_mul(Dual a, Dual b) {
    return dual_create(
        a.f * b.f,
        a.s * b.f + a.f * b.s
    );
}

Dual dual_mul_scalar(Dual a, double b) {
    return dual_create(a.f * b, a.s * b);
}

Dual dual_div(Dual a, Dual b) {
    return dual_create(
        a.f / b.f,
        (a.s * b.f - a.f * b.s) / (b.f * b.f)
    );
}

Dual dual_div_scalar(Dual a, double b) {
    return dual_create(a.f / b, a.s / b);
}

Dual dual_scalar_div(double a, Dual b) {
    return dual_div(dual_constant(a), b);
}

Dual dual_neg(Dual a) {
    return dual_create(-a.f, -a.s);
}

Dual dual_pow(Dual a, double power) {
    return dual_create(
        pow(a.f, power),
        a.s * power * pow(a.f, power - 1.0)
    );
}

Dual dual_sin(Dual a) {
    return dual_create(sin(a.f), a.s * cos(a.f));
}

Dual dual_cos(Dual a) {
    return dual_create(cos(a.f), -a.s * sin(a.f));
}

Dual dual_tan(Dual a) {
    return dual_div(dual_sin(a), dual_cos(a));
}

Dual dual_sqrt(Dual a) {
    double sqrt_f = sqrt(a.f);
    return dual_create(sqrt_f, a.s / (2.0 * sqrt_f));
}

Dual dual_exp(Dual a) {
    double exp_f = exp(a.f);
    return dual_create(exp_f, a.s * exp_f);
}

Dual dual_log(Dual a) {
    return dual_create(log(a.f), a.s / a.f);
}

