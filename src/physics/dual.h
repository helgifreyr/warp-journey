#ifndef DUAL_H
#define DUAL_H

#include <math.h>

typedef struct {
    double f;
    double s;
} Dual;

Dual dual_create(double first, double second);
Dual dual_constant(double value);
Dual dual_variable(double value);

Dual dual_add(Dual a, Dual b);
Dual dual_add_scalar(Dual a, double b);
Dual dual_sub(Dual a, Dual b);
Dual dual_sub_scalar(Dual a, double b);
Dual dual_scalar_sub(double a, Dual b);
Dual dual_mul(Dual a, Dual b);
Dual dual_mul_scalar(Dual a, double b);
Dual dual_div(Dual a, Dual b);
Dual dual_div_scalar(Dual a, double b);
Dual dual_scalar_div(double a, Dual b);
Dual dual_neg(Dual a);
Dual dual_pow(Dual a, double power);

Dual dual_sin(Dual a);
Dual dual_cos(Dual a);
Dual dual_tan(Dual a);
Dual dual_sqrt(Dual a);
Dual dual_exp(Dual a);
Dual dual_log(Dual a);

#endif

