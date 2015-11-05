#ifndef INTERPOLATIONMB_H
#define INTERPOLATIONMB_H

#include <iostream>

inline double interpolatormb(double t, double td)
{
  double coefficients[6];
  double interpolation = 0.0;
  coefficients[0] = 0.00000309;
  coefficients[1] = 0.00519100;
  coefficients[2] = -0.5098000;
  coefficients[3] = 11.9900000;
  coefficients[4] = -17.470000;
  coefficients[5] = 6.98900000;

  if (t < 0.0 )
  {
    return 0.0;
  }

  if (t >= td)
  {
    return 1.0;
  }

  interpolation = coefficients[5] * std::pow(t/td,5) +
                  coefficients[4] * std::pow(t/td,4) +
                  coefficients[3] * std::pow(t/td,3) +
                  coefficients[2] * std::pow(t/td,2) +
                  coefficients[1] * t +
                  coefficients[0];

  if (interpolation > 1)
    return 1.0;
  
  return interpolation;
}

#endif