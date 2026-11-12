// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FRENET_PLANNER_CURVES_FRENET_POLYNOMIAL_HPP
#define FRENET_PLANNER_CURVES_FRENET_POLYNOMIAL_HPP

namespace frenet_planner
{

// Lateral offset d(t), t = (s - s_start) / delta_s in [0, 1]. Derivative accessors are d/dt.
struct FrenetPolynomial
{
  double coeffs[6]{};
  double delta_s = 0.0;

  double evaluate(double t) const;
  double evaluateDerivative(double t) const;
  double evaluateSecondDerivative(double t) const;
};

// Quintic with matched position, slope, and curvature at both ends; monotone so max|d| = |D|.
FrenetPolynomial computeQuintic(
  double d_start,
  double slope_start,
  double second_derivative_start,
  double d_end,
  double slope_end,
  double second_derivative_end,
  double delta_s);

// d'' from vehicle curvature and reference geometry (steering-aware start boundary).
double frenetSecondDerivativeForVehicleCurvature(
  double vehicle_curvature,
  double lateral_offset,
  double lateral_slope,
  double reference_curvature,
  double reference_curvature_derivative);

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_CURVES_FRENET_POLYNOMIAL_HPP
