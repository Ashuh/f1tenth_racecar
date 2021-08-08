#include <string>
#include <vector>

#include <ifopt/ipopt_solver.h>
#include <ifopt/variable_set.h>
#include <ifopt/cost_term.h>

#include "local_planner/cubic_spiral_optimizer.h"

/* -------------------------------------------------------------------------- */
/*                                 CubicSpiral                                */
/* -------------------------------------------------------------------------- */

CubicSpiral::CubicSpiral(const Eigen::Vector4d& coeffs, const double length)
{
  a_ = coeffs;
  length_ = length;
}

Path CubicSpiral::toPath(const int num_samples)
{
  std::vector<double> distance;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> curvature;

  double spacing = length_ / (num_samples - 1);

  for (int i = 0; i < num_samples; ++i)
  {
    double s = i * spacing;
    distance.push_back(s);
    x.push_back(getX(s));
    y.push_back(getY(s));
    yaw.push_back(getHeading(s));
    curvature.push_back(getCurvature(s));
  }

  return Path("base_link", distance, x, y, yaw, curvature);
}

// Eigen::VectorXd CubicSpiral::Optimizer::CubicSpiral::sampleX(const Eigen::VectorXd& s_points)
// {
//   Eigen::VectorXd x_points;
//   x_points.resize(s_points.size());

//   for (int i = 0; i < s_points.size(); ++i)
//   {
//     x_points(i) = getX(s_points(i));
//   }

//   return x_points;
// }

// Eigen::VectorXd CubicSpiral::Optimizer::CubicSpiral::sampleY(const Eigen::VectorXd& s_points)
// {
//   Eigen::VectorXd y_points;
//   y_points.resize(s_points.size());

//   for (int i = 0; i < s_points.size(); ++i)
//   {
//     y_points(i) = getY(s_points(i));
//   }

//   return y_points;
// }

// Eigen::VectorXd CubicSpiral::Optimizer::CubicSpiral::sampleHeading(const Eigen::VectorXd& s_points)
// {
//   Eigen::VectorXd headings;
//   headings.resize(s_points.size());

//   for (int i = 0; i < s_points.size(); ++i)
//   {
//     headings(i) = getHeading(s_points(i));
//   }

//   return headings;
// }

double CubicSpiral::getX(const double s)
{
  return (s / 24) *
         (cos(getHeading(0)) + (4 * cos(getHeading(1 * s / 8))) + (2 * cos(getHeading(s / 4))) +
          (4 * cos(getHeading(3 * s / 8))) + (2 * (cos(getHeading(s / 2)))) + (4 * cos(getHeading(5 * s / 8))) +
          (2 * cos(getHeading(3 * s / 4))) + (4 * cos(getHeading(7 * s / 8))) + (cos(getHeading(s))));
}

double CubicSpiral::getY(const double s)
{
  return (s / 24) *
         (sin(getHeading(0)) + (4 * sin(getHeading(1 * s / 8))) + (2 * sin(getHeading(s / 4))) +
          (4 * sin(getHeading(3 * s / 8))) + (2 * (sin(getHeading(s / 2)))) + (4 * sin(getHeading(5 * s / 8))) +
          (2 * sin(getHeading(3 * s / 4))) + (4 * sin(getHeading(7 * s / 8))) + (sin(getHeading(s))));
}

double CubicSpiral::getHeading(const double s)
{
  return (a_(3) * pow(s, 4) / 4) + ((a_(2) * pow(s, 3) / 3) + (a_(1) * pow(s, 2) / 2) + (a_(0) * s));
}

double CubicSpiral::getCurvature(const double s)
{
  return (a_(3) * pow(s, 3)) + (a_(2) * pow(s, 2)) + (a_(1) * s) + a_(0);
}

/* -------------------------------------------------------------------------- */
/*                            CubicSpiralOptimizer                            */
/* -------------------------------------------------------------------------- */

CubicSpiral::Optimizer::Optimizer(double max_curvature) : max_curvature_(max_curvature)
{
}

Path CubicSpiral::Optimizer::generateCubicSpiralPath(const double initial_curvature, const double goal_curvature,
                                                     const double goal_x, const double goal_y,
                                                     const double goal_heading, const unsigned int num_samples)
{
  Eigen::Matrix<double, 5, 1> p =
      optimizeCubicSpiralParams(initial_curvature, goal_curvature, goal_x, goal_y, goal_heading);

  return CubicSpiral(paramsToCoeffs(p), p(4)).toPath(num_samples);
}

Eigen::Matrix<double, 5, 1> CubicSpiral::Optimizer::optimizeCubicSpiralParams(const double initial_curvature,
                                                                              const double goal_curvature,
                                                                              const double goal_x, const double goal_y,
                                                                              const double goal_heading)
{
  double min_dist = sqrt(pow(goal_x, 2) + pow(goal_y, 2));

  ifopt::Problem nlp;
  SimpsonsRuleHelpers helpers;
  nlp.AddVariableSet(
      std::make_shared<CubicSpiralVariableSet>(min_dist, initial_curvature, goal_curvature, max_curvature_, helpers));
  nlp.AddCostSet(
      std::make_shared<CubicSpiralCostTerm>(initial_curvature, goal_curvature, goal_x, goal_y, goal_heading, helpers));

  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("limited_memory_update_type", "bfgs");
  ipopt.SetOption("print_level", 0);
  ipopt.Solve(nlp);
  Eigen::Vector3d vars = nlp.GetOptVariables()->GetValues();
  Eigen::Matrix<double, 5, 1> p;
  p << initial_curvature, vars(0), vars(1), goal_curvature, vars(2);

  return p;
}

Eigen::Vector4d CubicSpiral::Optimizer::paramsToCoeffs(const Eigen::Matrix<double, 5, 1>& p)
{
  Eigen::Vector4d a;
  a(0) = p(0);
  a(1) = -(11.0 * p(0) / 2.0 - 9.0 * p(1) + 9.0 * p(2) / 2.0 - p(3)) / p(4);
  a(2) = (9.0 * p(0) - 45.0 * p(1) / 2.0 + 18.0 * p(2) - 9.0 * p(3) / 2.0) / pow(p(4), 2);
  a(3) = -(9.0 * p(0) / 2.0 - 27.0 * p(1) / 2.0 + 27.0 * p(2) / 2.0 - 9.0 * p(3) / 2.0) / pow(p(4), 3);

  return a;
}

/* -------------------------------------------------------------------------- */
/*                           CubicSpiralVariableSet                           */
/* -------------------------------------------------------------------------- */

CubicSpiral::Optimizer::CubicSpiralVariableSet::CubicSpiralVariableSet(const double min_dist,
                                                                       const double initial_curvature,
                                                                       const double goal_curvature,
                                                                       const double max_curvature,
                                                                       SimpsonsRuleHelpers& helpers)
  : min_dist_(min_dist)
  , p_0_(initial_curvature)
  , p_3_(goal_curvature)
  , max_curvature_(max_curvature)
  , helpers_(helpers)
  , VariableSet(3, "vars")
{
  p_1_ = 0.0;
  p_2_ = 0.0;
  s_f_ = min_dist_;

  Eigen::Matrix<double, 5, 1> p;
  p << p_0_, p_1_, p_2_, p_3_, s_f_;

  // updateCommonTerms(p);
  helpers_ = SimpsonsRuleHelpers(p);
}

void CubicSpiral::Optimizer::CubicSpiralVariableSet::SetVariables(const Eigen::VectorXd& x)
{
  if (p_1_ == x(0) && p_2_ == x(1) && s_f_ == x(2))
  {
    return;
  }

  p_1_ = x(0);
  p_2_ = x(1);
  s_f_ = x(2);

  Eigen::Matrix<double, 5, 1> p;
  p << p_0_, x(0), x(1), p_3_, x(2);
  // updateCommonTerms(p);
  helpers_ = SimpsonsRuleHelpers(p);
}

Eigen::VectorXd CubicSpiral::Optimizer::CubicSpiralVariableSet::GetValues() const
{
  return Eigen::Vector3d(p_1_, p_2_, s_f_);
}

ifopt::Component::VecBound CubicSpiral::Optimizer::CubicSpiralVariableSet::GetBounds() const
{
  ifopt::Component::VecBound bounds(GetRows());
  bounds.at(0) = ifopt::Bounds(-max_curvature_, max_curvature_);
  bounds.at(1) = ifopt::Bounds(-max_curvature_, max_curvature_);
  bounds.at(2) = ifopt::Bounds(min_dist_, ifopt::inf);

  return bounds;
}

/* -------------------------------------------------------------------------- */
/*                             CubicSpiralCostTerm                            */
/* -------------------------------------------------------------------------- */

constexpr unsigned int CubicSpiral::Optimizer::CubicSpiralCostTerm::K_BE_;
constexpr unsigned int CubicSpiral::Optimizer::CubicSpiralCostTerm::K_X_;
constexpr unsigned int CubicSpiral::Optimizer::CubicSpiralCostTerm::K_Y_;
constexpr unsigned int CubicSpiral::Optimizer::CubicSpiralCostTerm::K_HDG_;

CubicSpiral::Optimizer::CubicSpiralCostTerm::CubicSpiralCostTerm(const double initial_curvature,
                                                                 const double goal_curvature, const double goal_x,
                                                                 const double goal_y, const double goal_heading,
                                                                 SimpsonsRuleHelpers& helpers)
  : p_0_(initial_curvature)
  , p_3_(goal_curvature)
  , goal_x_(goal_x)
  , goal_y_(goal_y)
  , goal_heading_(goal_heading)
  , helpers_(helpers)
  , ifopt::CostTerm("cost")
{
}

double CubicSpiral::Optimizer::CubicSpiralCostTerm::bendingEnergyCost(const Eigen::Matrix<double, 5, 1>& p) const
{
  return (p(4) * (64 * pow(p(0), 2) + 99 * p(0) * p(1) - 36 * p(0) * p(2) + 19 * p(0) * p(3) + 324 * pow(p(1), 2) -
                  81 * p(1) * p(2) - 36 * p(1) * p(3) + 324 * pow(p(2), 2) + 99 * p(2) * p(3) + 64 * pow(p(3), 2))) /
         840;
}

Eigen::Vector3d
CubicSpiral::Optimizer::CubicSpiralCostTerm::bendingEnergyCostGrad(const Eigen::Matrix<double, 5, 1>& p) const
{
  double grad_p_1 = (p(4) * (99 * p(0) + 648 * p(1) - 81 * p(2) - 36 * p(3))) / 840;

  double grad_p_2 = -(p(4) * (36 * p(0) + 81 * p(1) - 648 * p(2) - 99 * p(3))) / 840;

  double grad_s_f = (8 * pow(p(0), 2)) / 105 + (33 * p(0) * p(1)) / 280 - (3 * p(0) * p(2)) / 70 +
                    (19 * p(0) * p(3)) / 840 + (27 * pow(p(1), 2)) / 70 - (27 * p(1) * p(2)) / 280 -
                    (3 * p(1) * p(3)) / 70 + (27 * pow(p(2), 2)) / 70 + (33 * p(2) * p(3)) / 280 +
                    (8 * pow(p(3), 2)) / 105;

  return Eigen::Vector3d(grad_p_1, grad_p_2, grad_s_f);
}

double CubicSpiral::Optimizer::CubicSpiralCostTerm::xCost(const Eigen::Matrix<double, 5, 1>& p) const
{
  double x_f =
      (p(4) * (helpers_.cos_yaw8_ + 2 * helpers_.cos_yaw4_ + 2 * helpers_.cos_yaw6_ + 2 * helpers_.cos_yaw2_ +
               4 * helpers_.cos_yaw5_ + 4 * helpers_.cos_yaw7_ + 4 * helpers_.cos_yaw3_ + 4 * helpers_.cos_yaw1_ + 1)) /
      24;

  return pow((goal_x_ - x_f), 2);
}

Eigen::Vector3d CubicSpiral::Optimizer::CubicSpiralCostTerm::xCostGrad(const Eigen::Matrix<double, 5, 1>& p) const
{
  double grad_p_1 =
      -(pow(p(4), 2) *
        (1024 * helpers_.sin_yaw8_ + 2176 * helpers_.sin_yaw4_ + 2376 * helpers_.sin_yaw6_ + 968 * helpers_.sin_yaw2_ +
         4825 * helpers_.sin_yaw5_ + 4361 * helpers_.sin_yaw7_ + 3321 * helpers_.sin_yaw3_ + 617 * helpers_.sin_yaw1_) *
        (p(4) / 24 - goal_x_ + (p(4) * helpers_.cos_yaw4_) / 12 + (p(4) * helpers_.cos_yaw6_) / 12 +
         (p(4) * helpers_.cos_yaw2_) / 12 + (p(4) * helpers_.cos_yaw5_) / 6 + (p(4) * helpers_.cos_yaw7_) / 6 +
         (p(4) * helpers_.cos_yaw3_) / 6 + (p(4) * helpers_.cos_yaw1_) / 6 + (p(4) * helpers_.cos_yaw8_) / 24)) /
      32768;

  double grad_p_2 =
      -(pow(p(4), 2) *
        (1024 * helpers_.sin_yaw8_ - 128 * helpers_.sin_yaw4_ + 1080 * helpers_.sin_yaw6_ - 328 * helpers_.sin_yaw2_ +
         775 * helpers_.sin_yaw5_ + 3479 * helpers_.sin_yaw7_ - 729 * helpers_.sin_yaw3_ - 265 * helpers_.sin_yaw1_) *
        (p(4) / 24 - goal_x_ + (p(4) * helpers_.cos_yaw4_) / 12 + (p(4) * helpers_.cos_yaw6_) / 12 +
         (p(4) * helpers_.cos_yaw2_) / 12 + (p(4) * helpers_.cos_yaw5_) / 6 + (p(4) * helpers_.cos_yaw7_) / 6 +
         (p(4) * helpers_.cos_yaw3_) / 6 + (p(4) * helpers_.cos_yaw1_) / 6 + (p(4) * helpers_.cos_yaw8_) / 24)) /
      32768;

  double grad_s_f =
      2 *
      (helpers_.cos_yaw8_ / 24 + helpers_.cos_yaw4_ / 12 + helpers_.cos_yaw6_ / 12 + helpers_.cos_yaw2_ / 12 +
       helpers_.cos_yaw5_ / 6 + helpers_.cos_yaw7_ / 6 + helpers_.cos_yaw3_ / 6 + helpers_.cos_yaw1_ / 6 -
       (p(4) * (helpers_.sin_yaw8_ * helpers_.d_yaw8_x1_ + helpers_.sin_yaw4_ * helpers_.d_yaw4_x2_ +
                helpers_.sin_yaw2_ * helpers_.d_yaw2_x2_ + helpers_.sin_yaw6_ * helpers_.d_yaw6_x2_ +
                helpers_.sin_yaw1_ * helpers_.d_yaw1_x4_ + helpers_.sin_yaw3_ * helpers_.d_yaw3_x4_ +
                helpers_.sin_yaw5_ * helpers_.d_yaw5_x4_ + helpers_.sin_yaw7_ * helpers_.d_yaw7_x4_)) /
           24 +
       1 / 24.0) *
      (p(4) / 24 - goal_x_ + (p(4) * helpers_.cos_yaw4_) / 12 + (p(4) * helpers_.cos_yaw6_) / 12 +
       (p(4) * helpers_.cos_yaw2_) / 12 + (p(4) * helpers_.cos_yaw5_) / 6 + (p(4) * helpers_.cos_yaw7_) / 6 +
       (p(4) * helpers_.cos_yaw3_) / 6 + (p(4) * helpers_.cos_yaw1_) / 6 + (p(4) * helpers_.cos_yaw8_) / 24);

  return Eigen::Vector3d(grad_p_1, grad_p_2, grad_s_f);
}

double CubicSpiral::Optimizer::CubicSpiralCostTerm::yCost(const Eigen::Matrix<double, 5, 1>& p) const
{
  double y_f =
      (p(4) * (helpers_.sin_yaw8_ + 2 * helpers_.sin_yaw4_ + 2 * helpers_.sin_yaw6_ + 2 * helpers_.sin_yaw2_ +
               4 * helpers_.sin_yaw5_ + 4 * helpers_.sin_yaw7_ + 4 * helpers_.sin_yaw3_ + 4 * helpers_.sin_yaw1_)) /
      24;

  return pow((goal_y_ - y_f), 2);
}

Eigen::Vector3d CubicSpiral::Optimizer::CubicSpiralCostTerm::yCostGrad(const Eigen::Matrix<double, 5, 1>& p) const
{
  double grad_p_1 =
      (pow(p(4), 2) *
       (1024 * helpers_.cos_yaw8_ + 2176 * helpers_.cos_yaw4_ + 2376 * helpers_.cos_yaw6_ + 968 * helpers_.cos_yaw2_ +
        4825 * helpers_.cos_yaw5_ + 4361 * helpers_.cos_yaw7_ + 3321 * helpers_.cos_yaw3_ + 617 * helpers_.cos_yaw1_) *
       ((p(4) * helpers_.sin_yaw4_) / 12 - goal_y_ + (p(4) * helpers_.sin_yaw6_) / 12 +
        (p(4) * helpers_.sin_yaw2_) / 12 + (p(4) * helpers_.sin_yaw5_) / 6 + (p(4) * helpers_.sin_yaw7_) / 6 +
        (p(4) * helpers_.sin_yaw3_) / 6 + (p(4) * helpers_.sin_yaw1_) / 6 + (p(4) * helpers_.sin_yaw8_) / 24)) /
      32768;

  double grad_p_2 =
      (pow(p(4), 2) *
       (1024 * helpers_.cos_yaw8_ - 128 * helpers_.cos_yaw4_ + 1080 * helpers_.cos_yaw6_ - 328 * helpers_.cos_yaw2_ +
        775 * helpers_.cos_yaw5_ + 3479 * helpers_.cos_yaw7_ - 729 * helpers_.cos_yaw3_ - 265 * helpers_.cos_yaw1_) *
       ((p(4) * helpers_.sin_yaw4_) / 12 - goal_y_ + (p(4) * helpers_.sin_yaw6_) / 12 +
        (p(4) * helpers_.sin_yaw2_) / 12 + (p(4) * helpers_.sin_yaw5_) / 6 + (p(4) * helpers_.sin_yaw7_) / 6 +
        (p(4) * helpers_.sin_yaw3_) / 6 + (p(4) * helpers_.sin_yaw1_) / 6 + (p(4) * helpers_.sin_yaw8_) / 24)) /
      32768;

  double grad_s_f =
      2 *
      (helpers_.sin_yaw8_ / 24 + helpers_.sin_yaw4_ / 12 + helpers_.sin_yaw6_ / 12 + helpers_.sin_yaw2_ / 12 +
       helpers_.sin_yaw5_ / 6 + helpers_.sin_yaw7_ / 6 + helpers_.sin_yaw3_ / 6 + helpers_.sin_yaw1_ / 6 +
       (p(4) * (helpers_.cos_yaw8_ * helpers_.d_yaw8_x1_ + helpers_.cos_yaw4_ * helpers_.d_yaw4_x2_ +
                helpers_.cos_yaw2_ * helpers_.d_yaw2_x2_ + helpers_.cos_yaw6_ * helpers_.d_yaw6_x2_ +
                helpers_.cos_yaw1_ * helpers_.d_yaw1_x4_ + helpers_.cos_yaw3_ * helpers_.d_yaw3_x4_ +
                helpers_.cos_yaw5_ * helpers_.d_yaw5_x4_ + helpers_.cos_yaw7_ * helpers_.d_yaw7_x4_)) /
           24) *
      ((p(4) * helpers_.sin_yaw4_) / 12 - goal_y_ + (p(4) * helpers_.sin_yaw6_) / 12 +
       (p(4) * helpers_.sin_yaw2_) / 12 + (p(4) * helpers_.sin_yaw5_) / 6 + (p(4) * helpers_.sin_yaw7_) / 6 +
       (p(4) * helpers_.sin_yaw3_) / 6 + (p(4) * helpers_.sin_yaw1_) / 6 + (p(4) * helpers_.sin_yaw8_) / 24);

  return Eigen::Vector3d(grad_p_1, grad_p_2, grad_s_f);
}

double CubicSpiral::Optimizer::CubicSpiralCostTerm::headingCost(const Eigen::Matrix<double, 5, 1>& p) const
{
  return pow(goal_heading_ - helpers_.yaw8_, 2);
}

Eigen::Vector3d CubicSpiral::Optimizer::CubicSpiralCostTerm::headingCostGrad(const Eigen::Matrix<double, 5, 1>& p) const
{
  double grad_p_1 = -(3 * p(4) * (goal_heading_ - helpers_.yaw8_)) / 4;

  double grad_p_2 = -(3 * p(4) * (goal_heading_ - helpers_.yaw8_)) / 4;

  double grad_s_f = -2 * (goal_heading_ - helpers_.yaw8_) * helpers_.d_yaw8_x1_;

  return Eigen::Vector3d(grad_p_1, grad_p_2, grad_s_f);
}

double CubicSpiral::Optimizer::CubicSpiralCostTerm::GetCost() const
{
  Eigen::Vector3d vars = GetVariables()->GetComponent("vars")->GetValues();
  Eigen::Matrix<double, 5, 1> p;
  p << p_0_, vars(0), vars(1), p_3_, vars(2);

  return (K_BE_ * bendingEnergyCost(p)) + (K_X_ * xCost(p)) + (K_Y_ * yCost(p)) + (K_HDG_ * headingCost(p));
}

void CubicSpiral::Optimizer::CubicSpiralCostTerm::FillJacobianBlock(std::string var_set,
                                                                    ifopt::Component::Jacobian& jac) const
{
  if (var_set == "vars")
  {
    Eigen::Vector3d vars = GetVariables()->GetComponent("vars")->GetValues();
    Eigen::Matrix<double, 5, 1> p;
    p << p_0_, vars(0), vars(1), p_3_, vars(2);

    Eigen::Vector3d grad = (K_BE_ * bendingEnergyCostGrad(p)) + (K_X_ * xCostGrad(p)) + (K_Y_ * yCostGrad(p)) +
                           (K_HDG_ * headingCostGrad(p));

    jac.coeffRef(0, 0) = grad(0);
    jac.coeffRef(0, 1) = grad(1);
    jac.coeffRef(0, 2) = grad(2);
  }
}

/* -------------------------------------------------------------------------- */
/*                             SimpsonsRuleHelpers                            */
/* -------------------------------------------------------------------------- */

CubicSpiral::Optimizer::SimpsonsRuleHelpers::SimpsonsRuleHelpers()
{
}

CubicSpiral::Optimizer::SimpsonsRuleHelpers::SimpsonsRuleHelpers(const Eigen::Matrix<double, 5, 1>& p)
{
  yaw1_ = (p(4) * (2871 * p(0) + 1851 * p(1) - 795 * p(2) + 169 * p(3))) / 32768;
  yaw2_ = (p(4) * (247 * p(0) + 363 * p(1) - 123 * p(2) + 25 * p(3))) / 2048;
  yaw3_ = (3 * p(4) * (1357 * p(0) + 3321 * p(1) - 729 * p(2) + 147 * p(3))) / 32768;
  yaw4_ = (p(4) * (15 * p(0) + 51 * p(1) - 3 * p(2) + p(3))) / 128;
  yaw5_ = (5 * p(4) * (731 * p(0) + 2895 * p(1) + 465 * p(2) + 5 * p(3))) / 32768;
  yaw6_ = (3 * p(4) * (77 * p(0) + 297 * p(1) + 135 * p(2) + 3 * p(3))) / 2048;
  yaw7_ = (7 * p(4) * (561 * p(0) + 1869 * p(1) + 1491 * p(2) + 175 * p(3))) / 32768;
  yaw8_ = (p(4) * (p(0) + 3 * p(1) + 3 * p(2) + p(3))) / 8;

  d_yaw1_x4_ = (2871 * p(0)) / 8192 + (1851 * p(1)) / 8192 - (795 * p(2)) / 8192 + (169 * p(3)) / 8192;
  d_yaw2_x2_ = (247 * p(0)) / 1024 + (363 * p(1)) / 1024 - (123 * p(2)) / 1024 + (25 * p(3)) / 1024;
  d_yaw3_x4_ = (4071 * p(0)) / 8192 + (9963 * p(1)) / 8192 - (2187 * p(2)) / 8192 + (441 * p(3)) / 8192;
  d_yaw4_x2_ = (15 * p(0)) / 64 + (51 * p(1)) / 64 - (3 * p(2)) / 64 + p(3) / 64;
  d_yaw5_x4_ = (3655 * p(0)) / 8192 + (14475 * p(1)) / 8192 + (2325 * p(2)) / 8192 + (25 * p(3)) / 8192;
  d_yaw6_x2_ = (231 * p(0)) / 1024 + (891 * p(1)) / 1024 + (405 * p(2)) / 1024 + (9 * p(3)) / 1024;
  d_yaw7_x4_ = (3927 * p(0)) / 8192 + (13083 * p(1)) / 8192 + (10437 * p(2)) / 8192 + (1225 * p(3)) / 8192;
  d_yaw8_x1_ = p(0) / 8 + (3 * p(1)) / 8 + (3 * p(2)) / 8 + p(3) / 8;

  sin_yaw1_ = sin(yaw1_);
  sin_yaw2_ = sin(yaw2_);
  sin_yaw3_ = sin(yaw3_);
  sin_yaw4_ = sin(yaw4_);
  sin_yaw5_ = sin(yaw5_);
  sin_yaw6_ = sin(yaw6_);
  sin_yaw7_ = sin(yaw7_);
  sin_yaw8_ = sin(yaw8_);

  cos_yaw1_ = cos(yaw1_);
  cos_yaw2_ = cos(yaw2_);
  cos_yaw3_ = cos(yaw3_);
  cos_yaw4_ = cos(yaw4_);
  cos_yaw5_ = cos(yaw5_);
  cos_yaw6_ = cos(yaw6_);
  cos_yaw7_ = cos(yaw7_);
  cos_yaw8_ = cos(yaw8_);
}
