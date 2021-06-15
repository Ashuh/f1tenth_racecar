#include <string>
#include <vector>

#include <ifopt/ipopt_solver.h>
#include <ifopt/variable_set.h>
#include <ifopt/cost_term.h>

#include "local_planner/cubic_spiral_optimizer.h"

/* -------------------------------------------------------------------------- */
/*                            CubicSpiralOptimizer                            */
/* -------------------------------------------------------------------------- */

CubicSpiralOptimizer::CubicSpiralOptimizer(double max_curvature) : max_curvature_(max_curvature)
{
}

Path CubicSpiralOptimizer::generateCubicSpiralPath(const double goal_x, const double goal_y, const double goal_heading,
                                                   const unsigned int num_samples)
{
  Eigen::Matrix<double, 5, 1> p = optimizeCubicSpiralParams(goal_x, goal_y, goal_heading);
  Eigen::Vector4d coeffs = paramsToCoeffs(p);
  CubicSpiral spiral(coeffs);

  Eigen::VectorXd s_points;
  s_points.resize(num_samples);
  double delta_s = p(4) / (num_samples - 1);

  for (int i = 0; i < num_samples; ++i)
  {
    s_points(i) = i * delta_s;
  }

  Path path = spiral.sampleCubicSpiral(s_points);

  return path;
}

Eigen::Matrix<double, 5, 1> CubicSpiralOptimizer::optimizeCubicSpiralParams(const double goal_x, const double goal_y,
                                                                            const double goal_heading)
{
  double min_dist = sqrt(pow(goal_x, 2) + pow(goal_y, 2));

  ifopt::Problem nlp;
  CommonTerms common_terms;
  nlp.AddVariableSet(std::make_shared<CubicSpiralVariableSet>(min_dist, max_curvature_, common_terms));
  nlp.AddCostSet(std::make_shared<CubicSpiralCostTerm>(goal_x, goal_y, goal_heading, common_terms));

  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("limited_memory_update_type", "bfgs");
  ipopt.SetOption("print_level", 0);
  ipopt.Solve(nlp);
  Eigen::Vector3d vars = nlp.GetOptVariables()->GetValues();
  Eigen::Matrix<double, 5, 1> p;
  p << 0.0, vars(0), vars(1), 0.0, vars(2);

  return p;
}

Eigen::Vector4d CubicSpiralOptimizer::paramsToCoeffs(const Eigen::Matrix<double, 5, 1> p)
{
  Eigen::Vector4d a;
  a(0) = p(0);
  a(1) = -(11.0 * p(0) / 2.0 - 9.0 * p(1) + 9.0 * p(2) / 2.0 - p(3)) / p(4);
  a(2) = (9.0 * p(0) - 45.0 * p(1) / 2.0 + 18.0 * p(2) - 9.0 * p(3) / 2.0) / pow(p(4), 2);
  a(3) = -(9.0 * p(0) / 2.0 - 27.0 * p(1) / 2.0 + 27.0 * p(2) / 2.0 - 9.0 * p(3) / 2.0) / pow(p(4), 3);

  return a;
}

/* -------------------------------------------------------------------------- */
/*                                 CubicSpiral                                */
/* -------------------------------------------------------------------------- */

CubicSpiralOptimizer::CubicSpiral::CubicSpiral(const Eigen::Vector4d coeffs)
{
  a_ = coeffs;
}

Path CubicSpiralOptimizer::CubicSpiral::sampleCubicSpiral(const Eigen::VectorXd& s_points)
{
  std::vector<Waypoint> waypoints;

  std::vector<double> distance;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> curvature;

  for (int i = 0; i < s_points.size(); ++i)
  {
    // Waypoint wp;
    distance.push_back(s_points(i));
    x.push_back(getX(s_points(i)));
    y.push_back(getY(s_points(i)));
    yaw.push_back(getHeading(s_points(i)));
    curvature.push_back(getCurvature(s_points(i)));

    // wp.x_ = getX(s_points(i));
    // wp.y_ = getY(s_points(i));
    // wp.yaw_ = getHeading(s_points(i));
    // wp.curvature_ = getCurvature(s_points(i));
    // wp.distance_ = s_points(i);

    // waypoints.push_back(wp);
  }

  return Path("base_link", distance, x, y, yaw, curvature);

  // return CubicSpiralPath("base_link", waypoints);
}

// Eigen::VectorXd CubicSpiralOptimizer::CubicSpiral::sampleX(const Eigen::VectorXd& s_points)
// {
//   Eigen::VectorXd x_points;
//   x_points.resize(s_points.size());

//   for (int i = 0; i < s_points.size(); ++i)
//   {
//     x_points(i) = getX(s_points(i));
//   }

//   return x_points;
// }

// Eigen::VectorXd CubicSpiralOptimizer::CubicSpiral::sampleY(const Eigen::VectorXd& s_points)
// {
//   Eigen::VectorXd y_points;
//   y_points.resize(s_points.size());

//   for (int i = 0; i < s_points.size(); ++i)
//   {
//     y_points(i) = getY(s_points(i));
//   }

//   return y_points;
// }

// Eigen::VectorXd CubicSpiralOptimizer::CubicSpiral::sampleHeading(const Eigen::VectorXd& s_points)
// {
//   Eigen::VectorXd headings;
//   headings.resize(s_points.size());

//   for (int i = 0; i < s_points.size(); ++i)
//   {
//     headings(i) = getHeading(s_points(i));
//   }

//   return headings;
// }

double CubicSpiralOptimizer::CubicSpiral::getX(const double s)
{
  return (s / 24) *
         (cos(getHeading(0)) + (4 * cos(getHeading(1 * s / 8))) + (2 * cos(getHeading(s / 4))) +
          (4 * cos(getHeading(3 * s / 8))) + (2 * (cos(getHeading(s / 2)))) + (4 * cos(getHeading(5 * s / 8))) +
          (2 * cos(getHeading(3 * s / 4))) + (4 * cos(getHeading(7 * s / 8))) + (cos(getHeading(s))));
}

double CubicSpiralOptimizer::CubicSpiral::getY(const double s)
{
  return (s / 24) *
         (sin(getHeading(0)) + (4 * sin(getHeading(1 * s / 8))) + (2 * sin(getHeading(s / 4))) +
          (4 * sin(getHeading(3 * s / 8))) + (2 * (sin(getHeading(s / 2)))) + (4 * sin(getHeading(5 * s / 8))) +
          (2 * sin(getHeading(3 * s / 4))) + (4 * sin(getHeading(7 * s / 8))) + (sin(getHeading(s))));
}

double CubicSpiralOptimizer::CubicSpiral::getHeading(const double s)
{
  return (a_(3) * pow(s, 4) / 4) + ((a_(2) * pow(s, 3) / 3) + (a_(1) * pow(s, 2) / 2) + (a_(0) * s));
}

double CubicSpiralOptimizer::CubicSpiral::getCurvature(const double s)
{
  return (a_(3) * pow(s, 3)) + (a_(2) * pow(s, 2)) + (a_(1) * s) + a_(0);
}

/* -------------------------------------------------------------------------- */
/*                           CubicSpiralVariableSet                           */
/* -------------------------------------------------------------------------- */

CubicSpiralOptimizer::CubicSpiralVariableSet::CubicSpiralVariableSet(const double min_dist, const double max_curvature,
                                                                     CommonTerms& common_terms)
  : min_dist_(min_dist), max_curvature_(max_curvature), common_terms_(common_terms), VariableSet(3, "vars")
{
  p_1_ = 0;
  p_2_ = 0;
  s_f_ = min_dist;

  Eigen::Matrix<double, 5, 1> p;
  p << 0.0, p_1_, p_2_, 0.0, s_f_;

  updateCommonTerms(p);
}

void CubicSpiralOptimizer::CubicSpiralVariableSet::SetVariables(const Eigen::VectorXd& x)
{
  if (p_1_ == x(0) && p_2_ == x(1) && s_f_ == x(2))
  {
    return;
  }

  p_1_ = x(0);
  p_2_ = x(1);
  s_f_ = x(2);

  Eigen::Matrix<double, 5, 1> p;
  p << 0.0, x(0), x(1), 0.0, x(2);
  updateCommonTerms(p);
}

void CubicSpiralOptimizer::CubicSpiralVariableSet::updateCommonTerms(const Eigen::Matrix<double, 5, 1>& p)
{
  common_terms_.t_1_ = (p(4) * (p(0) + 3 * p(1) + 3 * p(2) + p(3))) / 8;
  common_terms_.t_2_ = (p(4) * (15 * p(0) + 51 * p(1) - 3 * p(2) + p(3))) / 128;
  common_terms_.t_3_ = (3 * p(4) * (77 * p(0) + 297 * p(1) + 135 * p(2) + 3 * p(3))) / 2048;
  common_terms_.t_4_ = (p(4) * (247 * p(0) + 363 * p(1) - 123 * p(2) + 25 * p(3))) / 2048;
  common_terms_.t_5_ = (5 * p(4) * (731 * p(0) + 2895 * p(1) + 465 * p(2) + 5 * p(3))) / 32768;
  common_terms_.t_6_ = (7 * p(4) * (561 * p(0) + 1869 * p(1) + 1491 * p(2) + 175 * p(3))) / 32768;
  common_terms_.t_7_ = (3 * p(4) * (1357 * p(0) + 3321 * p(1) - 729 * p(2) + 147 * p(3))) / 32768;
  common_terms_.t_8_ = (p(4) * (2871 * p(0) + 1851 * p(1) - 795 * p(2) + 169 * p(3))) / 32768;
  common_terms_.t_9_ = p(0) / 8 + (3 * p(1)) / 8 + (3 * p(2)) / 8 + p(3) / 8;
  common_terms_.t_10_ = (15 * p(0)) / 64 + (51 * p(1)) / 64 - (3 * p(2)) / 64 + p(3) / 64;
  common_terms_.t_11_ = (247 * p(0)) / 1024 + (363 * p(1)) / 1024 - (123 * p(2)) / 1024 + (25 * p(3)) / 1024;
  common_terms_.t_12_ = (231 * p(0)) / 1024 + (891 * p(1)) / 1024 + (405 * p(2)) / 1024 + (9 * p(3)) / 1024;
  common_terms_.t_13_ = (2871 * p(0)) / 8192 + (1851 * p(1)) / 8192 - (795 * p(2)) / 8192 + (169 * p(3)) / 8192;
  common_terms_.t_14_ = (4071 * p(0)) / 8192 + (9963 * p(1)) / 8192 - (2187 * p(2)) / 8192 + (441 * p(3)) / 8192;
  common_terms_.t_15_ = (3655 * p(0)) / 8192 + (14475 * p(1)) / 8192 + (2325 * p(2)) / 8192 + (25 * p(3)) / 8192;
  common_terms_.t_16_ = (3927 * p(0)) / 8192 + (13083 * p(1)) / 8192 + (10437 * p(2)) / 8192 + (1225 * p(3)) / 8192;

  common_terms_.sin_t_1_ = sin(common_terms_.t_1_);
  common_terms_.sin_t_2_ = sin(common_terms_.t_2_);
  common_terms_.sin_t_3_ = sin(common_terms_.t_3_);
  common_terms_.sin_t_4_ = sin(common_terms_.t_4_);
  common_terms_.sin_t_5_ = sin(common_terms_.t_5_);
  common_terms_.sin_t_6_ = sin(common_terms_.t_6_);
  common_terms_.sin_t_7_ = sin(common_terms_.t_7_);
  common_terms_.sin_t_8_ = sin(common_terms_.t_8_);

  common_terms_.cos_t_1_ = cos(common_terms_.t_1_);
  common_terms_.cos_t_2_ = cos(common_terms_.t_2_);
  common_terms_.cos_t_3_ = cos(common_terms_.t_3_);
  common_terms_.cos_t_4_ = cos(common_terms_.t_4_);
  common_terms_.cos_t_5_ = cos(common_terms_.t_5_);
  common_terms_.cos_t_6_ = cos(common_terms_.t_6_);
  common_terms_.cos_t_7_ = cos(common_terms_.t_7_);
  common_terms_.cos_t_8_ = cos(common_terms_.t_8_);
}

Eigen::VectorXd CubicSpiralOptimizer::CubicSpiralVariableSet::GetValues() const
{
  return Eigen::Vector3d(p_1_, p_2_, s_f_);
}

ifopt::Component::VecBound CubicSpiralOptimizer::CubicSpiralVariableSet::GetBounds() const
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

CubicSpiralOptimizer::CubicSpiralCostTerm::CubicSpiralCostTerm(const double goal_x, const double goal_y,
                                                               const double goal_heading, CommonTerms& common_terms)
  : goal_x_(goal_x), goal_y_(goal_y), goal_heading_(goal_heading), common_terms_(common_terms), ifopt::CostTerm("cost")
{
}

double CubicSpiralOptimizer::CubicSpiralCostTerm::bendingEnergyCost(const Eigen::Matrix<double, 5, 1>& p) const
{
  return (p(4) * (64 * pow(p(0), 2) + 99 * p(0) * p(1) - 36 * p(0) * p(2) + 19 * p(0) * p(3) + 324 * pow(p(1), 2) -
                  81 * p(1) * p(2) - 36 * p(1) * p(3) + 324 * pow(p(2), 2) + 99 * p(2) * p(3) + 64 * pow(p(3), 2))) /
         840;
}

Eigen::Vector3d
CubicSpiralOptimizer::CubicSpiralCostTerm::bendingEnergyCostGrad(const Eigen::Matrix<double, 5, 1>& p) const
{
  Eigen::Vector3d grad;

  grad(0) = (p(4) * (99 * p(0) + 648 * p(1) - 81 * p(2) - 36 * p(3))) / 840;

  grad(1) = -(p(4) * (36 * p(0) + 81 * p(1) - 648 * p(2) - 99 * p(3))) / 840;

  grad(2) = (8 * pow(p(0), 2)) / 105 + (33 * p(0) * p(1)) / 280 - (3 * p(0) * p(2)) / 70 + (19 * p(0) * p(3)) / 840 +
            (27 * pow(p(1), 2)) / 70 - (27 * p(1) * p(2)) / 280 - (3 * p(1) * p(3)) / 70 + (27 * pow(p(2), 2)) / 70 +
            (33 * p(2) * p(3)) / 280 + (8 * pow(p(3), 2)) / 105;

  return grad;
}

double CubicSpiralOptimizer::CubicSpiralCostTerm::xCost(const Eigen::Matrix<double, 5, 1>& p) const
{
  double x_f = (p(4) * (cos(common_terms_.t_1_) + 2 * cos(common_terms_.t_2_) + 2 * cos(common_terms_.t_3_) +
                        2 * cos(common_terms_.t_4_) + 4 * cos(common_terms_.t_5_) + 4 * cos(common_terms_.t_6_) +
                        4 * cos(common_terms_.t_7_) + 4 * cos(common_terms_.t_8_) + 1)) /
               24;

  return pow((goal_x_ - x_f), 2);
}

Eigen::Vector3d CubicSpiralOptimizer::CubicSpiralCostTerm::xCostGrad(const Eigen::Matrix<double, 5, 1>& p) const
{
  Eigen::Vector3d grad;

  grad(0) = -(pow(p(4), 2) *
              (1024 * common_terms_.sin_t_1_ + 2176 * common_terms_.sin_t_2_ + 2376 * common_terms_.sin_t_3_ +
               968 * common_terms_.sin_t_4_ + 4825 * common_terms_.sin_t_5_ + 4361 * common_terms_.sin_t_6_ +
               3321 * common_terms_.sin_t_7_ + 617 * common_terms_.sin_t_8_) *
              (p(4) / 24 - goal_x_ + (p(4) * common_terms_.cos_t_2_) / 12 + (p(4) * common_terms_.cos_t_3_) / 12 +
               (p(4) * common_terms_.cos_t_4_) / 12 + (p(4) * common_terms_.cos_t_5_) / 6 +
               (p(4) * common_terms_.cos_t_6_) / 6 + (p(4) * common_terms_.cos_t_7_) / 6 +
               (p(4) * common_terms_.cos_t_8_) / 6 + (p(4) * common_terms_.cos_t_1_) / 24)) /
            32768;

  grad(1) = -(pow(p(4), 2) *
              (1024 * common_terms_.sin_t_1_ - 128 * common_terms_.sin_t_2_ + 1080 * common_terms_.sin_t_3_ -
               328 * common_terms_.sin_t_4_ + 775 * common_terms_.sin_t_5_ + 3479 * common_terms_.sin_t_6_ -
               729 * common_terms_.sin_t_7_ - 265 * common_terms_.sin_t_8_) *
              (p(4) / 24 - goal_x_ + (p(4) * common_terms_.cos_t_2_) / 12 + (p(4) * common_terms_.cos_t_3_) / 12 +
               (p(4) * common_terms_.cos_t_4_) / 12 + (p(4) * common_terms_.cos_t_5_) / 6 +
               (p(4) * common_terms_.cos_t_6_) / 6 + (p(4) * common_terms_.cos_t_7_) / 6 +
               (p(4) * common_terms_.cos_t_8_) / 6 + (p(4) * common_terms_.cos_t_1_) / 24)) /
            32768;

  grad(2) = 2 *
            (common_terms_.cos_t_1_ / 24 + common_terms_.cos_t_2_ / 12 + common_terms_.cos_t_3_ / 12 +
             common_terms_.cos_t_4_ / 12 + common_terms_.cos_t_5_ / 6 + common_terms_.cos_t_6_ / 6 +
             common_terms_.cos_t_7_ / 6 + common_terms_.cos_t_8_ / 6 -
             (p(4) * (common_terms_.sin_t_1_ * (p(0) / 8 + (3 * p(1)) / 8 + (3 * p(2)) / 8 + p(3) / 8) +
                      common_terms_.sin_t_2_ * ((15 * p(0)) / 64 + (51 * p(1)) / 64 - (3 * p(2)) / 64 + p(3) / 64) +
                      common_terms_.sin_t_4_ *
                          ((247 * p(0)) / 1024 + (363 * p(1)) / 1024 - (123 * p(2)) / 1024 + (25 * p(3)) / 1024) +
                      common_terms_.sin_t_3_ *
                          ((231 * p(0)) / 1024 + (891 * p(1)) / 1024 + (405 * p(2)) / 1024 + (9 * p(3)) / 1024) +
                      common_terms_.sin_t_8_ *
                          ((2871 * p(0)) / 8192 + (1851 * p(1)) / 8192 - (795 * p(2)) / 8192 + (169 * p(3)) / 8192) +
                      common_terms_.sin_t_7_ *
                          ((4071 * p(0)) / 8192 + (9963 * p(1)) / 8192 - (2187 * p(2)) / 8192 + (441 * p(3)) / 8192) +
                      common_terms_.sin_t_5_ *
                          ((3655 * p(0)) / 8192 + (14475 * p(1)) / 8192 + (2325 * p(2)) / 8192 + (25 * p(3)) / 8192) +
                      common_terms_.sin_t_6_ * ((3927 * p(0)) / 8192 + (13083 * p(1)) / 8192 + (10437 * p(2)) / 8192 +
                                                (1225 * p(3)) / 8192))) /
                 24 +
             1 / 24.0) *
            (p(4) / 24 - goal_x_ + (p(4) * common_terms_.cos_t_2_) / 12 + (p(4) * common_terms_.cos_t_3_) / 12 +
             (p(4) * common_terms_.cos_t_4_) / 12 + (p(4) * common_terms_.cos_t_5_) / 6 +
             (p(4) * common_terms_.cos_t_6_) / 6 + (p(4) * common_terms_.cos_t_7_) / 6 +
             (p(4) * common_terms_.cos_t_8_) / 6 + (p(4) * common_terms_.cos_t_1_) / 24);

  return grad;
}

double CubicSpiralOptimizer::CubicSpiralCostTerm::yCost(const Eigen::Matrix<double, 5, 1>& p) const
{
  double y_f = (p(4) * (common_terms_.sin_t_1_ + 2 * common_terms_.sin_t_2_ + 2 * common_terms_.sin_t_3_ +
                        2 * common_terms_.sin_t_4_ + 4 * common_terms_.sin_t_5_ + 4 * common_terms_.sin_t_6_ +
                        4 * common_terms_.sin_t_7_ + 4 * common_terms_.sin_t_8_)) /
               24;

  return pow((goal_y_ - y_f), 2);
}

Eigen::Vector3d CubicSpiralOptimizer::CubicSpiralCostTerm::yCostGrad(const Eigen::Matrix<double, 5, 1>& p) const
{
  Eigen::Vector3d grad;

  grad(0) = (pow(p(4), 2) *
             (1024 * common_terms_.cos_t_1_ + 2176 * common_terms_.cos_t_2_ + 2376 * common_terms_.cos_t_3_ +
              968 * common_terms_.cos_t_4_ + 4825 * common_terms_.cos_t_5_ + 4361 * common_terms_.cos_t_6_ +
              3321 * common_terms_.cos_t_7_ + 617 * common_terms_.cos_t_8_) *
             ((p(4) * common_terms_.sin_t_2_) / 12 - goal_y_ + (p(4) * common_terms_.sin_t_3_) / 12 +
              (p(4) * common_terms_.sin_t_4_) / 12 + (p(4) * common_terms_.sin_t_5_) / 6 +
              (p(4) * common_terms_.sin_t_6_) / 6 + (p(4) * common_terms_.sin_t_7_) / 6 +
              (p(4) * common_terms_.sin_t_8_) / 6 + (p(4) * common_terms_.sin_t_1_) / 24)) /
            32768;

  grad(1) = (pow(p(4), 2) *
             (1024 * common_terms_.cos_t_1_ - 128 * common_terms_.cos_t_2_ + 1080 * common_terms_.cos_t_3_ -
              328 * common_terms_.cos_t_4_ + 775 * common_terms_.cos_t_5_ + 3479 * common_terms_.cos_t_6_ -
              729 * common_terms_.cos_t_7_ - 265 * common_terms_.cos_t_8_) *
             ((p(4) * common_terms_.sin_t_2_) / 12 - goal_y_ + (p(4) * common_terms_.sin_t_3_) / 12 +
              (p(4) * common_terms_.sin_t_4_) / 12 + (p(4) * common_terms_.sin_t_5_) / 6 +
              (p(4) * common_terms_.sin_t_6_) / 6 + (p(4) * common_terms_.sin_t_7_) / 6 +
              (p(4) * common_terms_.sin_t_8_) / 6 + (p(4) * common_terms_.sin_t_1_) / 24)) /
            32768;

  grad(2) = 2 *
            (common_terms_.sin_t_1_ / 24 + common_terms_.sin_t_2_ / 12 + common_terms_.sin_t_3_ / 12 +
             common_terms_.sin_t_4_ / 12 + common_terms_.sin_t_5_ / 6 + common_terms_.sin_t_6_ / 6 +
             common_terms_.sin_t_7_ / 6 + common_terms_.sin_t_8_ / 6 +
             (p(4) * (common_terms_.cos_t_1_ * (p(0) / 8 + (3 * p(1)) / 8 + (3 * p(2)) / 8 + p(3) / 8) +
                      common_terms_.cos_t_2_ * ((15 * p(0)) / 64 + (51 * p(1)) / 64 - (3 * p(2)) / 64 + p(3) / 64) +
                      common_terms_.cos_t_4_ *
                          ((247 * p(0)) / 1024 + (363 * p(1)) / 1024 - (123 * p(2)) / 1024 + (25 * p(3)) / 1024) +
                      common_terms_.cos_t_3_ *
                          ((231 * p(0)) / 1024 + (891 * p(1)) / 1024 + (405 * p(2)) / 1024 + (9 * p(3)) / 1024) +
                      common_terms_.cos_t_8_ *
                          ((2871 * p(0)) / 8192 + (1851 * p(1)) / 8192 - (795 * p(2)) / 8192 + (169 * p(3)) / 8192) +
                      common_terms_.cos_t_7_ *
                          ((4071 * p(0)) / 8192 + (9963 * p(1)) / 8192 - (2187 * p(2)) / 8192 + (441 * p(3)) / 8192) +
                      common_terms_.cos_t_5_ *
                          ((3655 * p(0)) / 8192 + (14475 * p(1)) / 8192 + (2325 * p(2)) / 8192 + (25 * p(3)) / 8192) +
                      common_terms_.cos_t_6_ * ((3927 * p(0)) / 8192 + (13083 * p(1)) / 8192 + (10437 * p(2)) / 8192 +
                                                (1225 * p(3)) / 8192))) /
                 24) *
            ((p(4) * common_terms_.sin_t_2_) / 12 - goal_y_ + (p(4) * common_terms_.sin_t_3_) / 12 +
             (p(4) * common_terms_.sin_t_4_) / 12 + (p(4) * common_terms_.sin_t_5_) / 6 +
             (p(4) * common_terms_.sin_t_6_) / 6 + (p(4) * common_terms_.sin_t_7_) / 6 +
             (p(4) * common_terms_.sin_t_8_) / 6 + (p(4) * common_terms_.sin_t_1_) / 24);

  return grad;
}

double CubicSpiralOptimizer::CubicSpiralCostTerm::headingCost(const Eigen::Matrix<double, 5, 1>& p) const
{
  return pow(goal_heading_ - common_terms_.t_1_, 2);
}

Eigen::Vector3d CubicSpiralOptimizer::CubicSpiralCostTerm::headingCostGrad(const Eigen::Matrix<double, 5, 1>& p) const
{
  Eigen::Vector3d grad;

  grad(0) = -(3 * p(4) * (goal_heading_ - common_terms_.t_1_)) / 4;

  grad(1) = -(3 * p(4) * (goal_heading_ - common_terms_.t_1_)) / 4;

  grad(2) = -2 * (goal_heading_ - common_terms_.t_1_) * (p(0) / 8 + (3 * p(1)) / 8 + (3 * p(2)) / 8 + p(3) / 8);

  return grad;
}

double CubicSpiralOptimizer::CubicSpiralCostTerm::GetCost() const
{
  Eigen::Vector3d vars = GetVariables()->GetComponent("vars")->GetValues();
  Eigen::Matrix<double, 5, 1> p;
  p << 0.0, vars(0), vars(1), 0.0, vars(2);

  return (K_BE_ * bendingEnergyCost(p)) + (K_X_ * xCost(p)) + (K_Y_ * yCost(p)) + (K_HDG_ * headingCost(p));
}

void CubicSpiralOptimizer::CubicSpiralCostTerm::FillJacobianBlock(std::string var_set,
                                                                  ifopt::Component::Jacobian& jac) const
{
  if (var_set == "vars")
  {
    Eigen::Vector3d vars = GetVariables()->GetComponent("vars")->GetValues();
    Eigen::Matrix<double, 5, 1> p;
    p << 0.0, vars(0), vars(1), 0.0, vars(2);

    Eigen::Vector3d grad = (K_BE_ * bendingEnergyCostGrad(p)) + (K_X_ * xCostGrad(p)) + (K_Y_ * yCostGrad(p)) +
                           (K_HDG_ * headingCostGrad(p));

    jac.coeffRef(0, 0) = grad(0);
    jac.coeffRef(0, 1) = grad(1);
    jac.coeffRef(0, 2) = grad(2);
  }
}
