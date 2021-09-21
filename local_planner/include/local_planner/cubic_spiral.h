#ifndef LOCAL_PLANNER_CUBIC_SPIRAL_H
#define LOCAL_PLANNER_CUBIC_SPIRAL_H

#include <string>

#include <ifopt/ipopt_solver.h>
#include <ifopt/variable_set.h>
#include <ifopt/cost_term.h>

#include "local_planner/path.h"

class CubicSpiral
{
private:
  Eigen::Vector4d a_;

  double length_;

  // Eigen::VectorXd sampleX(const Eigen::VectorXd& s_points);

  // Eigen::VectorXd sampleY(const Eigen::VectorXd& s_points);

  // Eigen::VectorXd sampleHeading(const Eigen::VectorXd& s_points);

  double getX(const double s);

  double getY(const double s);

  double getHeading(const double s);

  double getCurvature(const double s);

public:
  CubicSpiral(const Eigen::Vector4d& coeffs, const double length);

  Path toPath(const int num_samples);

  class AbstractOptimizer
  {
  public:
    explicit AbstractOptimizer(const double max_curvature);

    virtual CubicSpiral optimizeCubicSpiral(const double initial_curvature, const double goal_curvature,
                                            const double goal_x, const double goal_y, const double goal_yaw) const = 0;

  protected:
    double max_curvature_;

    static constexpr unsigned int K_BE_ = 1;
    static constexpr unsigned int K_X_ = 25;
    static constexpr unsigned int K_Y_ = 25;
    static constexpr unsigned int K_HDG_ = 30;

    struct SimpsonsRuleHelpers;

    static Eigen::Vector4d paramsToCoeffs(const Eigen::Matrix<double, 5, 1>& p);

    static double bendingEnergyCost(const Eigen::Matrix<double, 5, 1>& p);

    static double xCost(const Eigen::Matrix<double, 5, 1>& p, const double goal_x);

    static double xCost(const double p4, const SimpsonsRuleHelpers& h, const double goal_x);

    static double yCost(const Eigen::Matrix<double, 5, 1>& p, const double goal_y);

    static double yCost(const double p4, const SimpsonsRuleHelpers& h, const double goal_y);

    static double yawCost(const Eigen::Matrix<double, 5, 1>& p, const double goal_yaw);

    static double yawCost(const double yaw_f, const double goal_yaw);

    static Eigen::Vector3d bendingEnergyCostGrad(const Eigen::Matrix<double, 5, 1>& p);

    static Eigen::Vector3d xCostGrad(const Eigen::Matrix<double, 5, 1>& p, const double goal_x);

    static Eigen::Vector3d xCostGrad(const double p4, const SimpsonsRuleHelpers& h, const double goal_x);

    static Eigen::Vector3d yCostGrad(const Eigen::Matrix<double, 5, 1>& p, const double goal_y);

    static Eigen::Vector3d yCostGrad(const double p4, const SimpsonsRuleHelpers& h, const double goal_y);

    static Eigen::Vector3d yawCostGrad(const Eigen::Matrix<double, 5, 1>& p, const double goal_yaw);

    static Eigen::Vector3d yawCostGrad(const double p4, const SimpsonsRuleHelpers& h, const double goal_yaw);

    struct SimpsonsRuleHelpers
    {
      double yaw1_;
      double yaw2_;
      double yaw3_;
      double yaw4_;
      double yaw5_;
      double yaw6_;
      double yaw7_;
      double yaw8_;

      double sin_yaw1_;
      double sin_yaw2_;
      double sin_yaw3_;
      double sin_yaw4_;
      double sin_yaw5_;
      double sin_yaw6_;
      double sin_yaw7_;
      double sin_yaw8_;

      double cos_yaw1_;
      double cos_yaw2_;
      double cos_yaw3_;
      double cos_yaw4_;
      double cos_yaw5_;
      double cos_yaw6_;
      double cos_yaw7_;
      double cos_yaw8_;

      double d_yaw1_x4_;
      double d_yaw2_x2_;
      double d_yaw3_x4_;
      double d_yaw4_x2_;
      double d_yaw5_x4_;
      double d_yaw6_x2_;
      double d_yaw7_x4_;
      double d_yaw8_x1_;

      SimpsonsRuleHelpers();

      explicit SimpsonsRuleHelpers(const Eigen::Matrix<double, 5, 1>& p);
    };
  };

  class OptimizerIFOPT : public AbstractOptimizer
  {
  public:
    explicit OptimizerIFOPT(double max_curvature);

    CubicSpiral optimizeCubicSpiral(const double initial_curvature, const double goal_curvature, const double goal_x,
                                    const double goal_y, const double goal_heading) const override;

  private:
    class CubicSpiralVariableSet : public ifopt::VariableSet
    {
    public:
      CubicSpiralVariableSet(const double min_dist, const double initial_curvature, const double goal_curvature,
                             const double max_curvature, SimpsonsRuleHelpers& helpers);

      void SetVariables(const Eigen::VectorXd& vars) override;

      Eigen::VectorXd GetValues() const override;

      ifopt::Component::VecBound GetBounds() const override;

    private:
      const double min_dist_;
      const double max_curvature_;

      const double p_0_;  // curvature at s = 0
      double p_1_;        // curvature at s = (1 / 3) * s_f_
      double p_2_;        // curvature at s = (2 / 3) * s_f_
      const double p_3_;  // curvature at s =  s_f_
      double s_f_;        // length

      SimpsonsRuleHelpers& helpers_;
    };

    class CubicSpiralCostTerm : public ifopt::CostTerm
    {
    public:
      CubicSpiralCostTerm(const double initial_curvature, const double goal_curvature, const double goal_x,
                          const double goal_y, const double goal_yaw, SimpsonsRuleHelpers& common_terms);

      double GetCost() const override;

      void FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& jac) const override;

    private:
      const double p_0_;  // curvature at s = 0
      const double p_3_;  // curvature at s =  s_f_
      const double goal_x_;
      const double goal_y_;
      const double goal_yaw_;

      SimpsonsRuleHelpers& helpers_;
    };
  };

  class OptimizerNLOPT : public AbstractOptimizer
  {
  public:
    explicit OptimizerNLOPT(double max_curvature);

    CubicSpiral optimizeCubicSpiral(const double initial_curvature, const double goal_curvature, const double goal_x,
                                    const double goal_y, const double goal_heading) const override;

    struct Constraints
    {
      double curvature_i_;
      double curvature_f_;
      double x_f_;
      double y_f_;
      double yaw_f_;

      Constraints(const double curvature_i, const double curvature_f, const double x_f, const double y_f,
                  const double yaw_f);
    };

  private:
    static double objective(const std::vector<double>& x, std::vector<double>& grad, void* f_data);
  };
};

#endif  // LOCAL_PLANNER_CUBIC_SPIRAL_H
