#ifndef LOCAL_PLANNER_CUBIC_SPIRAL_OPTIMIZER_H
#define LOCAL_PLANNER_CUBIC_SPIRAL_OPTIMIZER_H

#include <string>

#include <ifopt/ipopt_solver.h>
#include <ifopt/variable_set.h>
#include <ifopt/cost_term.h>

namespace f1tenth_racecar
{
class CubicSpiralOptimizer
{
private:
  class CubicSpiral
  {
  private:
    Eigen::Vector4d a_;

    Eigen::VectorXd sampleX(const Eigen::VectorXd& s_points);

    Eigen::VectorXd sampleY(const Eigen::VectorXd& s_points);

    Eigen::VectorXd sampleHeading(const Eigen::VectorXd& s_points);

    double getX(const double s);

    double getY(const double s);

    double getHeading(const double s);

  public:
    explicit CubicSpiral(const Eigen::Vector4d coeffs);

    Eigen::Matrix3Xd sampleCubicSpiral(const Eigen::VectorXd& s_points);
  };

  struct CommonTerms
  {
    double t_1_;
    double t_2_;
    double t_3_;
    double t_4_;
    double t_5_;
    double t_6_;
    double t_7_;
    double t_8_;
    double t_9_;
    double t_10_;
    double t_11_;
    double t_12_;
    double t_13_;
    double t_14_;
    double t_15_;
    double t_16_;

    double sin_t_1_;
    double sin_t_2_;
    double sin_t_3_;
    double sin_t_4_;
    double sin_t_5_;
    double sin_t_6_;
    double sin_t_7_;
    double sin_t_8_;

    double cos_t_1_;
    double cos_t_2_;
    double cos_t_3_;
    double cos_t_4_;
    double cos_t_5_;
    double cos_t_6_;
    double cos_t_7_;
    double cos_t_8_;
  };

  class CubicSpiralVariableSet : public ifopt::VariableSet
  {
  private:
    const double min_dist_;
    const double max_curvature_;

    double p_1_;  // curvature at s = 1/3
    double p_2_;  // curvature at s = 2/3
    double s_f_;  // length

    CommonTerms& common_terms_;

    void updateCommonTerms(const Eigen::Matrix<double, 5, 1>& p);

  public:
    CubicSpiralVariableSet(const double min_dist, const double max_curvature, CommonTerms& common_terms);

    void SetVariables(const Eigen::VectorXd& vars) override;

    Eigen::VectorXd GetValues() const override;

    ifopt::Component::VecBound GetBounds() const override;
  };

  class CubicSpiralCostTerm : public ifopt::CostTerm
  {
  private:
    static constexpr unsigned int K_BE_ = 1;
    static constexpr unsigned int K_X_ = 25;
    static constexpr unsigned int K_Y_ = 25;
    static constexpr unsigned int K_HDG_ = 30;

    const double goal_x_;
    const double goal_y_;
    const double goal_heading_;

    CommonTerms& common_terms_;

    double bendingEnergyCost(const Eigen::Matrix<double, 5, 1>& p) const;

    Eigen::Vector3d bendingEnergyCostGrad(const Eigen::Matrix<double, 5, 1>& p) const;

    double xCost(const Eigen::Matrix<double, 5, 1>& p) const;

    Eigen::Vector3d xCostGrad(const Eigen::Matrix<double, 5, 1>& p) const;

    double yCost(const Eigen::Matrix<double, 5, 1>& p) const;

    Eigen::Vector3d yCostGrad(const Eigen::Matrix<double, 5, 1>& p) const;

    double headingCost(const Eigen::Matrix<double, 5, 1>& p) const;

    Eigen::Vector3d headingCostGrad(const Eigen::Matrix<double, 5, 1>& p) const;

  public:
    CubicSpiralCostTerm(const double goal_x, const double goal_y, const double goal_heading, CommonTerms& common_terms);

    double GetCost() const override;

    void FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& jac) const override;
  };

  const double max_curvature_;

  Eigen::Matrix<double, 5, 1> optimizeCubicSpiralParams(const double goal_x, const double goal_y,
                                                        const double goal_heading);

  static Eigen::Vector4d paramsToCoeffs(const Eigen::Matrix<double, 5, 1> p);

public:
  explicit CubicSpiralOptimizer(double max_curvature);

  Eigen::Matrix3Xd generateCubicSpiralPath(const double goal_x, const double goal_y, const double goal_heading,
                                           const unsigned int num_samples);
};
}  // namespace f1tenth_racecar

#endif  // LOCAL_PLANNER_CUBIC_SPIRAL_OPTIMIZER_H
