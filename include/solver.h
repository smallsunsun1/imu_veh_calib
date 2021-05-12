#ifndef INCLUDE_SOLVER_
#define INCLUDE_SOLVER_

#include "csm/csm_all.h"
#include "data_selection.h"
#include "utils.h"

class CSolver {
 public:
  CSolver();

  struct SolverParams {
    int mode;
    double max_cond_number;
    int outliers_iterations;
    double outliers_percentage;
  };

  struct CalibResult {
    double radius_l, radius_r;
    double axle;
    /** externel paras lx ly theta between Cam and odo */
    double l[3];
  };

  bool Solve(const std::vector<data_selection::SyncData> &calib_data, int mode, double max_cond_number,
             struct CalibResult &res);
  void Calib(std::vector<data_selection::SyncData> &calib_data, int outliers_iterations, CalibResult &res);
  // void calib(std::vector<data_selection::sync_data> &calib_data, int outliers_iterations);

 public:
  Eigen::VectorXd full_calibration_min(const Eigen::MatrixXd &M);
  Eigen::VectorXd numeric_calibration(const Eigen::MatrixXd &H);
  double calculate_error(const Eigen::VectorXd &x, const Eigen::MatrixXd &M);
  Eigen::VectorXd x_given_lambda(const Eigen::MatrixXd &M, const double &lambda, const Eigen::MatrixXd &W);
  void compute_disagreement(data_selection::SyncData &calib_data, const struct CalibResult &res);
  void estimate_noise(std::vector<data_selection::SyncData> &calib_data, const struct CalibResult &res, double &std_x,
                      double &std_y, double &std_th);
  double calculate_sd(const double array[], const int s, const int e);
  Eigen::MatrixXd compute_fim(const std::vector<data_selection::SyncData> &calib_data, const struct CalibResult &res,
                              const Eigen::Matrix3d &inf_sm);
};

#endif /* INCLUDE_SOLVER_ */
