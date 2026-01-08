#pragma once

#include <cmath>
#include <vector>

namespace pdcsu::util::math {

class LS3x3 {
public:
  struct LS3x3Solution {
    double A;
    double B;
    double C;
    double r2;
  };
  struct LS3x3Input {
    double x;
    double y;
    double z;
    double value;
  };

  static inline LS3x3Solution solve(const std::vector<LS3x3Input>& inputs) {
    double Sxx = 0.0, Sxy = 0.0, Sxz = 0.0;
    double Syy = 0.0, Syz = 0.0, Szz = 0.0;
    double Sxq = 0.0, Syq = 0.0, Szq = 0.0;
    double Sq = 0.0;  //, Sqq = 0.0;

    for (const auto& s : inputs) {
      const double x = s.x;
      const double y = s.y;
      const double z = s.z;
      const double q = s.value;

      Sxx += x * x;
      Sxy += x * y;
      Sxz += x * z;
      Syy += y * y;
      Syz += y * z;
      Szz += z * z;
      Sxq += x * q;
      Syq += y * q;
      Szq += z * q;
      Sq += q;
      // Sqq += q * q;
    }

    const int N = inputs.size();
    const double eps = 1e-12;

    LS3x3Solution sol{0.0, 0.0, 0.0, -1.0};

    const bool ux = Sxx > eps;
    const bool uy = Syy > eps;
    const bool uz = Szz > eps;

    if (ux && uy && uz) {
      const double det = Sxx * (Syy * Szz - Syz * Syz) -
                         Sxy * (Sxy * Szz - Sxz * Syz) +
                         Sxz * (Sxy * Syz - Sxz * Syy);

      if (std::abs(det) > eps) {
        sol.A = (Sxq * (Syy * Szz - Syz * Syz) - Sxy * (Syq * Szz - Szq * Syz) +
                    Sxz * (Syq * Syz - Szq * Syy)) /
                det;

        sol.B = (Sxx * (Syq * Szz - Szq * Syz) - Sxq * (Sxy * Szz - Sxz * Syz) +
                    Sxz * (Sxy * Szq - Sxq * Syz)) /
                det;

        sol.C = (Sxx * (Syy * Szq - Syq * Syz) - Sxy * (Sxy * Szq - Sxq * Syz) +
                    Sxq * (Sxy * Syz - Sxz * Syy)) /
                det;
      }
    } else if (!ux && uy && uz) {
      const double det = Syy * Szz - Syz * Syz;
      if (std::abs(det) > eps) {
        sol.B = (Syq * Szz - Szq * Syz) / det;
        sol.C = (Syy * Szq - Syq * Syz) / det;
      }
    } else if (ux && !uy && uz) {
      const double det = Sxx * Szz - Sxz * Sxz;
      if (std::abs(det) > eps) {
        sol.A = (Sxq * Szz - Szq * Sxz) / det;
        sol.C = (Sxx * Szq - Sxq * Sxz) / det;
      }
    } else if (ux && uy && !uz) {
      const double det = Sxx * Syy - Sxy * Sxy;
      if (std::abs(det) > eps) {
        sol.A = (Sxq * Syy - Syq * Sxy) / det;
        sol.B = (Sxx * Syq - Sxq * Sxy) / det;
      }
    } else if (ux) {
      sol.A = Sxq / Sxx;
    } else if (uy) {
      sol.B = Syq / Syy;
    } else if (uz) {
      sol.C = Szq / Szz;
    }

    const double q_mean = Sq / N;
    double ss_tot = 0.0;
    double ss_res = 0.0;

    for (const auto& s : inputs) {
      const double q_hat = sol.A * s.x + sol.B * s.y + sol.C * s.z;
      const double dq = s.value - q_hat;
      const double dqt = s.value - q_mean;
      ss_res += dq * dq;
      ss_tot += dqt * dqt;
    }

    sol.r2 = (ss_tot > eps) ? (1.0 - ss_res / ss_tot) : 1.0;

    return sol;
  }
};

}