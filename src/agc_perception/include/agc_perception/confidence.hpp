// agc_perception 感知置信度模型（W4 共享头，spec R5）
//
// 旧树 `air_perception` 与 `air_fusion` **各自**重新导入 sensors_file 重算同一置信度，
// 属重复实现。W4 收敛为单一来源：二者只引用本头。
//
// 口径（对齐旧树行为）：2D 径向（Rayleigh）覆盖 P(|err| <= tol) = 1 - exp(-tol²/(2σ²))。
#pragma once

#include <algorithm>
#include <cmath>

namespace agc
{

// 由通道 x/y 噪声 σ（m）与容差 tol（m）解析得到置信度（经验覆盖）
inline double coverageConfidence(double sigma_xy_m, double tol_m)
{
  const double s2 = sigma_xy_m * sigma_xy_m;
  if (s2 <= 1e-12) {return 1.0;}
  const double c = 1.0 - std::exp(-(tol_m * tol_m) / (2.0 * s2));
  return std::min(1.0, std::max(0.0, c));
}

}  // namespace agc
