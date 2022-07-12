#pragma once

#include "utils.h"

struct ImageRays 
{
  ImageRays(const double _fx, const double _fy, const double _cx, const double _cy,
      const int _img_cols, const int _img_rows)
    : m_fx(_fx), m_fy(_fy), m_cx(_cx), m_cy(_cy)
    , m_img_cols(_img_cols), m_img_rows(_img_rows)
  {
    // Init computation of bearing vectors 
    // (i.e. rays for each image pixels centered at the center of each pixels)
    const size_t nb_px = m_img_cols * m_img_rows;
    m_vbvs.resize(nb_px);

    const double inv_fx = 1. / m_fx;
    const double inv_fy = 1. / m_fy;

    #pragma omp parallel for
    for (int r = 0 ; r < m_img_rows ; r++)
    {
      for (int c = 0 ; c < m_img_cols ; c++)
      {
        const double u = static_cast<double>(c) + 0.5;
        const double v = static_cast<double>(r) + 0.5;

        const double x = (u - m_cx) * inv_fx;
        const double y = (v - m_cy) * inv_fy;

        const Eigen::Vector3d bv(x, y, 1.);

        m_vbvs.at(r * m_img_cols + c) = bv.normalized();
      }
    }
  }

  double m_fx, m_fy, m_cx, m_cy;
  int m_img_cols, m_img_rows;
  VectorAlignV3d m_vbvs;
};
