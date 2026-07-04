/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#ifndef GCOPTER_HPP
#define GCOPTER_HPP

#include "gcopter/flatness.hpp"
#include "gcopter/geo_utils.hpp"
#include "gcopter/lbfgs.hpp"
#include "gcopter/minco.hpp"

#include <Eigen/Eigen>

#include <cfloat>
#include <cmath>
#include <iostream>
#include <spdlog/spdlog.h>
#include <vector>

namespace gcopter {

class OMNI_PolytopeSFC {
public:
  typedef Eigen::Matrix3Xd PolyhedronV;
  typedef Eigen::MatrixX4d PolyhedronH;
  typedef std::vector<PolyhedronV> PolyhedraV;
  typedef std::vector<PolyhedronH> PolyhedraH;

private:
  minco::MINCO_S3NU minco;

  double rho;              //时间权重
  Eigen::Matrix3d headPVA; //初始状态
  Eigen::Matrix3d tailPVA; //最终状态

  PolyhedraV vPolytopes;
  PolyhedraH hPolytopes; //凸约束
  Eigen::Matrix3Xd shortPath;

  Eigen::VectorXi pieceIdx;
  Eigen::VectorXi vPolyIdx;
  Eigen::VectorXi hPolyIdx;

  int polyN;
  int pieceN;

  int spatialDim;  //空间变量数
  int temporalDim; //时间变量数

  double smoothEps;            //平滑系数
  int integralRes;             //离散化点数
  Eigen::VectorXd magnitudeBd; //约束边界
  Eigen::VectorXd penaltyWt;   //惩罚权重
  double allocSpeed;           // 分配速度 = 3倍最大速度

  lbfgs::lbfgs_parameter_t lbfgs_params;

  Eigen::Matrix3Xd points;
  Eigen::VectorXd times;
  Eigen::Matrix3Xd gradByPoints;
  Eigen::VectorXd gradByTimes;
  Eigen::MatrixX3d partialGradByCoeffs;
  Eigen::VectorXd partialGradByTimes;

private:
  //正向变换: τ → T (forwardT)
  static inline void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T) {
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    for (int i = 0; i < sizeTau; i++) {
      T(i) = tau(i) > 0.0 ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                          : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
    return;
  }
  //逆向变换: T → τ (backwardT)
  template <typename EIGENVEC>
  static inline void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau) {
    const int sizeT = T.size();
    tau.resize(sizeT);
    for (int i = 0; i < sizeT; i++) {
      tau(i) = T(i) > 1.0 ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / T(i) - 1.0));
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradT(const Eigen::VectorXd &tau, // 无约束时间变量
                                   const Eigen::VectorXd &gradT, // ∂cost/∂T
                                   EIGENVEC &gradTau) // ∂cost/∂τ (输出)
  {
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    double denSqrt;
    for (int i = 0; i < sizeTau; i++) {
      if (tau(i) > 0) {
        gradTau(i) = gradT(i) * (tau(i) + 1.0);
      } else {
        denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
        gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
      }
    }

    return;
  }

  static inline void forwardP(const Eigen::VectorXd &xi,   // 无约束变量
                              const Eigen::VectorXi &vIdx, // 顶点集索引
                              const PolyhedraV &vPolys, // 顶点表示的多面体
                              Eigen::Matrix3Xd &P)      // 输出：物理点
  {
    const int sizeP = vIdx.size();
    P.resize(3, sizeP);
    Eigen::VectorXd q;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();
      q = xi.segment(j, k).normalized().head(k - 1);
      P.col(i) =
          vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) + vPolys[l].col(0);
    }
    return;
  }

  static inline double costTinyNLS(void *ptr, const Eigen::VectorXd &xi,
                                   Eigen::VectorXd &gradXi) {
    const int n = xi.size();
    const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

    const double sqrNormXi = xi.squaredNorm();
    const double invNormXi = 1.0 / sqrt(sqrNormXi);
    const Eigen::VectorXd unitXi = xi * invNormXi;
    const Eigen::VectorXd r = unitXi.head(n - 1);
    const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                  ovPoly.col(1) - ovPoly.col(0);

    double cost = delta.squaredNorm();
    gradXi.head(n - 1) =
        (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
        r.array() * 2.0;
    gradXi(n - 1) = 0.0;
    gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

    const double sqrNormViolation = sqrNormXi - 1.0;
    if (sqrNormViolation > 0.0) {
      double c = sqrNormViolation * sqrNormViolation;
      const double dc = 3.0 * c;
      c *= sqrNormViolation;
      cost += c;
      gradXi += dc * 2.0 * xi;
    }

    return cost;
  }

  template <typename EIGENVEC>
  static inline void backwardP(const Eigen::Matrix3Xd &P,
                               const Eigen::VectorXi &vIdx,
                               const PolyhedraV &vPolys, EIGENVEC &xi) {
    const int sizeP = P.cols();

    double minSqrD;
    lbfgs::lbfgs_parameter_t tiny_nls_params;
    tiny_nls_params.past = 0;
    tiny_nls_params.delta = 1.0e-5;
    tiny_nls_params.g_epsilon = FLT_EPSILON;
    tiny_nls_params.max_iterations = 128;

    Eigen::Matrix3Xd ovPoly;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();

      ovPoly.resize(3, k + 1);
      ovPoly.col(0) = P.col(i);
      ovPoly.rightCols(k) = vPolys[l];
      Eigen::VectorXd x(k);
      x.setConstant(sqrt(1.0 / k));
      lbfgs::lbfgs_optimize(x, minSqrD, &OMNI_PolytopeSFC::costTinyNLS, nullptr,
                            nullptr, &ovPoly, tiny_nls_params);

      xi.segment(j, k) = x;
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void
  backwardGradP(const Eigen::VectorXd &xi, const Eigen::VectorXi &vIdx,
                const PolyhedraV &vPolys, const Eigen::Matrix3Xd &gradP,
                EIGENVEC &gradXi) {
    const int sizeP = vIdx.size();
    gradXi.resize(xi.size());

    double normInv;
    Eigen::VectorXd q, gradQ, unitQ;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();
      q = xi.segment(j, k);
      normInv = 1.0 / q.norm();
      unitQ = q * normInv;
      gradQ.resize(k);
      gradQ.head(k - 1) =
          (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
          unitQ.head(k - 1).array() * 2.0;
      gradQ(k - 1) = 0.0;
      gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys, double &cost,
                                         EIGENVEC &gradXi) {
    const int sizeP = vIdx.size();
    gradXi.resize(xi.size());

    double sqrNormQ, sqrNormViolation, c, dc;
    Eigen::VectorXd q;
    for (int i = 0, j = 0, k; i < sizeP; i++, j += k) {
      k = vPolys[vIdx(i)].cols();

      q = xi.segment(j, k);
      sqrNormQ = q.squaredNorm();
      sqrNormViolation = sqrNormQ - 1.0;
      if (sqrNormViolation > 0.0) {
        c = sqrNormViolation * sqrNormViolation;
        dc = 3.0 * c;
        c *= sqrNormViolation;
        cost += c;
        gradXi.segment(j, k) += dc * 2.0 * q;
      }
    }

    return;
  }

  static inline bool smoothedL1(const double &x, const double &mu, double &f,
                                double &df) {
    if (x < 0.0) {
      return false;
    } else if (x > mu) {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    } else {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return true;
    }
  }

  // 修改后的 magnitudeBounds 含义：
  //   magnitudeBounds(0) = v_max (最大速度)
  //   magnitudeBounds(1) = a_max (最大加速度)
  // 修改后的 penaltyWeights 含义：
  //   penaltyWeights(0) = pos_weight (位置约束权重)
  //   penaltyWeights(1) = vel_weight (速度约束权重)
  //   penaltyWeights(2) = acc_weight (加速度约束权重)
  // 注意：不再使用 thrust、omega、theta 等相关量
  static inline void attachPenaltyFunctional(
      const Eigen::VectorXd &T, const Eigen::MatrixX3d &coeffs,
      const Eigen::VectorXi &hIdx, const PolyhedraH &hPolys,
      const double &smoothFactor, const int &integralResolution,
      const Eigen::VectorXd &magnitudeBounds, // [v_max, a_max]
      const Eigen::VectorXd &penaltyWeights,  // [pos, vel, acc]
      double &cost, Eigen::VectorXd &gradT, Eigen::MatrixX3d &gradC) {
    const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
    const double accSqrMax = magnitudeBounds(1) * magnitudeBounds(1);

    const double weightPos = penaltyWeights(0);
    const double weightVel = penaltyWeights(1);
    const double weightAcc = penaltyWeights(2);

    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d gradPos, gradVel,
        gradAcc; // 直接存储对位置、速度、加速度的梯度

    double step, alpha;
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::Vector3d outerNormal;
    int K, L;
    double violaPos, violaVel, violaAcc;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD;
    double violaPosPena, violaVelPena, violaAccPena;
    double node, pena;

    const int pieceNum = T.size();
    const double integralFrac = 1.0 / integralResolution;
    for (int i = 0; i < pieceNum; i++) {
      const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);
      step = T(i) * integralFrac;
      for (int j = 0; j <= integralResolution; j++) {
        // 计算基函数 beta0~beta4 (同原代码)
        s1 = j * step;
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;

        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;

        // 初始化梯度累加器
        gradPos.setZero();
        gradVel.setZero();
        gradAcc.setZero();
        pena = 0.0;

        // ----- 位置约束（凸多面体碰撞）-----
        L = hIdx(i);
        K = hPolys[L].rows();
        for (int k = 0; k < K; k++) {
          outerNormal = hPolys[L].block<1, 3>(k, 0);
          violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
          if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD)) {
            gradPos += weightPos * violaPosPenaD * outerNormal;
            pena += weightPos * violaPosPena;
          }
        }

        // ----- 速度约束 -----
        violaVel = vel.squaredNorm() - velSqrMax;
        if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
          gradVel += weightVel * violaVelPenaD * 2.0 * vel;
          pena += weightVel * violaVelPena;
        }

        // ----- 加速度约束 -----
        violaAcc = acc.squaredNorm() - accSqrMax;
        if (smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD)) {
          gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
          pena += weightAcc * violaAccPena;
        }

        // 注：如果需要加加速度约束，可在此添加类似代码（使用 jer 和相应的权重）

        // 将梯度通过基函数投影到系数梯度 gradC 和时间梯度 gradT
        node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
        alpha = j * integralFrac;

        // 对轨迹系数 c 的梯度：每个 beta 分量乘以对应的梯度向量
        gradC.block<6, 3>(i * 6, 0) +=
            (beta0 * gradPos.transpose() + beta1 * gradVel.transpose() +
             beta2 * gradAcc.transpose()) *
            node * step;

        // 对时间 T(i) 的梯度：通过链式法则 d/dT = d/d_s * ds/dT，其中 s
        // 为归一化时间 这里利用 cost 对时间的导数 = (∂cost/∂pos)·vel +
        // (∂cost/∂vel)·acc + (∂cost/∂acc)·jer
        gradT(i) += (gradPos.dot(vel) + gradVel.dot(acc) + gradAcc.dot(jer)) *
                        alpha * node * step +
                    node * integralFrac * pena;

        cost += node * step * pena;
      }
    }

    return;
  }

  static inline double costFunctional(void *ptr, const Eigen::VectorXd &x,
                                      Eigen::VectorXd &g) {
    OMNI_PolytopeSFC &obj = *(OMNI_PolytopeSFC *)ptr;
    const int dimTau = obj.temporalDim;
    const int dimXi = obj.spatialDim;
    const double weightT = obj.rho;
    Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
    Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
    Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
    Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

    forwardT(tau, obj.times);
    forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

    double cost;
    obj.minco.setParameters(obj.points, obj.times);
    obj.minco.getEnergy(cost);
    obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
    obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

    attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(), obj.hPolyIdx,
                            obj.hPolytopes, obj.smoothEps, obj.integralRes,
                            obj.magnitudeBd, obj.penaltyWt, cost,
                            obj.partialGradByTimes, obj.partialGradByCoeffs);

    obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                            obj.gradByPoints, obj.gradByTimes);

    cost += weightT * obj.times.sum();
    obj.gradByTimes.array() += weightT;

    backwardGradT(tau, obj.gradByTimes, gradTau);
    backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
    normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

    return cost;
  }

  static inline double costDistance(void *ptr, const Eigen::VectorXd &xi,
                                    Eigen::VectorXd &gradXi) {
    void **dataPtrs = (void **)ptr;
    const double &dEps = *((const double *)(dataPtrs[0]));
    const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
    const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
    const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

    double cost = 0.0;
    const int overlaps = vPolys.size() / 2;

    Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
    Eigen::Vector3d a, b, d;
    Eigen::VectorXd r;
    double smoothedDistance;
    for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k) {
      a = i == 0 ? ini : b;
      if (i < overlaps) {
        k = vPolys[2 * i + 1].cols();
        Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
        r = q.normalized().head(k - 1);
        b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
            vPolys[2 * i + 1].col(0);
      } else {
        b = fin;
      }

      d = b - a;
      smoothedDistance = sqrt(d.squaredNorm() + dEps);
      cost += smoothedDistance;

      if (i < overlaps) {
        gradP.col(i) += d / smoothedDistance;
      }
      if (i > 0) {
        gradP.col(i - 1) -= d / smoothedDistance;
      }
    }

    Eigen::VectorXd unitQ;
    double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
    for (int i = 0, j = 0, k; i < overlaps; i++, j += k) {
      k = vPolys[2 * i + 1].cols();
      Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
      Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
      sqrNormQ = q.squaredNorm();
      invNormQ = 1.0 / sqrt(sqrNormQ);
      unitQ = q * invNormQ;
      gradQ.head(k - 1) =
          (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i))
              .array() *
          unitQ.head(k - 1).array() * 2.0;
      gradQ(k - 1) = 0.0;
      gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

      sqrNormViolation = sqrNormQ - 1.0;
      if (sqrNormViolation > 0.0) {
        c = sqrNormViolation * sqrNormViolation;
        dc = 3.0 * c;
        c *= sqrNormViolation;
        cost += c;
        gradQ += dc * 2.0 * q;
      }
    }

    return cost;
  }

  static inline void getShortestPath(const Eigen::Vector3d &ini,
                                     const Eigen::Vector3d &fin,
                                     const PolyhedraV &vPolys,
                                     const double &smoothD,
                                     Eigen::Matrix3Xd &path) {
    const int overlaps = vPolys.size() / 2;
    Eigen::VectorXi vSizes(overlaps);
    for (int i = 0; i < overlaps; i++) {
      vSizes(i) = vPolys[2 * i + 1].cols();
    }
    Eigen::VectorXd xi(vSizes.sum());
    for (int i = 0, j = 0; i < overlaps; i++) {
      xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
      j += vSizes(i);
    }

    double minDistance;
    void *dataPtrs[4];
    dataPtrs[0] = (void *)(&smoothD);
    dataPtrs[1] = (void *)(&ini);
    dataPtrs[2] = (void *)(&fin);
    dataPtrs[3] = (void *)(&vPolys);
    lbfgs::lbfgs_parameter_t shortest_path_params;
    shortest_path_params.past = 3;
    shortest_path_params.delta = 1.0e-3;
    shortest_path_params.g_epsilon = 1.0e-5;

    lbfgs::lbfgs_optimize(xi, minDistance, &OMNI_PolytopeSFC::costDistance,
                          nullptr, nullptr, dataPtrs, shortest_path_params);

    path.resize(3, overlaps + 2);
    path.leftCols<1>() = ini;
    path.rightCols<1>() = fin;
    Eigen::VectorXd r;
    for (int i = 0, j = 0, k; i < overlaps; i++, j += k) {
      k = vPolys[2 * i + 1].cols();
      Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
      r = q.normalized().head(k - 1);
      path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
    }

    return;
  }
  /**
  @brief:半空间转顶点表示
  @param: hPs 输入：半空间表示
  @param: vPs 输出：顶点表示
  */
  static inline bool processCorridor(const PolyhedraH &hPs, PolyhedraV &vPs) {
    const int sizeCorridor = hPs.size() - 1;

    vPs.clear();
    vPs.reserve(2 * sizeCorridor + 1);

    int nv;

    PolyhedronH curIH;
    PolyhedronV curIV, curIOB;
    for (int i = 0; i < sizeCorridor; i++) {
      if (!geo_utils::enumerateVs(hPs[i], curIV)) {
        
        return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB.col(0) = curIV.col(0);
      curIOB.rightCols(nv - 1) =
          curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);

      curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
      curIH.topRows(hPs[i].rows()) = hPs[i];
      curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
      if (!geo_utils::enumerateVs(curIH, curIV)) {
        return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB.col(0) = curIV.col(0);
      curIOB.rightCols(nv - 1) =
          curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);
    }

    if (!geo_utils::enumerateVs(hPs.back(), curIV)) {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);

    return true;
  }
  /**
  @param path   最短路径点 [3×(N+1)]，N为凸多面体数
                  path.col(0): 起点
                  path.col(i): 第i个凸多面体内的点
                  path.col(N): 终点
  @param speed  分配速度（通常是最大速度的3倍）
                  影响初始时间估计
  @param intervalNs 每个路径段的分段数 [N维]
                  intervalNs(i): 第i段路径分成多少小段
                  例如：intervalNs = [2, 3, 2] 表示
                  第0段分2小段，第1段分3小段，第2段分2小段
  @param innerPoints 输出：中间点 [3×(总段数-1)]
                  所有细分后的路径点（不含起点）
  @param timeAlloc   输出：每段时间 [总段数]
                  每小段的估计时间
  */
  static inline void setInitial(const Eigen::Matrix3Xd &path,
                                const double &speed,
                                const Eigen::VectorXi &intervalNs,
                                Eigen::Matrix3Xd &innerPoints,
                                Eigen::VectorXd &timeAlloc) {
    const int sizeM = intervalNs.size();
    const int sizeN = intervalNs.sum();
    innerPoints.resize(3, sizeN - 1);
    timeAlloc.resize(sizeN);

    Eigen::Vector3d a, b, c;
    for (int i = 0, j = 0, k = 0, l; i < sizeM; i++) {
      l = intervalNs(i);
      a = path.col(i);
      b = path.col(i + 1);
      c = (b - a) / l;
      timeAlloc.segment(j, l).setConstant(c.norm() / speed);
      j += l;
      for (int m = 0; m < l; m++) {
        if (i > 0 || m > 0) {
          innerPoints.col(k++) = a + c * m;
        }
      }
    }
  }

public:
  // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
  // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight,
  // thrust_weight]^T physicalParams = [vehicle_mass,
  // gravitational_acceleration, horitonral_drag_coeff,
  //                   vertical_drag_coeff, parasitic_drag_coeff,
  //                   speed_smooth_factor]^T
  /**
  magnitudeBounds(5) [v_max, ω_max, θ_max, thrust_min, thrust_max]
  penaltyWeights(5) 各成本项权重 [位置权重 速度权重 角速度权重 角度权重
  推力权重]
  */
  /**
  @brief:
  @param: timeWeight 时间权重
  @param: initialPVA 初始状态
  @param: terminalPVA 最终状态
  @param: safeCorridor 凸约束集
  @param: lengthPerPiece 最大时间（无限制）
  @param: smoothingFactor 平滑系数
  @param: integralResolution 离散化点数
  @param: magnitudeBounds 约束边界
  @param: penaltyWeights 惩罚权重
  @return:
  */
  inline bool setup(const double &timeWeight, const Eigen::Matrix3d &initialPVA,
                    const Eigen::Matrix3d &terminalPVA,
                    const PolyhedraH &safeCorridor,
                    const double &lengthPerPiece, const double &smoothingFactor,
                    const int &integralResolution,
                    const Eigen::VectorXd &magnitudeBounds,
                    const Eigen::VectorXd &penaltyWeights) {
    rho = timeWeight;
    headPVA = initialPVA;
    tailPVA = terminalPVA;

    hPolytopes = safeCorridor;
    //归一化所有凸多面体的半空间表示
    for (size_t i = 0; i < hPolytopes.size(); i++) {
      const Eigen::ArrayXd norms = hPolytopes[i].leftCols<3>().rowwise().norm();
      hPolytopes[i].array().colwise() /= norms;
    }
    //半空间转顶点表示
    if (!processCorridor(hPolytopes, vPolytopes)) {
      spdlog::warn("半空间转顶点表示 失败");
      return false;
    }
    //凸多面体总数
    polyN = hPolytopes.size();

    smoothEps = smoothingFactor;
    integralRes = integralResolution;
    magnitudeBd = magnitudeBounds;
    penaltyWt = penaltyWeights;

    allocSpeed = magnitudeBd(0) * 3.0; // 分配速度 = 3倍最大速度
    //最短路径生成
    getShortestPath(headPVA.col(0), // 起点位置
                    tailPVA.col(0), // 终点位置
                    vPolytopes,     // 顶点表示的安全走廊
                    smoothEps,      // 平滑参数
                    shortPath);     // 输出：最短路径点

    // 计算每段的距离
    const Eigen::Matrix3Xd deltas =
        shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
    // 根据期望长度自动分配段数
    pieceIdx =
        (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
    pieceIdx.array() += 1;   // 每段至少1个
    pieceN = pieceIdx.sum(); // 总轨迹段数
    //索引映射建立
    temporalDim = pieceN;        // 时间变量数 = 轨迹段数
    spatialDim = 0;              // 空间变量数（累加）
    vPolyIdx.resize(pieceN - 1); // 每段对应的顶点集索引
    hPolyIdx.resize(pieceN);     // 每段对应的半空间索引
    for (int i = 0, j = 0, k; i < polyN; i++) {
      k = pieceIdx(i); // 当前凸多面体分成的段数
      for (int l = 0; l < k; l++, j++) {
        if (l < k - 1) {
          // 非最后一段：使用当前多面体自身
          vPolyIdx(j) = 2 * i;
          spatialDim += vPolytopes[2 * i].cols();
        } else if (i < polyN - 1) {
          // 最后一段（非最后一个多面体）：使用重叠区域
          vPolyIdx(j) = 2 * i + 1;
          spatialDim += vPolytopes[2 * i + 1].cols();
        }
        // 半空间始终用当前多面体
        hPolyIdx(j) = i;
      }
    }

    // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
    // 设置边界条件：起点和终点的位置、速度、加速度
    minco.setConditions(headPVA, tailPVA, pieceN);

    // Allocate temp variables
    points.resize(3, pieceN - 1);       // 中间点（3D，数量=段数-1）
    times.resize(pieceN);               // 每段时间
    gradByPoints.resize(3, pieceN - 1); // 点的梯度
    gradByTimes.resize(pieceN);         // 时间的梯度
    partialGradByCoeffs.resize(6 * pieceN, 3); // 系数梯度
    partialGradByTimes.resize(pieceN);         // 时间梯度部分

    return true;
  }

  inline double optimize(Trajectory<5> &traj, const double &relCostTol) {
    Eigen::VectorXd x(temporalDim + spatialDim);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

    setInitial(shortPath,  // 最短路径点
               allocSpeed, // 分配速度 = 3 * v_max
               pieceIdx,   // 每段分配数
               points,     // 输出：中间点
               times);     // 输出：每段时间

    backwardT(times, tau);                       // 时间：物理→无约束
    backwardP(points, vPolyIdx, vPolytopes, xi); // 空间：物理→无约束

    double minCostFunctional;
    lbfgs_params.mem_size = 256; // 存储256个历史梯度（收敛快）
    lbfgs_params.past = 3;       // 检查过去3次迭代的成本变化
    lbfgs_params.min_step = 1.0e-32; // 最小步长（很小，允许精细调整）
    lbfgs_params.g_epsilon = 0.0;    // 梯度容差（0表示禁用）
    lbfgs_params.delta = relCostTol; // 成本相对变化容差

    int ret =
        lbfgs::lbfgs_optimize(x, // 优化变量（输入/输出）
                              minCostFunctional, // 输出：最优成本
                              &OMNI_PolytopeSFC::costFunctional, // 成本函数
                              nullptr, // 进度回调（可选）
                              nullptr, // 数据指针（可选）
                              this,    // 传递给成本函数的this指针
                              lbfgs_params); // L-BFGS参数

    if (ret >= 0) {
      forwardT(tau, times);
      forwardP(xi, vPolyIdx, vPolytopes, points);
      minco.setParameters(points, times);
      minco.getTrajectory(traj);
    } else {
      traj.clear();
      minCostFunctional = INFINITY;
      std::cout << "Optimization Failed: " << lbfgs::lbfgs_strerror(ret)
                << std::endl;
    }

    return minCostFunctional;
  }
};
class GCOPTER_PolytopeSFC {
public:
  typedef Eigen::Matrix3Xd PolyhedronV;
  typedef Eigen::MatrixX4d PolyhedronH;
  typedef std::vector<PolyhedronV> PolyhedraV;
  typedef std::vector<PolyhedronH> PolyhedraH;

private:
  minco::MINCO_S3NU minco;
  flatness::FlatnessMap flatmap;

  double rho;              //时间权重
  Eigen::Matrix3d headPVA; //初始状态
  Eigen::Matrix3d tailPVA; //最终状态

  PolyhedraV vPolytopes;
  PolyhedraH hPolytopes; //凸约束集
  Eigen::Matrix3Xd shortPath;

  Eigen::VectorXi pieceIdx;
  Eigen::VectorXi vPolyIdx;
  Eigen::VectorXi hPolyIdx;

  int polyN;
  int pieceN;

  int spatialDim;  //空间变量数
  int temporalDim; //时间变量数

  double smoothEps;            //平滑系数
  int integralRes;             //离散化点数
  Eigen::VectorXd magnitudeBd; //约束边界
  Eigen::VectorXd penaltyWt;   //惩罚权重
  Eigen::VectorXd physicalPm;  //物理参数
  double allocSpeed;           // 分配速度 = 3倍最大速度

  lbfgs::lbfgs_parameter_t lbfgs_params;

  Eigen::Matrix3Xd points;
  Eigen::VectorXd times;
  Eigen::Matrix3Xd gradByPoints;
  Eigen::VectorXd gradByTimes;
  Eigen::MatrixX3d partialGradByCoeffs;
  Eigen::VectorXd partialGradByTimes;

private:
  //正向变换: τ → T (forwardT)
  static inline void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T) {
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    for (int i = 0; i < sizeTau; i++) {
      T(i) = tau(i) > 0.0 ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                          : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
    return;
  }
  //逆向变换: T → τ (backwardT)
  template <typename EIGENVEC>
  static inline void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau) {
    const int sizeT = T.size();
    tau.resize(sizeT);
    for (int i = 0; i < sizeT; i++) {
      tau(i) = T(i) > 1.0 ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / T(i) - 1.0));
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradT(const Eigen::VectorXd &tau, // 无约束时间变量
                                   const Eigen::VectorXd &gradT, // ∂cost/∂T
                                   EIGENVEC &gradTau) // ∂cost/∂τ (输出)
  {
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    double denSqrt;
    for (int i = 0; i < sizeTau; i++) {
      if (tau(i) > 0) {
        gradTau(i) = gradT(i) * (tau(i) + 1.0);
      } else {
        denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
        gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
      }
    }

    return;
  }

  static inline void forwardP(const Eigen::VectorXd &xi,   // 无约束变量
                              const Eigen::VectorXi &vIdx, // 顶点集索引
                              const PolyhedraV &vPolys, // 顶点表示的多面体
                              Eigen::Matrix3Xd &P)      // 输出：物理点
  {
    const int sizeP = vIdx.size();
    P.resize(3, sizeP);
    Eigen::VectorXd q;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();
      q = xi.segment(j, k).normalized().head(k - 1);
      P.col(i) =
          vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) + vPolys[l].col(0);
    }
    return;
  }

  static inline double costTinyNLS(void *ptr, const Eigen::VectorXd &xi,
                                   Eigen::VectorXd &gradXi) {
    const int n = xi.size();
    const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

    const double sqrNormXi = xi.squaredNorm();
    const double invNormXi = 1.0 / sqrt(sqrNormXi);
    const Eigen::VectorXd unitXi = xi * invNormXi;
    const Eigen::VectorXd r = unitXi.head(n - 1);
    const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                  ovPoly.col(1) - ovPoly.col(0);

    double cost = delta.squaredNorm();
    gradXi.head(n - 1) =
        (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
        r.array() * 2.0;
    gradXi(n - 1) = 0.0;
    gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

    const double sqrNormViolation = sqrNormXi - 1.0;
    if (sqrNormViolation > 0.0) {
      double c = sqrNormViolation * sqrNormViolation;
      const double dc = 3.0 * c;
      c *= sqrNormViolation;
      cost += c;
      gradXi += dc * 2.0 * xi;
    }

    return cost;
  }

  template <typename EIGENVEC>
  static inline void backwardP(const Eigen::Matrix3Xd &P,
                               const Eigen::VectorXi &vIdx,
                               const PolyhedraV &vPolys, EIGENVEC &xi) {
    const int sizeP = P.cols();

    double minSqrD;
    lbfgs::lbfgs_parameter_t tiny_nls_params;
    tiny_nls_params.past = 0;
    tiny_nls_params.delta = 1.0e-5;
    tiny_nls_params.g_epsilon = FLT_EPSILON;
    tiny_nls_params.max_iterations = 128;

    Eigen::Matrix3Xd ovPoly;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();

      ovPoly.resize(3, k + 1);
      ovPoly.col(0) = P.col(i);
      ovPoly.rightCols(k) = vPolys[l];
      Eigen::VectorXd x(k);
      x.setConstant(sqrt(1.0 / k));
      lbfgs::lbfgs_optimize(x, minSqrD, &GCOPTER_PolytopeSFC::costTinyNLS,
                            nullptr, nullptr, &ovPoly, tiny_nls_params);

      xi.segment(j, k) = x;
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void
  backwardGradP(const Eigen::VectorXd &xi, const Eigen::VectorXi &vIdx,
                const PolyhedraV &vPolys, const Eigen::Matrix3Xd &gradP,
                EIGENVEC &gradXi) {
    const int sizeP = vIdx.size();
    gradXi.resize(xi.size());

    double normInv;
    Eigen::VectorXd q, gradQ, unitQ;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();
      q = xi.segment(j, k);
      normInv = 1.0 / q.norm();
      unitQ = q * normInv;
      gradQ.resize(k);
      gradQ.head(k - 1) =
          (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
          unitQ.head(k - 1).array() * 2.0;
      gradQ(k - 1) = 0.0;
      gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys, double &cost,
                                         EIGENVEC &gradXi) {
    const int sizeP = vIdx.size();
    gradXi.resize(xi.size());

    double sqrNormQ, sqrNormViolation, c, dc;
    Eigen::VectorXd q;
    for (int i = 0, j = 0, k; i < sizeP; i++, j += k) {
      k = vPolys[vIdx(i)].cols();

      q = xi.segment(j, k);
      sqrNormQ = q.squaredNorm();
      sqrNormViolation = sqrNormQ - 1.0;
      if (sqrNormViolation > 0.0) {
        c = sqrNormViolation * sqrNormViolation;
        dc = 3.0 * c;
        c *= sqrNormViolation;
        cost += c;
        gradXi.segment(j, k) += dc * 2.0 * q;
      }
    }

    return;
  }

  static inline bool smoothedL1(const double &x, const double &mu, double &f,
                                double &df) {
    if (x < 0.0) {
      return false;
    } else if (x > mu) {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    } else {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return true;
    }
  }

  // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
  // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight,
  // thrust_weight]^T physicalParams = [vehicle_mass,
  // gravitational_acceleration, horitonral_drag_coeff,
  //                   vertical_drag_coeff, parasitic_drag_coeff,
  //                   speed_smooth_factor]^T
  static inline void attachPenaltyFunctional(
      const Eigen::VectorXd &T, const Eigen::MatrixX3d &coeffs,
      const Eigen::VectorXi &hIdx, const PolyhedraH &hPolys,
      const double &smoothFactor, const int &integralResolution,
      const Eigen::VectorXd &magnitudeBounds,
      const Eigen::VectorXd &penaltyWeights, flatness::FlatnessMap &flatMap,
      double &cost, Eigen::VectorXd &gradT, Eigen::MatrixX3d &gradC) {
    const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
    const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
    const double thetaMax = magnitudeBounds(2);
    const double thrustMean = 0.5 * (magnitudeBounds(3) + magnitudeBounds(4));
    const double thrustRadi =
        0.5 * fabs(magnitudeBounds(4) - magnitudeBounds(3));
    const double thrustSqrRadi = thrustRadi * thrustRadi;

    const double weightPos = penaltyWeights(0);
    const double weightVel = penaltyWeights(1);
    const double weightOmg = penaltyWeights(2);
    const double weightTheta = penaltyWeights(3);
    const double weightThrust = penaltyWeights(4);

    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
    double totalGradPsi, totalGradPsiD;
    double thr, cos_theta;
    Eigen::Vector4d quat;
    Eigen::Vector3d omg;
    double gradThr;
    Eigen::Vector4d gradQuat;
    Eigen::Vector3d gradPos, gradVel, gradOmg;

    double step, alpha;
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::Vector3d outerNormal;
    int K, L;
    double violaPos, violaVel, violaOmg, violaTheta, violaThrust;
    double violaPosPenaD, violaVelPenaD, violaOmgPenaD, violaThetaPenaD,
        violaThrustPenaD;
    double violaPosPena, violaVelPena, violaOmgPena, violaThetaPena,
        violaThrustPena;
    double node, pena;

    const int pieceNum = T.size();
    const double integralFrac = 1.0 / integralResolution;
    for (int i = 0; i < pieceNum; i++) {
      const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);
      step = T(i) * integralFrac;
      for (int j = 0; j <= integralResolution; j++) {
        s1 = j * step;
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3,
        beta0(4) = s4, beta0(5) = s5;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1,
        beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
        beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1,
        beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
        beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0,
        beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
        beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0,
        beta4(4) = 24.0, beta4(5) = 120.0 * s1;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;

        flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);

        violaVel = vel.squaredNorm() - velSqrMax;
        violaOmg = omg.squaredNorm() - omgSqrMax;
        cos_theta = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
        violaTheta = acos(cos_theta) - thetaMax;
        violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

        gradThr = 0.0;
        gradQuat.setZero();
        gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();
        pena = 0.0;

        L = hIdx(i);
        K = hPolys[L].rows();
        for (int k = 0; k < K; k++) {
          outerNormal = hPolys[L].block<1, 3>(k, 0);
          violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
          if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD)) {
            gradPos += weightPos * violaPosPenaD * outerNormal;
            pena += weightPos * violaPosPena;
          }
        }

        if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
          gradVel += weightVel * violaVelPenaD * 2.0 * vel;
          pena += weightVel * violaVelPena;
        }

        if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD)) {
          gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
          pena += weightOmg * violaOmgPena;
        }

        if (smoothedL1(violaTheta, smoothFactor, violaThetaPena,
                       violaThetaPenaD)) {
          gradQuat += weightTheta * violaThetaPenaD /
                      sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
                      Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
          pena += weightTheta * violaThetaPena;
        }

        if (smoothedL1(violaThrust, smoothFactor, violaThrustPena,
                       violaThrustPenaD)) {
          gradThr += weightThrust * violaThrustPenaD * 2.0 * (thr - thrustMean);
          pena += weightThrust * violaThrustPena;
        }

        flatMap.backward(gradPos, gradVel, gradThr, gradQuat, gradOmg,
                         totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                         totalGradPsi, totalGradPsiD);

        node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
        alpha = j * integralFrac;
        gradC.block<6, 3>(i * 6, 0) += (beta0 * totalGradPos.transpose() +
                                        beta1 * totalGradVel.transpose() +
                                        beta2 * totalGradAcc.transpose() +
                                        beta3 * totalGradJer.transpose()) *
                                       node * step;
        gradT(i) += (totalGradPos.dot(vel) + totalGradVel.dot(acc) +
                     totalGradAcc.dot(jer) + totalGradJer.dot(sna)) *
                        alpha * node * step +
                    node * integralFrac * pena;
        cost += node * step * pena;
      }
    }

    return;
  }

  static inline double costFunctional(void *ptr, const Eigen::VectorXd &x,
                                      Eigen::VectorXd &g) {
    GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
    const int dimTau = obj.temporalDim;
    const int dimXi = obj.spatialDim;
    const double weightT = obj.rho;
    Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
    Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
    Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
    Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

    forwardT(tau, obj.times);
    forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

    double cost;
    obj.minco.setParameters(obj.points, obj.times);
    obj.minco.getEnergy(cost);
    obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
    obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

    attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(), obj.hPolyIdx,
                            obj.hPolytopes, obj.smoothEps, obj.integralRes,
                            obj.magnitudeBd, obj.penaltyWt, obj.flatmap, cost,
                            obj.partialGradByTimes, obj.partialGradByCoeffs);

    obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                            obj.gradByPoints, obj.gradByTimes);

    cost += weightT * obj.times.sum();
    obj.gradByTimes.array() += weightT;

    backwardGradT(tau, obj.gradByTimes, gradTau);
    backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
    normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

    return cost;
  }

  static inline double costDistance(void *ptr, const Eigen::VectorXd &xi,
                                    Eigen::VectorXd &gradXi) {
    void **dataPtrs = (void **)ptr;
    const double &dEps = *((const double *)(dataPtrs[0]));
    const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
    const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
    const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

    double cost = 0.0;
    const int overlaps = vPolys.size() / 2;

    Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
    Eigen::Vector3d a, b, d;
    Eigen::VectorXd r;
    double smoothedDistance;
    for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k) {
      a = i == 0 ? ini : b;
      if (i < overlaps) {
        k = vPolys[2 * i + 1].cols();
        Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
        r = q.normalized().head(k - 1);
        b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
            vPolys[2 * i + 1].col(0);
      } else {
        b = fin;
      }

      d = b - a;
      smoothedDistance = sqrt(d.squaredNorm() + dEps);
      cost += smoothedDistance;

      if (i < overlaps) {
        gradP.col(i) += d / smoothedDistance;
      }
      if (i > 0) {
        gradP.col(i - 1) -= d / smoothedDistance;
      }
    }

    Eigen::VectorXd unitQ;
    double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
    for (int i = 0, j = 0, k; i < overlaps; i++, j += k) {
      k = vPolys[2 * i + 1].cols();
      Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
      Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
      sqrNormQ = q.squaredNorm();
      invNormQ = 1.0 / sqrt(sqrNormQ);
      unitQ = q * invNormQ;
      gradQ.head(k - 1) =
          (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i))
              .array() *
          unitQ.head(k - 1).array() * 2.0;
      gradQ(k - 1) = 0.0;
      gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

      sqrNormViolation = sqrNormQ - 1.0;
      if (sqrNormViolation > 0.0) {
        c = sqrNormViolation * sqrNormViolation;
        dc = 3.0 * c;
        c *= sqrNormViolation;
        cost += c;
        gradQ += dc * 2.0 * q;
      }
    }

    return cost;
  }

  static inline void getShortestPath(const Eigen::Vector3d &ini,
                                     const Eigen::Vector3d &fin,
                                     const PolyhedraV &vPolys,
                                     const double &smoothD,
                                     Eigen::Matrix3Xd &path) {
    const int overlaps = vPolys.size() / 2;
    Eigen::VectorXi vSizes(overlaps);
    for (int i = 0; i < overlaps; i++) {
      vSizes(i) = vPolys[2 * i + 1].cols();
    }
    Eigen::VectorXd xi(vSizes.sum());
    for (int i = 0, j = 0; i < overlaps; i++) {
      xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
      j += vSizes(i);
    }

    double minDistance;
    void *dataPtrs[4];
    dataPtrs[0] = (void *)(&smoothD);
    dataPtrs[1] = (void *)(&ini);
    dataPtrs[2] = (void *)(&fin);
    dataPtrs[3] = (void *)(&vPolys);
    lbfgs::lbfgs_parameter_t shortest_path_params;
    shortest_path_params.past = 3;
    shortest_path_params.delta = 1.0e-3;
    shortest_path_params.g_epsilon = 1.0e-5;

    lbfgs::lbfgs_optimize(xi, minDistance, &GCOPTER_PolytopeSFC::costDistance,
                          nullptr, nullptr, dataPtrs, shortest_path_params);

    path.resize(3, overlaps + 2);
    path.leftCols<1>() = ini;
    path.rightCols<1>() = fin;
    Eigen::VectorXd r;
    for (int i = 0, j = 0, k; i < overlaps; i++, j += k) {
      k = vPolys[2 * i + 1].cols();
      Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
      r = q.normalized().head(k - 1);
      path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
    }

    return;
  }
  /**
  @brief:半空间转顶点表示
  @param: hPs 输入：半空间表示
  @param: vPs 输出：顶点表示
  */
  static inline bool processCorridor(const PolyhedraH &hPs, PolyhedraV &vPs) {
    const int sizeCorridor = hPs.size() - 1;

    vPs.clear();
    vPs.reserve(2 * sizeCorridor + 1);

    int nv;

    PolyhedronH curIH;
    PolyhedronV curIV, curIOB;
    for (int i = 0; i < sizeCorridor; i++) {
      if (!geo_utils::enumerateVs(hPs[i], curIV)) {
        return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB.col(0) = curIV.col(0);
      curIOB.rightCols(nv - 1) =
          curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);

      curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
      curIH.topRows(hPs[i].rows()) = hPs[i];
      curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
      if (!geo_utils::enumerateVs(curIH, curIV)) {
        return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB.col(0) = curIV.col(0);
      curIOB.rightCols(nv - 1) =
          curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);
    }

    if (!geo_utils::enumerateVs(hPs.back(), curIV)) {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);

    return true;
  }
  /**
  @param path   最短路径点 [3×(N+1)]，N为凸多面体数
                  path.col(0): 起点
                  path.col(i): 第i个凸多面体内的点
                  path.col(N): 终点
  @param speed  分配速度（通常是最大速度的3倍）
                  影响初始时间估计
  @param intervalNs 每个路径段的分段数 [N维]
                  intervalNs(i): 第i段路径分成多少小段
                  例如：intervalNs = [2, 3, 2] 表示
                  第0段分2小段，第1段分3小段，第2段分2小段
  @param innerPoints 输出：中间点 [3×(总段数-1)]
                  所有细分后的路径点（不含起点）
  @param timeAlloc   输出：每段时间 [总段数]
                  每小段的估计时间
  */
  static inline void setInitial(const Eigen::Matrix3Xd &path,
                                const double &speed,
                                const Eigen::VectorXi &intervalNs,
                                Eigen::Matrix3Xd &innerPoints,
                                Eigen::VectorXd &timeAlloc) {
    const int sizeM = intervalNs.size();
    const int sizeN = intervalNs.sum();
    innerPoints.resize(3, sizeN - 1);
    timeAlloc.resize(sizeN);

    Eigen::Vector3d a, b, c;
    for (int i = 0, j = 0, k = 0, l; i < sizeM; i++) {
      l = intervalNs(i);
      a = path.col(i);
      b = path.col(i + 1);
      c = (b - a) / l;
      timeAlloc.segment(j, l).setConstant(c.norm() / speed);
      j += l;
      for (int m = 0; m < l; m++) {
        if (i > 0 || m > 0) {
          innerPoints.col(k++) = a + c * m;
        }
      }
    }
  }

public:
  // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
  // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight,
  // thrust_weight]^T physicalParams = [vehicle_mass,
  // gravitational_acceleration, horitonral_drag_coeff,
  //                   vertical_drag_coeff, parasitic_drag_coeff,
  //                   speed_smooth_factor]^T
  /**
  magnitudeBounds(5) [v_max, ω_max, θ_max, thrust_min, thrust_max]
  penaltyWeights(5) 各成本项权重 [位置权重 速度权重 角速度权重 角度权重
  推力权重] physicalParams(6) 物理参数 [质量 重力加速度 水平阻力系数
  垂直阻力系数 寄生阻力 速度平滑因子]
  */
  /**
  @brief:
  @param: timeWeight 时间权重
  @param: initialPVA 初始状态
  @param: terminalPVA 最终状态
  @param: safeCorridor 凸约束集
  @param: lengthPerPiece 最大时间（无限制）
  @param: smoothingFactor 平滑系数
  @param: integralResolution 离散化点数
  @param: magnitudeBounds 约束边界
  @param: penaltyWeights 惩罚权重
  @param: physicalParams  物理参数
  @return:
  */
  inline bool setup(const double &timeWeight, const Eigen::Matrix3d &initialPVA,
                    const Eigen::Matrix3d &terminalPVA,
                    const PolyhedraH &safeCorridor,
                    const double &lengthPerPiece, const double &smoothingFactor,
                    const int &integralResolution,
                    const Eigen::VectorXd &magnitudeBounds,
                    const Eigen::VectorXd &penaltyWeights,
                    const Eigen::VectorXd &physicalParams) {
    rho = timeWeight;
    headPVA = initialPVA;
    tailPVA = terminalPVA;

    hPolytopes = safeCorridor;
    //归一化所有凸多面体的半空间表示
    for (size_t i = 0; i < hPolytopes.size(); i++) {
      const Eigen::ArrayXd norms = hPolytopes[i].leftCols<3>().rowwise().norm();
      hPolytopes[i].array().colwise() /= norms;
    }
    //半空间转顶点表示
    if (!processCorridor(hPolytopes, vPolytopes)) {
      return false;
    }
    //凸多面体总数
    polyN = hPolytopes.size();

    smoothEps = smoothingFactor;
    integralRes = integralResolution;
    magnitudeBd = magnitudeBounds;
    penaltyWt = penaltyWeights;
    physicalPm = physicalParams;

    allocSpeed = magnitudeBd(0) * 3.0; // 分配速度 = 3倍最大速度
    //最短路径生成
    getShortestPath(headPVA.col(0), // 起点位置
                    tailPVA.col(0), // 终点位置
                    vPolytopes,     // 顶点表示的安全走廊
                    smoothEps,      // 平滑参数
                    shortPath);     // 输出：最短路径点

    // 计算每段的距离
    const Eigen::Matrix3Xd deltas =
        shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
    // 根据期望长度自动分配段数
    pieceIdx =
        (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
    pieceIdx.array() += 1;   // 每段至少1个
    pieceN = pieceIdx.sum(); // 总轨迹段数
    //索引映射建立
    temporalDim = pieceN;        // 时间变量数 = 轨迹段数
    spatialDim = 0;              // 空间变量数（累加）
    vPolyIdx.resize(pieceN - 1); // 每段对应的顶点集索引
    hPolyIdx.resize(pieceN);     // 每段对应的半空间索引
    for (int i = 0, j = 0, k; i < polyN; i++) {
      k = pieceIdx(i); // 当前凸多面体分成的段数
      for (int l = 0; l < k; l++, j++) {
        if (l < k - 1) {
          // 非最后一段：使用当前多面体自身
          vPolyIdx(j) = 2 * i;
          spatialDim += vPolytopes[2 * i].cols();
        } else if (i < polyN - 1) {
          // 最后一段（非最后一个多面体）：使用重叠区域
          vPolyIdx(j) = 2 * i + 1;
          spatialDim += vPolytopes[2 * i + 1].cols();
        }
        // 半空间始终用当前多面体
        hPolyIdx(j) = i;
      }
    }

    // Setup for MINCO_S3NU, FlatnessMap, and L-BFGS solver
    // 设置边界条件：起点和终点的位置、速度、加速度
    minco.setConditions(headPVA, tailPVA, pieceN);
    // 微分平坦性映射 设置物理参数
    flatmap.reset(physicalPm(0), physicalPm(1), physicalPm(2), physicalPm(3),
                  physicalPm(4), physicalPm(5));

    // Allocate temp variables
    points.resize(3, pieceN - 1);       // 中间点（3D，数量=段数-1）
    times.resize(pieceN);               // 每段时间
    gradByPoints.resize(3, pieceN - 1); // 点的梯度
    gradByTimes.resize(pieceN);         // 时间的梯度
    partialGradByCoeffs.resize(6 * pieceN, 3); // 系数梯度
    partialGradByTimes.resize(pieceN);         // 时间梯度部分

    return true;
  }

  inline double optimize(Trajectory<5> &traj, const double &relCostTol) {
    Eigen::VectorXd x(temporalDim + spatialDim);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

    setInitial(shortPath,  // 最短路径点
               allocSpeed, // 分配速度 = 3 * v_max
               pieceIdx,   // 每段分配数
               points,     // 输出：中间点
               times);     // 输出：每段时间

    backwardT(times, tau);                       // 时间：物理→无约束
    backwardP(points, vPolyIdx, vPolytopes, xi); // 空间：物理→无约束

    double minCostFunctional;
    lbfgs_params.mem_size = 256; // 存储256个历史梯度（收敛快）
    lbfgs_params.past = 3;       // 检查过去3次迭代的成本变化
    lbfgs_params.min_step = 1.0e-32; // 最小步长（很小，允许精细调整）
    lbfgs_params.g_epsilon = 0.0;    // 梯度容差（0表示禁用）
    lbfgs_params.delta = relCostTol; // 成本相对变化容差

    int ret =
        lbfgs::lbfgs_optimize(x, // 优化变量（输入/输出）
                              minCostFunctional, // 输出：最优成本
                              &GCOPTER_PolytopeSFC::costFunctional, // 成本函数
                              nullptr, // 进度回调（可选）
                              nullptr, // 数据指针（可选）
                              this,    // 传递给成本函数的this指针
                              lbfgs_params); // L-BFGS参数

    if (ret >= 0) {
      forwardT(tau, times);
      forwardP(xi, vPolyIdx, vPolytopes, points);
      minco.setParameters(points, times);
      minco.getTrajectory(traj);
    } else {
      traj.clear();
      minCostFunctional = INFINITY;
      std::cout << "Optimization Failed: " << lbfgs::lbfgs_strerror(ret)
                << std::endl;
    }

    return minCostFunctional;
  }
};

} // namespace gcopter

#endif
