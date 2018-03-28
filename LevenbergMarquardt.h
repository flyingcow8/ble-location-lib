/***********************************************************************************************************************
* 版权信息：版权所有(c) 2015, 杭州海康威视数字技术股份有限公司，保留所有权利
*
* 文件名称：bls.h
*
* 摘    要：海康威视蓝牙定位算法组件接口文件
*
*
* 当前版本：1.0.0
* 作    者：詹源
* 日    期：2015-06-01
* 备    注：初始版本
***********************************************************************************************************************/

#ifndef HIK_LevenbergMarquardt_LIB_H
#define HIK_LevenbergMarquardt_LIB_H

#include "Eigen/Core"
#include "Eigen/Dense"

#include "unsupported/Eigen/NonLinearOptimization"
#include "unsupported/Eigen/NumericalDiff"

template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>

struct Functor {

    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }
    int values() const { return m_values; }
};

struct distance_functor : Functor<double> {

    Eigen::MatrixXd matrix;

    distance_functor( Eigen::MatrixXd &m, int count) : matrix(m), Functor<double>(count, count) {}

    int operator()(const Eigen::VectorXd &b, Eigen::VectorXd &fvec) const {
        for(int i = 0; i < matrix.rows(); i++) {
//            fvec[i] = 1 - sqrt(pow(matrix(i, 0) - b[0], 2) + pow(matrix(i, 1) - b[1], 2)) / matrix(i, 2);
            fvec[i] = 1 - (pow(matrix(i, 0) - b[0], 2) + pow(matrix(i, 1) - b[1], 2)) / pow(matrix(i, 2), 2);
        }

        return 0;
    }
};

#endif
