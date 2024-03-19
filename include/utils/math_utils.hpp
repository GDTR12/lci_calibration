#pragma once

#include <deque>
#include <map>
#include <unordered_map>
#include <vector>

#include <math.h>
#include <Eigen/Dense>
#include <sophus/so3.hpp>

namespace MathUtils {

template <typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename T>
using aligned_deque = std::deque<T, Eigen::aligned_allocator<T>>;

template <typename K, typename V>
using aligned_map = std::map<K, V, std::less<K>,
                             Eigen::aligned_allocator<std::pair<K const, V>>>;

template <typename K, typename V>
using aligned_unordered_map =
    std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
                       Eigen::aligned_allocator<std::pair<K const, V>>>;

/** sorts vectors from large to small
 * vec: vector to be sorted
 * sorted_vec: sorted results
 * ind: the position of each element in the sort result in the original vector
 * https://www.programmersought.com/article/343692646/
 */
inline void sort_vec(const Eigen::Vector3d& vec, Eigen::Vector3d& sorted_vec,
                     Eigen::Vector3i& ind) {
  ind = Eigen::Vector3i::LinSpaced(vec.size(), 0, vec.size() - 1);  //[0 1 2]
  auto rule = [vec](int i, int j) -> bool {
    return vec(i) > vec(j);
  };  // regular expression, as a predicate of sort

  std::sort(ind.data(), ind.data() + ind.size(), rule);

  // The data member function returns a pointer to the first element of
  // VectorXd, similar to begin()
  for (int i = 0; i < vec.size(); i++) {
    sorted_vec(i) = vec(ind(i));
  }
}

template<typename Scalar = float>
Scalar getSO3Distance(const Sophus::SO3<Scalar> p0, const Sophus::SO3<Scalar> p1){
    Eigen::Vector3f p0_qua = p0.unit_quaternion().vec();
    Eigen::Vector3f p1_qua = p1.unit_quaternion().vec();
    return (p0_qua - p1_qua).norm();
}


// template<typename Scalar = float>
// Scalar getRd3Distance(const So)
// {
// }

template<typename Scalar>
Eigen::Matrix<Scalar, 4, 4> quaLeftMultiMat(
        const Eigen::Quaternion<Scalar>& qua){
    Eigen::Matrix<Scalar, 4, 4> mat = Eigen::Matrix<Scalar, 4, 4>::Zero();   
    // std::cout << qua.w() << " " << qua.vec().transpose() << std::endl;
    mat.block(0,1,1,3) = -qua.vec().transpose();
    mat.block(1,0,3,1) = qua.vec();
    mat.block(1,1,3,3) = (qua.w() * Eigen::Matrix<Scalar, 3, 3>::Identity()) + Sophus::SO3<Scalar>::hat(qua.vec()); 
    mat(0,0) = qua.w();
    return mat;
}

template<typename Scalar>
Eigen::Matrix<Scalar, 4, 4> quaRightMultiMat(
        const Eigen::Quaternion<Scalar>& qua){
    Eigen::Matrix<Scalar, 4, 4> mat = Eigen::Matrix<Scalar, 4, 4>::Zero();   
    mat.block(0,1,1,3) = -qua.vec().transpose();
    mat.block(1,0,3,1) = qua.vec();
    mat.block(1,1,3,3) = -Sophus::SO3<Scalar>::hat(qua.vec()) + qua.w() * Eigen::Matrix<Scalar,3,3>::Identity(); 
    mat(0,0) = qua.w();
    return mat;
}
template<typename Scalar>
Scalar rand_float(Scalar min, Scalar max){
    std::random_device rd;
    std::mt19937 gen(rd());  // Mersenne Twister 引擎
    std::uniform_real_distribution<> dis(min, max);  // 0.0到1.0之间的均匀分布
    return dis(gen);
}

template<typename Scalar>
Eigen::Matrix<Scalar, 4, 1> QuaSubtraction(const Eigen::Quaternion<Scalar>& qua0, const Eigen::Quaternion<Scalar>& qua1)
{
  Eigen::Matrix<Scalar, 4, 1> ret;
  ret << qua0.w() - qua1.w(),
         qua0.x() - qua1.x(),
         qua0.y() - qua1.y(),
         qua0.z() - qua1.z();
  return ret;
}




}  // namespace Eigen
