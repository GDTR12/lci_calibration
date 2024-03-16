#include "Eigen/Core"
#include "sophus/so3.hpp"
#include "iostream"
#include "utils/math_utils.hpp"
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <iomanip>

using namespace MathUtils;

void testQuaMultiMat()
{
    for(int64_t i = 0; i < 1000000; i++)
    {
        using double100_t = boost::multiprecision::cpp_dec_float_100;
        const Eigen::Quaternion<double100_t> qua0 = Eigen::Quaternion<double100_t>::UnitRandom();
        const Eigen::Quaternion<double100_t> qua2 = Eigen::Quaternion<double100_t>::UnitRandom();
        auto aa0 = qua0 * qua2;

        auto qua2Vec = Eigen::Matrix<double100_t, 4,1>(qua2.w(), qua2.x(),qua2.y(), qua2.z());
        auto qua0Vec = Eigen::Matrix<double100_t, 4,1>(qua0.w(), qua0.x(), qua0.y(), qua0.z());
        // std::cout << qua0.w() << " " << qua0.vec().transpose() << std::endl;
        auto aa1 = MathUtils::quaLeftMultiMat(qua0) * qua2Vec;

        auto aa2 = MathUtils::quaRightMultiMat(qua2) * qua0Vec;

        Eigen::Matrix<double100_t, 4,1> a0(aa0.w(), aa0.vec()[0], aa0.vec()[1], aa0.vec()[2]);

        std::cout << std::fixed << std::setprecision(60);

        std::cout << "test" << std::endl;
        std::cout << a0.transpose() << std::endl;
        std::cout <<aa1.transpose() << std::endl;
        std::cout <<aa2.transpose() << std::endl;
        // 使用 norm会失败
        // if((a0 - aa1).norm() > 1e-11 ||  (a0 - aa2).norm() > 1e-11){
        // }
    }
}

void testEigenQuaAndSophusQua()
{
    Eigen::Quaterniond qua = Eigen::Quaterniond::UnitRandom();
    Sophus::SO3d so3 = qua.matrix();
    Eigen::Quaterniond qua2 = so3.unit_quaternion();
    std::cout << qua.w() << " " << qua.vec() << std::endl;
    std::cout << qua2.w() << " " << qua2.vec() << std::endl;
}

void testFormulation()
{
    using double100_t = double;
    for(int i = 0; i < 10000; i++)
    {
        auto R = Sophus::SO3<double100_t>(Eigen::Quaternion<double100_t>::UnitRandom()).matrix();
        auto theta = Eigen::Matrix<double100_t,3,1>::Random();
        std::cout << "\n";
        std::cout << R << "\n" << R.transpose() << std::endl;
        // std::cout << (R * Sophus::SO3d::hat(theta).matrix() * R.transpose() - Sophus::SO3d::hat(R * theta).matrix()).norm() << std::endl;
        std::cout << "\n";
        auto r1 = R * Sophus::SO3<double100_t>::exp(theta).matrix() * R.transpose();
        auto r2 = Sophus::SO3<double100_t>::exp(R * theta);
        // std::cout << r1 << std::endl;
        // std::cout << r2.matrix() << std::endl;

    }
}

void testQuaMap()
{
    double a[4] = {0,1,2,3};
    Eigen::Map<Eigen::Quaterniond> r(a);
    // Eigen::Map<Eigen::Vector4d> cof(r.coeffs());
    auto d = r.coeffs().data();
    a[3] = 5;
    std::cout << r.w() << " " << r.vec().transpose() << std::endl;
    // std::cout << cof.transpose() << std::endl;
    std::cout << d[0] << " " << d[1] << " " << d[2] << " " << d[3] << std::endl;
}
#include "lci_cali/spline/lidar_spline.hpp"
#include "utils/cpp_utils.hpp"


// int main(int argc, char const *argv[])
// {
//     // testQuaMultiMat();
//     // testEigenQuaAndSophusQua();
//     // testFormulation();
//     // testQuaMap();
//     // Sophus::SO3d::Tangent t;
//     // t.setIdentity();
//     // Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
//     // Eigen::Vector3d v;
//     // v << 1,3,4;
//     // std::cout << (t - v).transpose() << " " << (mat * t).transpose() << std::endl;
//     // std::cout << Sophus::SO3d::hat(v) << std::endl;
//     Sophus::SO3d a;
    


//     // Eigen::Quaterniond q;
//     // q.ma
//     return 0;
// }

#include <unordered_map>
#include <pcl/point_types.h>

using namespace pcl;

// class A{
// public:
//     A(){std::cout << "construct" << std::endl;}
//     A(const A& a){std::cout << "copy construct" << std::endl;}
//     A(A&& a){std::cout << "move construct" << std::endl;}
//     ~A(){std::cout << "delete" << std::endl;}
// };

// void func(std::vector<A>& vec)
// {
//     A a;
//     vec.emplace_back(std::move(a));
// }
#include "utils/ndt_utils.hpp"
#include "utils/ui_utils.hpp"
int main(int argc, char* argv[]) {
    // using PointT = pcl::PointXYZ;
	// //创建点云对象PointCloud<PointXYZ> boost共享指针并进行实例化
	// PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	// //点云生成
	// cloud->width = 100230;
	// cloud->height = 1;    //无序点云
	// cloud->points.resize(cloud->width*cloud->height); //总的点数

	// for (size_t i = 0; i < cloud->points.size(); i++)
	// {
	// 	cloud->points[i].x = 12.0f*rand() / (RAND_MAX + 1.0f);
	// 	cloud->points[i].y = 12.0f*rand() / (RAND_MAX + 1.0f);
	// 	cloud->points[i].z = 12.0f*rand() / (RAND_MAX + 1.0f);
	// }
    // slam_utils::showPcl(*cloud);

    // pclomp::NormalDistributionsTransform<PointT, PointT> ndt;
    // ndt.setResolution(2);
    // ndt.setStepSize(0.1);
    // ndt.setTransformationEpsilon(0.01);
    // ndt.setMaximumIterations(30);
    // ndt.setNumThreads(10);
    // ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);

    // ndt.setInputTarget(cloud);
    // // for(const auto& leaf_ : ndt.getTargetCells().getLeaves()){
    // //     auto count = leaf_.first;
    // //     auto leaf = leaf_.second.pointList_.makeShared();
    // //     // std::cout << count << " ," << leaf->size()<< std::endl;
    // //     // slam_utils::showPcl(*leaf);
    // // }
    // auto centroids = ndt.getTargetCells().getCentroids();
    // std::cout << centroids->size() << std::endl;
    // slam_utils::showPcl(*cloud,*centroids);
    // std::vector<A> vec;
    // func(vec); 

    // Sophus::SE3d se3(Eigen::Quaterniond::UnitRandom(), Sophus::Vector3d(1,2,3));
    // Sophus::SO3d::exp() 

    Eigen::Quaterniond qua(0.993, 0.088, 0.055, 0.057);
    std::cout << qua.w() << " " << qua.vec().transpose() << std::endl;
    auto qua_inverse = qua.inverse();
    double* a = qua.coeffs().data();
    std::cout << "p: " << qua.coeffs().data() << std::endl;
    std::cout << qua.w() << " " << qua.vec().transpose() << std::endl;
    std::cout << qua_inverse.w() << " " << qua_inverse.vec().transpose() << std::endl;
    // double* p = qua.coeffs().data();
    // std::cout << p[0] << " " << p[1] << " " <<  p[2] << " " << p[3] << std::endl;
    // std::cout << qua.vec().transpose() << " " << qua.w() << std::endl;
    // Eigen::Map<Eigen::Quaterniond> qua2(p);
    // std::cout << qua2.vec() << " " << qua2.w() << std::endl;
    
    // std::cout << se3.matrix() << std::endl;
    // std::cout << se3.log().transpose() << std::endl;
    // std::cout << se3.so3().log().transpose() << std::endl;
    // std::cout << se3.translation().transpose() << std::endl;
    // Eigen::Vector3d a,b;
    // a << 1,2,3;
    // b << 2,3,4;
    // auto mat = Eigen::Matrix4d::Random();
    // Sophus::SE3d se3(mat);
    // std::cout << mat.block<3,1>(0,3).transpose() << std::endl;
    // std::cout << se3.log().transpose() << std::endl;
    // std::cout << a.array().pow(2) << std::endl;
    // std::cout << b.dot(Eigen::Vector3d(a.array().pow(2))) << std::endl;
    // std::cout << "still alive" << std::endl; 
    // std::unordered_map<std::string, int> l;

    // std::cout << sizeof(pcl::PointXYZ) << std::endl;
    // pcl::PointCloud<pcl::PointXYZ> pcl;
    // for(int i = 0; i < 1000; i++)
    // {
    //     pcl::PointXYZ x((float)rand()/RAND_MAX, (float)rand()/RAND_MAX, (float)rand()/RAND_MAX);
    //     pcl.push_back(x);
    //     l[pclSerialization(x)] = i;
    // }
    // for(int i = 0; i < 1000; i++)
    // {
    //     std::cout << l[pclSerialization(pcl.at(i))] << " ";
    // }
    // std::cout << l.size() << std::endl;
    // std::cout << l.max_size() << std::endl;
    // std::cout << sizeof(l)/56.0 << std::endl;
    // sleep(10);
    return 0;

}

