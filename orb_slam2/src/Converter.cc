/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Converter.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>

namespace ORB_SLAM2
{
/// for VI-ORB_SLAM2
/********************************************************************************/
/**************************** for VI-ORB_SLAM2 Start ****************************/
/********************************************************************************/

/**
 * @brief 将imupreint积分得到的dR,dP,dV作用到NavState(IMU坐标系)
 * 公式(33)
 */
//void Converter::updateNS(NavState& ns, const IMUPreintegrator& imupreint, const Vector3d& gw)
//{
//    Matrix3d dR = imupreint.getDeltaR();
//    Vector3d dP = imupreint.getDeltaP();
//    Vector3d dV = imupreint.getDeltaV();
//    double dt = imupreint.getDeltaTime();

//    Vector3d Pwbpre = ns.Get_P();
//    Matrix3d Rwbpre = ns.Get_RotMatrix();
//    Vector3d Vwbpre = ns.Get_V();

//    Matrix3d Rwb = Rwbpre * dR;
//    Vector3d Pwb = Pwbpre + Vwbpre*dt + 0.5*gw*dt*dt + Rwbpre*dP;
//    Vector3d Vwb = Vwbpre + gw*dt + Rwbpre*dV;

//    // Here assume that the pre-integration is re-computed after bias updated, so the bias term is ignored
//    ns.Set_Pos(Pwb);
//    ns.Set_Vel(Vwb);
//    ns.Set_Rot(Rwb);

//    // Test log
//    if(ns.Get_dBias_Gyr().norm()>1e-6 || ns.Get_dBias_Acc().norm()>1e-6) std::cerr<<"delta bias in updateNS is not zero"<<ns.Get_dBias_Gyr().transpose()<<", "<<ns.Get_dBias_Acc().transpose()<<std::endl;
//}

void Converter::updateNS(NavState& ns, const IMUPreintegrator& imupreint, const Vector3d& gw)
{
    Matrix3d dR = imupreint.getDeltaR();
    Vector3d dP = imupreint.getDeltaP();
    Vector3d dV = imupreint.getDeltaV();
    double dt = imupreint.getDeltaTime();

    Vector3d Pwbpre = ns.Get_P();
    Matrix3d Rwbpre = ns.Get_RotMatrix();
    Vector3d Vwbpre = ns.Get_V();

    Matrix3d Rwb = Rwbpre * dR;
    Vector3d Pwb = Pwbpre + Vwbpre*dt + 0.5*gw*dt*dt + Rwbpre*dP;
    Vector3d Vwb = Vwbpre + gw*dt + Rwbpre*dV;

    // Here assume that the pre-integration is re-computed after bias updated, so the bias term is ignored
    ns.Set_Pos(Pwb);
    ns.Set_Vel(Vwb);
    ns.Set_Rot(Rwb);

    // Test log
    if(ns.Get_dBias_Gyr().norm()>1e-6 || ns.Get_dBias_Acc().norm()>1e-6) std::cerr<<"delta bias in updateNS is not zero"<<ns.Get_dBias_Gyr().transpose()<<", "<<ns.Get_dBias_Acc().transpose()<<std::endl;
}

void Converter::updateNS(NavState& ns, const IMUPreintegrator& imupreint, const Vector3d& gw,
                         const Eigen::Vector3d& dbiasg, const Eigen::Vector3d& dbiasa)
{
    Matrix3d dR = imupreint.getDeltaR();
    Vector3d dP = imupreint.getDeltaP();
    Vector3d dV = imupreint.getDeltaV();
    double dt = imupreint.getDeltaTime();

    Vector3d Pwbpre = ns.Get_P();
    Matrix3d Rwbpre = ns.Get_RotMatrix();
    Vector3d Vwbpre = ns.Get_V();

    Matrix3d Rwb = Rwbpre * dR;
    Vector3d Pwb = Pwbpre + Vwbpre*dt + 0.5*gw*dt*dt + Rwbpre*dP;
    Vector3d Vwb = Vwbpre + gw*dt + Rwbpre*dV;

//    // Here assume that the pre-integration is re-computed after bias updated, so the bias term is ignored
//    ns.Set_Pos(Pwb);
//    ns.Set_Vel(Vwb);
//    ns.Set_Rot(Rwb);


    // Here we consider the bias updated
    Rwb *= imupreint.Expmap(imupreint.getJRBiasg()*dbiasg);
    Vwb += imupreint.getJVBiasg()*dbiasg + imupreint.getJVBiasa()*dbiasa;
    Pwb += imupreint.getJPBiasg()*dbiasg + imupreint.getJPBiasa()*dbiasa;

    ns.Set_Pos(Pwb);
    ns.Set_Vel(Vwb);
    ns.Set_Rot(Rwb);

    // Test log
    if(ns.Get_dBias_Gyr().norm()>1e-6 || ns.Get_dBias_Acc().norm()>1e-6) std::cerr<<"delta bias in updateNS is not zero"<<ns.Get_dBias_Gyr().transpose()<<", "<<ns.Get_dBias_Acc().transpose()<<std::endl;
}

cv::Mat Converter::toCvMatInverse(const cv::Mat &Tcw)
{
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat twc = -Rwc*tcw;

    cv::Mat Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    twc.copyTo(Twc.rowRange(0,3).col(3));

    return Twc.clone();
}

// part_index: 0, 1, ,,, total_parts-1
std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors, int total_parts, int part_index)
{
//    std::vector<cv::Mat> vDesc;
//    vDesc.reserve(Descriptors.rows);
//    for (int j=0;j<Descriptors.rows;j++)
//        vDesc.push_back(Descriptors.row(j));

//    return vDesc;

    std::vector<cv::Mat> vDesc;
    int i_start, i_end;
    i_start = Descriptors.rows / total_parts * part_index;
    if(part_index == total_parts-1)
        i_end = Descriptors.rows-1;
    else
        i_end = Descriptors.rows / total_parts * (part_index+1) - 1;


    vDesc.reserve(i_end-i_start+1);
    for(int j=i_start; j<=i_end; j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;

}
/********************************************************************************/
/***************************** for VI-ORB_SLAM2 End *****************************/
/********************************************************************************/


std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}


g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix4d Converter::toMatrix4d(const cv::Mat &cvMat4)
{
    Eigen::Matrix<double,3,3> R;
    R <<    cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2),
            cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2),
            cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2);

    Eigen::Matrix<double,3,1> t( cvMat4.at<float>(0,3), cvMat4.at<float>(1,3), cvMat4.at<float>(2,3) );
    Eigen::Matrix4d EigTbc = Eigen::Matrix4d::Identity();
    EigTbc.block<3,3>(0,0) = R;
    EigTbc.block<3,1>(0,3) = t;

    return EigTbc;
}

cv::Mat Converter::toSkew(const Eigen::Vector3d v)
{
//    cv::Mat Omega = cv::Mat::zeros(3,3,CV_32F);
//    Omega.at<float>(0,1) = -v(2);
//    Omega.at<float>(0,2) = v(1);
//    Omega.at<float>(1,2) = -v(0);
//    Omega.at<float>(1,0) = v(2);
//    Omega.at<float>(2,0) = -v(1);
//    Omega.at<float>(2,1) = v(0);

//    return Omega;

    return (cv::Mat_<float>(3,3) <<
            0,  -v(2),  v(1),
            v(2),   0,  -v(0),
            -v(1),  v(0),   0
            );
}

cv::Mat Converter::toSkew(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}


Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}
Eigen::Vector3d Converter::toEulerAngles(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(cvMat3);
    Eigen::Matrix3d rotation_matrix(eigMat);
    Eigen::Vector3d euler = rotation_matrix.eulerAngles(2, 1, 0);

    return euler;
}




} //namespace ORB_SLAM
