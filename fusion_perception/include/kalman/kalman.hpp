#pragma once

#include "iostream"
#include "vector"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
class KalmanModel{
private:
    cv::KalmanFilter KF;
    cv::Mat state_pre,state_;
    cv::Mat F;
    cv::Mat H;
    cv::Mat P;
    cv::Mat Q;
    cv::Mat R;
    int dim_state;
    int dim_measure;
public:
    KalmanModel(int stateNum,int measureNum);
    void kalmanFilterSetup(float dt,float p,float q,float r);
    void kalmaninitstate(cv::Mat &init_state);
    void kalmanPredict();
    void kalmanUpdate(cv::Mat &measures);
    cv::Mat getPreState();
};
KalmanModel::KalmanModel(int stateNum, int measureNum) {
//    std::cout<<"kalman init"<<std::endl;
    dim_measure=measureNum;
    dim_state=stateNum;
    KF=cv::KalmanFilter(stateNum,measureNum);
    F=cv::Mat(dim_state,dim_state,CV_32FC1,cv::Scalar(0));
    H=cv::Mat(dim_measure,dim_state,CV_32FC1,cv::Scalar(0));
    P=cv::Mat(dim_state,dim_state,CV_32FC1,cv::Scalar(0));
    Q=cv::Mat(dim_state,dim_state,CV_32FC1,cv::Scalar(0));
    R=cv::Mat(dim_measure,dim_measure,CV_32FC1,cv::Scalar(0));
}
void KalmanModel::kalmanFilterSetup(float dt, float p, float q, float r) {
    for (int i = 0; i < F.rows; ++i) {
        for (int j = 0; j < F.cols; ++j) {
            if (i==j)
                F.at<float>(i,j)=1.0;
            if (i<dim_state/2){
                if (j-i==dim_state/2)
                    F.at<float>(i,j)=dt;//根据时间间隔设置
            }
        }
    }
    for (int i = 0; i < H.rows; ++i) {
        for (int j = 0; j < H.cols; ++j) {
            if (i==j)
                H.at<float>(i,j)=1.0;
        }
    }
    setIdentity(Q, cv::Scalar::all(q));            //系统噪声方差矩阵Q
    setIdentity(R, cv::Scalar::all(r));        //测量噪声方差矩阵R
    setIdentity(P, cv::Scalar::all(p));                  //后验错误估计协方差矩阵P

    /*
    std::cout<<"Q:"<<std::endl;
    for (int i = 0; i < Q.rows; ++i) {
        for (int j = 0; j < Q.cols; ++j) {
            std::cout<<Q.at<float>(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
    std::cout<<"P:"<<std::endl;
    for (int i = 0; i < P.rows; ++i) {
        for (int j = 0; j < P.cols; ++j) {
            std::cout<<P.at<float>(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
    std::cout<<"R:"<<std::endl;
    for (int i = 0; i < R.rows; ++i) {
        for (int j = 0; j < R.cols; ++j) {
            std::cout<<R.at<float>(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
    std::cout<<"F:"<<std::endl;
    for (int i = 0; i < F.rows; ++i) {
        for (int j = 0; j < F.cols; ++j) {
            std::cout<<F.at<float>(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
    std::cout<<"H:"<<std::endl;
    for (int i = 0; i < H.rows; ++i) {
        for (int j = 0; j < H.cols; ++j) {
            std::cout<<H.at<float>(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
     */
    KF.transitionMatrix=F;                            //转移矩阵A
    KF.measurementMatrix=H;                           //测量矩阵H
    KF.processNoiseCov=Q;                            //系统噪声方差矩阵Q
    KF.measurementNoiseCov=R;                        //测量噪声方差矩阵R
    KF.errorCovPost=P;                                //后验错误估计协方差矩阵P
}
void KalmanModel::kalmaninitstate(cv::Mat &init_state) {
    state_=init_state;
    KF.statePost=init_state;
    KF.statePre=init_state;
//    std::cout<<"state init:"<<std::endl;
//    for (int i=0;i<init_state.rows;i++) {
//        std::cout<<init_state.at<float>(i)<<" ";
//    }
//    std::cout<<std::endl;
}
void KalmanModel::kalmanPredict() {
//    std::cout<<"state before predict:"<<std::endl;
//    for (int i=0;i<KF.statePre.rows;i++) {
//        std::cout<<KF.statePre.at<float>(i)<<" ";
//    }
//    std::cout<<std::endl;

    state_pre=KF.predict();

//    std::cout<<"state after predict::"<<std::endl;
//    for (int i=0;i<KF.statePre.rows;i++) {
//        std::cout<<KF.statePre.at<float>(i)<<" ";
//    }
//    std::cout<<std::endl;
}
void KalmanModel::kalmanUpdate(cv::Mat &measures) {
//    std::cout<<"state before update:"<<std::endl;
//    for (int i=0;i<KF.statePost.rows;i++) {
//        std::cout<<KF.statePost.at<float>(i)<<" ";
//    }
//    std::cout<<std::endl;
    state_=KF.correct(measures);
//    std::cout<<"state measure:"<<std::endl;
//    for (int i=0;i<measures.rows;i++) {
//        std::cout<<measures.at<float>(i)<<" ";
//    }
//    std::cout<<std::endl;
//    std::cout<<"state after update:"<<std::endl;
//    for (int i=0;i<KF.statePost.rows;i++) {
//        std::cout<<KF.statePost.at<float>(i)<<" ";
//    }
//    std::cout<<std::endl;
}
cv::Mat KalmanModel::getPreState(){
    return  state_pre;
}
