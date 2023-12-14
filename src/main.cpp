//----------------------------------------------------------------------------------------
//
//  Copyright (c) 2018 Zheng Tang <zhtang@uw.edu>.  All rights reserved.
//
//  Description:
//      Implementation of semi-automatic camera calibration based on Perspective-n-Point
//
//----------------------------------------------------------------------------------------

#include <fstream>
#include "Cfg.h"
#include "CamCal.h"

int main(int argc, char *argv[])
{
	// 定义cv::Mat类型的变量oImgFrm、oCamInParam和ovDistCoeff用于存储图像帧、相机内参和畸变系数
	cv::Mat oImgFrm;
	cv::Mat_<float> oCamInParam;
	cv::Mat_<float> ovDistCoeff;
	// 创建相机校准器CCamCal oCamCal和配置对象CCfg oCfg
	CCamCal oCamCal;
	CCfg oCfg;

	// read configuration file
	// 读取配置文件。如果参数数量超过2个，则显示用法信息并退出；否则，加载配置文件
	if (2 < argc)
	{
		std::cout << "usage: " << argv[0] << " <cfg_file_path>" << std::endl;
		return 0;
	}
	else if (2 == argc)
		oCfg.ldCfgFl(argv[1]);
	else
		oCfg.ldCfgFl(NULL);

	// read frame image
	oImgFrm = cv::imread(oCfg.getInFrmPth(), cv::IMREAD_COLOR);

	// resize frame if necessary
	// 如果配置中指定了调整帧大小，则对图像进行缩放
	if (0 < oCfg.getRszFrmHei())
	{
		cv::Size oFrmSz((((float)oImgFrm.size().width / (float)oImgFrm.size().height) * oCfg.getRszFrmHei()), oCfg.getRszFrmHei());
		cv::resize(oImgFrm, oImgFrm, oFrmSz);
	}

	// correct camera distortion
	// 如果配置中指定了校正相机畸变，使用opencv的undistort函数进行图像畸变校正
	if (oCfg.getCalDistFlg())
	{
		cv::Mat oImgUndist;
		oCamInParam = oCfg.getCalIntMat();
		ovDistCoeff = oCfg.getCalDistCoeffMat();
		cv::undistort(oImgFrm, oImgUndist, oCamInParam, ovDistCoeff);
		oImgFrm = oImgUndist.clone();
	}

	// initialize the camera calibrator
	// 使用配置和读取的图像帧初始化相机校准器
	oCamCal.initialize(oCfg, oImgFrm);

	// run camera calibration
	// 运行相机校准过程
	oCamCal.process();

	// output calibration results
	// 输出校准结果
	oCamCal.output();

	return 0;
}
