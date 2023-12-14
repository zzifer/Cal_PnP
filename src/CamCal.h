#pragma once

#include <opencv2/calib3d/calib3d.hpp>
#include "Cfg.h"

// camera calibrator
class CCamCal
{
public:
	// 构造函数和析构函数
	CCamCal(void);
	~CCamCal(void);

	//! initializes the calibrator 初始化校准器，接收配置参数和图像帧
	void initialize(CCfg oCfg, cv::Mat oImgFrm);
	//! perform camera calibration 执行相机校准过程
	void process(void);
	//! output homography matrix and display image 输出校准结果，包括单应矩阵和显示图像
	void output(void);

private:
	//! runs all calibration types
	void runAllCalTyp(void);
	//! calculates reprojection error 计算重投影误差
	double calcReprojErr(cv::Mat oHomoMat, int nCalTyp, double fCalRansacReprojThld);
	//! outputs text file of homography matrix 输出文本文件的单应矩阵
	void outTxt(void);
	//! plots a display grid on the ground plane 在地面上绘制显示网格
	void pltDispGrd(void);

	//! configuration parameters
	CCfg m_oCfg;
	//! frame image
	cv::Mat m_oImgFrm;
	//! list of 3D points for PnP 用于透视-n-点（PnP）的3D和2D点列表
	std::vector<cv::Point2f> m_vo3dPt;
	//! list of 2D points for PnP
	std::vector<cv::Point2f> m_vo2dPt;
	//! homography matrix
	cv::Mat m_oHomoMat;
	//! reprojection error 重投影误差
	double m_fReprojErr;
};

// selector of 2D points for PnP 主要用于选择和管理在图像上用于相机校准的二维点
class C2dPtSel
{
public:
	C2dPtSel(void) = default;
	~C2dPtSel(void) = default;

	//! initializes the 2D point selector
	void initialize(CCfg oCfg, cv::Mat oImgFrm);
	//! selects 2D points 执行点选择流程，返回选择的点列表
	std::vector<cv::Point> process(void);
	//! pushes a node to the list and draw the circle 添加一个节点（点）到列表并在图像上绘制圆圈
	void addNd(int nX, int nY);
	//! checks if the background image is loaded 内联函数，检查图像是否已经加载
	inline bool chkImgLd(void)
	{
		if (!m_oImgFrm.empty())
			return true;
		else
			return false;
	}

private:
	//! configuration parameters
	CCfg m_oCfg;
	//! frame image for plotting results  用于绘制结果的图像帧
	cv::Mat m_oImgFrm;
	//! list of nodes 节点（点）的列表
	std::vector<cv::Point> m_voNd;
};

extern C2dPtSel o2dPtSel;
