/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap/core/Odometry.h"
#include <rtabmap/core/odometry/OdometryF2M.h>
#include "rtabmap/core/odometry/OdometryF2F.h"
#include "rtabmap/core/odometry/OdometryFovis.h"
#include "rtabmap/core/odometry/OdometryViso2.h"
#include "rtabmap/core/odometry/OdometryDVO.h"
#include "rtabmap/core/odometry/OdometryOkvis.h"
#include "rtabmap/core/odometry/OdometryORBSLAM3.h"
#include "rtabmap/core/odometry/OdometryLOAM.h"
#include "rtabmap/core/odometry/OdometryFLOAM.h"
#include "rtabmap/core/odometry/OdometryMSCKF.h"
#include "rtabmap/core/odometry/OdometryVINS.h"
#include "rtabmap/core/odometry/OdometryOpenVINS.h"
#include "rtabmap/core/odometry/OdometryOpen3D.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_mapping.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UProcessInfo.h"
#include "rtabmap/core/ParticleFilter.h"
#include "rtabmap/core/util2d.h"

#include <pcl/pcl_base.h>
#include <rtabmap/core/odometry/OdometryORBSLAM2.h>

namespace rtabmap {

// 此方法根据传入的 parameters 创建一个里程计对象，并返回。它先解析并确定使用的里程计策略类型，然后调用另一个重载的 create 方法来创建具体的里程计类型对象。
Odometry * Odometry::create(const ParametersMap & parameters)
{	
	// 获取 odometry 类型的整数值，默认为默认的里程计策略
	int odomTypeInt = Parameters::defaultOdomStrategy();
	// 从参数中解析 odometry 类型，默认为 Parameters::kOdomStrategy()
	Parameters::parse(parameters, Parameters::kOdomStrategy(), odomTypeInt);
	// 将类型转换为枚举类型
	Odometry::Type type = (Odometry::Type)odomTypeInt;
	return create(type, parameters);
}

Odometry * Odometry::create(Odometry::Type & type, const ParametersMap & parameters)
{
	UDEBUG("type=%d", (int)type);
	Odometry * odometry = 0;
	 // 根据类型创建不同的里程计对象
	switch(type)
	{
	case Odometry::kTypeF2M:
		odometry = new OdometryF2M(parameters);
		break;
	case Odometry::kTypeF2F:
		odometry = new OdometryF2F(parameters);
		break;
	case Odometry::kTypeFovis:
		odometry = new OdometryFovis(parameters);
		break;
	case Odometry::kTypeViso2:
		odometry = new OdometryViso2(parameters);
		break;
	case Odometry::kTypeDVO:
		odometry = new OdometryDVO(parameters);
		break;
	case Odometry::kTypeORBSLAM:
#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 2
		odometry = new OdometryORBSLAM2(parameters);
#else
		odometry = new OdometryORBSLAM3(parameters);
#endif
		break;
	case Odometry::kTypeOkvis:
		odometry = new OdometryOkvis(parameters);
		break;
	case Odometry::kTypeLOAM:
		odometry = new OdometryLOAM(parameters);
		break;
	case Odometry::kTypeFLOAM:
		odometry = new OdometryFLOAM(parameters);
		break;
	case Odometry::kTypeMSCKF:
		odometry = new OdometryMSCKF(parameters);
		break;
	case Odometry::kTypeVINS:
		odometry = new OdometryVINS(parameters);
		break;
	case Odometry::kTypeOpenVINS:
		odometry = new OdometryOpenVINS(parameters);
		break;
	case Odometry::kTypeOpen3D:
		odometry = new OdometryOpen3D(parameters);
		break;
	default:
	 	// 如果类型不明，打印错误信息并默认创建 OdometryF2M 类型的对象
		UERROR("Unknown odometry type %d, using F2M instead...", (int)type);
		odometry = new OdometryF2M(parameters);
		type = Odometry::kTypeF2M;
		break;
	}
	return odometry;
}

Odometry::Odometry(const rtabmap::ParametersMap & parameters) :
		_resetCountdown(Parameters::defaultOdomResetCountdown()),
		_force3DoF(Parameters::defaultRegForce3DoF()),
		_holonomic(Parameters::defaultOdomHolonomic()),
		guessFromMotion_(Parameters::defaultOdomGuessMotion()),
		guessSmoothingDelay_(Parameters::defaultOdomGuessSmoothingDelay()),
		_filteringStrategy(Parameters::defaultOdomFilteringStrategy()),
		_particleSize(Parameters::defaultOdomParticleSize()),
		_particleNoiseT(Parameters::defaultOdomParticleNoiseT()),
		_particleLambdaT(Parameters::defaultOdomParticleLambdaT()),
		_particleNoiseR(Parameters::defaultOdomParticleNoiseR()),
		_particleLambdaR(Parameters::defaultOdomParticleLambdaR()),
		_fillInfoData(Parameters::defaultOdomFillInfoData()),
		_kalmanProcessNoise(Parameters::defaultOdomKalmanProcessNoise()),
		_kalmanMeasurementNoise(Parameters::defaultOdomKalmanMeasurementNoise()),
		_imageDecimation(Parameters::defaultOdomImageDecimation()),
		_alignWithGround(Parameters::defaultOdomAlignWithGround()),
		_publishRAMUsage(Parameters::defaultRtabmapPublishRAMUsage()),
		_imagesAlreadyRectified(Parameters::defaultRtabmapImagesAlreadyRectified()),
		_deskewing(Parameters::defaultOdomDeskewing()),
		_pose(Transform::getIdentity()),
		_resetCurrentCount(0),
		previousStamp_(0),
		distanceTravelled_(0),
		framesProcessed_(0)
{
	Parameters::parse(parameters, Parameters::kOdomResetCountdown(), _resetCountdown);

	Parameters::parse(parameters, Parameters::kRegForce3DoF(), _force3DoF);
	Parameters::parse(parameters, Parameters::kOdomHolonomic(), _holonomic);
	Parameters::parse(parameters, Parameters::kOdomGuessMotion(), guessFromMotion_);
	Parameters::parse(parameters, Parameters::kOdomGuessSmoothingDelay(), guessSmoothingDelay_);
	Parameters::parse(parameters, Parameters::kOdomFillInfoData(), _fillInfoData);
	Parameters::parse(parameters, Parameters::kOdomFilteringStrategy(), _filteringStrategy);
	Parameters::parse(parameters, Parameters::kOdomParticleSize(), _particleSize);
	Parameters::parse(parameters, Parameters::kOdomParticleNoiseT(), _particleNoiseT);
	Parameters::parse(parameters, Parameters::kOdomParticleLambdaT(), _particleLambdaT);
	Parameters::parse(parameters, Parameters::kOdomParticleNoiseR(), _particleNoiseR);
	Parameters::parse(parameters, Parameters::kOdomParticleLambdaR(), _particleLambdaR);
	UASSERT(_particleNoiseT>0);
	UASSERT(_particleLambdaT>0);
	UASSERT(_particleNoiseR>0);
	UASSERT(_particleLambdaR>0);
	Parameters::parse(parameters, Parameters::kOdomKalmanProcessNoise(), _kalmanProcessNoise);
	Parameters::parse(parameters, Parameters::kOdomKalmanMeasurementNoise(), _kalmanMeasurementNoise);
	Parameters::parse(parameters, Parameters::kOdomImageDecimation(), _imageDecimation);
	Parameters::parse(parameters, Parameters::kOdomAlignWithGround(), _alignWithGround);
	Parameters::parse(parameters, Parameters::kRtabmapPublishRAMUsage(), _publishRAMUsage);
	Parameters::parse(parameters, Parameters::kRtabmapImagesAlreadyRectified(), _imagesAlreadyRectified);
	Parameters::parse(parameters, Parameters::kOdomDeskewing(), _deskewing);

	if(_imageDecimation == 0)
	{
		_imageDecimation = 1;
	}

	if(_filteringStrategy == 2)
	{
		// Initialize the Particle filters
		particleFilters_.resize(6);
		for(unsigned int i = 0; i<particleFilters_.size(); ++i)
		{
			if(i<3)
			{
				particleFilters_[i] = new ParticleFilter(_particleSize, _particleNoiseT, _particleLambdaT);
			}
			else
			{
				particleFilters_[i] = new ParticleFilter(_particleSize, _particleNoiseR, _particleLambdaR);
			}
		}
	}
	else if(_filteringStrategy == 1)
	{
		initKalmanFilter();
	}
}

Odometry::~Odometry()
{
	for(unsigned int i=0; i<particleFilters_.size(); ++i)
	{
		delete particleFilters_[i];
	}
}

void Odometry::reset(const Transform & initialPose)
{
	UDEBUG("");
	UASSERT(!initialPose.isNull());
	previousVelocities_.clear();
	velocityGuess_.setNull();
	previousGroundTruthPose_.setNull();
	_resetCurrentCount = 0;
	previousStamp_ = 0;
	distanceTravelled_ = 0;
	framesProcessed_ = 0;
	imuLastTransform_.setNull();
	imus_.clear();
	if(_force3DoF || particleFilters_.size())
	{
		float x,y,z, roll,pitch,yaw;
		initialPose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);

		if(_force3DoF)
		{
			if(z != 0.0f || roll != 0.0f || pitch != 0.0f)
			{
				UWARN("Force2D=true and the initial pose contains z, roll or pitch values (%s). They are set to null.", initialPose.prettyPrint().c_str());
			}
			z = 0;
			roll = 0;
			pitch = 0;
			Transform pose(x, y, z, roll, pitch, yaw);
			_pose = pose;
		}
		else
		{
			_pose = initialPose;
		}

		if(particleFilters_.size())
		{
			UASSERT(particleFilters_.size() == 6);
			particleFilters_[0]->init(x);
			particleFilters_[1]->init(y);
			particleFilters_[2]->init(z);
			particleFilters_[3]->init(roll);
			particleFilters_[4]->init(pitch);
			particleFilters_[5]->init(yaw);
		}

		if(_filteringStrategy == 1)
		{
			initKalmanFilter(initialPose);
		}
	}
	else
	{
		_pose = initialPose;
	}
}

const Transform & Odometry::previousVelocityTransform() const
{
	return getVelocityGuess();
}

Transform getMeanVelocity(const std::list<std::pair<std::vector<float>, double> > & transforms)
{
	if(transforms.size())
	{
		float tvx=0.0f,tvy=0.0f,tvz=0.0f, tvroll=0.0f,tvpitch=0.0f,tvyaw=0.0f;
		for(std::list<std::pair<std::vector<float>, double> >::const_iterator iter=transforms.begin(); iter!=transforms.end(); ++iter)
		{
			UASSERT(iter->first.size() == 6);
			tvx+=iter->first[0];
			tvy+=iter->first[1];
			tvz+=iter->first[2];
			tvroll+=iter->first[3];
			tvpitch+=iter->first[4];
			tvyaw+=iter->first[5];
		}
		tvx/=float(transforms.size());
		tvy/=float(transforms.size());
		tvz/=float(transforms.size());
		tvroll/=float(transforms.size());
		tvpitch/=float(transforms.size());
		tvyaw/=float(transforms.size());
		return Transform(tvx, tvy, tvz, tvroll, tvpitch, tvyaw);
	}
	return Transform();
}

Transform Odometry::process(SensorData & data, OdometryInfo * info)
{
	// 调用带有空变换的处理方法，默认假设没有初始猜测变换
	return process(data, Transform(), info);
}

Transform Odometry::process(SensorData & data, const Transform & guessIn, OdometryInfo * info)
{
	// 确保数据id有效
	UASSERT_MSG(data.id() >= 0, uFormat("Input data should have ID greater or equal than 0 (id=%d)!", data.id()).c_str());

	// cache imu data
	// 如果imu不为空，如果不能异步处理
	if(!data.imu().empty() && !this->canProcessAsyncIMU())
	{
		if(!(data.imu().orientation()[0] == 0.0 && data.imu().orientation()[1] == 0.0 && data.imu().orientation()[2] == 0.0))
		{
			// 创建IMU的局部变换，包括姿态（只包含滚转和俯仰，不包括偏航）
			Transform orientation(0,0,0, data.imu().orientation()[0], data.imu().orientation()[1], data.imu().orientation()[2], data.imu().orientation()[3]);
			// orientation includes roll and pitch but not yaw in local transform
			// 计算imu的变换 将 IMU 数据从 原始传感器坐标系 转换到 目标坐标系
			Transform imuT = Transform(data.imu().localTransform().x(),data.imu().localTransform().y(),data.imu().localTransform().z(), 0,0,data.imu().localTransform().theta()) *
					orientation*
					data.imu().localTransform().rotation().inverse();

			// 初始化位姿
			// 如果位姿的旋转矩阵是单位矩阵并且是第一次处理帧，更新初始位姿
			if(	this->getPose().r11() == 1.0f && this->getPose().r22() == 1.0f && this->getPose().r33() == 1.0f &&
				this->framesProcessed() == 0)
			{
				Eigen::Quaterniond imuQuat = imuT.getQuaterniond();
				Transform previous = this->getPose();
				Transform newFramePose = Transform(previous.x(), previous.y(), previous.z(), imuQuat.x(), imuQuat.y(), imuQuat.z(), imuQuat.w());
				UWARN("Updated initial pose from %s to %s with IMU orientation", previous.prettyPrint().c_str(), newFramePose.prettyPrint().c_str());
				this->reset(newFramePose);
			}

			// 缓存IMU数据
			imus_.insert(std::make_pair(data.stamp(), imuT));
			if(imus_.size() > 1000) // 限制缓存大小，防止内存泄漏
			{
				imus_.erase(imus_.begin());
			}
		}
		else
		{
			UWARN("Received IMU doesn't have orientation set! It is ignored.");
		}
	}

	// 图像数据处理
	if(!data.imageRaw().empty())
	{
		UDEBUG("Processing image data %dx%d: rgbd models=%ld, stereo models=%ld",
			data.imageRaw().cols,
			data.imageRaw().rows,
			data.cameraModels().size(),
			data.stereoCameraModels().size());
	}

	// 如果图像没有矫正且不能处理原始图像，进行立体图像矫正
	if(!_imagesAlreadyRectified && !this->canProcessRawImages() && !data.imageRaw().empty())
	{
		if(!data.stereoCameraModels().empty()) // 如果使用的是立体相机模型
		{
			bool valid = true;
			if(data.stereoCameraModels().size() != stereoModels_.size()) // 检查模型是否一致
			{
				stereoModels_.clear();
				valid = false;
			}
			else
			{
				// 遍历数据中的每个立体相机模型，检查其是否初始化了校正映射，并且当前立体模型的左图像尺寸是否与数据中的左图像尺寸匹配
				for(size_t i=0; i<data.stereoCameraModels().size() && valid; ++i)
				{
					// 检查当前立体模型的校正映射是否已经初始化，并且左图像的尺寸是否与数据中的左图像尺寸匹配
					valid = stereoModels_[i].isRectificationMapInitialized() &&
							stereoModels_[i].left().imageSize() == data.stereoCameraModels()[i].left().imageSize();
				}
			}

			// 如果模型无效，执行以下操作
			if(!valid)
			{
				// 将数据中的立体相机模型复制到本地的stereoModels_中
				stereoModels_ = data.stereoCameraModels();
				valid = true;
				// 初始化每个相机模型的校正映射，并检查是否成功初始化
				for(size_t i=0; i<stereoModels_.size() && valid; ++i)
				{
					stereoModels_[i].initRectificationMap();
					// 检查校正映射是否初始化成功
					valid = stereoModels_[i].isRectificationMapInitialized();
				}
				// 如果所有模型的校正映射都成功初始化
				if(valid)
				{
					UWARN("%s parameter is set to false but the selected odometry approach cannot "
							"process raw stereo images. We will rectify them for convenience.",
							Parameters::kRtabmapImagesAlreadyRectified().c_str());
				}
				else
				{
					UERROR("Odometry approach chosen cannot process raw stereo images (not rectified images) "
							"and we cannot rectify them as the rectification map failed to initialize (valid calibration?). "
							"Make sure images are rectified and set %s parameter back to true, or "
							"make sure calibration is valid for rectification",
							Parameters::kRtabmapImagesAlreadyRectified().c_str());
					// 清空立体相机模型列表
					stereoModels_.clear();
				}
			}
			if(valid)
			{
				// 如果只有一个立体相机模型
				if(stereoModels_.size()==1)
				{
					data.setStereoImage(
							stereoModels_[0].left().rectifyImage(data.imageRaw()),
							stereoModels_[0].right().rectifyImage(data.rightRaw()),
							stereoModels_,
							false);
				}
				else
				{
					// 校正多个相机模型的图像
					UASSERT(int((data.imageRaw().cols/data.stereoCameraModels().size())*data.stereoCameraModels().size()) == data.imageRaw().cols);
					 // 计算每个子图像的宽度
					int subImageWidth = data.imageRaw().cols/data.stereoCameraModels().size();
					// 克隆原始图像，用于存储校正后的图像
					cv::Mat rectifiedLeftImages = data.imageRaw().clone();
					cv::Mat rectifiedRightImages = data.imageRaw().clone();
					// 对每个立体相机模型进行图像校正
					for(size_t i=0; i<stereoModels_.size() && valid; ++i)
					{
						// 对左图像和右图像分别进行校正
						cv::Mat rectifiedLeft = stereoModels_[i].left().rectifyImage(cv::Mat(data.imageRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows)));
						cv::Mat rectifiedRight = stereoModels_[i].right().rectifyImage(cv::Mat(data.rightRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, data.rightRaw().rows)));
						// 将校正后的图像复制到最终的图像矩阵中
						rectifiedLeft.copyTo(cv::Mat(rectifiedLeftImages, cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows)));
						rectifiedRight.copyTo(cv::Mat(rectifiedRightImages, cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows)));
					}
					data.setStereoImage(rectifiedLeftImages, rectifiedRightImages, stereoModels_, false);
				}
			}
		}
		// 如果没有立体相机模型，则跳过
		else if(!data.cameraModels().empty())
		{
			bool valid = true;
			// 如果相机模型的数量与当前存储的模型数量不相同，清空现有模型
			if(data.cameraModels().size() != models_.size())
			{
				models_.clear();
				valid = false;
			}
			else
			{
				// 校验每个相机模型的校正映射是否已初始化，并且图像尺寸是否一致
				for(size_t i=0; i<data.cameraModels().size() && valid; ++i)
				{
					valid = models_[i].isRectificationMapInitialized() &&
							models_[i].imageSize() == data.cameraModels()[i].imageSize();
				}
			}

			// 如果校验失败，则尝试初始化新的模型并进行图像校正
			if(!valid)
			{
				models_ = data.cameraModels();
				valid = true;
				// 初始化新的相机模型的校正映射
				for(size_t i=0; i<models_.size() && valid; ++i)
				{
					valid = models_[i].initRectificationMap();
				}
				if(valid) // 如果校正映射初始化成功，显示警告并继续处理
				{
					UWARN("%s parameter is set to false but the selected odometry approach cannot "
							"process raw images. We will rectify them for convenience (only "
							"rgb is rectified, we assume depth image is already rectified!).",
							Parameters::kRtabmapImagesAlreadyRectified().c_str());
				}
				else // 如果校正映射初始化失败，显示错误并清空模型
				{
					UERROR("Odometry approach chosen cannot process raw images (not rectified images) "
							"and we cannot rectify them as the rectification map failed to initialize (valid calibration?). "
							"Make sure images are rectified and set %s parameter back to true, or "
							"make sure calibration is valid for rectification",
							Parameters::kRtabmapImagesAlreadyRectified().c_str());
					models_.clear();
				}
			}
			// 如果校正成功，进行图像的处理和设置
			if(valid)
			{
				// Note that only RGB image is rectified, the depth image is assumed to be already registered to rectified RGB camera.
				// 只有RGB图像被校正，深度图像假设已经与校正的RGB相机对齐
				if(models_.size()==1)
				{
					// 对单个模型进行图像校正
					data.setRGBDImage(models_[0].rectifyImage(data.imageRaw()), data.depthRaw(), models_, false);
				}
				else
				{
					// 校正多个模型的图像
					UASSERT(int((data.imageRaw().cols/data.cameraModels().size())*data.cameraModels().size()) == data.imageRaw().cols);
					int subImageWidth = data.imageRaw().cols/data.cameraModels().size(); // 每个图像子部分的宽度
					cv::Mat rectifiedImages = data.imageRaw().clone(); // 克隆原始图像
					for(size_t i=0; i<models_.size() && valid; ++i)
					{
						// 对每个子图像进行校正
						cv::Mat rectifiedImage = models_[i].rectifyImage(cv::Mat(data.imageRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows)));
						rectifiedImage.copyTo(cv::Mat(rectifiedImages, cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows))); // 将校正后的图像拷贝到完整的校正图像矩阵中
					}
					data.setRGBDImage(rectifiedImages, data.depthRaw(), models_, false);
				}
			}
		}
		else
		{
			UERROR("Odometry approach chosen cannot process raw images (not rectified images). Make sure images "
					"are rectified, and set %s parameter back to true, or make sure that calibration is valid "
					"for rectification so we can rectifiy them for convenience",
					Parameters::kRtabmapImagesAlreadyRectified().c_str());
		}
	}

	// Ground alignment
	// 如果位姿为初始状态（0, 0, 0），且尚未处理任何帧，并且启用了与地面对齐的选项
	if(_pose.x() == 0 && _pose.y() == 0 && _pose.z() == 0 && this->framesProcessed() == 0 && _alignWithGround)
	{
		// 如果深度信息为空，则跳过地面对齐过程
		if(data.depthOrRightRaw().empty())
		{
			UWARN("\"%s\" is true but the input has no depth information, ignoring alignment with ground...", Parameters::kOdomAlignWithGround().c_str());
		}
		else
		{
			// 创建一个定时器，用于计算对齐过程的时间
			UTimer alignTimer;
			pcl::IndicesPtr indices(new std::vector<int>); // 用于存储点云的索引
			pcl::IndicesPtr ground, obstacles; // 用于存储分割出的地面和障碍物点云
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::cloudFromSensorData(data, 1, 10, 0, indices.get()); // 从传感器数据创建点云
			bool success = false; // 用于标识地面对齐是否成功
			// 如果点云数据非空，进行点云处理
			if(indices->size())
			{
				// 对点云进行体素化（降低点云分辨率，减少计算量）
				cloud = util3d::voxelize(cloud, indices, 0.01);
				// 如果已有位姿，则将点云变换到当前位姿下
				if(!_pose.isIdentity())
				{
					// In case we are already aligned with gravity
					cloud = util3d::transformPointCloud(cloud, _pose);
				}
				// 分割点云中的地面和障碍物
				util3d::segmentObstaclesFromGround<pcl::PointXYZ>(cloud, ground, obstacles, 20, M_PI/4.0f, 0.02, 200, true);
				// 如果分割出了地面点云
				if(ground->size())
				{
					 // 提取地面平面模型的系数
					pcl::ModelCoefficients coefficients;
					util3d::extractPlane(cloud, ground, 0.02, 100, &coefficients);
					// 根据平面系数判断地面或天花板
					if(coefficients.values.at(3) >= 0) // 如果检测到地面，输出警告信息
					{
						UWARN("Ground detected! coefficients=(%f, %f, %f, %f) time=%fs",
								coefficients.values.at(0),
								coefficients.values.at(1),
								coefficients.values.at(2),
								coefficients.values.at(3),
								alignTimer.ticks());
					}
					else // 如果检测到天花板，输出警告信息
					{
						UWARN("Ceiling detected! coefficients=(%f, %f, %f, %f) time=%fs",
								coefficients.values.at(0),
								coefficients.values.at(1),
								coefficients.values.at(2),
								coefficients.values.at(3),
								alignTimer.ticks());
					}
					// 获取平面法向量（地面或天花板）
					Eigen::Vector3f n(coefficients.values.at(0), coefficients.values.at(1), coefficients.values.at(2));
					Eigen::Vector3f z(0,0,1); // z轴单位向量
					//get rotation from z to n;
					// 获取从z轴到n向量的旋转矩阵
					Eigen::Matrix3f R;
					R = Eigen::Quaternionf().setFromTwoVectors(n,z);
					// 如果位姿是初始状态（单位矩阵），则进行地面对齐
					if(_pose.r11() == 1.0f && _pose.r22() == 1.0f && _pose.r33() == 1.0f)
					{
						// 创建地面对齐后的变换
						Transform rotation(
								R(0,0), R(0,1), R(0,2), 0,
								R(1,0), R(1,1), R(1,2), 0,
								R(2,0), R(2,1), R(2,2), coefficients.values.at(3));
						this->reset(rotation); // 重置位姿
					}
					else
					{
						// Rotation is already set (e.g., from IMU/gravity), just update Z
						// 如果位姿已经初始化（如通过IMU），则仅更新z坐标
						UWARN("Rotation was already initialized, just offseting z to %f", coefficients.values.at(3));
						Transform pose = _pose;
						pose.z() = coefficients.values.at(3);
						this->reset(pose);
					}
					success = true;
				}
			}
			if(!success)
			{
				UERROR("Odometry failed to detect the ground. You have this "
						"error because parameter \"%s\" is true. "
						"Make sure the camera is seeing the ground (e.g., tilt ~30 "
						"degrees toward the ground).", Parameters::kOdomAlignWithGround().c_str());
			}
		}
	}

	// KITTI datasets start with stamp=0
	// 计算时间间隔dt
	// 如果previousStamp_大于0.0，则时间间隔为当前时间戳与上一帧时间戳的差值
	// 如果previousStamp_为0.0且已处理了第一帧，则认为时间间隔为0
	// 否则，时间间隔dt为0
	double dt = previousStamp_>0.0f || (previousStamp_==0.0f && framesProcessed()==1)?data.stamp() - previousStamp_:0.0;
	// 根据时间间隔和猜测是否来自运动的条件来决定是否设置初始猜测（默认为单位变换）
	Transform guess = dt>0.0 && guessFromMotion_ && !velocityGuess_.isNull()?Transform::getIdentity():Transform();
	// 如果时间间隔无效，且不是从运动中进行猜测的情况
	if(!(dt>0.0 || (dt == 0.0 && velocityGuess_.isNull())))
	{
		// 如果启用了从运动中进行猜测，但当前时间间隔无效，输出错误信息
		if(guessFromMotion_ && (!data.imageRaw().empty() || !data.laserScanRaw().isEmpty()))
		{
			UERROR("Guess from motion is set but dt is invalid! Odometry is then computed without guess. (dt=%f previous transform=%s)", dt, velocityGuess_.prettyPrint().c_str());
		}
		// 如果启用了卡尔曼滤波，但时间间隔无效，输出错误信息
		else if(_filteringStrategy==1)
		{
			UERROR("Kalman filtering is enabled but dt is invalid! Odometry is then computed without Kalman filtering. (dt=%f previous transform=%s)", dt, velocityGuess_.prettyPrint().c_str());
		}
		dt=0;  // 设置时间间隔为0，并清除以前的速度数据
		previousVelocities_.clear();
		velocityGuess_.setNull();
	}
	// 检查速度猜测值velocityGuess_是否非空（非零变换）
	if(!velocityGuess_.isNull())
	{
		// 如果启用了从运动中进行猜测（guessFromMotion_为true）
		if(guessFromMotion_)
		{
			// 使用滤波策略1（卡尔曼滤波）
			if(_filteringStrategy == 1)
			{
				// use Kalman predict transform
				float vx,vy,vz, vroll,vpitch,vyaw;
				predictKalmanFilter(dt, &vx,&vy,&vz,&vroll,&vpitch,&vyaw);
				guess = Transform(vx*dt, vy*dt, vz*dt, vroll*dt, vpitch*dt, vyaw*dt);
			}
			else // 不使用卡尔曼滤波时，直接从velocityGuess_获取速度
			{
				float vx,vy,vz, vroll,vpitch,vyaw;
				// 从当前速度猜测值获取线速度和欧拉角速度
				velocityGuess_.getTranslationAndEulerAngles(vx,vy,vz, vroll,vpitch,vyaw);
				guess = Transform(vx*dt, vy*dt, vz*dt, vroll*dt, vpitch*dt, vyaw*dt); // 计算位移和旋转变化量，构造变换矩阵
			}
		}
		else if(_filteringStrategy == 1)
		{
			predictKalmanFilter(dt);
		}
	}

	Transform imuCurrentTransform;  // 声明当前IMU变换变量
	// 检查是否提供了初始猜测变换
	if(!guessIn.isNull())
	{
		guess = guessIn;
	}
	// 如果没有初始猜测但有IMU数据可用
	else if(!imus_.empty())
	{
		// replace orientation guess with IMU (if available)
		imuCurrentTransform = Transform::getTransform(imus_, data.stamp()); // 尝试从IMU数据中获取当前时间戳对应的变换
		// 如果成功获取到当前和上一时刻的IMU变换
		if(!imuCurrentTransform.isNull() && !imuLastTransform_.isNull())
		{
			// 计算两帧IMU之间的相对旋转变化
        	// 使用上一帧的逆变换乘以当前帧变换得到相对变换
			Transform orientation = imuLastTransform_.inverse() * imuCurrentTransform;

			// 构建新的猜测变换：
        	// 1. 使用IMU计算的旋转矩阵部分(3x3)
        	// 2. 保留原有猜测的平移部分(x,y,z)
			guess = Transform(
					orientation.r11(), orientation.r12(), orientation.r13(), guess.x(),
					orientation.r21(), orientation.r22(), orientation.r23(), guess.y(),
					orientation.r31(), orientation.r32(), orientation.r33(), guess.z());
			if(_force3DoF)
			{
				guess = guess.to3DoF(); //将6自由度变换转换为3自由度变换（2D平面运动）返回只包含x,y平移和yaw旋转的变换矩阵
			}
		}
		else if(!imuLastTransform_.isNull())
		{
			UWARN("Could not find imu transform at %f", data.stamp());
		}
	}

	UTimer time;

	// Deskewing lidar
	// 雷达
	if( _deskewing &&
		!data.laserScanRaw().empty() &&
		data.laserScanRaw().hasTime() &&
		dt > 0 &&
		!guess.isNull())
	{
		UDEBUG("Deskewing begin");
		// Recompute velocity
		float vx,vy,vz, vroll,vpitch,vyaw;
		guess.getTranslationAndEulerAngles(vx,vy,vz, vroll,vpitch,vyaw);

		// transform to velocity
		vx /= dt;
		vy /= dt;
		vz /= dt;
		vroll /= dt;
		vpitch /= dt;
		vyaw /= dt;

		if(!imus_.empty())
		{
			float scanTime =
				data.laserScanRaw().data().ptr<float>(0, data.laserScanRaw().size()-1)[data.laserScanRaw().getTimeOffset()] -
				data.laserScanRaw().data().ptr<float>(0, 0)[data.laserScanRaw().getTimeOffset()];

			// replace orientation velocity based on IMU (if available)
			Transform imuFirstScan = Transform::getTransform(imus_,
					data.stamp() +
					data.laserScanRaw().data().ptr<float>(0, 0)[data.laserScanRaw().getTimeOffset()]);
			Transform imuLastScan = Transform::getTransform(imus_,
					data.stamp() +
					data.laserScanRaw().data().ptr<float>(0, data.laserScanRaw().size()-1)[data.laserScanRaw().getTimeOffset()]);
			if(!imuFirstScan.isNull() && !imuLastScan.isNull())
			{
				Transform orientation = imuFirstScan.inverse() * imuLastScan;
				orientation.getEulerAngles(vroll, vpitch, vyaw);
				if(_force3DoF)
				{
					vroll=0;
					vpitch=0;
					vyaw /= scanTime;
				}
				else
				{
					vroll /= scanTime;
					vpitch /= scanTime;
					vyaw /= scanTime;
				}
			}
		}

		Transform velocity(vx,vy,vz,vroll,vpitch,vyaw);
		LaserScan scanDeskewed = util3d::deskew(data.laserScanRaw(), data.stamp(), velocity);
		if(!scanDeskewed.isEmpty())
		{
			data.setLaserScan(scanDeskewed);
		}
		info->timeDeskewing = time.ticks();
		UDEBUG("Deskewing end");
	}
	if(data.laserScanRaw().isOrganized())
	{
		// Laser scans should be dense passing this point
		data.setLaserScan(data.laserScanRaw().densify());
	}


	
	Transform t;
	// 检查是否需要进行图像降采样（满足以下两个条件）：
	// 1. 降采样系数_imageDecimation大于1
	// 2. 原始图像数据非空
	if(_imageDecimation > 1 && !data.imageRaw().empty())
	{
		// Decimation of images with calibrations
		// 创建数据副本用于降采样处理
		SensorData decimatedData = data;
		int decimationDepth = _imageDecimation; // 深度图降采样系数（默认与RGB相同）

		// 如果有有效的相机模型且图像尺寸已知
		if(	!data.cameraModels().empty() &&
			data.cameraModels()[0].imageHeight()>0 &&
			data.cameraModels()[0].imageWidth()>0)
		{
			// decimate from RGB image size 根据RGB图像尺寸计算目标尺寸
			int targetSize = data.cameraModels()[0].imageHeight() / _imageDecimation;

			// 如果目标尺寸大于等于原始深度图尺寸，则不降采样深度图
			if(targetSize >= data.depthRaw().rows)
			{
				decimationDepth = 1;
			}
			else
			{
				// 否则按比例计算深度图降采样系数
				decimationDepth = (int)ceil(float(data.depthRaw().rows) / float(targetSize));
			}
		}
		UDEBUG("decimation rgbOrLeft(rows=%d)=%d, depthOrRight(rows=%d)=%d", data.imageRaw().rows, _imageDecimation, data.depthOrRightRaw().rows, decimationDepth);

		// =============== 执行图像降采样 ===============
        // 对RGB/左目图像进行降采样
		cv::Mat rgbLeft = util2d::decimate(decimatedData.imageRaw(), _imageDecimation);
		// 对深度图/右目图像进行降采样
		cv::Mat depthRight = util2d::decimate(decimatedData.depthOrRightRaw(), decimationDepth);
		// =============== 相机模型缩放 ===============
    	// 处理单目相机模型
		std::vector<CameraModel> cameraModels = decimatedData.cameraModels();
		for(unsigned int i=0; i<cameraModels.size(); ++i)
		{
			// 按降采样比例缩放相机内参
			cameraModels[i] = cameraModels[i].scaled(1.0/double(_imageDecimation));
		}
		if(!cameraModels.empty())
		{
			// 更新降采样后的RGBD数据
			decimatedData.setRGBDImage(rgbLeft, depthRight, cameraModels);
		}
		else
		{
			std::vector<StereoCameraModel> stereoModels = decimatedData.stereoCameraModels();
			for(unsigned int i=0; i<stereoModels.size(); ++i)
			{
				stereoModels[i].scale(1.0/double(_imageDecimation)); // 按降采样比例缩放双目相机内参
			}
			if(!stereoModels.empty())
			{
				decimatedData.setStereoImage(rgbLeft, depthRight, stereoModels); // 更新降采样后的双目数据
			}
		}


		// compute transform  使用降采样后的数据计算位姿变换
		t = this->computeTransform(decimatedData, guess, info);

		// transform back the keypoints in the original image 将特征点坐标映射回原始图像尺寸
		std::vector<cv::KeyPoint> kpts = decimatedData.keypoints();
		double log2value = log(double(_imageDecimation))/log(2.0);
		for(unsigned int i=0; i<kpts.size(); ++i)
		{
			kpts[i].pt.x *= _imageDecimation; // 还原x坐标
			kpts[i].pt.y *= _imageDecimation; // 还原y坐标
			kpts[i].size *= _imageDecimation;  // 还原特征点尺寸
			kpts[i].octave += log2value; // 更新金字塔层级
		}
		}
		// 更新原始数据中的特征信息
		data.setFeatures(kpts, decimatedData.keypoints3D(), decimatedData.descriptors());
		data.setLaserScan(decimatedData.laserScanRaw());

		if(info)
		{
			// 确保匹配点数量一致
			UASSERT(info->newCorners.size() == info->refCorners.size() || info->refCorners.empty());
			for(unsigned int i=0; i<info->newCorners.size(); ++i) // 还原匹配点坐标
			{
				info->newCorners[i].x *= _imageDecimation;
				info->newCorners[i].y *= _imageDecimation;
				if(!info->refCorners.empty())
				{
					info->refCorners[i].x *= _imageDecimation;
					info->refCorners[i].y *= _imageDecimation;
				}
			}
			// 还原视觉词典特征点坐标
			for(std::multimap<int, cv::KeyPoint>::iterator iter=info->words.begin(); iter!=info->words.end(); ++iter)
			{
				iter->second.pt.x *= _imageDecimation;
				iter->second.pt.y *= _imageDecimation;
				iter->second.size *= _imageDecimation;
				iter->second.octave += log2value;
			}
		}
	}
	// 处理包含有效传感器数据的情况：
	// 1. 有原始图像数据 或
	// 2. 有激光雷达数据 或
	// 3. 支持异步IMU处理且IMU数据非空
	else if(!data.imageRaw().empty() || !data.laserScanRaw().isEmpty() || (this->canProcessAsyncIMU() && !data.imu().empty()))
	{
		// 调用核心算法计算当前帧的位姿变换 默认F2M
		// data: 包含传感器数据的结构体
		// guess: 初始位姿猜测
		// info: 存储计算过程和结果的调试信息
		t = this->computeTransform(data, guess, info);
	}

	// 特殊处理：仅有IMU数据而无视觉/激光数据时
	if(data.imageRaw().empty() && data.laserScanRaw().isEmpty() && !data.imu().empty())
	{
		return Transform(); // Return null on IMU-only updates  返回空变换（表示仅IMU更新不产生位姿变化）
	}

	// 如果启用了调试信息记录
	if(info)
	{
		info->timeEstimation = time.ticks(); // 记录位姿估计耗时（毫秒）
		info->lost = t.isNull(); // 标记是否跟踪丢失（变换矩阵是否为空）
		info->stamp = data.stamp(); // 记录当前帧时间戳
		info->interval = dt; // 记录与上一帧的时间间隔
		info->transform = t; // 存储计算得到的变换矩阵
		info->guess = guess; // 存储使用的初始猜测值
		if(_publishRAMUsage) // 如果启用了内存监控
		{
			info->memoryUsage = UProcessInfo::getMemoryUsage()/(1024*1024); // 记录当前内存使用量（转换为MB）
		}

		// 如果有真实轨迹数据（用于评估算法精度）
		if(!data.groundTruth().isNull())
		{
			// 计算相对于上一帧的真实位姿变化
			if(!previousGroundTruthPose_.isNull())
			{
				info->transformGroundTruth = previousGroundTruthPose_.inverse() * data.groundTruth();
			}
			previousGroundTruthPose_ = data.groundTruth();
		}
	}

	// 检查变换矩阵t是否有效（非空）
	if(!t.isNull())
	{
		// 重置失败计数器（当获得有效位姿时重置）
		_resetCurrentCount = _resetCountdown;

		// 分解变换矩阵为平移和欧拉角（6自由度参数）
		float vx,vy,vz, vroll,vpitch,vyaw;
		t.getTranslationAndEulerAngles(vx,vy,vz, vroll,vpitch,vyaw);

		// transform to velocity
		// 如果有时间间隔dt，将位移转换为速度（单位时间变化量）
		if(dt)
		{
			vx /= dt;
			vy /= dt;
			vz /= dt;
			vroll /= dt;
			vpitch /= dt;
			vyaw /= dt;
		}

		/* 运动模型处理部分 */
    	// 处理3DoF/非全向/粒子滤波/Kalman滤波等情况
		if(_force3DoF || !_holonomic || particleFilters_.size() || _filteringStrategy==1)
		{
			if(_filteringStrategy == 1)
			{
				if(velocityGuess_.isNull())
				{
					// reset Kalman
					if(dt)
					{
						initKalmanFilter(t, vx,vy,vz,vroll,vpitch,vyaw);
					}
					else
					{
						initKalmanFilter(t);
					}
				}
				else
				{
					// Kalman filtering
					updateKalmanFilter(vx,vy,vz,vroll,vpitch,vyaw);
				}
			}
			else
			{
				// 粒子滤波策略
				if(particleFilters_.size())
				{
					// Particle filtering
					UASSERT(particleFilters_.size()==6);
					// 初始化粒子滤波器
					if(velocityGuess_.isNull())
					{
						particleFilters_[0]->init(vx);
						particleFilters_[1]->init(vy);
						particleFilters_[2]->init(vz);
						particleFilters_[3]->init(vroll);
						particleFilters_[4]->init(vpitch);
						particleFilters_[5]->init(vyaw);
					}
					else
					{
						// 粒子滤波更新
						vx = particleFilters_[0]->filter(vx);
						vy = particleFilters_[1]->filter(vy);
						vyaw = particleFilters_[5]->filter(vyaw);

						// 非全向运动模型处理（弧线轨迹）
						if(!_holonomic)
						{
							// arc trajectory around ICR
							// 计算瞬时曲率中心(ICR)的弧线轨迹
							float tmpY = vyaw!=0.0f ? vx / tan((CV_PI-vyaw)/2.0f) : 0.0f;
							// 处理y方向速度约束
							if(fabs(tmpY) < fabs(vy) || (tmpY<=0 && vy >=0) || (tmpY>=0 && vy<=0))
							{
								vy = tmpY;
							}
							else
							{
								vyaw = (atan(vx/vy)*2.0f-CV_PI)*-1;
							}
						}

						if(!_force3DoF) // 如果不强制3DoF，更新其他自由度
						{
							vz = particleFilters_[2]->filter(vz);
							vroll = particleFilters_[3]->filter(vroll);
							vpitch = particleFilters_[4]->filter(vpitch);
						}
					}

					if(info)
					{
						info->timeParticleFiltering = time.ticks();
					}
				}
				// 纯非全向运动模型
				else if(!_holonomic)
				{
					// arc trajectory around ICR
					vy = vyaw!=0.0f ? vx / tan((CV_PI-vyaw)/2.0f) : 0.0f;
				}

				if(_force3DoF)
				{
					vz = 0.0f;
					vroll = 0.0f;
					vpitch = 0.0f;
				}
			}

			if(dt)
			{
				// 根据dt将速度转换回位移量
				t = Transform(vx*dt, vy*dt, vz*dt, vroll*dt, vpitch*dt, vyaw*dt);
			}
			else
			{
				t = Transform(vx, vy, vz, vroll, vpitch, vyaw);
			}

			/* 数据记录和更新部分 */
    		// 更新滤波后的位姿
			if(info)
			{
				info->transformFiltered = t;
			}
		}
		// 时间戳检查
		if(data.stamp() == 0 && framesProcessed_ != 0)
		{
			UWARN("Null stamp detected");
		}

		previousStamp_ = data.stamp();

		// 速度估计处理
		if(dt)
		{
			if(dt >= (guessSmoothingDelay_/2.0) || particleFilters_.size() || _filteringStrategy==1)
			{
				// 直接使用当前速度估计
				velocityGuess_ = Transform(vx, vy, vz, vroll, vpitch, vyaw);
				previousVelocities_.clear();
			}
			else
			{
				// 平滑速度估计（滑动窗口平均）
				// smooth velocity estimation over the past X seconds
				std::vector<float> v(6);
				v[0] = vx;
				v[1] = vy;
				v[2] = vz;
				v[3] = vroll;
				v[4] = vpitch;
				v[5] = vyaw;
				previousVelocities_.push_back(std::make_pair(v, data.stamp()));
				// 移除超出时间窗口的旧数据
				while(previousVelocities_.size() > 1 && previousVelocities_.front().second < previousVelocities_.back().second-guessSmoothingDelay_)
				{
					previousVelocities_.pop_front();
				}
				velocityGuess_ = getMeanVelocity(previousVelocities_);
			}
		}
		else
		{
			previousVelocities_.clear();
			velocityGuess_.setNull();
		}

		// 更新里程计信息
		if(info)
		{
			distanceTravelled_ += t.getNorm(); // 累计行驶距离
			info->distanceTravelled = distanceTravelled_; 
			info->guessVelocity = velocityGuess_;
		}
		++framesProcessed_;

		imuLastTransform_ = imuCurrentTransform; // 更新IMU数据

		return _pose *= t; // update  更新全局位姿
	}
	else if(_resetCurrentCount > 0) // 位姿丢失处理
	{
		UWARN("Odometry lost! Odometry will be reset after next %d consecutive unsuccessful odometry updates...", _resetCurrentCount);
		// 位姿丢失计数
		--_resetCurrentCount;
		if(_resetCurrentCount == 0) // 达到重置阈值
		{
			UWARN("Odometry automatically reset to latest pose!");
			this->reset(_pose);
			_resetCurrentCount = _resetCountdown;
			if(info)
			{
				*info = OdometryInfo();
			}
			return this->computeTransform(data, Transform(), info);
		}
	}
	// 位姿无效时的清理
	previousVelocities_.clear();
	velocityGuess_.setNull();
	previousStamp_ = 0;

	return Transform();
}

void Odometry::initKalmanFilter(const Transform & initialPose, float vx, float vy, float vz, float vroll, float vpitch, float vyaw)
{
	UDEBUG("");
	// See OpenCV tutorial: http://docs.opencv.org/master/dc/d2c/tutorial_real_time_pose.html
	// See Kalman filter pose/orientation estimation theory: http://campar.in.tum.de/Chair/KalmanFilter

	// initialize the Kalman filter
	int nStates = 18;            // the number of states (x,y,z,x',y',z',x'',y'',z'',roll,pitch,yaw,roll',pitch',yaw',roll'',pitch'',yaw'')
	int nMeasurements = 6;       // the number of measured states (x',y',z',roll',pitch',yaw')
	if(_force3DoF)
	{
		nStates = 9;             // the number of states (x,y,x',y',x'',y'',yaw,yaw',yaw'')
		nMeasurements = 3;       // the number of measured states (x',y',yaw')
	}
	int nInputs = 0;             // the number of action control

	/* From viso2, measurement covariance
	 * static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
	{ { 0.1, 0, 0, 0, 0, 0,
	    0, 0.1, 0, 0, 0, 0,
	    0, 0, 0.1, 0, 0, 0,
	    0, 0, 0, 0.17, 0, 0,
	    0, 0, 0, 0, 0.17, 0,
	    0, 0, 0, 0, 0, 0.17 } };
	static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
	{ { 0.05, 0, 0, 0, 0, 0,
	    0, 0.05, 0, 0, 0, 0,
	    0, 0, 0.05, 0, 0, 0,
	    0, 0, 0, 0.09, 0, 0,
	    0, 0, 0, 0, 0.09, 0,
	    0, 0, 0, 0, 0, 0.09 } };
	 */


	kalmanFilter_.init(nStates, nMeasurements, nInputs);                 // init Kalman Filter
	cv::setIdentity(kalmanFilter_.processNoiseCov, cv::Scalar::all(_kalmanProcessNoise));  // set process noise
	cv::setIdentity(kalmanFilter_.measurementNoiseCov, cv::Scalar::all(_kalmanMeasurementNoise));   // set measurement noise
	cv::setIdentity(kalmanFilter_.errorCovPost, cv::Scalar::all(1));             // error covariance

	float x,y,z,roll,pitch,yaw;
	initialPose.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);

	if(_force3DoF)
	{
        /* MEASUREMENT MODEL (velocity) */
		//  [0 0 1 0 0 0 0 0 0]
		//  [0 0 0 1 0 0 0 0 0]
		//  [0 0 0 0 0 0 0 1 0]
		kalmanFilter_.measurementMatrix.at<float>(0,2) = 1;  // x'
		kalmanFilter_.measurementMatrix.at<float>(1,3) = 1;  // y'
		kalmanFilter_.measurementMatrix.at<float>(2,7) = 1; // yaw'

		kalmanFilter_.statePost.at<float>(0) = x;
		kalmanFilter_.statePost.at<float>(1) = y;
		kalmanFilter_.statePost.at<float>(6) = yaw;

		kalmanFilter_.statePost.at<float>(2) = vx;
		kalmanFilter_.statePost.at<float>(3) = vy;
		kalmanFilter_.statePost.at<float>(7) = vyaw;
	}
	else
	{
	    /* MEASUREMENT MODEL (velocity) */
		//  [0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
		//  [0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0]
		//  [0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0]
		//  [0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0]
		//  [0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0]
		//  [0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0]
		kalmanFilter_.measurementMatrix.at<float>(0,3) = 1;  // x'
		kalmanFilter_.measurementMatrix.at<float>(1,4) = 1;  // y'
		kalmanFilter_.measurementMatrix.at<float>(2,5) = 1;  // z'
		kalmanFilter_.measurementMatrix.at<float>(3,12) = 1; // roll'
		kalmanFilter_.measurementMatrix.at<float>(4,13) = 1; // pitch'
		kalmanFilter_.measurementMatrix.at<float>(5,14) = 1; // yaw'

		kalmanFilter_.statePost.at<float>(0) = x;
		kalmanFilter_.statePost.at<float>(1) = y;
		kalmanFilter_.statePost.at<float>(2) = z;
		kalmanFilter_.statePost.at<float>(9) = roll;
		kalmanFilter_.statePost.at<float>(10) = pitch;
		kalmanFilter_.statePost.at<float>(11) = yaw;

		kalmanFilter_.statePost.at<float>(3) = vx;
		kalmanFilter_.statePost.at<float>(4) = vy;
		kalmanFilter_.statePost.at<float>(5) = vz;
		kalmanFilter_.statePost.at<float>(12) = vroll;
		kalmanFilter_.statePost.at<float>(13) = vpitch;
		kalmanFilter_.statePost.at<float>(14) = vyaw;
	}
}

/**
 * 使用卡尔曼滤波预测运动状态
 * @param dt 时间间隔（单位：秒）
 * @param vx 输出x轴方向速度（可选）
 * @param vy 输出y轴方向速度（可选）
 * @param vz 输出z轴方向速度（可选）
 * @param vroll 输出绕x轴旋转速度（可选）
 * @param vpitch 输出绕y轴旋转速度（可选）
 * @param vyaw 输出绕z轴旋转速度（可选）
 */
void Odometry::predictKalmanFilter(float dt, float * vx, float * vy, float * vz, float * vroll, float * vpitch, float * vyaw)
{
	// Set transition matrix with current dt
	// 根据当前dt设置状态转移矩阵
	if(_force3DoF) // 如果是强制3自由度（2D平面运动）
	{
		/* 2D运动状态转移矩阵说明（9x9矩阵）：
         * 状态向量：[x, y, x', y', x'', y'', yaw, yaw', yaw'']
         * 其中：'表示一阶导数（速度），''表示二阶导数（加速度）
         *
         * 矩阵结构：
         * [1 0 dt  0 dt2    0   0    0     0]  // x位置 = x + x'*dt + 0.5*x''*dt2
         * [0 1  0 dt   0  dt2   0    0     0]  // y位置 = y + y'*dt + 0.5*y''*dt2
         * [0 0  1  0   dt   0   0    0     0]  // x速度 = x' + x''*dt
         * [0 0  0  1   0   dt   0    0     0]  // y速度 = y' + y''*dt
         * [0 0  0  0   1    0   0    0     0]  // x加速度保持不变
         * [0 0  0  0   0    1   0    0     0]  // y加速度保持不变
         * [0 0  0  0   0    0   1   dt   dt2]  // 偏航角 = yaw + yaw'*dt + 0.5*yaw''*dt2
         * [0 0  0  0   0    0   0    1    dt]  // 偏航速度 = yaw' + yaw''*dt
         * [0 0  0  0   0    0   0    0     1]  // 偏航加速度保持不变
         */

		// 设置位置相关转移系数
		kalmanFilter_.transitionMatrix.at<float>(0,2) = dt; // x = x + x'*dt
		kalmanFilter_.transitionMatrix.at<float>(1,3) = dt; // y = y + y'*dt
		kalmanFilter_.transitionMatrix.at<float>(2,4) = dt; // x' = x' + x''*dt
		kalmanFilter_.transitionMatrix.at<float>(3,5) = dt; // y' = y' + y''*dt
		kalmanFilter_.transitionMatrix.at<float>(0,4) = 0.5*pow(dt,2); // x = x + 0.5*x''*dt2
		kalmanFilter_.transitionMatrix.at<float>(1,5) = 0.5*pow(dt,2); // y = y + 0.5*y''*dt2
		// orientation设置姿态相关转移系数
		kalmanFilter_.transitionMatrix.at<float>(6,7) = dt; // yaw = yaw + yaw'*dt
		kalmanFilter_.transitionMatrix.at<float>(7,8) = dt; // yaw' = yaw' + yaw''*dt
		kalmanFilter_.transitionMatrix.at<float>(6,8) = 0.5*pow(dt,2); // yaw = yaw + 0.5*ya
	} 
	else // 6自由度（3D空间运动）

	/* 3D运动状态转移矩阵说明（18x18矩阵）：
         * 状态向量：[x,y,z, x',y',z', x'',y'',z'', roll,pitch,yaw, roll',pitch',yaw', roll'',pitch'',yaw'']
         *
         * 矩阵结构类似2D情况，但扩展到3D空间：
         * 前9行对应位置、速度、加速度（x,y,z三个维度）
         * 后9行对应姿态角、角速度、角加速度（roll,pitch,yaw三个维度）
         */

	{
		//  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0] x
		//  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0] y
		//  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0] z
		//  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0] x'
		//  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0] y'
		//  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0] z'
		//  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0] x''
		//  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0] y''
		//  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0] z''
		//  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
		//  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
		//  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
		// position
		kalmanFilter_.transitionMatrix.at<float>(0,3) = dt; // x = x + x'*dt
		kalmanFilter_.transitionMatrix.at<float>(1,4) = dt; // y = y + y'*dt
		kalmanFilter_.transitionMatrix.at<float>(2,5) = dt; // z = z + z'*dt
		kalmanFilter_.transitionMatrix.at<float>(3,6) = dt; // x' = x' + x''*dt
		kalmanFilter_.transitionMatrix.at<float>(4,7) = dt; // y' = y' + y''*dt
		kalmanFilter_.transitionMatrix.at<float>(5,8) = dt; // z' = z' + z''*dt
		kalmanFilter_.transitionMatrix.at<float>(0,6) = 0.5*pow(dt,2); // x = x + 0.5*x''*dt2
		kalmanFilter_.transitionMatrix.at<float>(1,7) = 0.5*pow(dt,2); // y = y + 0.5*y''*dt2
		kalmanFilter_.transitionMatrix.at<float>(2,8) = 0.5*pow(dt,2); // z = z + 0.5*z''*dt2
		// orientation
		kalmanFilter_.transitionMatrix.at<float>(9,12) = dt; // roll = roll + roll'*dt
		kalmanFilter_.transitionMatrix.at<float>(10,13) = dt; // pitch = pitch + pitch'*dt
		kalmanFilter_.transitionMatrix.at<float>(11,14) = dt; // yaw = yaw + yaw'*dt
		kalmanFilter_.transitionMatrix.at<float>(12,15) = dt; // roll' = roll' + roll''*dt
		kalmanFilter_.transitionMatrix.at<float>(13,16) = dt; // pitch' = pitch' + pitch''*dt
		kalmanFilter_.transitionMatrix.at<float>(14,17) = dt;  // yaw' = yaw' + yaw''*dt
		kalmanFilter_.transitionMatrix.at<float>(9,15) = 0.5*pow(dt,2); // roll = roll + 0.5*roll''*dt2
		kalmanFilter_.transitionMatrix.at<float>(10,16) = 0.5*pow(dt,2); // pitch = pitch + 0.5*pitch''*dt2
		kalmanFilter_.transitionMatrix.at<float>(11,17) = 0.5*pow(dt,2); // yaw = yaw + 0.5*yaw''*dt2
	}

	// First predict, to update the internal statePre variable
	// 执行预测步骤，更新内部状态(statePre)
	UDEBUG("Predict");
	const cv::Mat & prediction = kalmanFilter_.predict();

	// 从预测结果中提取需要的速度值（如果输出参数不为空）
	if(vx)
		*vx = prediction.at<float>(3);                      // x'
	if(vy)
		*vy = prediction.at<float>(4);                      // y'
	if(vz)
		*vz = _force3DoF?0.0f:prediction.at<float>(5);      // z'
	if(vroll)
		*vroll = _force3DoF?0.0f:prediction.at<float>(12);  // roll'
	if(vpitch)
		*vpitch = _force3DoF?0.0f:prediction.at<float>(13); // pitch'
	if(vyaw)
		*vyaw = prediction.at<float>(_force3DoF?7:14);      // yaw'
}

void Odometry::updateKalmanFilter(float & vx, float & vy, float & vz, float & vroll, float & vpitch, float & vyaw)
{
	// Set measurement to predict
	cv::Mat measurements;
	if(!_force3DoF)
	{
		measurements = cv::Mat(6,1,CV_32FC1);
		measurements.at<float>(0) = vx;     // x'
		measurements.at<float>(1) = vy;     // y'
		measurements.at<float>(2) = vz;     // z'
		measurements.at<float>(3) = vroll;  // roll'
		measurements.at<float>(4) = vpitch; // pitch'
		measurements.at<float>(5) = vyaw;   // yaw'
	}
	else
	{
		measurements = cv::Mat(3,1,CV_32FC1);
		measurements.at<float>(0) = vx;     // x'
		measurements.at<float>(1) = vy;     // y'
		measurements.at<float>(2) = vyaw;   // yaw',
	}

	// The "correct" phase that is going to use the predicted value and our measurement
	UDEBUG("Correct");
	const cv::Mat & estimated = kalmanFilter_.correct(measurements);


	vx = estimated.at<float>(3);                      // x'
	vy = estimated.at<float>(4);                      // y'
	vz = _force3DoF?0.0f:estimated.at<float>(5);      // z'
	vroll = _force3DoF?0.0f:estimated.at<float>(12);  // roll'
	vpitch = _force3DoF?0.0f:estimated.at<float>(13); // pitch'
	vyaw = estimated.at<float>(_force3DoF?7:14);      // yaw'
}

} /* namespace rtabmap */
