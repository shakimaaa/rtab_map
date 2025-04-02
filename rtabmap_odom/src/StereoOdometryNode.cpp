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

#include "rtabmap_odom/stereo_odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#ifdef RTABMAP_PYTHON
#include <rtabmap/core/PythonInterface.h>
#endif

int main(int argc, char **argv)
{
	ULogger::setType(ULogger::kTypeConsole); // 设置输出为控制台类型
	ULogger::setLevel(ULogger::kWarning); // 日志级别为警告

	// process "--params" argument
	// 处理 参数
	std::vector<std::string> arguments; // 创建一个字符串向量来储存命令行参数
	for(int i=1;i<argc;++i)
	{
		if(strcmp(argv[i], "--params") == 0)
		{
			rtabmap::ParametersMap parametersOdom = rtabmap::Parameters::getDefaultOdometryParameters(true); // 获取默认里程计参数
			for(rtabmap::ParametersMap::iterator iter=parametersOdom.begin(); iter!=parametersOdom.end(); ++iter)
			{
				std::string str = "Param: " + iter->first + " = \"" + iter->second + "\""; // 构建参数输出字符串
				std::cout << 
						str <<
						std::setw(60 - str.size()) <<
						" [" <<
						rtabmap::Parameters::getDescription(iter->first).c_str() <<  // 获取附加参数的描述
						"]" <<
						std::endl;
			}
			UWARN("Node will now exit after showing default odometry parameters because "
					 "argument \"--params\" is detected!");
			exit(0);
		}
		else if(strcmp(argv[i], "--udebug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
		}
		else if(strcmp(argv[i], "--uinfo") == 0)
		{
			ULogger::setLevel(ULogger::kInfo);
		}
		arguments.push_back(argv[i]); // 将命令行参数添加到aruements中

	}

#ifdef RTABMAP_PYTHON
	rtabmap::PythonInterface pythonInterface;
#endif
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	options.arguments(arguments);
	auto node = std::make_shared<rtabmap_odom::StereoOdometry>(options);
	rclcpp::executors::MultiThreadedExecutor executor; // 多线程执行器
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
