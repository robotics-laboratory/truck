#pragma once

#include "icp_odometry/common.h"


namespace truck::icp_odometry {
	enum RobustFct {
		Cauchy,
		Welsch,
		SwitchableConstaint,
		GemanMcClure,
		Tukey,
		Huber,
		L1,
		Student,
	};

	extern std::map <RobustFct, std::string> RobustFctMap;

	enum CustomOutlierFilter {
		TrimmedDistOutlierFilter,
		VarTrimmedDistOutlierFilter,
		MedianDistOutlierFilter,
		SurfaceNormalOutlierFilter,
	};

	extern std::map <CustomOutlierFilter, std::string> CustomOutlierFilterMap;

	enum DistType {
		Point2Point,
		Point2Plane,
	};

	Matcher::ICP getDefaultICP(const DistType);

	Matcher::ICP getICPWithRobustOutlierFilter(const DistType, const RobustFct, const float = 1.0);

	Matcher::ICP getICPWithCustomOutlierFilter(const DistType, const CustomOutlierFilter);

	std::vector <TransformationParameters> mapToTransformations(Matcher::ICP, const std::vector <DataPoints> &);
}