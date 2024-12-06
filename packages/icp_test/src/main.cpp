#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pointmatcher/PointMatcher.h>
#include <cstring>
#include <rosbag2_cpp/writer.hpp>
#include <yaml-cpp/yaml.h>


using namespace std;
using namespace pcl;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

sensor_msgs::msg::PointCloud2 toPointCloud2(const DP& dataPoints, std::string frame_id) {
    sensor_msgs::msg::PointCloud2 result;

    result.header.frame_id = frame_id;
    result.height = 1; 
    result.width = dataPoints.features.count();
    if (result.width == 0) {
        std::cerr << "Нет точек для записи!" << std::endl;
    }

    result.fields.resize(3);
    result.fields[0].name = "x";
    result.fields[0].offset = 0;
    result.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[0].count = 1;

    result.fields[1].name = "y";
    result.fields[1].offset = 4;
    result.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[1].count = 1;

    result.fields[2].name = "z";
    result.fields[2].offset = 8;
    result.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    result.fields[2].count = 1;

    result.point_step = 12;
    result.row_step = result.point_step * result.width;  
    result.is_dense = true; 
    result.data.resize(result.row_step * result.height);
    std::cout << "dataPoints.features.rows(): " << dataPoints.features.rows() << '\n';
    std::cout << "dataPoints.features.cols(): " << dataPoints.features.cols() << '\n';
    for (size_t i = 0; i <dataPoints.features.cols(); ++i) {
       
        float* point_data = reinterpret_cast<float*>(result.data.data() + i * result.point_step);
        point_data[0] = dataPoints.features(0, i);
        point_data[1] = dataPoints.features(1, i);
        point_data[2] = dataPoints.features(2, i);

    }

    return result;
}

DP pcdToPointMatcherData(const std::string& filePath) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    if (io::loadPCDFile<PointXYZ>(filePath, *cloud) == -1) {
        cerr << "Не удалось загрузить PCD файл." << endl;
        exit(EXIT_FAILURE);
    }
    DP dataPoints;
    std::cout << "Размер облака: " << cloud->points.size() << '\n';
    dataPoints.features = PM::Matrix(4, cloud->points.size());

    for (size_t i = 0; i < cloud->points.size(); ++i) {
  		
        dataPoints.features(0, i) = cloud->points[i].x;
        dataPoints.features(1, i) = cloud->points[i].y;
        dataPoints.features(2, i) = cloud->points[i].z;
        dataPoints.features(3, i) = 1;
    
    }
    return dataPoints;
}

void writeToMCAP(
    const std::string& mcap_path, 
    const DP& dataPoints, 
    const std::string& cloud_topic_name) {
    const rclcpp::Time time;
    rosbag2_cpp::Writer writer; 
    writer.open(mcap_path); 
    auto cloud_msg = toPointCloud2(dataPoints, "test");
    writer.write(cloud_msg, cloud_topic_name, time);
    std::cout << "Запись завершена" << std::endl;
}


void writecloudsToMCAP(
    const std::string& mcap_path, 
    const std::vector<DP>& dataPointsList, 
    const std::vector<std::string>& cloud_topic_names,
    std::vector<std::string> colors) {
    
    if (dataPointsList.size() != cloud_topic_names.size()) {
        std::cerr << "Ошибка: количество облаков и названий топиков не совпадает." << std::endl;
        return;
    }

    const rclcpp::Time time;
    rosbag2_cpp::Writer writer; 
    writer.open(mcap_path); 

    for (size_t i = 0; i < dataPointsList.size(); ++i) {
        auto cloud_msg = toPointCloud2(dataPointsList[i], "test");
        writer.write(cloud_msg, cloud_topic_names[i], time);
        std::cout << "Запись облака " << i + 1 << " в топик '" << cloud_topic_names[i] << "' завершена." << std::endl;
    }
}

void getResidualError(const DP &dpref, const DP &dppointcloud_out, PM::ICP &icp) {
    icp.matcher->init(dpref);
    PM::Matches matches = icp.matcher->findClosests(dppointcloud_out);
    PM::OutlierWeights outlierWeights = icp.outlierFilters.compute(dppointcloud_out, dpref, matches);
    float error = icp.errorMinimizer->getResidualError(dppointcloud_out, dpref, outlierWeights, matches);
    std::cout << "Final residual error: " << error << std::endl;
}


int main() {
    std::string filePath1 = "/truck/packages/icp_test/files/cloud_1.pcd"; 
    DP cloud_1 = pcdToPointMatcherData(filePath1);
    std::string filePath2 = "/truck/packages/icp_test/files/cloud_2.pcd"; 
    DP cloud_2 = pcdToPointMatcherData(filePath2);
	PM::ICP icp;
    // std::ifstream icp_config_stream("/truck/packages/icp_test/conf/icp_config.yaml");
    std::ifstream icp_config_stream("/truck/packages/icp_test/conf/icp_config_truck.yaml");

    icp.loadFromYaml(icp_config_stream);
    // icp.setDefault();

    std::cout << "cloud_1 rows: " << cloud_1.features.rows() 
    << " cloud_1 cols: " << cloud_1.features.cols()
    << " cloud_1 nb: " << cloud_1.getNbPoints() << '\n';
    std::cout << "cloud_2 rows: " << cloud_2.features.rows()
     << " cloud_2 cols: " << cloud_2.features.cols()
     << " cloud_2 nb: " << cloud_2.getNbPoints() << '\n';


	PM::TransformationParameters T = icp(cloud_1, cloud_2);

	DP cloud_3(cloud_1);
	icp.transformations.apply(cloud_3, T);

    std::vector<float> v(6);
    v[0] = T(0, 3); 
    v[1] = T(1, 3); 
    v[2] = T(2, 3);

    // Вычисление углов Эйлера (roll, pitch, yaw) из матрицы T
    float sy = sqrt(T(0, 0) * T(0, 0) + T(1, 0) * T(1, 0)); 
    bool singular = sy < 1e-6; // Проверка на сингулярность

    if (!singular) {
        v[3] = atan2(T(2, 1), T(2, 2)); // roll
        v[4] = atan2(-T(2, 0), sy);      // pitch
        v[5] = atan2(T(1, 0), T(0, 0));  // yaw
    } else {
        v[3] = atan2(-T(1, 2), T(1, 1)); // roll
        v[4] = atan2(-T(2, 0), sy);      // pitch
        v[5] = 0;                         // yaw
    }
    std::cout << "v: ";
    for (float value : v) {
        std::cout << value << '\t';
    }

    std::cout << "\n\n";
    icp.referenceDataPointsFilters.apply(cloud_2);
    getResidualError(cloud_2, cloud_3, icp);
    std::cout << "\n\n";

    std::vector<DP> clouds = {cloud_1, cloud_2, cloud_3};
    std::vector<std::string> topics = {"cloud_1", "cloud_2", "cloud_3"};
    std::vector<std::string> colors = {"red", "green", "blue"};
    writecloudsToMCAP("/truck/packages/icp_test/results/truck_conf.mcap", clouds, topics, colors);
    return 0;
}
