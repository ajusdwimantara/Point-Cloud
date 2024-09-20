#include "point_cloud/unit_test.h"

void testRotateYAxiz() {
    PCLManager* pcl_manager = new PCLManager();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr expected_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // add points to the input cloud
    input_cloud->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));
    input_cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    input_cloud->push_back(pcl::PointXYZ(0.0, 0.0, 1.0));

    // expected points after rotation by 90 degrees around the Y axis
    expected_cloud->push_back(pcl::PointXYZ(0.0, 0.0, -1.0));
    expected_cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    expected_cloud->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud = pcl_manager->rotateYAxiz(90.0, input_cloud);

    assert(rotated_cloud->size() == expected_cloud->size());

    for (size_t i = 0; i < rotated_cloud->size(); ++i) {
        assert(rotated_cloud->points[i].x == expected_cloud->points[i].x);
        assert(rotated_cloud->points[i].y == expected_cloud->points[i].y);
        assert(rotated_cloud->points[i].z == expected_cloud->points[i].z);
    }

    std::cout << "Test rotateYAxiz passed!" << std::endl;
    delete pcl_manager;
}

void testRotateYAxizV2() {
    PCLManager* pcl_manager = new PCLManager();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr expected_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // add points to the input cloud
    input_cloud->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));
    input_cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    input_cloud->push_back(pcl::PointXYZ(0.0, 0.0, 1.0));

    // expected points after rotation by 90 degrees around the Y axis
    expected_cloud->push_back(pcl::PointXYZ(0.0, 0.0, -1.0));
    expected_cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    expected_cloud->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud = pcl_manager->rotateYAxizV2(90.0, input_cloud);

    assert(rotated_cloud->size() == expected_cloud->size());

    for (size_t i = 0; i < rotated_cloud->size(); ++i) {
        assert(rotated_cloud->points[i].x == expected_cloud->points[i].x);
        assert(rotated_cloud->points[i].y == expected_cloud->points[i].y);
        assert(rotated_cloud->points[i].z == expected_cloud->points[i].z);
    }

    std::cout << "Test rotateYAxizV2 passed!" << std::endl;
    delete pcl_manager;
}

void testSaveRotatedPCD(std::string filepath) {
    PCLManager* pcl_manager = new PCLManager();

    // mock request and response
    point_cloud::SavePCD::Request req;
    point_cloud::SavePCD::Response res;

    // set the request filename
    req.filename = filepath + "test_output.pcd";

    // create a simple rotated point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    rotated_cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    pcl_manager->setRotatedPCL(rotated_cloud);

    // call the function
    bool result = pcl_manager->saveRotatedPCD(req, res);

    // check the results
    assert(result == true);
    assert(res.success == true);
    assert(res.message == "PCD file saved successfully.");

    std::cout << "Test saveRotatedPCD passed!" << std::endl;
    delete pcl_manager;
}

void testInsertFile(std::string filepath) {
    PCLManager* pcl_manager = new PCLManager();

    std::string test_file = "test_file.pcd";
    int test_scale = 3;

    pcl_manager->insertFile(test_file, filepath, test_scale);

    assert(pcl_manager->getFilename() == test_file);
    assert(pcl_manager->getScale() == test_scale);

    std::cout << "Test insertFile passed!" << std::endl;
    delete pcl_manager;
}

void testDeg2Rad() {
    PCLManager* pcl_manager = new PCLManager();

    float degree = 180.0;
    float expected_radian = M_PI;

    float radian = pcl_manager->deg2Rad(degree);

    assert(radian == expected_radian);

    std::cout << "Test deg2Rad passed!" << std::endl;
    delete pcl_manager;
}