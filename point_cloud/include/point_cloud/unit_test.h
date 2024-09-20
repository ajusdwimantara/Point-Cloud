#ifndef UNIT_TEST_H_
#define UNIT_TEST_H_

#include <cassert>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "point_cloud/pcl_manager.h"
#include "point_cloud/SavePCD.h"

void testRotateYAxiz();

void testRotateYAxizV2();

void testSaveRotatedPCD(std::string filepath);

void testInsertFile(std::string filepath);

void testDeg2Rad();

#endif // UNIT_TEST_H_