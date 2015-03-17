/*
 * test_data_octomap.cpp
 *
 *  Created on: 23 Aug 2012
 *      Author: andreas
 */

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <octomap/octomap.h>
#include <boost/algorithm/string/replace.hpp>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

using std::string;
using boost::algorithm::replace_first;

string meshlabscript_filename;
string map_path;
string model_path;
double target_resolution = 0.05;
bool filter_z = true;

double map_resolution = 0.05;

namespace fs = boost::filesystem;

void supersample(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, boost::shared_ptr<octomap::OcTree> ocTree, const octomap::OcTree::iterator& it)
{
    double half_resolution = map_resolution / 2.0;
    double quarter_resolution = half_resolution / 2.0;
    for (int x = 0; x < 2; x++)
    {
        for (int y = 0; y < 2; y++)
        {
            for (int z = 0; z < 2; z++)
            {
                pclCloud->push_back(pcl::PointXYZ());
                pcl::PointXYZ& point = pclCloud->back();
                point.x = it.getX() - quarter_resolution + x * half_resolution;
                point.y = it.getY() - quarter_resolution + y * half_resolution;
                point.z = it.getZ() - quarter_resolution + z * half_resolution;
            }
        }
    }
}

bool isNodeOccupied(const boost::shared_ptr<octomap::OcTree> ocTree, octomap::OcTreeKey key, unsigned int depth)
{
    const octomap::OcTreeNode* node = ocTree->search(key, depth);
    if (node != NULL)
    {
        return ocTree->isNodeOccupied(node);
    }
    return false;
}

void extract_surface(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, const boost::shared_ptr<octomap::OcTree> ocTree, const octomap::OcTree::iterator& it)
{
    unsigned int depth = it.getDepth();
    double cube_size = map_resolution / 2.0;
    double half_cube_size = cube_size / 2.0;
    // check faces neighbour
    octomap::point3d point = it.getCoordinate();
    point += octomap::point3d(map_resolution, 0, 0);
    octomap::OcTreeKey neighbour_key = ocTree->coordToKey(point, depth);
    bool inner_node = true;
    if (! isNodeOccupied(ocTree, neighbour_key, depth))
    {
        for (int y = 0; y < 2; y++)
        {
            for (int z = 0; z < 2; z++)
            {
                pclCloud->push_back(pcl::PointXYZ());
                pcl::PointXYZ& point = pclCloud->back();
                point.x = it.getX() - half_cube_size + cube_size;
                point.y = it.getY() - half_cube_size + y * cube_size;
                point.z = it.getZ() - half_cube_size + z * cube_size;
            }
        }
    }
    point = it.getCoordinate();
    point += octomap::point3d(-map_resolution, 0, 0);
    neighbour_key = ocTree->coordToKey(point, depth);
    if (! isNodeOccupied(ocTree, neighbour_key, depth))
    {
        for (int y = 0; y < 2; y++)
        {
            for (int z = 0; z < 2; z++)
            {
                pclCloud->push_back(pcl::PointXYZ());
                pcl::PointXYZ& point = pclCloud->back();
                point.x = it.getX() - half_cube_size;
                point.y = it.getY() - half_cube_size + y * cube_size;
                point.z = it.getZ() - half_cube_size + z * cube_size;
            }
        }
    }
    point = it.getCoordinate();
    point += octomap::point3d(0, map_resolution, 0);
    neighbour_key = ocTree->coordToKey(point, depth);
    if (! isNodeOccupied(ocTree, neighbour_key, depth))
    {
        string model_path;

        for (int x = 0; x < 2; x++)
        {
            for (int z = 0; z < 2; z++)
            {
                pclCloud->push_back(pcl::PointXYZ());
                pcl::PointXYZ& point = pclCloud->back();
                point.x = it.getX() - half_cube_size + x * cube_size;
                point.y = it.getY() - half_cube_size + cube_size;
                point.z = it.getZ() - half_cube_size + z * cube_size;
            }
        }
    }
    point = it.getCoordinate();
    point += octomap::point3d(0, -map_resolution, 0);
    neighbour_key = ocTree->coordToKey(point, depth);
    if (! isNodeOccupied(ocTree, neighbour_key, depth))
    {
        for (int x = 0; x < 2; x++)
        {
            for (int z = 0; z < 2; z++)
            {
                pclCloud->push_back(pcl::PointXYZ());
                pcl::PointXYZ& point = pclCloud->back();
                point.x = it.getX() - half_cube_size + x * cube_size;
                point.y = it.getY() - half_cube_size;
                point.z = it.getZ() - half_cube_size + z * cube_size;
            }
        }
    }
    point = it.getCoordinate();
    point += octomap::point3d(0, 0, map_resolution);
    neighbour_key = ocTree->coordToKey(point, depth);
    if (! isNodeOccupied(ocTree, neighbour_key, depth))
    {
        for (int x = 0; x < 2; x++)
        {
            for (int y = 0; y < 2; y++)
            {
                pclCloud->push_back(pcl::PointXYZ());
                pcl::PointXYZ& point = pclCloud->back();
                point.x = it.getX() - half_cube_size + x * cube_size;
                point.y = it.getY() - half_cube_size + y * cube_size;
                point.z = it.getZ() - half_cube_size + cube_size;
            }
        }
    }
    point = it.getCoordinate();
    point += octomap::point3d(0, 0, -map_resolution);
    neighbour_key = ocTree->coordToKey(point, depth);
    if (! isNodeOccupied(ocTree, neighbour_key, depth))
    {
        for (int x = 0; x < 2; x++)
        {
            for (int y = 0; y < 2; y++)
            {
                pclCloud->push_back(pcl::PointXYZ());
                pcl::PointXYZ& point = pclCloud->back();
                point.x = it.getX() - half_cube_size + x * cube_size;
                point.y = it.getY() - half_cube_size + y * cube_size;
                point.z = it.getZ() - half_cube_size;
            }
        }
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr octomap_to_pointcloud(boost::shared_ptr<octomap::OcTree> ocTree, double target_resolution = 0.1)
{
    ocTree->expand();
    map_resolution = ocTree->getResolution();
    int max_depth = 16;
    for (; max_depth > 1 && map_resolution < target_resolution; max_depth--)
    {
        map_resolution *= 2;
    }
    ROS_INFO_STREAM("resolution: map: "<<ocTree->getResolution()<<" target: "<<target_resolution<<" corrected: "<<map_resolution<<" max_depth: "<< max_depth);

    // init pointcloud:
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (octomap::OcTree::iterator it = ocTree->begin(max_depth); it != ocTree->end(); it++)
    {
        if (ocTree->isNodeOccupied(*it))
        {
            if (filter_z && (it.getZ() < 0.0 || it.getZ() > 2.0))
            {
                continue;
            }
//            supersample(pclCloud, ocTree, it);
            extract_surface(pclCloud, ocTree, it);
        }
    }
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (pclCloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter (*cloud_filtered);
    ROS_INFO("octomap converted to point cloud.");
    return cloud_filtered;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_sdf");
    ros::NodeHandle nh;

    // Initialize ROS
    ROS_ASSERT(argc == 2);
    fs::path package(argv[1]);
    ros::param::get("~script_filename", meshlabscript_filename);
    ros::param::get("~map_path", map_path);
    ros::param::get("~model_path", model_path);
    ros::param::get("~target_resolution", target_resolution);
    ros::param::get("~filter_z", filter_z);

    fs::path map_abs = package / fs::path(map_path);
    ROS_ASSERT_MSG(fs::exists(map_abs), ("octomap file not found: "+map_abs.string()).c_str());
    fs::path model_abs = package / fs::path(model_path);
    fs::create_directories(model_abs.parent_path());


    ROS_INFO("load_octomap");
    boost::shared_ptr<octomap::OcTree> ocTree(new octomap::OcTree(0.05));
    ocTree->readBinary(map_abs.string());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = octomap_to_pointcloud(ocTree, target_resolution);
    pcl::io::savePLYFile("/tmp/cloud.ply", *cloud, true);
    string meshlab_command = "meshlabserver -s [scrip_file] -i /tmp/cloud.ply -o [model_path]";
    replace_first(meshlab_command, "[scrip_file]", meshlabscript_filename);
    replace_first(meshlab_command, "[model_path]", model_abs.string());
    int value = system(meshlab_command.c_str());
    ROS_ASSERT(value == 0);
    ROS_INFO("octomap conversion finished.");

    return 0;
}

