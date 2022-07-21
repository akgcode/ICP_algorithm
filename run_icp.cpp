
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/time.h> //TicToc
 
int main (int argc, char** argv){
   //input ply files
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::io::loadPLYFile("../BALL BAR/4.ply", *cloud1);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::io::loadPLYFile("../BALL BAR/1.ply", *cloud2);
   pcl::console::TicToc time;
   pcl::console::TicToc timeF;
   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
   //Create the filtering object for DOWNSAMPLING
   pcl::console::TicToc timeDS;
   timeDS.tic();
   pcl::VoxelGrid<pcl::PointXYZ> down;
   down.setInputCloud (cloud1);
   down.setLeafSize (0.5f, 0.5f, 0.5f);
   down.filter (*cloud1);
   down.setInputCloud (cloud2);
   down.setLeafSize (0.5f, 0.5f, 0.5f);
   down.filter (*cloud2);
   std::cout <<"Downsampling took time(in ms) = " <<timeDS.toc() <<std::endl;
   // for BALL BAR1-4
   Eigen::Matrix4d transformation_matrix = (Eigen::Matrix4d() << 0.995802,-0.0913516,-0.0057416,60.3384,0.0914189,0.99573,0.0128293,-6.61557,0.00454511,-0.0133004,0.999901,8.3568,0,0,0,1).finished();
   int iterations = 25;
   int count=0; //count no. of iterations
 
   icp.setInputSource(cloud1);
   icp.setInputTarget(cloud2);
   std::cout<< icp.hasConverged() <<std::endl;
   pcl::PointCloud<pcl::PointXYZ> Final;
   icp.setMaximumIterations(1);
  
   timeF.tic();
   while(count<iterations){
       //1 iteration for ICP
       time.tic ();
       icp.align(Final,transformation_matrix.cast<float>());
       std::cout << "Applied iteration " << count <<  " in " << time.toc () << " ms "<< std::endl;
       if(icp.hasConverged()){
           //the mean squared distance from each point in source to its closest point in target
           std::cout << "Has converged with Fitness score: " << icp.getFitnessScore() << std::endl;
           transformation_matrix = icp.getFinalTransformation ().cast<double>();
           count++;
       }
       else{
           PCL_ERROR ("\nICP has not converged.\n");
           return (-1);
       }
   }
   std::cout << "Final Matrix= \n"<< icp.getFinalTransformation() <<std::endl;
   std:: cout << "total time(ms)= " <<timeF.toc()<< std::endl;
   return (0);
}
