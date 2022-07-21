#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
 
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
bool next_iteration = false;
 
void print4x4Matrix (const Eigen::Matrix4d & matrix){
 for(int i=0;i<4;i++){
   for(int j=0;j<4;j++){
     std::cout << matrix(i,j) << "\t";
   }
   std::cout << std::endl;
 }
}
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing){
 if (event.getKeySym () == "space" && event.keyDown ())
   next_iteration = true;
}
 
int main (int argc,char* argv[]){
 
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::io::loadPLYFile("../data/2.ply", *cloud1);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::io::loadPLYFile("../data/1.ply", *cloud2);
 PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
 
 pcl::console::TicToc time;
 time.tic ();
 // Coarse registration matrix for BALL BAR1-4
 Eigen::Matrix4d transformation_matrix = (Eigen::Matrix4d() << 0.995802,-0.0913516,-0.0057416,60.3384,0.0914189,0.99573,0.0128293,-6.61557,0.00454511,-0.0133004,0.999901,8.3568,0,0,0,1).finished();
 
 //Display in terminal the transformation matrix
 std::cout << "Applying this rigid transformation to: cloud1 -> cloud_icp" << std::endl;
 print4x4Matrix (transformation_matrix);
 
 // Executing the transformation
 pcl::transformPointCloud (*cloud1, *cloud_icp, transformation_matrix);
 *cloud2 = *cloud_icp;  // We backup cloud_icp into cloud2 for later use
 
 // The Iterative Closest Point algorithm
 time.tic ();
 int iterations =1;
 pcl::IterativeClosestPoint<PointT, PointT> icp;
 icp.setMaximumIterations (iterations);
 icp.setInputSource (cloud_icp);
 icp.setInputTarget (cloud1);
 icp.align (*cloud_icp);
 icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
 std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
 
 if (icp.hasConverged ()){
   std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
   std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud1" << std::endl;
   transformation_matrix = icp.getFinalTransformation ().cast<double>();
   print4x4Matrix (transformation_matrix);
 }
 else{
   PCL_ERROR ("\nICP has not converged.\n");
   return (-1);
 }
 // Visualization
 pcl::visualization::PCLVisualizer viewer ("ICP demo");
 // Create two vertically separated viewports
 int v1 (0);
 int v2 (1);
 viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
 viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
 
 // The color we will be using
 float bckgr_gray_level = 0.0;  // Black
 float txt_gray_lvl = 1.0 - bckgr_gray_level;
 
 // Original point cloud is white
 pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud1, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                            (int) 255 * txt_gray_lvl);
 viewer.addPointCloud (cloud1, cloud_in_color_h, "cloud_in_v1", v1);
 viewer.addPointCloud (cloud1, cloud_in_color_h, "cloud_in_v2", v2);
 
 // Transformed point cloud is green
 pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud2, 20, 180, 20);
 viewer.addPointCloud (cloud2, cloud_tr_color_h, "cloud_tr_v1", v1);
 
 // ICP aligned point cloud is red
 pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
 viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);
 
 // Adding text descriptions in each viewport
 viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
 viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
 
 std::stringstream ss;
 ss << iterations;
 std::string iterations_cnt = "ICP iterations = " + ss.str ();
 viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
 
 // Set background color
 viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
 viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
 
 // Set camera position and orientation
 viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
 viewer.setSize (1280, 1024);  // Visualiser window size
 
 // Register keyboard callback :
 viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
 
 // Display the visualiser
 while (!viewer.wasStopped ()){
   viewer.spinOnce ();
   // The user pressed "space" :
   if (next_iteration){
     time.tic ();
     icp.align (*cloud_icp);
     std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;
 
     if (icp.hasConverged ()){
       printf ("\033[11A");  // Go up 11 lines in terminal output.
       printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
       std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud1 \n Trasnformation Matrix is = \n" << std::endl;
       transformation_matrix *= icp.getFinalTransformation ().cast<double>();//dubugged the printing error
 
       ss.str ("");
       ss << iterations;
       std::string iterations_cnt = "ICP iterations = " + ss.str ();
       viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
       viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
       std::cout <<"\n \n \n \n \n "<<std::endl;
       print4x4Matrix ( transformation_matrix);  // Print the transformation between original pose and current pose
     }
     else{
       PCL_ERROR ("\nICP has not converged.\n");
       return (-1);
     }
   }
   next_iteration = false;
 }
 return (0);
}