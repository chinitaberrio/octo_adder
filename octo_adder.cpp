#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/simple_filter.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf/tf.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <custom_point_types/point_xyzlu.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/ClassificationOcTree.h>

#include <thread>
#include <functional>

sensor_msgs::PointCloud2 octo_pc;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "octo_adder");
  ros::NodeHandle nh;

  //Parsing arguments
  std::string bagname;
  for (int i = 1; i < argc; i++) {
    if (i + 1 != argc){
        if (std::strcmp(argv[1], "-b") == 0)           //bag_name
           bagname = argv[i + 1];
     }
  }

  //Open bags
  std::string bagname_out = bagname;
  bagname_out.insert(bagname_out.length()-4,"_octomap");
  std::cout << "bag directory " << bagname <<"\n";
  std::cout << "out_bag directory " << bagname_out <<"\n";
  rosbag::Bag bag,bag_out;
  bag.open(bagname, rosbag::bagmode::Read);
  bag_out.open(bagname_out, rosbag::bagmode::Write);

  // Topics' name
  std::string velodyne_p =  "/velodyne/front/label_uncertainties";
  std::string tf_msg     =  "/tf";
  std::string tf_msg_stat=  "/tf_static";

  // Topics to load
  std::vector<std::string> topics;
  topics.push_back(velodyne_p);
  topics.push_back(tf_msg_stat);
  topics.push_back(tf_msg);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // Publishers needed by the "Position filter"
  ros::Publisher Octomap_publisher;
  ros::Publisher Point_Cloud_publisher;

  ros::Publisher m_markerPub= nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 100);
  ros::Publisher m_pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 100);
  ros::Publisher o_pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("origin_point_cloud_centers", 100);
  tf2_ros::TransformBroadcaster tf_static_br;
  tf2_ros::TransformBroadcaster tf_br;


  // tf message to be included in the bag
  sensor_msgs::PointCloud2 Octomap_pc;

  //transformations
  geometry_msgs::TransformStamped base_vel, odom_base;
  Eigen::Matrix4d  Matrix_bl_vl, Matrix_od_bl, Matrix_od_vl, Matrix_vl_od ;

  //Octomap
  octomap::ClassificationOcTree* m_octree = new octomap::ClassificationOcTree(0.1); // resolution
  m_octree->setProbHit(0.9);
  m_octree->setProbMiss(0.2);
  m_octree->setClampingThresMin(0.12);
  m_octree->setClampingThresMax(0.97);
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;
  unsigned m_treeDepth = m_octree->getTreeDepth();
  unsigned m_maxTreeDepth = m_treeDepth;

  int number = 0;
  std::cout << "Starting to read the bag  \n";




  // Load all messages
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {

    // from tf_static extract only base_link to velodyne_link
   if (m.getTopic() == tf_msg_stat || ("/" + m.getTopic() == tf_msg_stat))
    {
      tf2_msgs::TFMessage::ConstPtr tf_info = m.instantiate<tf2_msgs::TFMessage>();
      if (tf_info != NULL)
      {
        for(int i=0; i < tf_info->transforms.size();i++)
        {
          if (tf_info->transforms[i].header.frame_id == "base_link"){
            if (tf_info->transforms[i].child_frame_id == "velodyne_front_link"){
                tf::Matrix3x3 Rotation_bl_vl;
                Rotation_bl_vl.setRotation(tf::Quaternion(tf_info->transforms[i].transform.rotation.x, tf_info->transforms[i].transform.rotation.y, tf_info->transforms[i].transform.rotation.z,tf_info->transforms[i].transform.rotation.w));
                tf::Vector3 traslation_bl_vl = tf::Vector3(tf_info->transforms[i].transform.translation.x,tf_info->transforms[i].transform.translation.y,tf_info->transforms[i].transform.translation.z);
                Matrix_bl_vl << Rotation_bl_vl.getRow(0)[0],Rotation_bl_vl.getRow(0)[1] ,Rotation_bl_vl.getRow(0)[2],traslation_bl_vl.getX(),
                                Rotation_bl_vl.getRow(1)[0],Rotation_bl_vl.getRow(1)[1] ,Rotation_bl_vl.getRow(1)[2],traslation_bl_vl.getY(),
                                Rotation_bl_vl.getRow(2)[0],Rotation_bl_vl.getRow(2)[1] ,Rotation_bl_vl.getRow(2)[2],traslation_bl_vl.getZ(),
                                0,0,0,1;
                std::cout << "I got the baselink to velodyne \n" ;
            }
          }
        }
        tf_static_br.sendTransform(tf_info->transforms);
        bag_out.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
      }
    }

   // from tf extract only odom to base_link
   if (m.getTopic() == tf_msg || ("/" + m.getTopic() == tf_msg))
    {
      tf2_msgs::TFMessage::ConstPtr tf_info = m.instantiate<tf2_msgs::TFMessage>();
      if (tf_info != NULL)
      {
        for(int i=0; i < tf_info->transforms.size();i++)
        {         
          if (tf_info->transforms[i].header.frame_id == "odom"){
            if (tf_info->transforms[i].child_frame_id == "base_link"){
                tf::Matrix3x3 Rotation_bl_vl;
                Rotation_bl_vl.setRotation(tf::Quaternion(tf_info->transforms[i].transform.rotation.x, tf_info->transforms[i].transform.rotation.y, tf_info->transforms[i].transform.rotation.z,tf_info->transforms[i].transform.rotation.w));
                tf::Vector3 traslation_bl_vl = tf::Vector3(tf_info->transforms[i].transform.translation.x,tf_info->transforms[i].transform.translation.y,tf_info->transforms[i].transform.translation.z);
                Matrix_od_bl << Rotation_bl_vl.getRow(0)[0],Rotation_bl_vl.getRow(0)[1] ,Rotation_bl_vl.getRow(0)[2],traslation_bl_vl.getX(),
                                Rotation_bl_vl.getRow(1)[0],Rotation_bl_vl.getRow(1)[1] ,Rotation_bl_vl.getRow(1)[2],traslation_bl_vl.getY(),
                                Rotation_bl_vl.getRow(2)[0],Rotation_bl_vl.getRow(2)[1] ,Rotation_bl_vl.getRow(2)[2],traslation_bl_vl.getZ(),
                                0,0,0,1;
                std::cout << "I got the odom to base_link \n" ;
            }
          }
        }
        tf_br.sendTransform(tf_info->transforms);
        bag_out.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
      }
    }

   // writing point cloud original topic
   if (m.getTopic() == velodyne_p || ("/" + m.getTopic() == velodyne_p )) // /velodyne/front/labelled
   {
     std::cout << " velodyne msg \n" ;
     sensor_msgs::PointCloud2::ConstPtr vel_p = m.instantiate<sensor_msgs::PointCloud2>();
     if (vel_p != NULL)
     {

       pcl::PointCloud<pcl::PointXYZLU> pc;
       pcl::PointCloud<pcl::PointXYZ>  origin;
       pcl::fromROSMsg(*vel_p, pc);
       Matrix_od_vl = Matrix_od_bl*Matrix_bl_vl;
       Matrix_vl_od = Matrix_od_vl.pow(-1);
       pcl::transformPointCloud(pc, pc, Matrix_od_vl);

       octomap::point3d sensorOrigin; // = pointTfToOctomap(sensorOriginTf); ojo aqui
       sensorOrigin.x()=Matrix_od_vl(0,3);
       sensorOrigin.y()=Matrix_od_vl(1,3);
       sensorOrigin.z()=Matrix_od_vl(2,3);

       // Origin
       pcl::PointXYZ origin_point;
       origin_point.x = Matrix_od_vl(0,3);
       origin_point.y = Matrix_od_vl(1,3);
       origin_point.z = Matrix_od_vl(2,3);
       origin.push_back(origin_point);

       sensor_msgs::PointCloud2 cloud2;
       pcl::toROSMsg (origin, cloud2);
       cloud2.header.frame_id = "odom";
       cloud2.header.stamp = vel_p->header.stamp;
       o_pointCloudPub.publish(cloud2);

       octomap::KeySet free_cells, occupied_cells;

       for (pcl::PointCloud<pcl::PointXYZLU>::const_iterator it = pc.begin(); it != pc.end(); ++it){
         octomap::point3d point(it->x, it->y, it->z);
         octomap::KeyRay m_keyRay;
         // maxrange check

         // free cells
         if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
           free_cells.insert(m_keyRay.begin(), m_keyRay.end());
         }
         // occupied endpoint
         octomap::OcTreeKey key;
         if (m_octree->coordToKeyChecked(point, key)){
           occupied_cells.insert(key);
           m_octree->updateNode(key, true);
           uint16_t label = it->label-1;
           if (label == 0)
             label = 1;

           float prob = 255.0/400.0 + 0.15; //pasarlo a log
          // std::cout << "probability  " << prob << ".\n";
           m_octree->averageNodeClassification(it->x, it->y, it->z,int(label), prob, true);
         }
       }

       std::cout <<" ya ingrese la nube de puntos\n ";

       // mark free cells only if not seen occupied in this cloud
       for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
         if (occupied_cells.find(*it) == occupied_cells.end()){
           m_octree->updateNode(*it, false);
         }
       }

       // now mark all occupied cells:
       for (octomap::KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
         m_octree->updateNode(*it, true);
       }

       size_t octomapSize = m_octree->size();
       std::cout << " Octomap size "<< octomapSize <<" \n" ;

       visualization_msgs::MarkerArray occupiedNodesVis;

       occupiedNodesVis.markers.resize(m_treeDepth+1);

       pcl::PointCloud<pcl::PointXYZL> pclCloud;

     for (octomap::ClassificationOcTree::iterator it = m_octree->begin(),
           end = m_octree->end(); it != end; ++it)
       {
         if (m_octree->isNodeOccupied(*it)){
           double z = it.getZ();
           if (z > -std::numeric_limits<double>::max() && z < std::numeric_limits<double>::max())
           {
             double size = it.getSize();
             double x = it.getX();
             double y = it.getY();
             double n_prob = it->getNodeClassification().probability;
             int n_class = it->getNodeClassification().num_class;

             //create marker:
             unsigned idx = it.getDepth();
             assert(idx < occupiedNodesVis.markers.size());

             geometry_msgs::Point cubeCenter;
             cubeCenter.x = x;
             cubeCenter.y = y;
             cubeCenter.z = z;

             occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

             std_msgs::ColorRGBA _color;
             double porcentage_prob = 1-(1/(1+exp(n_prob))) ;

             if (n_class == 1){
               _color.r = (0); _color.g = (0); _color.b = (0); _color.a = 0;   //Unlabel //Sky Sin check
             } else if (n_class == 2){
               _color.r = (1); _color.g = (1); _color.b = (1); _color.a =  porcentage_prob;   //Building white
             } else if (n_class == 3){
               _color.r = (0); _color.g = (1); _color.b = (1); _color.a =  porcentage_prob; //Pole cyan
             } else if (n_class == 4){
               _color.r = (0.4); _color.g = (0.2); _color.b = (0); _color.a = porcentage_prob;   //Road brown
             } else if (n_class == 5) {
               _color.r = (0.5); _color.g = (1); _color.b = (0.5); _color.a = porcentage_prob; //U. road
             } else if (n_class == 6){
               _color.r = (0); _color.g = (1); _color.b = (0); _color.a = porcentage_prob;   //Vegetation green
             } else if (n_class == 7){
               _color.r = (0); _color.g = (1); _color.b = (1); _color.a = porcentage_prob;   //Sign cyan
             } else if (n_class == 8){
               _color.r = (0.5); _color.g = (0.5); _color.b = (0.5); _color.a = porcentage_prob;   //Fence gray
             } else if (n_class == 9) {
               _color.r = (1); _color.g = (0); _color.b = (0); _color.a = porcentage_prob;  //Vehicle red
             } else if (n_class == 10) {
               _color.r = (1); _color.g = (1); _color.b = (0); _color.a = porcentage_prob;   //Pedestrian yellow
             } else {
               _color.r = (1); _color.g = (1); _color.b = (0); _color.a = porcentage_prob;   //Rider yewllow
             }

             occupiedNodesVis.markers[idx].colors.push_back(_color);

             pcl::PointXYZL _point = pcl::PointXYZL();
             _point.x = x; _point.y = y; _point.z = z;
             _point.label = n_class;
             pclCloud.push_back(_point);



           }
         }
       }


      for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
        double size = m_octree->getNodeSize(i);

        occupiedNodesVis.markers[i].header.frame_id = "odom";
        occupiedNodesVis.markers[i].header.stamp = vel_p->header.stamp;
        occupiedNodesVis.markers[i].ns = "map";
        occupiedNodesVis.markers[i].id = i;
        occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        occupiedNodesVis.markers[i].scale.x = size;
        occupiedNodesVis.markers[i].scale.y = size;
        occupiedNodesVis.markers[i].scale.z = size;
        if (occupiedNodesVis.markers[i].points.size() > 0)
          occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
        else
          occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
      }

      // mensaje del octomapS
      m_markerPub.publish(occupiedNodesVis);

      bag_out.write("/Octomap", m.getTime(), occupiedNodesVis);

      sensor_msgs::PointCloud2 cloud;
      pcl::toROSMsg (pclCloud, cloud);
      cloud.header.frame_id = "odom";
      cloud.header.stamp = vel_p->header.stamp;
      m_pointCloudPub.publish(cloud);

      if (number > 10000)
        bag_out.write("/Octomap_point_cloud", m.getTime(), cloud);

      number = number +1 ;
     }
   }
  }

 bag_out.close();
 bag.close();
 return 0;
}
