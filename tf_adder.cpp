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

#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf/tf.h>

#include <thread>
#include <functional>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_adder");
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
  bagname_out.insert(bagname_out.length()-4,"_tf");
  std::cout << "bag directory " << bagname <<"\n";
  std::cout << "out_bag directory " << bagname_out <<"\n";
  rosbag::Bag bag,bag_out;
  bag.open(bagname, rosbag::bagmode::Read);
  bag_out.open(bagname_out, rosbag::bagmode::Write);

  // Topics' name
  std::string velodyne_p =  "/velodyne/front/labelled";
  std::string tf_msg     =  "/tf";
  std::string tf_msg_stat=  "/tf_static";
  std::string odo        =  "/zio/odometry/rear";
  std::string odom_r     =  "/vn100/odometry";
  std::string gps        =  "/ublox_gps/fix";
  std::string imu        =  "/vn100/imu";

  // Topics to load
  std::vector<std::string> topics;
  topics.push_back(velodyne_p);
  topics.push_back(tf_msg_stat);
  topics.push_back(odo);
  topics.push_back(odom_r);
  topics.push_back(gps);
  topics.push_back(imu);
  rosbag::View view(bag, rosbag::TopicQuery(topics));


  // Publishers needed by the "Position filter"
  ros::Publisher imu_pub, gps_pub, odom_pub;
  imu_pub= nh.advertise<sensor_msgs::Imu>("/vn100/imu",1000);
  gps_pub= nh.advertise<sensor_msgs::NavSatFix>("/ublox_gps/fix",10);
  odom_pub= nh.advertise<nav_msgs::Odometry>("/zio/odometry/rear",100);

  // tf message to be included in the bag
  tf2_msgs::TFMessage horizon;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped base_vel;
  base_vel.child_frame_id = "none";

  std::cout << "Starting to read the bag  \n";
  // Load all messages
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {

    //cleaning all the transforms
    horizon.transforms.clear();

    // writing original topics
    bag_out.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());

    if (m.getTopic() == imu || ("/" + m.getTopic() == imu )) // /vn100/imu
    {
      // Reading the Imu data
      sensor_msgs::Imu::ConstPtr imu_data = m.instantiate<sensor_msgs::Imu>();
      if (imu_data != NULL)
      {
        tf::Quaternion q1(imu_data->orientation.x, imu_data->orientation.y, imu_data->orientation.z, imu_data->orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 mat(q1);
        mat.getRPY(roll, pitch, yaw);
        tf2::Quaternion q;
        q.setRPY(-roll, -pitch, 0.);
        geometry_msgs::TransformStamped baselink, footprint, utm_map;
        baselink.transform.rotation.x = footprint.transform.rotation.x = q.x();
        baselink.transform.rotation.y = footprint.transform.rotation.y = q.y();
        baselink.transform.rotation.z = footprint.transform.rotation.z = q.z();
        baselink.transform.rotation.w = footprint.transform.rotation.w = q.w();
        baselink.header.stamp = footprint.header.stamp = imu_data->header.stamp;
        baselink.header.frame_id = "base_link";
        footprint.header.frame_id = "base_footprint";
        baselink.child_frame_id = "base_link_horizon";
        footprint.child_frame_id = "base_footprint_horizon";
        // adding the horizon transforms;
        horizon.transforms.push_back(baselink);
        horizon.transforms.push_back(footprint);
        // publishing imu data
        imu_pub.publish(imu_data);
        //utm to map
        utm_map.transform.rotation.x = 0;
        utm_map.transform.rotation.y = 0;
        utm_map.transform.rotation.z = 0;
        utm_map.transform.rotation.w = 1;
        utm_map.transform.translation.x = 331275.383734;
        utm_map.transform.translation.y = 6250099.29659;
        utm_map.transform.translation.z = 0;
        utm_map.header.stamp = imu_data->header.stamp;
        utm_map.header.frame_id = "utm";
        utm_map.child_frame_id = "map";
        horizon.transforms.push_back(utm_map);


        if (base_vel.child_frame_id == "velodyne_front_link")
        {
         base_vel.header.stamp = m.getTime();
         horizon.transforms.push_back(base_vel);
        }

      }
    }

    if (m.getTopic() == gps || ("/" + m.getTopic() == gps )) // /ublox_gps/fix
    {
       sensor_msgs::NavSatFix::ConstPtr gps_data = m.instantiate<sensor_msgs::NavSatFix>();
      if (gps_data != NULL)
      {
        // publishing gps data
        gps_pub.publish(gps_data);
      }
    }

    if (m.getTopic() == odo || ("/" + m.getTopic() == odo )) // /zio/odometry/rear
    {
      nav_msgs::Odometry::ConstPtr od = m.instantiate<nav_msgs::Odometry>();
      if (od != NULL)
      {
        // publishing odometry data
        odom_pub.publish(od);
      }
    }
    
    if (m.getTopic() == tf_msg_stat || ("/" + m.getTopic() == tf_msg_stat))
    {
      tf2_msgs::TFMessage::ConstPtr tf_info = m.instantiate<tf2_msgs::TFMessage>();
      if (tf_info != NULL)
      {
        for(int i=0; i < tf_info->transforms.size();i++){
          if (tf_info->transforms[i].header.frame_id == "base_link"){
            if (tf_info->transforms[i].child_frame_id == "velodyne_front_link"){
                base_vel = tf_info->transforms[i];
                horizon.transforms.push_back(base_vel);
            }
          }
        }
      }
    }

    // Looking for the required transforms and adding them to the tf
    geometry_msgs::TransformStamped map_odom, odom_basel;
    // Map to Odom
    try{
      map_odom = tfBuffer.lookupTransform("map", "odom",ros::Time(0));
      map_odom.header.stamp = m.getTime();
      horizon.transforms.push_back(map_odom);
    }
    catch (tf2::TransformException &ex) {

    }
   // Odom to base_link
    try{
      odom_basel = tfBuffer.lookupTransform("odom", "base_link",ros::Time(0));
      odom_basel.header.stamp = m.getTime();
      horizon.transforms.push_back(odom_basel);
    }
    catch (tf2::TransformException &ex) {

    }

    if (horizon.transforms.size()>0)
      bag_out.write("/tf", m.getTime(), horizon);

  }

 bag_out.close();
 bag.close();
 return 0;
}
