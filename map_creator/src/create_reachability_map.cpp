// The spheres and poses are fused in a single dataset, instead of two datasets for sphere and poses
#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseArray.h"
#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <map_creator/sphere_discretization.h>
#include <map_creator/kinematics.h>
#include <map_creator/hdf5_dataset.h>
#include <boost/format.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

// write to file
#include <fstream>
#include <iterator>

// read csv
#include <string>
#include <vector>
#include <sstream> //istringstream
#include <iostream> // cout
#include <fstream> // ifstream

//struct stat st;

typedef std::vector<std::pair<std::vector<double>, const std::vector<double> *>> MultiVector;
//typedef std::multimap< const std::vector< double >*, const std::vector< double >* > MultiMap;

bool isFloat(std::string s)
{
  std::istringstream iss(s);
  float dummy;
  iss >> std::noskipws >> dummy;
  return iss && iss.eof(); // Result converted to bool
}

using namespace std;
 
/**
 * Reads csv file into table, exported as a vector of vector of doubles.
 * @param inputFileName input file name (full path).
 * @return data as vector of vector of doubles.
 */
vector<vector<double>> parse2DCsvFile(string inputFileName) {
 
    vector<vector<double> > data;
    ifstream inputFile(inputFileName);
    int l = 0;
 
    while (inputFile) {
        l++;
        string s;
        if (!getline(inputFile, s)) break;
        if (s[0] != '#') {
            istringstream ss(s);
            vector<double> record;
 
            while (ss) {
                string line;
                if (!getline(ss, line, ','))
                    break;
                try {
                    record.push_back(stof(line));
                }
                catch (const std::invalid_argument e) {
                    cout << "NaN found in file " << inputFileName << " line " << l
                         << endl;
                    e.what();
                }
            }
 
            data.push_back(record);
        }
    }
 
    if (!inputFile.eof()) {
        cerr << "Could not read file " << inputFileName << "\n";
        __throw_invalid_argument("File not found.");
    }
 
    return data;
}

void 
makeSpherePosesToVector(
  const octomap::point3d& origin, 
  std::vector< double > &pose, 
  std::vector< double > &data)
{
  data.resize(7);
  data[0]=double(pose[0]+origin.x());
  data[1]=double(pose[1]+origin.y());
  data[2]=double(pose[2]+origin.z());
  data[3]=double(pose[4]);
  data[4]=double(pose[5]);
  data[5]=double(pose[6]);
  data[6]=double(pose[3]);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "workspace");
  ros::NodeHandle n;
  ros::Time startit = ros::Time::now();
  float resolution = 0.08;
  // resolution = 0.2; // set default resolution
  n.getParam("create_reachability_map/resolution", resolution);

  kinematics::Kinematics k;
  std::string file = str(boost::format("%s_r%d_reachability_sample.h5") % k.getRobotName() % resolution);
  std::string path(ros::package::getPath("map_creator") + "/maps/");
  std::string filename;
  if (argc == 2)
  {
    if (!isFloat(argv[1]))
    {
      ROS_ERROR_STREAM("Probably you have just provided only the map filename. Hey!! The first argument is the "
                       "resolution.");
      return 0;
    }
    resolution = atof(argv[1]);
    file = str(boost::format("%s_r%d_reachability.h5") % k.getRobotName() % resolution);
    filename = path + file;
  }

  else if (argc == 3)
  {
    std::string name;
    name = argv[2];
    if (!isFloat(argv[1]) && isFloat(argv[2]))
    {
      ROS_ERROR_STREAM("Hey!! The first argument is the resolution and the second argument is the map filename. You "
                       "messed up.");
      return 0;
    }

    else
    {
      resolution = atof(argv[1]);
      std::string str(argv[2]);
      if (std::strchr(str.c_str(), '/'))
      {
        filename = argv[2];
      }
      else
        filename = path + str;
    }
  }
  else if (argc < 2)
  {
    ROS_INFO("You have not provided any argument. So taking default values.");
    filename = path + file;
  }

  

  // MOVEIT planning scene for collision checking
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  // ros::Publisher workspace_pub = n.advertise<map_creator::WorkSpace>("workspace", 10);


  ROS_INFO_STREAM("Will save the reachability map to " << filename);


  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok())
  {
    unsigned char max_depth = 16;
    unsigned char minDepth = 0;

    // A box of radius 1 is created. It will be the size of the robot+1.5. Then the box is discretized by voxels of
    // specified resolution
    // TODO resolution will be user argument
    // The center of every voxels are stored in a vector

    sphere_discretization::SphereDiscretization sd;
    float r = 1;
    octomap::point3d origin = octomap::point3d(0, 0, 0); // This point will be the base of the robot
    octomap::OcTree *tree = sd.generateBoxTree(origin, r, resolution);
    std::vector<octomap::point3d> new_data;
    ROS_INFO("Creating the box and discretizing with resolution: %f", resolution);
    int sphere_count = 0;
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
      sphere_count++;
    }
    new_data.reserve(sphere_count);
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
      new_data.push_back(it.getCoordinate());
    }

    ROS_INFO("Total no of spheres now: %lu", new_data.size());
    ROS_INFO("Please hold ON. Spheres are discretized and all of the poses are checked for Ik solutions. May take some "
             "time");

    // A sphere is created in every voxel. The sphere may be created by default or other techniques.
    // TODO Other techniques need to be modified. the user can specifiy which technique they want to use
    // TODO The sphere discretization parameter and rotation of every poses will be taken as argument. If the final
    // joints can rotate (0, 2pi) we dont need to rotate the poses.
    // Every discretized points on spheres are converted to pose and all the poses are saved in a multimap with their
    // corresponding sphere centers
    // If the resolution is 0.01 the programs not responds

    // std::cout << new_data << std::endl;
    std::string sample_filename = path + "sample_poses.csv";
    ROS_INFO_STREAM("Reading sample poses from file: " << sample_filename);
    vector<vector<double>> sample_poses = parse2DCsvFile(sample_filename);

    float radius = resolution;

    VectorOfVectors sphere_coord;
    sphere_coord.resize(new_data.size());

    MultiVector pose_col;
    pose_col.reserve(new_data.size() * 50);

    for (int i = 0; i < new_data.size(); i++)
    {
      // std::cout << "new data: " << new_data[i] << std::endl;
      // static std::vector<geometry_msgs::Pose> pose;
      sd.convertPointToVector(new_data[i], sphere_coord[i]);

      // sd.make_sphere_poses(new_data[i], radius, pose);
      // for (int j = 0; j < pose.size(); j++)
      // {
      //   static std::vector<double> point_on_sphere;
      //   sd.convertPoseToVector(pose[j], point_on_sphere);
        
      //   pose_col.push_back(std::make_pair(point_on_sphere, &sphere_coord[i]));
      //   // TODO make sure the set of poses is identical
      // }
      for (int j = 0; j < sample_poses.size(); j++)
      {
        static std::vector<double> point_on_sphere;
        makeSpherePosesToVector(new_data[i], sample_poses[j], point_on_sphere);

        pose_col.push_back(std::make_pair(point_on_sphere, &sphere_coord[i]));
      }
    }

    // Every pose is checked for IK solutions. The reachable poses and the their corresponsing joint solutions are
    // stored. Only the First joint solution is stored. We may need this solutions in the future. Otherwise we can show
    // the robot dancing with the joint solutions in a parallel thread
    // TODO Support for more than 6DOF robots needs to be implemented.

    // Kinematics k;

    // boolean indices
    std::vector<
        std::pair<
            std::vector<int>,
            const std::vector<double> *>>
        voxel_bool_indices;

    vector<int> v1;
    v1.reserve(50);

    MultiMapPtr pose_col_filter;
    VectorOfVectors ik_solutions;
    ik_solutions.reserve(pose_col.size());

    int progress = (int)(pose_col.size() / 100);
    int count = 1;
    size_t i = 0;
    for (MultiVector::iterator it = pose_col.begin(); it != pose_col.end(); ++it)
    {

      if (count % progress == 0)
        ROS_INFO("Progress: %2f\%", count / (double)pose_col.size() * 100);

      static std::vector<double> joints(6);
      int solns;
      // if (k.isIKSuccess(it->first, joints, solns))
      bool found_sol = k.isIKSuccess(
          planning_scene,
          collision_request,
          collision_result,
          it->first,
          joints,
          solns);
      if (found_sol)
      {
        // std::vector< double > points_on_sphere = it->first;
        // std::cout << "############################" << std::endl;
        // for(double c: points_on_sphere)
        //   std::cout << c << " ";
        // std::cout << std::endl;

        pose_col_filter.insert(std::make_pair(it->second, &(it->first)));
        ik_solutions.push_back(joints);
        // cout<<it->first[0]<<" "<<it->first[1]<<" "<<it->first[2]<<" "<<it->first[3]<<" "<<it->first[4]<<"
        // "<<it->first[5]<<" "<<it->first[6]<<endl;
        // std::cout << "1" << " ";
        v1.push_back(1);
      }
      else
      {
        // std::cout << "0" << " ";
        v1.push_back(0);
      }

      if (i == 49)
      {
        voxel_bool_indices.push_back(
            std::make_pair(v1, it->second));
        i = -1;
        v1.clear();
      }

      ++count;
      ++i;
    }

    ROS_INFO("Total number of poses: %lu", pose_col.size());
    ROS_INFO("Total number of reachable poses: %lu", pose_col_filter.size());

    // visualization ONLY for less spheres!
    // for (std::pair<
    //          std::vector<int>,
    //          const std::vector<double> *>
    //          p : voxel_bool_indices)
    // {
    //   // std::vector<int> *bl = p.first;
    //   std::vector<int> vec = p.first;
    //   // for (int b: vec)
    //   //   std::cout << b << " ";
    //   // std::cout << std::endl;
    //   // std::cout << vec.size() << std::endl;
    //   // std::cout << vec[0] << std::endl;
    //   const vector<double> *sphere_coord = p.second;
    //   int ri = accumulate(vec.begin(), vec.end(), 0);
    //   // std::cout << "RI: " << ri << std::endl;

    //   if (ri > 0)
    //   {
    //     std::cout << ri
    //               << "\t(" << sphere_coord->at(0) << ", " << sphere_coord->at(1)
    //               << ", " << sphere_coord->at(2) << ")" << std::endl;
    //     std::cout << std::endl;
    //   }
    // }

    ROS_INFO("#####################################");

    // The centers of reachable spheres are stored in a map. This data will be utilized in visualizing the spheres in
    // the visualizer.
    // TODO there are several maps are implemented. We can get rid of few maps and run large loops. The complexity of
    // accessing map is Olog(n)

    MapVecDoublePtr sphere_color;

    for (MultiMapPtr::iterator it = pose_col_filter.begin(); it != pose_col_filter.end(); ++it)
    {
      const std::vector<double> *sphere_coord = it->first;
      //const std::vector<double>* point_on_sphere = it->second;

      // Reachability Index D=R/N*100;
      // TODO write a bool array for R
      float d = float(pose_col_filter.count(sphere_coord)) / (pose_col.size() / new_data.size()) * 100;
      // std::cout << float(pose_col_filter.count(sphere_coord))
      //           << "\t(" << sphere_coord->at(0) << ", " << sphere_coord->at(1)
      //           << ", " << sphere_coord->at(2) << ")" << std::endl;
      sphere_color.insert(std::make_pair(it->first, double(d)));
    }

    ROS_INFO("No of spheres reachable: %lu", sphere_color.size());

    // Creating maps now
    //Saving map to dataset
    hdf5_dataset::Hdf5Dataset h5(filename);
    h5.saveReachMapsToDataset(pose_col_filter, sphere_color, resolution);

    double dif = ros::Duration(ros::Time::now() - startit).toSec();
    ROS_INFO("Elasped time is %.2lf seconds.", dif);
    ROS_INFO("Completed");


    ROS_INFO("Write Boolean Indices to file");
    std::string file_bl = str(boost::format("bl_%s_r%d_reachability.txt") % k.getRobotName() % resolution);
    std::string filename_bl = path + file_bl;
    // // write boolean indices to file
    std::ofstream output_file(filename_bl);

    for (std::pair<
             std::vector<int>,
             const std::vector<double> *>
             p : voxel_bool_indices)
    {
      std::vector<int> vec = p.first;
      const vector<double> *sphere_coord = p.second;

      int ri = accumulate(vec.begin(), vec.end(), 0);

      std::ostringstream oss;
      std::copy(vec.begin(), vec.end(), std::ostream_iterator<int>(oss, " "));
      output_file << ri << "," << sphere_coord->at(0) << " " 
                  << sphere_coord->at(1) << " " << sphere_coord->at(2) << "," 
                  << oss.str() << "\n";

      // std::cout << ri << "," << sphere_coord->at(0) << " " 
      //           << sphere_coord->at(1) << " " << sphere_coord->at(2) << "," 
      //           << oss.str() << "\n";
    }
    output_file.close();

    ros::spinOnce();
    // sleep(10000);
    return 1;
    loop_rate.sleep();
    count;
  }
  return 0;
}
