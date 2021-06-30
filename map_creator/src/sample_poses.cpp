#include <ros/ros.h>
#include <ros/package.h>
#include <map_creator/sphere_discretization.h>



// read csv
#include <string>
#include <vector>
#include <sstream> //istringstream
#include <iostream> // cout
#include <fstream> // ifstream
 
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



int main()
{
  // print the original poses
  // sphere_discretization::SphereDiscretization sd;
  
  // const octomap::point3d origin = octomap::point3d(0, 0, 0);
  // double r = 0.05;
  // std::vector<geometry_msgs::Pose> pose_Col;

  // sd.make_sphere_poses(origin, r, pose_Col);
  // for (geometry_msgs::Pose p: pose_Col)
  // {
  //   std::cout << p.position.x << " " << p.position.y << " " << p.position.z 
  //             << std::endl;
  // }


  // import from csv zacharias method
  // Read three_cols.csv and ones.csv
  std::string path(ros::package::getPath("map_creator") + "/maps/");
  std::string filename = path + "sample_poses.csv";
  std::cout << "Reading sample poses from file: "+ filename << std::endl;
  vector<vector<double>> data = parse2DCsvFile(filename);
 
  for (auto l : data) {
      for (auto x : l)
          cout << x << " ";
      cout << endl;
  }

  return 0;
}