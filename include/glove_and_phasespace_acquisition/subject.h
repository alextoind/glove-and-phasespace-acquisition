/*  Copyright (C) 2015 Alessandro Tondo
 *  email: tondo.codes+ros <at> gmail.com
 *
 *  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public 
 *  License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any 
 *  later version.
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more 
 *  details.
 *  You should have received a copy of the GNU General Public License along with this program.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GUARD_SUBJECT_H
#define GUARD_SUBJECT_H

// Standard libraries
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <ctime>
// ROS libraries
#include <ros/ros.h>
#include <ros/time.h>
// item library
#include "item.h"

// default values for ROS params (if not specified by the user)
#define DEFAULT_SUBJECT_NAME "undefined"
#define DEFAULT_EXPERIMENT_NAME "experiment"
#define DEFAULT_START_TIME 0
#define DEFAULT_STOP_TIME 60*60  // a sufficiently large ending time (1 hour)
#define DEFAULT_WAIT_TIME_NEXT 5  // wait at least 5 seconds between 2 distinct acquisition

/*  This class purpose is to provide a simple logging interface to manage multiple acquisition for a given subject.
 *  Log files are named and stored in a hierarchical pattern based on subject name, experiment name and current date
 *  and time of the acquisition. This class is as generic as possible, but has been developed for the Glove and
 *  PhaseSpace acquisition packages, therefore has some specific filed like the object list retrieved with a YAML file.
 *  In this list are stored every all the information for a given experiment. A simple example is the following:
 *  
 *  items:
 *    item_1: {
 *      size: 5,
 *      directions: ["approaching_direction_11","approaching_direction_12","...",]
 *    }
 *    item_2: {
 *      size: 5,
 *      directions: ["approaching_direction_21","approaching_direction_22","...",]
 *    }
 *
 *  If you edit this class, please try to follow these C++ style guidelines: http://wiki.ros.org/CppStyleGuide.
 *
 *  ROS params:
 *    + subject_name
 *    + experiment
 *    + wait_time_next
 *    + start_time
 *    + stop_time
 */
class Subject {
public:
  Subject(std::map<std::string, std::ofstream*> log_files_map);
  ~Subject();

  int closeLogFiles();

  const ros::Duration & getAcquisitionStartTime() const { return acquisition_start_time_; }

  const ros::Duration & getAcquisitionStopTime() const { return acquisition_stop_time_;  }

  int getItemAndPop(Item &item);

  void initializeLogFiles(std::string item, std::string direction);

  void removeLastLogFiles();

  void updateDate();

  void waitForNextAcquisition();

private:
  ros::NodeHandle *private_node_handle_;

  // path element variables
  std::string date_time_;
  std::string name_;
  std::string experiment_;

  ros::Duration acquisition_start_time_;
  ros::Duration acquisition_stop_time_;
  double wait_time_next_;
  std::vector<Item> objects_list_;

  // log file variables
  std::string current_item_;
  std::string current_direction_;
  std::map<std::string, std::ofstream*> log_files_map_;  // terminal file name with the proper std::ofstream pointer


  void parseItems();

  void setAcquisitionTimes();

  void shuffleItems(std::time_t random_seed);
};

#endif
