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

#include "glove_and_phasespace_acquisition/subject.h"

Subject::Subject(std::map<std::string, std::ofstream*> log_files_map)
  : log_files_map_(log_files_map) {
  // handles server private parameters (private names are protected from accidental name collisions)
  private_node_handle_ = new ros::NodeHandle("~");

  // retrieves info (from ROS params if specified)
  private_node_handle_->param("subject_name", name_, std::string(DEFAULT_SUBJECT_NAME));
  private_node_handle_->param("experiment", experiment_, std::string(DEFAULT_EXPERIMENT_NAME));
  private_node_handle_->param("wait_time_next", wait_time_next_, (double)DEFAULT_WAIT_TIME_NEXT);

  parseItems();

  std::time_t random_seed = std::time(NULL);
  shuffleItems(random_seed);

  // sets start and stop times with ROS params or default values
  setAcquisitionTimes();

  updateDate();
}

Subject::~Subject() {
  delete private_node_handle_;
}

int Subject::closeLogFiles() {
  int num_log_files_closed = 0;
  std::string base_path = name_ + "/" + experiment_ + "/" + current_item_ + "/" + current_direction_ + "/";

  // closes log files previously opened
  for (auto &log_file : log_files_map_) {
    if (log_file.second->is_open()) {
      std::string complete_path = base_path + date_time_ + log_file.first + ".dat";
      log_file.second->close();
      num_log_files_closed++;

      // checks for emptiness
      std::ifstream last_file(complete_path);
      if (last_file.peek() == std::ifstream::traits_type::eof()) {
        ROS_DEBUG_STREAM("[Subject::closeLogFiles] Log file is empty.");
        if (std::remove(complete_path.c_str()) == 0) {
          ROS_DEBUG_STREAM("[Subject::closeLogFiles] Log file removed: " << complete_path);
          continue;
        }
        ROS_WARN_STREAM("[Subject::closeLogFiles] Can't remove log file: " << complete_path);
        continue;
      }
      ROS_DEBUG_STREAM("[Subject::closeLogFiles] Log file generated: " << complete_path);
    }
  }

  return num_log_files_closed;
}

int Subject::getItemAndPop(Item &item) {
  if (!objects_list_.empty()) {
    item = objects_list_.back();
    objects_list_.pop_back();
    return 0;
  }
  return -1;
}

void Subject::initializeLogFiles(std::string item, std::string direction) {
  // checks for previously opened log files (does not remove the last log files)
  closeLogFiles();

  // creates log files folder if it doesn't exist yet
  std::string base_path = name_ + "/" + experiment_ + "/" + item + "/" + direction + "/";
  std::string command = "mkdir -p " + base_path;
  system(command.c_str());

  // opens log files
  for (auto &log_file : log_files_map_) {
    log_file.second->open(base_path + date_time_ + log_file.first + ".dat");
  }

  // stores current item and direction (only for debug when the std::ofstream are closed)
  current_item_ = item;
  current_direction_ = direction;
}

void Subject::parseItems() {
  ROS_INFO_STREAM("[Subject::parseItems] Parsing object parameters from YAML configuration file...");

  std::string base_path = "items";
  std::vector<std::string> item_names;
  XmlRpc::XmlRpcValue list;
  if (!private_node_handle_->getParam("/" + base_path, list)) {
    ROS_ERROR_STREAM("[ubject::parseItems] Can't retrieve '" + base_path + "' from YAML configuration file.");
    return;
  }
  for (auto it = list.begin(); it != list.end(); it++) {
    item_names.push_back(it->first);
  }

  for (auto const& item_name : item_names) {
    std::string param_name;
    std::string field_name;

    std::vector<std::string> item_directions;
    field_name = "directions";
    param_name = "/" + base_path + "/" + item_name + "/" + field_name;
    if (!private_node_handle_->getParam(param_name, item_directions)) {
      ROS_WARN_STREAM("[Subject::parseItems] Can't retrieve '" + param_name + "' from YAML configuration file.");
      break;
    }

    int item_size = 0;
    field_name = "size";
    param_name = "/" + base_path + "/" + item_name + "/" + field_name;
    if (!private_node_handle_->getParam(param_name, item_size)) {
      ROS_WARN_STREAM("[Subject::parseItems] Can't retrieve '" + param_name + "' from YAML configuration file.");
      break;
    }

    Item item(item_name, item_size, item_directions);
    objects_list_.push_back(item);

    item.printItem();  // debug info
  }
}

void Subject::removeLastLogFiles() {
  std::string base_path = name_ + "/" + experiment_ + "/" + current_item_ + "/" + current_direction_ + "/";

  // closes log files previously opened
  for (auto &log_file : log_files_map_) {
    std::string complete_path = base_path + date_time_ + log_file.first + ".dat";

    if (std::remove(complete_path.c_str()) == 0) {
      ROS_DEBUG_STREAM("[Subject::closeLogFiles] Log file removed: " << complete_path);
      continue;
    }
    ROS_WARN_STREAM("[Subject::closeLogFiles] Can't remove log file: " << complete_path);
  }
}

void Subject::setAcquisitionTimes() {
  double acquisition_start_time = 0;
  double acquisition_stop_time = 0;
  // retrieves info (from ROS params if specified)
  private_node_handle_->param("start_time", acquisition_start_time, double(DEFAULT_START_TIME));
  private_node_handle_->param("stop_time", acquisition_stop_time, double(DEFAULT_STOP_TIME));

  if (acquisition_stop_time < acquisition_start_time) {
    ROS_WARN_STREAM("[Subject::setAcquisitionTimes] 'stop_time' is smaller than 'start_time'.");
    acquisition_stop_time = acquisition_start_time;
  }

  acquisition_start_time_ = ros::Duration(acquisition_start_time);
  acquisition_stop_time_ = ros::Duration(acquisition_stop_time);
}

void Subject::shuffleItems(std::time_t random_seed) {
  std::srand(unsigned(random_seed));
  std::random_shuffle(objects_list_.begin(), objects_list_.end());
}

void Subject::updateDate() {
  // stores in 'date_time_' the current time converted into a handful form (date/time format YYYYMMDD_HHMMSS)
  std::time_t rawtime;
  char buffer[16];
  std::time(&rawtime);
  std::strftime(buffer, 16, "%G%m%d_%H%M%S", std::localtime(&rawtime));
  date_time_ = buffer;
}

void Subject::waitForNextAcquisition() {
  ROS_INFO_STREAM("Wait " << wait_time_next_ << " seconds to continue...");
  ros::Duration(wait_time_next_).sleep();
}
