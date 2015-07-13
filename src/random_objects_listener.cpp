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

#include <glove_acquisition/glove_core.h>
#include <phasespace_acquisition/phasespace_core.h>
#include "glove_and_phasespace_acquisition/subject.h"

int main(int argc, char **argv) {
  std::cout << LICENSE_INFO << std::flush;

  ros::init(argc, argv, "random_objects_listener");
  GloveCore *glove_listener;
  try {
    glove_listener = new GloveCore("listener");
  } catch (const GloveCoreException& e) {
    ROS_FATAL_STREAM(e.what() << ": failure in 'constructor' function");
    ros::shutdown();
    std::exit(EXIT_FAILURE);
  }
  PhasespaceCore *phasespace_listener;
  try {
    phasespace_listener = new PhasespaceCore("listener");
  } catch (const PhasespaceCoreException& e) {
    ROS_FATAL_STREAM(e.what() << ": failure in 'constructor' function");
    ros::shutdown();
    std::exit(EXIT_FAILURE);
  }

  std::map<std::string, std::ofstream*> log_files_map = glove_listener->getLogFilesMap();
  for (auto const& log_file : phasespace_listener->getLogFilesMap()) {
    log_files_map.insert(log_file);
  }
  Subject subject(log_files_map);

  ROS_INFO_STREAM("Starting acquisition...");
  while (glove_listener->getStatus() || phasespace_listener->getStatus()) {
    Item current_item;
    if (subject.getItemAndPop(current_item) < 0) {
      ROS_INFO_STREAM("There are no more items in the list. Acquisition is terminated correctly. Exiting...");
      ros::shutdown();
      break;
    }

    std::time_t random_seed = std::time(NULL);
    current_item.shuffleDirections(random_seed);

    while (glove_listener->getStatus() || phasespace_listener->getStatus()) {
      std::string current_direction = "";
      if (current_item.getDirectionAndPop(current_direction) < 0) {
        ROS_INFO_STREAM("There are no more directions to be acquired. Skip to next item...");
        break;
      }

      std::string answer = "";
      std::cout << std::endl;
      ROS_INFO_STREAM("Acquisition for '" << current_item.getName() << "'");
      ROS_INFO_STREAM("Chosen direction: '" << current_direction << "'");
      ROS_INFO_STREAM("Staring time: " << subject.getAcquisitionStartTime());
      ROS_INFO_STREAM("Ending time: " << subject.getAcquisitionStopTime());
      while ((std::cout << "[ INFO] [" << ros::Time::now() << "]: Confirm acquisition? ['yes','chdir','chobj','exit'] ") 
              && (!(std::cin >> answer)
                  || (answer != "yes" && answer != "chdir" && answer != "chobj" && answer != "exit"))) {
        ROS_WARN_STREAM("Wrong input!");
        ROS_INFO_STREAM("Help:" << "\n + type 'yes' to confirm and start acquisition"
                                << "\n + type 'chdir' to change approaching direction"
                                << "\n + type 'chobj' to skip this object"
                                << "\n + type 'exit' to exit acquisition\n");
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }

      if (answer == "yes") {
        std::string repeat = "yes";
        while (repeat == "yes") {
          // updates log files base path with current time and creates new log files
          subject.updateDate();
          subject.initializeLogFiles(current_item.getName(), current_direction);

          ros::Time initial_time = ros::Time::now();
          ros::Time current_time = ros::Time::now();
          ros::Duration wait_time = subject.getAcquisitionStartTime();

          // prints on screen a simple countdown
          while (wait_time > ros::Duration(1.0)) {
            ROS_INFO_STREAM("Wait " << wait_time << " seconds to start...");
            ros::Duration(ros::Duration(1.0)).sleep();
            wait_time = wait_time - ros::Duration(1.0);
          }
          while (wait_time > ros::Duration(0.0)) {
            ROS_INFO_STREAM("Wait " << wait_time << " seconds to start...");
            ros::Duration(ros::Duration(0.1)).sleep();
            wait_time = wait_time - ros::Duration(0.1);
          }

          ROS_INFO_STREAM("Node is retrieving messages... (<ctrl+c> to terminate)");
          while ((glove_listener->getStatus() || phasespace_listener->getStatus())
                 && (current_time - initial_time) < subject.getAcquisitionStopTime()) {
            ros::spinOnce();
            current_time = ros::Time::now();
          }
          ROS_INFO_STREAM("Done!");

          phasespace_listener->printStats();

          int closed = subject.closeLogFiles();
          glove_listener->updateNumLogFiles(closed - phasespace_listener->getLogFilesMap().size());
          phasespace_listener->updateNumLogFiles(closed - glove_listener->getLogFilesMap().size());

          // checks if ROS has been shutdown 
          if (!glove_listener->getStatus() || !phasespace_listener->getStatus()) {
            std::cout << '\n' << std::endl;
            break;
          }
          subject.waitForNextAcquisition();

          repeat = "";
          while ((std::cout << "[ INFO] [" << ros::Time::now()
                            << "]: Repeat acquisition for '" << current_direction << "'? ['yes','no','remove'] ")
                  && (!(std::cin >> repeat) || (repeat != "yes" && repeat != "no" && repeat != "remove"))) {
            ROS_WARN_STREAM("Wrong input!");
            ROS_INFO_STREAM("Help:" << "\n + type 'yes' to confirm repetition"
                                    << "\n + type 'no' to continue"
                                    << "\n + type 'remove' to delete the current log files\n");
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          }

          // removes last log files and restarts acquisition with same settings
          if (repeat == "remove") {
            subject.removeLastLogFiles();
            repeat = "yes";
            ROS_INFO_STREAM("Restarting acquisition for '" << current_item.getName() << "'");
            ROS_INFO_STREAM("Chosen direction: '" << current_direction << "'");
          }
        }
      }
      else if (answer == "chdir") {
        continue;  // gets another item from the list
      }
      else if (answer == "chobj") {
        break;  // gets another item from the list
      }
      else if (answer == "exit") {
        ROS_INFO_STREAM("Terminating acquisition correctly. Exiting...");
        ros::shutdown();
        break;
      }
      else {
        ROS_FATAL_STREAM("Something went wrong while parsing input. Terminating...");
        ros::shutdown();
        std::exit(EXIT_FAILURE);
      }
    }
  }

  delete glove_listener;
  delete phasespace_listener;
  return 0;
}
