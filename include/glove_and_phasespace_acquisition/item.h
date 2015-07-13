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

#ifndef GUARD_ITEM_H
#define GUARD_ITEM_H

// Standard libraries
#include <string>
#include <iostream>
#include <vector>
// ROS libraries
#include <ros/ros.h>
#include <ros/time.h>

class Item {
public:
  Item();
  Item(std::string name, int size, std::vector<std::string> approaching_directions)
    : name_(name)
    , size_(size)
    , approaching_directions_(approaching_directions) {}
  ~Item() {}

  int getDirectionAndPop(std::string &direction);

  const std::string & getName() const { return name_; }

  void printItem();

  void shuffleDirections(std::time_t random_seed);

private:
  std::string name_;
  int size_;
  std::vector<std::string> approaching_directions_;
};

#endif
