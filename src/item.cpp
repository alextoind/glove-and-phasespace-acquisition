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

#include "glove_and_phasespace_acquisition/item.h"

Item::Item() {
  std::vector<std::string> directions;
  directions.push_back("undefined_dir");

  Item("undefined", 0, directions);
}

void Item::printItem() {
  std::string directions = "";
  for (auto const& dir : approaching_directions_) {
    directions += dir + " ";
  }

  ROS_DEBUG_STREAM("Object parameters\n"
                << " + Name: " << name_ << '\n'
                << " + Size: " << size_ << '\n'
                << " + Directions: " << directions);
}

void Item::shuffleDirections(std::time_t random_seed) {
  std::srand(unsigned(random_seed));
  std::random_shuffle(approaching_directions_.begin(), approaching_directions_.end());
}

int Item::getDirectionAndPop(std::string &direction) {
  if (!approaching_directions_.empty()) {
    direction = approaching_directions_.back();
    approaching_directions_.pop_back();
    return 0;
  }
  return -1;
}
