#ifndef BEHAVIOR_TREE_ORIENTED_H
#define BEHAVIOR_TREE_ORIENTED_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "terminalinfo.h"
#include "randomgenerator.h"
#include "template_calculator.h"
#define COMMAND_LOCAL 1  // Local frame

// Include behavior tree library
#include "bt/btFile.h"
using namespace BT;

class behavior_tree_oriented: public Controller
{
public:
  float v_x_ref, v_y_ref, ang;
  uint moving_timer; // Timer measuring how long a robot has been moving
  composite *tree; // Tree structure.
  blackboard BLKB; // Blackboard structure that ticks the tree during runtime.
  Template_Calculator t;

  /** Initialize
   * Initialize the behavior tree
   */
  behavior_tree_oriented();

  /**
   * Function for lattice to all robots
   *
   * @param closest vector of ID of all closest neighbors
   * @param v_x
   * @param v_y
   */
  void lattice_all(const int &ID, const std::vector<uint> &closest, float &v_x, float &v_y);

  /**
   * Update the inputs and run the behavior tree command
   */
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
  virtual void animation(const uint16_t ID);
};

#endif /*BEHAVIOR_TREE_ORIENTED_H*/
