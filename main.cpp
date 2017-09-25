/* Copyright (C)
 * 2017 - wuxingzhang
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Reference document:
 * [1](http://wiki.icub.org/wiki/KDL-simple)
 * [2](https://medium.com/@sarvagya.vaish/forward-kinematics-using-orocos-kdl-da7035f9c8e)
 * [3](http://www.orocos.org/kdl/examples)
 */
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#define DEBUG

using namespace KDL;

int main(int argc, char *argv[])
{
  //
  // Create a KDL kinematic Chain.
  //
  // A Chain is made up of Segments. Each Segment consists of a Joint and a Frame.
  // The Joint indicates how the Frame moves - rotation or translation about / along an axis.
  // Reference: RobotAnno-V5 dimension parameter of robot arm v1.0(with clip).pdf
  //

  Chain kdlChain = Chain();

  Joint joint1(Joint::RotY);
  Frame frame1 = Frame(Vector(0.0, 246.0, 0.0));
  kdlChain.addSegment(Segment(joint1, frame1));

  Joint joint2(Joint::RotZ);
  Frame frame2 = Frame(Vector(0.0, 225, 0.0));
  kdlChain.addSegment(Segment(joint2, frame2));

  Joint joint3(Joint::RotZ);
  Frame frame3 = Frame(Vector(0.0, 80, 0.0));
  kdlChain.addSegment(Segment(joint3, frame3));

  Joint joint4(Joint::RotY);
  Frame frame4 = Frame(Vector(0.0, 137.32, 0.0));
  kdlChain.addSegment(Segment(joint4, frame4));

  Joint joint5(Joint::RotX);
  Frame frame5 = Frame(Vector(0.0, 40, 0.0));
  kdlChain.addSegment(Segment(joint5, frame5));

  // Joint joint6(Joint::RotY);
  // Frame frame6 = Frame(Vector(0.0, 0.1, 0.0));
  // kdlChain.addSegment(Segment(joint6, frame6));

  //
  // Joint Angles
  //
  unsigned int nj = kdlChain.getNrOfJoints();
  JntArray jointInitAngles = JntArray(nj);
  for(int i; i<nj; i++)
    jointInitAngles(i) = 0;

  JntArray jointAngles = JntArray(nj);
  // jointAngles(0) = M_PI / 2.;       // Joint 1
  // jointAngles(1) = 0;//M_PI / 3.;       // Joint 2
  // jointAngles(2) = 0;//M_PI / 6.;      // Joint 3
  // jointAngles(3) = 0;//M_PI / 6.;       // Joint 4
  // jointAngles(4) = M_PI / 2.;       // Joint 5
  // jointAngles(5) = 0;//-M_PI / 6.;      // Joint 6

  // Assign some values to the joint positions
  // for(unsigned int i=0; i<nj; i++){
  // float myinput;
  // printf ("Enter the position of joint %i: ", i+1);
  // scanf ("%e",&myinput);
  // jointAngles(i) = (double)myinput*(M_PI/180);
  // }

  // eg:
  // ./SimpleRobotFK 90 0 0 0 90
  for(unsigned int i=1; i<nj+1; i++){
    if(i>=argc) {
      jointAngles(i-1) = 0.0;
      continue;
    }
#ifdef DEBUG
    printf("Argument position of joint %d is %f\n", i, atof(argv[i]));
#endif
    //if calc ik, should first set jointAngle to 0 0 -90 0 -90
    jointAngles(i-1) = (double)atof(argv[i])*(M_PI/180);
  }

  //
  // Perform Forward Kinematics
  //

#ifdef DEBUG
  std::cout << "/**********Forward kinematics**********/" << std::endl;
#endif
  ChainFkSolverPos_recursive FKSolver = ChainFkSolverPos_recursive(kdlChain);
  Frame eeFrame;
  // Calculate and print the init frame
  FKSolver.JntToCart(jointInitAngles, eeFrame);
#ifdef DEBUG
  std::cout << "Rotational Matrix of the init Frame:" << std::endl;
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++) {
      double a = eeFrame(i, j);
      if (a < 0.0001 && a > -0.001) {
        a = 0.0;
      }
      std::cout << std::setprecision(4) << a << "\t\t";
    }
    std::cout << std::endl;
  }
#endif

  // Calculate and print the final frame
  FKSolver.JntToCart(jointAngles, eeFrame);
#ifdef DEBUG
  std::cout << "Desired Angles:\n";
  for(int i=0; i<nj-1; i++)
  {
    std::cout << jointAngles(i)*(180/M_PI) << "\t\t";
  }
  std::cout << jointAngles(nj-1)*(180/M_PI) << std::endl;

  std::cout << "Rotational Matrix of the final Frame:" << std::endl;
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++) {
      double a = eeFrame(i, j);
      if (a < 0.0001 && a > -0.001) {
        a = 0.0;
      }
      std::cout << std::setprecision(4) << a << "\t\t";
    }
    std::cout << std::endl;
  }
#else
  for (int i = 0; i < 3; i++){
    double a = eeFrame(i, 3);
    if (a < 0.0001 && a > -0.001) {
      a = 0.0;
    }
    std::cout << std::setprecision(4) << a;
    if(i < 2)
      std::cout << ",";
  }
  std::cout << std::endl;
#endif

#ifdef DEBUG
  //
  // Perform Inverse Kinematics
  //

  std::cout << "/**********Inverse kinematics**********/" << std::endl;
#endif
  JntArray q_init = JntArray(nj);
  q_init = jointInitAngles;
#ifdef DEBUG
  std::cout << "Init Angles:\n";
  for(int i=0; i<nj-1; i++)
  {
    std::cout << q_init(i)*(180/M_PI) << "\t\t";
  }
  std::cout << q_init(nj-1)*(180/M_PI) << std::endl;
#endif

  ChainFkSolverPos_recursive fk(kdlChain);
  ChainIkSolverVel_pinv vik(kdlChain);
  ChainIkSolverPos_NR ik(kdlChain, fk, vik, 1000, 1e-6); //Maximum 100 iterations, stop at accuracy 1e-6

  // https://wenku.baidu.com/view/ef930264453610661ed9f4f9.html
  Frame desiredFrame = eeFrame;//Frame(Rotation::RPY(M_PI/2.0, 0.0, M_PI/2), Vector(0, 717.6, 129.5));
#ifdef DEBUG
  std::cout << "Desired Position:" << std::endl;
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++) {
      double a = desiredFrame(i, j);
      if (a < 0.0001 && a > -0.001) {
        a = 0.0;
      }
      std::cout << std::setprecision(4) << a << "\t\t";
    }
    std::cout << std::endl;
  }
#endif

  // https://git.physik3.gwdg.de/CNS/skynet_ros/raw/8f1ff07a1159846771a8e47e23a10c14bfbcd9ab/src/orocos_kdl/tests/solvertest.cpp
  JntArray q_out = JntArray(nj);
  ik.CartToJnt(q_init, desiredFrame, q_out);

#ifdef DEBUG
  std::cout << "Output Angles:\n";
  for(int i=0; i<nj-1; i++)
  {
    if (q_out(i) < 0.0001 && q_out(i) > -0.001) {
      q_out(i) = 0.0;
    }
    std::cout << q_out(i)*(180/M_PI) << "\t\t";
  }
  if (q_out(nj-1) < 0.0001 && q_out(nj-1) > -0.001) {
    q_out(nj-1) = 0.0;
  }
  std::cout << q_out(nj-1)*(180/M_PI) << std::endl;
#endif

  std::cout << "/**********Second Inverse kinematics**********/" << std::endl;
  // calc joint angles according real time coordinate us ik
  q_init = q_out;//jointAngles;//jointInitAngles;
#ifdef DEBUG
  std::cout << "Init Second Angles:\n";
  for(int i=0; i<nj-1; i++)
  {
    std::cout << q_init(i)*(180/M_PI) << "\t\t";
  }
  std::cout << q_init(nj-1)*(180/M_PI) << std::endl;
#endif
  // desiredFrame is set by current position(real time coordinate) slightly offset
  desiredFrame = Frame(eeFrame.M,Vector(eeFrame.p[0]+50, eeFrame.p[1]-200, eeFrame.p[2]));
#ifdef DEBUG
  std::cout << "Desired Second Position:" << std::endl;
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++) {
      double a = desiredFrame(i, j);
      if (a < 0.0001 && a > -0.001) {
        a = 0.0;
      }
      std::cout << std::setprecision(4) << a << "\t\t";
    }
    std::cout << std::endl;
  }
#endif
  JntArray q_out_2 = JntArray(nj);
  ik.CartToJnt(q_init, desiredFrame, q_out_2);

#ifdef DEBUG
  std::cout << "Output Second Angles:\n";
  for(int i=0; i<nj-1; i++)
  {
    if (q_out_2(i) < 0.0001 && q_out_2(i) > -0.001) {
      q_out_2(i) = 0.0;
    }
    std::cout << q_out_2(i)*(180/M_PI) << "\t\t";
  }
  if (q_out_2(nj-1) < 0.0001 && q_out_2(nj-1) > -0.001) {
    q_out_2(nj-1) = 0.0;
  }
  std::cout << q_out_2(nj-1)*(180/M_PI) << std::endl;
#endif

  return 0;
}

