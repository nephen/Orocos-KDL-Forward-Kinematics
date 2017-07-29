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

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

using namespace KDL;

int main(int argc, char *argv[])
{
	//
	// Create a KDL kinematic Chain.
	//
	// A Chain is made up of Segments. Each Segment consists of a Joint and a Frame.
	// The Joint indicates how the Frame moves - rotation or translation about / along an axis.
	//

	Chain kdlChain = Chain();

	Joint joint1(Joint::RotZ);
	Frame frame1 = Frame(Vector(0.2, 0.3, 0.0));
	kdlChain.addSegment(Segment(joint1, frame1));

	Joint joint2(Joint::RotZ);
	Frame frame2 = Frame(Vector(0.4, 0.0, 0.0));
	kdlChain.addSegment(Segment(joint2, frame2));

	Joint joint3(Joint::RotZ);
	Frame frame3 = Frame(Vector(0.1, 0.1, 0.0));
	kdlChain.addSegment(Segment(joint3, frame3));

	//
	// Joint Angles
	//

	JntArray jointAngles = JntArray(3);
	jointAngles(0) = M_PI / 6.;       // Joint 1
	jointAngles(1) = M_PI / 6.;        // Joint 2
	jointAngles(2) = -M_PI / 2.;             // Joint 3

	//
	// Perform Forward Kinematics
	//

  std::cout << "/**********Forward kinematics**********/" << std::endl;
	ChainFkSolverPos_recursive FKSolver = ChainFkSolverPos_recursive(kdlChain);
	Frame eeFrame;
	FKSolver.JntToCart(jointAngles, eeFrame);
  std::cout << "Desired Angles:\n" << jointAngles(0)*(180/M_PI) << "\t\t" << jointAngles(1)*(180/M_PI)
    << "\t\t" << jointAngles(2)*(180/M_PI) << std::endl;

	// Print the frame
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

	//
	// Perform Inverse Kinematics
	//

  std::cout << "/**********Inverse kinematics**********/" << std::endl;
  JntArray q_init = JntArray(3);
  q_init = jointAngles;

	ChainFkSolverPos_recursive fk(kdlChain);
  ChainIkSolverVel_pinv vik(kdlChain);
  ChainIkSolverPos_NR ik(kdlChain, fk, vik);

  Frame desiredFrame = Frame(Vector(0.4, 0.4, 0));
  std::cout << "Desired Position:\n" << desiredFrame.p(0) << "\t\t" << desiredFrame.p(1)
    << "\t\t" << desiredFrame.p(2) << std::endl;

  JntArray q_out = JntArray(3);
  ik.CartToJnt(q_init, desiredFrame, q_out);

  std::cout << "Output Angles:\n" << q_out(0)*(180/M_PI) << "\t\t" << q_out(1)*(180/M_PI)
    << "\t\t" << q_out(2)*(180/M_PI) << std::endl;

	return 0;
}

