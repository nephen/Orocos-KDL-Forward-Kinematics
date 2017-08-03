## Forward Kinematics using Orocos KDL

Check out the [blog post](https://medium.com/@sarvagya.vaish/forward-kinematics-using-orocos-kdl-da7035f9c8e)

## Build and Run

```
mkdir build
cd build
cmake ..
make
./SimpleRobotFK 90 0 0 0 90
```

Output:
```
Argument position of joint 1 is 90.000000
Argument position of joint 2 is 0.000000
Argument position of joint 3 is 0.000000
Argument position of joint 4 is 0.000000
Argument position of joint 5 is 90.000000
/**********Forward kinematics**********/
Rotational Matrix of the init Frame:
1		0		0		0		
0		1		0		847.1		
0		0		1		0		
0		0		0		1		
Desired Angles:
90		0		0		0		90
Rotational Matrix of the final Frame:
0		0		1		0		
1		0		0		717.6		
0		1		0		129.5		
0		0		0		1		
/**********Inverse kinematics**********/
Init Angles:
0		0		0		0		0
Desired Position:
0		0		1		0		
1		0		0		717.6		
0		1		0		129.5		
0		0		0		1		
Output Angles:
42.59		0		0		47.41		90
```

