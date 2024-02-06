#ifndef __STRUCTS_H__
#define __STRUCTS_H__

enum Block_name {X1_Y1_Z2, X1_Y2_Z1, X1_Y2_Z2, X1_Y2_Z2_CHAMFER, X1_Y2_Z2_TWINFILLET, X1_Y3_Z2, X1_Y3_Z2_FILLET, X1_Y4_Z1, X1_Y4_Z2, X2_Y2_Z2, X2_Y2_Z2_FILLET};

enum Face {BOTTOM, LEFT, RIGHT, FRONT, BACK, TOP};

struct ObjectPose{
	double x;
	double y;
	double z;
	double theta;
	Face face;
	Block_name name;
	std::string gazebo_name;

	public:
		ObjectPose(){}
		ObjectPose(double x, double y, double z, double theta, Face face, Block_name name, std::string gazebo_name){
			this -> x = x;
			this -> y = y;
			this -> z = z;
			this -> theta = theta;
			this -> face = face;
			this -> name = name;
			this -> gazebo_name = gazebo_name;
		}
};

#endif


