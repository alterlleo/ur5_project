#ifndef __STRUCTS_H__
#define __STRUCTS_H__

enum Block_name {X1_Y1_Z2, X1_Y2_Z1, X1_Y2_Z2, X1_Y2_Z2_CHAMFER, X1_Y2_Z2_TWINFILLET, X1_Y3_Z2, X1_Y3_Z2_FILLET, X1_Y4_Z1, X1_Y4_Z2, X2_Y2_Z2, X2_Y2_Z2_FILLET}:

enum Face {BOTTOM, LEFT, RIGHT, FRONT, BACK, TOP};

struct Pose_object{
	double x;
	double y;
	double z;
	double theta;
	Face face;
	Block_name name;
	std::string gazeboName;
};

#endif


