#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pose.h"
#include "base_module.h"

#define num_test_poses 8
#define num_test2_poses 18


void test_straight_and_rotational()
{
	long movements[num_test_poses][2] = {
		{ 30, 30 },
		{ 30, 30 },
		{ 74, -74 },
		{ 30, 30 },
		{ 75, -75 },
		{ 30, 30 },
		{ 74, -74 },
		{ 30, 30 } };
		  
    init_pose(0,0);

    base_data_type base_data;
    base_data.counterA = 0;
    base_data.counterB = 0;
    
    for (int i = 0; i < num_test_poses; i++)
    {	
		
		base_data.counterA += movements[i][0];
		base_data.counterB += movements[i][1];
		update_pose(&base_data);
		
		pose_type apose;
		get_pose(&apose);
	
		printf("%d. pose x y heading: %.2f %.2f %.2f\n", i, apose.x, apose.y, apose.heading / M_PI * 180.0);
	}
}


void test_circular()
{
	long movements[num_test2_poses][2] = {
		{ 30, 30 },
		{ 30, 30 },
		{ 74, -74 },
		{ 30, 30 },
		{ 37, -37 },
		{ 0, 298 },		
		{ 447, 149 },
		{ -447, -149 },
		{ 0, -298 },
		{ 298, 0 },
		{ 149, 447 },
		{ -149, -447 },
		{ -298, 0 },
		{ -37, 37 },
		{ -30, -30 },
		{ -74, 74 },
		{ -30, -30 },
		{ -30, -30 } };
		  
    set_pose(0, 0, 0);

    base_data_type base_data;
    base_data.counterA = 0;
    base_data.counterB = 0;
    
    for (int i = 0; i < num_test2_poses; i++)
    {	
		
		base_data.counterA += movements[i][0];
		base_data.counterB += movements[i][1];
		update_pose(&base_data);
		
		pose_type apose;
		get_pose(&apose);
	
		printf("----- %d. pose x y heading: %.2f %.2f %.2f\n", i, apose.x, apose.y, apose.heading / M_PI * 180.0);
	}
}


int main()
{
	test_straight_and_rotational();
	test_circular();
}

void get_base_data(base_data_type* buffer)
{
	base_data_type local_data;
	local_data.counterA = local_data.counterB = 0;
    memcpy(buffer, &local_data, sizeof(base_data_type));
}


