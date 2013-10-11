#include "core.h"

/**
 *	main()
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "testing_tf");
	std::string node_name = ros::this_node::getName();
	ros::NodeHandle node(node_name);

	tf::TransformBroadcaster br;
	tf::Transform Ta, Tb, aTb;

	Ta.setOrigin(	tf::Vector3(1.0, 1.0, 1.0) 			);
	Ta.setBasis( 	tf::Matrix3x3(0,0,1,-1,0,0,0,-1,0) 	);

	aTb.setOrigin( 	tf::Vector3(2.0, 0.0, 0.0) 			);
	aTb.setBasis( 	tf::Matrix3x3(1,0,0,0,1,0,0,0,1) 	);

	while (ros::ok())
	{
		br.sendTransform(
				tf::StampedTransform(Ta, ros::Time::now(), "world", "frameA"));
		br.sendTransform(
				tf::StampedTransform(aTb, ros::Time::now(), "frameA", "frameB"));
		ros::spinOnce();
	}

	return 0;
}
