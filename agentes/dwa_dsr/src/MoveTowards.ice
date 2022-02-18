//******************************************************************
// 
//  Generated by RoboCompDSL
//  
//  File name: MoveTowards.ice
//  Source: MoveTowards.idsl
//
//******************************************************************
#ifndef ROBOCOMPMOVETOWARDS_ICE
#define ROBOCOMPMOVETOWARDS_ICE
module RoboCompMoveTowards
{
	struct Command
	{
		float adv;
		float rot;
	};
	interface MoveTowards
	{
		Command move (float x, float y, float alpha);
	};
};

#endif