#include <iostream>
#include <string>
#include <math.h>
#include <wgs_conversions/wgs_conversions.h>
#include <geodesy/utm.h>


int main(int argc, char** argv)
{
	if (argc < 3)
	{
		std::cout << "Usage: geodetic_conversions lla2enu gps.txt." << std::endl;
		return EXIT_FAILURE;
	}
	std::string mode(argv[1]);
	if (mode == "lla2xyz")
	{
		
	}
	else if (mode == "lla2enu")
	{

	}
	else if (mode == "lla2utm")
	{

	}
	else
	{
		std::cout << "The mode cannot be recognized." << std::endl;
		return EXIT_FAILURE;
	}
}