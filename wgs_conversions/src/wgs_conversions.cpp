/*
 * A C++ implementation of the popular wgs conversion functions that were originally 
 * written by Andrew Barrows in MATLAB. 
 *
 * Ref: Decker, B. L., World Geodetic System 1984,
 *          Defense Mapping Agency Aerospace Center. 
 *
 * Note: Latitude/longitude are in units of degrees with values ranging between -180 and +360 
 *
 * Dan Pierce
 * 2017-03-13
 */
// #include "wgs_conversions/wgs_conversions.h"  //if in CMakeLists: include_directories(include ${catkin_INCLUDE_DIRS})
#include "wgs_conversions.h"  //if in CMakeLists: include_directories(include/wgs_conversions ${catkin_INCLUDE_DIRS})


//------------------------------------------------------------------------------------------------
// Class Constructor
//------------------------------------------------------------------------------------------------
WgsConversions::WgsConversions(){
  // std::cout << "WgsConversions::WgsConversions" << std::endl;
}

//------------------------------------------------------------------------------------------------
// Class Destructor
//------------------------------------------------------------------------------------------------
WgsConversions::~WgsConversions(){
  // std::cout << "WgsConversions::~WgsConversions" << std::endl;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::enu2lla [Public]  --- convert from (East,North,Up) to (Lat,Long,Alt)
//------------------------------------------------------------------------------------------------
bool WgsConversions::enu2lla(double lla[3], double enu[3], double ref_lla[3]){

	double ref_xyz[3],diff_xyz[3],xyz[3],R[3][3],Rt[3][3];

	// First, calculate the xyz of reflat, reflon, refalt
	if (!lla2xyz(ref_xyz,ref_lla))
		return 0;

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    matrixMultiply(diff_xyz,Rt,enu);

    xyz[0] = diff_xyz[0] + ref_xyz[0];
    xyz[1] = diff_xyz[1] + ref_xyz[1];
    xyz[2] = diff_xyz[2] + ref_xyz[2];

    if(!xyz2lla(lla,xyz))
    	return 0;

    return 1;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::lla2enu [Public]  --- convert from (Lat,Long,Alt) to (East,North,Up)
//------------------------------------------------------------------------------------------------
bool WgsConversions::lla2enu(double enu[3], double lla[3], double ref_lla[3]){
  
	double xyz[3];

	if(!lla2xyz(xyz,lla))
		return 0;

	if(!xyz2enu(enu,xyz,ref_lla))
		return 0;

  return 1;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::xyz2lla [Public]  --- convert from (ECEF X, ECEF Y, ECEF Z) to (Lat,Long,Alt)
//------------------------------------------------------------------------------------------------
bool WgsConversions::xyz2lla(double lla[3], double xyz[3]){

    //This dual-variable iteration seems to be 7 or 8 times faster than
    //a one-variable (in latitude only) iteration.  AKB 7/17/95

    double A_EARTH = 6378137.0;
    double flattening = 1.0 / 298.257223563;
    double NAV_E2 = (2.0 - flattening) * flattening; // also e^2
    double rad2deg = 180.0 / M_PI;

    if ((xyz[0] == 0.0) & (xyz[1] == 0.0)) {
        lla[1] = 0.0;
    } else {
        lla[1] = atan2(xyz[1], xyz[0]) * rad2deg;
    }

    if ((xyz[0] == 0.0) & (xyz[1] == 0.0) & (xyz[2] == 0.0)) {
		std::cout << "WGS xyz at center of earth" << std::endl;
		return 0;
    } else {
        // Make initial lat and alt guesses based on spherical earth.
        double rhosqrd = xyz[0] * xyz[0] + xyz[1] * xyz[1];
        double rho = sqrt(rhosqrd);
        double templat = atan2(xyz[2], rho);
        double tempalt = sqrt(rhosqrd + xyz[2] * xyz[2]) - A_EARTH;
        double rhoerror = 1000.0;
        double zerror = 1000.0;
        
        int iter = 0; // number of iterations

        //      %  Newton's method iteration on templat and tempalt makes
        //      %   the residuals on rho and z progressively smaller.  Loop
        //      %   is implemented as a 'while' instead of a 'do' to simplify
        //      %   porting to MATLAB

        while ((abs(rhoerror) > 1e-6) | (abs(zerror) > 1e-6)) {
            double slat = sin(templat);
            double clat = cos(templat);
            double q = 1.0 - NAV_E2 * slat*slat;
            double r_n = A_EARTH / sqrt(q);
            double drdl = r_n * NAV_E2 * slat * clat / q; // d(r_n)/d(latitutde)

            rhoerror = (r_n + tempalt) * clat - rho;
            zerror = (r_n * (1.0 - NAV_E2) + tempalt) * slat - xyz[2];

            //          %             --                               -- --      --
            //          %             |  drhoerror/dlat  drhoerror/dalt | |  a  b  |
            //                        % Find Jacobian           |                       |=|        |
            //          %             |   dzerror/dlat    dzerror/dalt  | |  c  d  |
            //          %             --                               -- --      --

            double aa = drdl * clat - (r_n + tempalt) * slat;
            double bb = clat;
            double cc = (1.0 - NAV_E2)*(drdl * slat + r_n * clat);
            double dd = slat;

            //Apply correction = inv(Jacobian)*errorvector

            double invdet = 1.0 / (aa * dd - bb * cc);
            templat = templat - invdet * (+dd * rhoerror - bb * zerror);
            tempalt = tempalt - invdet * (-cc * rhoerror + aa * zerror);

            iter++;

            if (iter>20){
            	std::cout << "xyz2lla could not converge" << std::endl;
            	return 0;
            }
        }

        lla[0] = templat*rad2deg;
        lla[2] = tempalt;
    }
    return 1;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::lla2xyz [Public]  --- convert from (Lat,Long,Alt) to (ECEF X, ECEF Y, ECEF Z)
//------------------------------------------------------------------------------------------------
bool WgsConversions::lla2xyz(double xyz[3], double lla[3]){

	if ((lla[0] < -90.0) | (lla[0] > +90.0) | (lla[1] < -180.0) | (lla[1] > +360.0)){
		std::cout << "WGS lat or WGS lon out of range" << std::endl;
		return 0;
	}

	double A_EARTH = 6378137.0;
	double flattening = 1.0/298.257223563;
	double NAV_E2 = (2.0-flattening)*flattening; // also e^2
	double deg2rad = M_PI/180.0;

	double slat = sin(lla[0]*deg2rad);
	double clat = cos(lla[0]*deg2rad);
	double r_n = A_EARTH/sqrt(1.0 - NAV_E2*slat*slat);
	xyz[0] = (r_n + lla[2])*clat*cos(lla[1]*deg2rad);  
    xyz[1] = (r_n + lla[2])*clat*sin(lla[1]*deg2rad);  
    xyz[2] = (r_n*(1.0 - NAV_E2) + lla[2])*slat;

    return 1;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::xyz2enu [Public]  --- convert from (ECEF X, ECEF Y, ECEF Z) to (East,North,Up)
//------------------------------------------------------------------------------------------------
bool WgsConversions::xyz2enu(double enu[3], double xyz[3], double ref_lla[3]){
  
  	double ref_xyz[3],diff_xyz[3],R[3][3];

	// First, calculate the xyz of reflat, reflon, refalt
    if (!lla2xyz(ref_xyz,ref_lla))
		return 0;
	
    //Difference xyz from reference point
    diff_xyz[0] = xyz[0] - ref_xyz[0];
    diff_xyz[1] = xyz[1] - ref_xyz[1];
    diff_xyz[2] = xyz[2] - ref_xyz[2];

    rot3d(R, ref_lla[0], ref_lla[1]);

    matrixMultiply(enu,R,diff_xyz);

    return 1;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::enu2xyz [Public]  --- convert from (East,North,Up) to (ECEF X, ECEF Y, ECEF Z)
//------------------------------------------------------------------------------------------------
bool WgsConversions::enu2xyz(double xyz[3], double enu[3], double ref_lla[3]){

  double lla[3];

  // first enu2lla
  if(!enu2lla(lla,enu, ref_lla))
    return 0;

  // then lla2xyz
  if(!lla2xyz(xyz,lla))
    return 0;

  return 1;
}


//--------------------------------------------------------------------------------------------------------------
// WgsConversions::xyz2enu_vel [Public]  --- convert velocities from (ECEF X, ECEF Y, ECEF Z) to (East,North,Up)
//--------------------------------------------------------------------------------------------------------------
void WgsConversions::xyz2enu_vel(double enu_vel[3], double xyz_vel[3], double ref_lla[3]){
  
    double R[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    matrixMultiply(enu_vel,R,xyz_vel);

}

//--------------------------------------------------------------------------------------------------------------
// WgsConversions::enu2xyz_vel [Public]  --- convert velocities from (East,North,Up) to (ECEF X, ECEF Y, ECEF Z)
//--------------------------------------------------------------------------------------------------------------
void WgsConversions::enu2xyz_vel(double xyz_vel[3], double enu_vel[3], double ref_lla[3]){
    
    double R[3][3],Rt[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    matrixMultiply(xyz_vel,Rt,enu_vel);

}

//--------------------------------------------------------------------------------------------------------------------------------
// WgsConversions::xyz2enu_cov [Public]  --- convert position/velocity covariance from (ECEF X, ECEF Y, ECEF Z) to (East,North,Up)
//--------------------------------------------------------------------------------------------------------------------------------
void WgsConversions::xyz2enu_cov(double enu_cov[3][3], double xyz_cov[3][3], double ref_lla[3]){
  
    double R[3][3],Rt[3][3],Tmp[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    matrixMultiply(Tmp,xyz_cov,Rt); // Tmp = xyz_cov*R'

    matrixMultiply(enu_cov,R,Tmp); // enu_cov = R*xyz_cov*R'

}

//--------------------------------------------------------------------------------------------------------------------------------
// WgsConversions::enu2xyz_cov [Public]  --- convert position/velocity covariance from (East,North,Up) to (ECEF X, ECEF Y, ECEF Z)
//--------------------------------------------------------------------------------------------------------------------------------
void WgsConversions::enu2xyz_cov(double xyz_cov[3][3], double enu_cov[3][3], double ref_lla[3]){
    
    double R[3][3],Rt[3][3],Tmp[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    matrixMultiply(Tmp,enu_cov,R); // Tmp = enu_cov*R

    matrixMultiply(xyz_cov,Rt,Tmp); // xyz_cov = R'*enu_cov*R

}

//--------------------------------------------------------------------------------------------
// WgsConversions::enu2xyz [Private]  --- return the 3D rotation matrix to/from ECEF/ENU frame
//--------------------------------------------------------------------------------------------
void WgsConversions::rot3d(double R[3][3], double reflat, double reflon){

    double R1[3][3],R2[3][3];

    rot(R1, 90 + reflon, 3);
    rot(R2, 90 - reflat, 1);

    matrixMultiply(R, R2, R1);
}

//------------------------------------------------------------------------------------------------
// WgsConversions::matrixMultiply [Private]  --- Multiply 3x3 matrix times another 3x3 matrix C=AB
//------------------------------------------------------------------------------------------------
void WgsConversions::matrixMultiply(double C[3][3], double A[3][3], double B[3][3]){

    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
    C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1];
    C[0][2] = A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
    C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1];
    C[1][2] = A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2];
    C[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0];
    C[2][1] = A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1];
    C[2][2] = A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2];

}

//------------------------------------------------------------------------------------------------
// WgsConversions::matrixMultiply [Private]  --- Multiply 3x3 matrix times a 3x1 vector c=Ab
//------------------------------------------------------------------------------------------------
void WgsConversions::matrixMultiply(double c[3], double A[3][3], double b[3]){

    c[0] = A[0][0] * b[0] + A[0][1] * b[1] + A[0][2] * b[2];
    c[1] = A[1][0] * b[0] + A[1][1] * b[1] + A[1][2] * b[2];
    c[2] = A[2][0] * b[0] + A[2][1] * b[1] + A[2][2] * b[2];

}

//------------------------------------------------------------------------------------------------
// WgsConversions::transposeMatrix [Private]  --- transpose a 3x3 matrix At = A'
//------------------------------------------------------------------------------------------------
void WgsConversions::transposeMatrix(double At[3][3], double A[3][3]){

    At[0][0] = A[0][0];
    At[0][1] = A[1][0];
    At[0][2] = A[2][0];
    At[1][0] = A[0][1];
    At[1][1] = A[1][1];
    At[1][2] = A[2][1];
    At[2][0] = A[0][2];
    At[2][1] = A[1][2];
    At[2][2] = A[2][2];

}

//------------------------------------------------------------------------------------------------
// WgsConversions::rot [Private]  --- rotation matrix
//------------------------------------------------------------------------------------------------
void WgsConversions::rot(double R[3][3], double angle, int axis) {

    double cang = cos(angle * M_PI / 180);
    double sang = sin(angle * M_PI / 180);

    if (axis == 1) {
        R[0][0] = 1;
        R[0][1] = 0;
        R[0][2] = 0;
        R[1][0] = 0;
        R[2][0] = 0;
        R[1][1] = cang;
        R[2][2] = cang;
        R[1][2] = sang;
        R[2][1] = -sang;
    } else if (axis == 2) {
        R[0][1] = 0;
        R[1][0] = 0;
        R[1][1] = 1;
        R[1][2] = 0;
        R[2][1] = 0;
        R[0][0] = cang;
        R[2][2] = cang;
        R[0][2] = -sang;
        R[2][0] = sang;
    } else if (axis == 3) {
        R[2][0] = 0;
        R[2][1] = 0;
        R[2][2] = 1;
        R[0][2] = 0;
        R[1][2] = 0;
        R[0][0] = cang;
        R[1][1] = cang;
        R[1][0] = -sang;
        R[0][1] = sang;
    }
}
