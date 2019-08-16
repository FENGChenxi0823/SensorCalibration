#ifndef LOCALIAZTION_LOCALIAZTION_H_
#define LOCALIAZTION_LOCALIAZTION_H_

#include <tf/tf.h>
#include "proj_api.h"

std::vector<std::string> split(const std::string &str, const std::string &delims)
{
  // str.erase(str.begin()); str.erase(str.end()-1);
  std::vector<std::string> ret;
  if (delims.empty()) return ret;
  size_t start = 0, index = str.find_first_of(delims, 0);
  while (index != str.npos){
    if (start != index)
      ret.push_back(str.substr(start, index - start));
    start = index + 1;
    index = str.find_first_of(delims, start);
  }
  if (!str.substr(start).empty())
    ret.push_back(str.substr(start));
  return ret;
}

std::string UTMZoneDesignator(const double &lat, const double &lon)
{
  char utm_letter;
  if     ((84 >= lat) && (lat >= 72))  utm_letter = 'X';
  else if ((72 > lat) && (lat >= 64))  utm_letter = 'W';
  else if ((64 > lat) && (lat >= 56))  utm_letter = 'V';
  else if ((56 > lat) && (lat >= 48))  utm_letter = 'U';
  else if ((48 > lat) && (lat >= 40))  utm_letter = 'T';
  else if ((40 > lat) && (lat >= 32))  utm_letter = 'S';
  else if ((32 > lat) && (lat >= 24))  utm_letter = 'R';
  else if ((24 > lat) && (lat >= 16))  utm_letter = 'Q';
  else if ((16 > lat) && (lat >= 8))   utm_letter = 'P';
  else if (( 8 > lat) && (lat >= 0))   utm_letter = 'N';
  else if (( 0 > lat) && (lat >= -8))  utm_letter = 'M';
  else if ((-8 > lat) && (lat >= -16)) utm_letter = 'L';
  else if ((-16 > lat) && (lat >= -24)) utm_letter = 'K';
  else if ((-24 > lat) && (lat >= -32)) utm_letter = 'J';
  else if ((-32 > lat) && (lat >= -40)) utm_letter = 'H';
  else if ((-40 > lat) && (lat >= -48)) utm_letter = 'G';
  else if ((-48 > lat) && (lat >= -56)) utm_letter = 'F';
  else if ((-56 > lat) && (lat >= -64)) utm_letter = 'E';
  else if ((-64 > lat) && (lat >= -72)) utm_letter = 'D';
  else if ((-72 > lat) && (lat >= -80)) utm_letter = 'C';
  // 'Z' is an error flag, the Latitude is outside the UTM limits
  else
    utm_letter = 'Z';

  // Make sure the longitude is between -180.00 .. 179.9
  double lon_tmp =
        (lon + 180) - static_cast<int>((lon + 180) / 360) * 360 - 180;

  int zone_number = static_cast<int>((lon_tmp + 180) / 6) + 1;

  if (lat >= 56.0 && lat < 64.0 && lon_tmp >= 3.0 && lon_tmp < 12.0)
    zone_number = 32;

  // Special zones for Svalbard
  if (lat >= 72.0 && lat < 84.0) {
    if (lon_tmp >= 0.0 && lon_tmp < 9.0)
      zone_number = 31;
    else if (lon_tmp >= 9.0 && lon_tmp < 21.0)
      zone_number = 33;
    else if (lon_tmp >= 21.0 && lon_tmp < 33.0)
      zone_number = 35;
    else if (lon_tmp >= 33.0 && lon_tmp < 42.0)
      zone_number = 37;
  }

  std::string utm_zone = std::to_string(zone_number) + utm_letter;
  return utm_zone;
}

// Converts from degrees to radians.
double DegToRad(double deg) { return M_PI * deg / 180.; }
// Converts form radians to degrees.
double RadToDeg(double rad) { return 180. * rad / M_PI; }

std::tuple<bool, tf::Vector3> LatLongAltToUtm(
                      const double latitude, const double longitude,
                      const double altitude, const int zone)
{
  projPJ pj_merc, pj_latlong;
  static const std::string param_str = "+proj=utm +lon_0=" +
        std::to_string(static_cast<int>(zone - 31) * 6 + 3)
        + " +x_0=500000 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
  if (!(pj_merc = pj_init_plus(param_str.c_str())))
      return std::make_tuple(false, tf::Vector3(0, 0, 0));
  if (!(pj_latlong = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs")))
      return std::make_tuple(false, tf::Vector3(0, 0, 0));

  double east = DegToRad(longitude);
  double north = DegToRad(latitude);

  pj_transform(pj_latlong, pj_merc, 1, 1, &east, &north, NULL);

  return std::make_tuple(true, tf::Vector3(east, north, altitude));
}



#endif
