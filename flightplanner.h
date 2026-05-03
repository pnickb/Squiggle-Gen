#pragma once

#include <cmath>
#include <vector>
#include <string>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------
// Degree-based trig helpers (mirrors MATLAB's sind/cosd/tand/asind/atan2d)
// ---------------------------------------------------------------------------
inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }
inline double sind(double d)    { return std::sin(deg2rad(d)); }
inline double cosd(double d)    { return std::cos(deg2rad(d)); }
inline double tand(double d)    { return std::tan(deg2rad(d)); }
inline double asind(double x)   { return rad2deg(std::asin(x)); }
inline double atan2d(double y, double x) { return rad2deg(std::atan2(y, x)); }

// ---------------------------------------------------------------------------
// Preset tables
// ---------------------------------------------------------------------------

struct AircraftPreset {
    const char* name;
    double cruise; // m/s
    double range;  // m
};

struct ScienceTarget {
    const char* name;
    double lat;
    double lon;
};

struct Airport {
    const char* name;
    double lat;
    double lon;
};

struct UnitConversion {
    const char* name;
    double speedConv; // user speed unit -> m/s
    double rangeConv; // user range unit -> m
};

// ---------------------------------------------------------------------------
// Flight plan input parameters
// ---------------------------------------------------------------------------
struct FlightPlanParams {
    double sciTarLat      = 0.0;
    double sciTarLong     = 0.0;
    double apLat          = 0.0;  // airport (not yet used in path calc but stored)
    double apLong         = 0.0;
    int    numLines       = 1;
    double lineHeading    = 0.0;  // compass degrees
    double lineLength     = 0.0;  // user units
    double lineSpace      = 0.0;  // user units
    double cruise         = 0.0;  // m/s  (always stored in SI)
    double range          = 0.0;  // m    (always stored in SI)
    double speedConversion = 1.0; // user_speed -> m/s
    double rangeConversion = 1.0; // user_range -> m
    double bankAngle      = 15.0; // degrees
    std::string fileName;         // empty means no file output
};

// ---------------------------------------------------------------------------
// A single arc segment: centre (cx,cy), radius r, angles a1→a2 in standard
// counter-clockwise degrees from the +x axis.
// ---------------------------------------------------------------------------
struct ArcSegment {
    double cx, cy, r;
    double a1, a2;
};

// ---------------------------------------------------------------------------
// Results returned by generateFlightPlan()
// ---------------------------------------------------------------------------
struct FlightPlanResult {
    // Lat/lon of every waypoint (2·numLines values)
    std::vector<double> lats;
    std::vector<double> longs;

    // Mercator-projected, scaled coordinates in user units
    std::vector<double> xCoords;
    std::vector<double> yCoords;

    // Turn arcs (empty when TurnRadius >= LineSpace*3)
    std::vector<ArcSegment> arcs;

    double totalDistance = 0.0; // user units
    double turnRadius    = 0.0; // user units
    double flightTime    = 0.0; // hours

    bool        valid = false;
    std::string error;
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
FlightPlanResult generateFlightPlan(const FlightPlanParams& p);
