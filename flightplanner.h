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

// ---------------------------------------------------------------------------
// Preset tables (shared between the planner and the GUI)
// ---------------------------------------------------------------------------

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

// Science target presets (from MATLAB bgScienceTargets_SelectionChangeFcn)
inline const ScienceTarget kScienceTargets[] = {
    { "Chamberlin Glacier",    76.743607,   -68.615041 },
    { "Camp Century",          77.166696,   -61.133369 },
    { "Jakobshavn",            69.215840,   -49.798696 },
    { "Russell Glacier",       67.101912,   -50.225496 },
    { "Columbia Glacier (AK)", 61.170380,  -147.026099 },
    { "Nuuk Glacier",          65.212518,   -50.662002 },
};

// Airport presets (from MATLAB bgAirports_SelectionChangeFcn)
inline const Airport kAirports[] = {
    { "Thule AB",         77.46666667,  -69.23055556 },
    { "Ilulissat",        69.21666667,  -51.10000000 },
    { "Kangerlussuaq",    67.00861111,  -50.68916667 },
    { "Nuuk",             64.17500000,  -51.73888889 },
    { "Valdez (AK)",      61.85000000, -146.34833333 },
    { "Barrow (AK)",      71.29055556, -156.78861111 },
};

// Aircraft presets (from MATLAB bgPlanes_SelectionChangeFcn)
inline const AircraftPreset kAircraftPresets[] = {
    { "Sierra",      30.8667,   1018600.0 },
    { "P-3",        169.7670,  5556000.0  },
    { "Twin Otter",  56.5889,  1574200.0  },
};

// Unit presets (from MATLAB bgUnits_SelectionChangeFcn)
inline const UnitConversion kUnitConversions[] = {
    { "Metres / m/s",  1.0,       1.0     },
    { "Miles / mph",   0.44704,   1609.34 },
    { "Knots / nm",    0.514444,  1852.0  },
    { "km / km/h",     0.277778,  1000.0  },
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
