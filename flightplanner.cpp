#include "flightplanner.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

// Approximate radius of the Earth in metres (WGS-84 mean)
static constexpr double EARTH_RADIUS_M = 6371000.0;

// Standard gravity in m/s²
static constexpr double GRAVITY_M_S2 = 9.8;

// Tolerances used when deciding whether two turn-circle centres coincide.
// The original MATLAB code used asymmetric values (.05 / .01); preserved here.
static constexpr double TURN_CENTER_X_TOLERANCE = 0.05;
static constexpr double TURN_CENTER_Y_TOLERANCE = 0.01;

// ---------------------------------------------------------------------------
// newPoint – spherical forward geodesy (mirrors MATLAB NewPoint)
// Returns the lat/lon reached by travelling distance d (metres) from
// (lat, lon) at compass bearing.
// ---------------------------------------------------------------------------
static std::pair<double,double> newPoint(double lat, double lon,
                                         double bearing, double d)
{
    constexpr double R = EARTH_RADIUS_M;
    double newlat = asind(sind(lat) * std::cos(d / R)
                        + cosd(lat) * std::sin(d / R) * cosd(bearing));
    double A = sind(bearing) * std::sin(d / R) * cosd(lat);
    double B = std::cos(d / R) - sind(lat) * sind(newlat);
    double newlon = lon + atan2d(A, B);
    return {newlat, newlon};
}

// ---------------------------------------------------------------------------
// flightDistance – haversine great-circle distance in metres
// (mirrors MATLAB FlightDistance)
// ---------------------------------------------------------------------------
static double flightDistance(double lat1, double lon1,
                              double lat2, double lon2)
{
    return 2.0 * EARTH_RADIUS_M
         * std::asin(std::sqrt(
               std::pow(sind((lat2 - lat1) / 2.0), 2)
             + cosd(lat1) * cosd(lat2)
               * std::pow(sind((lon2 - lon1) / 2.0), 2)));
}

// ---------------------------------------------------------------------------
// genFlightLines – fills lats/longs with 2·numLines waypoints
// (mirrors MATLAB GenFlightLines)
// ---------------------------------------------------------------------------
static void genFlightLines(const FlightPlanParams& p,
                            std::vector<double>& lats,
                            std::vector<double>& longs)
{
    int    n       = p.numLines;
    double bearing = p.lineHeading;
    double length  = p.lineLength * p.rangeConversion; // metres
    double space   = p.lineSpace  * p.rangeConversion; // metres

    lats.resize(2 * n);
    longs.resize(2 * n);

    // First waypoint: science target
    lats[0]  = p.sciTarLat;
    longs[0] = p.sciTarLong;

    // Second waypoint: end of first flight line
    auto [la, lo] = newPoint(lats[0], longs[0], bearing, length);
    lats[1] = la; longs[1] = lo;

    // Generate subsequent lines (MATLAB loop: for i = 2:numLines)
    // MATLAB 1-indexed lats(2*i-1) → C++ 0-indexed lats[2*i-2]
    for (int i = 2; i <= n; ++i) {
        // Step sideways by spacing (always 90° to the right of the bearing)
        auto [la1, lo1] = newPoint(lats[2*i-3], longs[2*i-3],
                                   bearing + 90.0, space);
        lats[2*i-2]  = la1;
        longs[2*i-2] = lo1;

        // Fly next line: even-numbered MATLAB lines go back (bearing+180)
        double nextBearing = (i % 2 == 0) ? (bearing + 180.0) : bearing;
        auto [la2, lo2] = newPoint(lats[2*i-2], longs[2*i-2],
                                   nextBearing, length);
        lats[2*i-1]  = la2;
        longs[2*i-1] = lo2;
    }
}

// ---------------------------------------------------------------------------
// mercatorConvert – converts lat/lon arrays to planar (x,y) in metres
// (mirrors MATLAB MercConv)
// ---------------------------------------------------------------------------
static void mercatorConvert(const std::vector<double>& lats,
                             const std::vector<double>& longs,
                             std::vector<double>& x,
                             std::vector<double>& y)
{
    constexpr double R = EARTH_RADIUS_M;
    int n = static_cast<int>(lats.size());
    x.resize(n); y.resize(n);

    double cosLat1 = std::cos(deg2rad(lats[0]));
    double minLon  = *std::min_element(longs.begin(), longs.end());

    for (int i = 0; i < n; ++i) {
        x[i] = R * cosLat1 * (longs[i] - minLon) * M_PI / 180.0;
        y[i] = R * cosLat1
             * std::log(std::tan(M_PI / 4.0 + deg2rad(lats[i]) / 2.0));
    }
    // Shift so that the first waypoint is at the origin
    double y0 = y[0];
    for (auto& v : y) v -= y0;
}

// ---------------------------------------------------------------------------
// scaleCoords – rescales Mercator (metres) to user units
// (mirrors MATLAB Scale)
// ---------------------------------------------------------------------------
static void scaleCoords(std::vector<double>& x, std::vector<double>& y,
                         const std::vector<double>& lats,
                         const std::vector<double>& longs,
                         double rangeConversion)
{
    double plotDist = std::sqrt(std::pow(x[1] - x[0], 2)
                              + std::pow(y[1] - y[0], 2));
    double realDist = flightDistance(lats[0], longs[0], lats[1], longs[1]);
    double sf = plotDist / realDist; // ~1 for small areas

    for (auto& v : x) v /= sf * rangeConversion;
    for (auto& v : y) v /= sf * rangeConversion;
}

// ---------------------------------------------------------------------------
// Helper: angle from centre (cx,cy) to point (px,py) in [0,360) degrees,
// measured counter-clockwise from the +x axis.
// Replaces the verbose quadrant-checking code in the original MATLAB turn().
// ---------------------------------------------------------------------------
static double angleFromCenter(double px, double py, double cx, double cy)
{
    double a = atan2d(py - cy, px - cx);
    if (a < 0.0) a += 360.0;
    return a;
}

// ---------------------------------------------------------------------------
// computeTurn – calculates the arc arc-length of one inter-line turn and
// appends the arc(s) to result.arcs.
// (mirrors MATLAB turn() + DrawArc())
//
// Parameters are in the scaled user-unit coordinate system.
// b1, b2 are compass bearings (degrees).
// r      is the turn radius in user units.
// dist   is the straight-line distance between the two points (pre-computed).
// ---------------------------------------------------------------------------
static double computeTurn(double x1, double y1, double b1,
                           double x2, double y2, double b2,
                           double r, double dist,
                           std::vector<ArcSegment>& arcs)
{
    b1 = std::fmod(b1, 360.0); if (b1 < 0) b1 += 360.0;
    b2 = std::fmod(b2, 360.0); if (b2 < 0) b2 += 360.0;

    // Right-hand turn centres for each waypoint
    double r1x = x1 + r * cosd(b1);
    double r1y = y1 - r * sind(b1);
    double r2x = x2 + r * cosd(b2);
    double r2y = y2 - r * sind(b2);

    // Left-hand turn centres for each waypoint
    double l1x = x1 - r * cosd(b1);
    double l1y = y1 + r * sind(b1);
    double l2x = x2 - r * cosd(b2);
    double l2y = y2 + r * sind(b2);

    if (dist < 1e-12) return 0.0; // degenerate: coincident points

    // ---- Left-left turn (both points share the same left-turn circle) ----
    if (std::abs(l2x - l1x) < TURN_CENTER_X_TOLERANCE * dist
     && std::abs(l2y - l1y) < TURN_CENTER_Y_TOLERANCE * dist)
    {
        double a1 = angleFromCenter(x1, y1, l1x, l1y);
        double a2 = angleFromCenter(x2, y2, l2x, l2y);
        if (a1 > a2) a1 -= 360.0; // ensure CCW traversal
        arcs.push_back({l1x, l1y, r, a1, a2});
        return (a2 - a1) * r * M_PI / 180.0;
    }

    // ---- Right-right turn (both points share the same right-turn circle) ---
    if (std::abs(r2x - r1x) < TURN_CENTER_X_TOLERANCE * dist
     && std::abs(r2y - r1y) < TURN_CENTER_Y_TOLERANCE * dist)
    {
        double a1 = angleFromCenter(x1, y1, r1x, r1y);
        double a2 = angleFromCenter(x2, y2, r2x, r2y);
        if (a2 > a1) a2 -= 360.0; // ensure CW traversal
        arcs.push_back({r1x, r1y, r, a1, a2});
        return (a1 - a2) * r * M_PI / 180.0;
    }

    // ---- S-turn case (unfinished in original; distance returned as
    //      straight-line between points) ---------------------------------
    if (dist > r * 2.0) {
        // Original MATLAB only converted the bearings to Cartesian angles
        // but did not implement the full S-turn geometry.
        return dist;
    }

    return dist;
}

// ---------------------------------------------------------------------------
// generateFlightPlan – top-level function called from the GUI
// ---------------------------------------------------------------------------
FlightPlanResult generateFlightPlan(const FlightPlanParams& p)
{
    FlightPlanResult res;

    // ---- Build waypoints ----
    genFlightLines(p, res.lats, res.longs);

    // ---- Project to Cartesian, then scale to user units ----
    mercatorConvert(res.lats, res.longs, res.xCoords, res.yCoords);
    scaleCoords(res.xCoords, res.yCoords,
                res.lats, res.longs, p.rangeConversion);

    // ---- Turn radius in user units ----
    // Standard formula: r = v² / (g·tan(phi)).
    // Note: the original MATLAB had "Cruise * 2" which appears to be a typo
    // for "Cruise^2"; the physically correct formula is used here.
    res.turnRadius = (p.cruise * p.cruise)
                   / (GRAVITY_M_S2 * tand(p.bankAngle))
                   / p.rangeConversion;

    // ---- Accumulate flight-line distances and turn distances ----
    res.totalDistance = p.numLines * p.lineLength; // user units (straight lines)

    if (res.turnRadius < p.lineSpace * 3.0) {
        // Turn j connects end of line j to start of line j+1.
        // In 0-indexed coords: end of line j is at index 2j-1,
        //                      start of line j+1 is at index 2j.
        for (int j = 1; j < p.numLines; ++j) {
            int ci1 = 2 * j - 1; // end of line j
            int ci2 = 2 * j;     // start of line j+1

            // Bearing at end of line j: odd-j lines run at lineHeading,
            // even-j lines run at lineHeading+180.
            double b1 = (j % 2 == 1) ? p.lineHeading
                                      : (p.lineHeading + 180.0);
            double b2 = (j % 2 == 1) ? (p.lineHeading + 180.0)
                                      : p.lineHeading;

            double dx = res.xCoords[ci2] - res.xCoords[ci1];
            double dy = res.yCoords[ci2] - res.yCoords[ci1];
            double dist = std::sqrt(dx * dx + dy * dy);

            res.totalDistance += computeTurn(
                res.xCoords[ci1], res.yCoords[ci1], b1,
                res.xCoords[ci2], res.yCoords[ci2], b2,
                res.turnRadius, dist, res.arcs);
        }
    }

    // ---- Flight time in hours (distance / cruise speed) ----
    // totalDistance is in user units; convert to metres for time calculation.
    res.flightTime = res.totalDistance * p.rangeConversion
                   / p.cruise / 3600.0;

    // ---- Optional file output ----
    if (!p.fileName.empty()) {
        std::ofstream f(p.fileName + ".txt");
        if (f.is_open()) {
            for (int i = 0; i < static_cast<int>(res.lats.size()); ++i) {
                f << std::fixed;
                f.precision(8);
                f << res.lats[i] << " " << res.longs[i] << " \n";
            }
        }
    }

    res.valid = true;
    return res;
}
