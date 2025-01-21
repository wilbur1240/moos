/************************************************************/
/*    NAME: Mikala Molina                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SendPoints.h                                          */
/*    DATE: June 1 2023                             */
/************************************************************/

#ifndef SendPoints_HEADER
#define SendPoints_HEADER
#include "XYFieldGenerator.h"
#include "XYPolygon.h"
#include "XYFormatUtilsPoly.h"
#include "VisitPoint.h"
#include "NodeReport.h"

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
using namespace std;

class SendPoints : public AppCastingMOOSApp
{
 public:
   SendPoints();
   ~SendPoints();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables
 string m_region; 
 int m_initial_points;
 int m_capture_radius;
 double m_min_dist;
 vector<string> m_vehicle_names;
 int m_number_of_vehicles;

 private: // State variables
 XYPolygon m_region_poly;
 XYFieldGenerator m_region_fld;
 string m_requesting_vehicle;
 bool m_need_to_send;
 map<string, bool> m_vehicle_awake;
 map<string, int> m_id_counter;
 map<string, vector<VisitPoint> > m_visit_points;
 map<string, vector<XYPoint> > m_nav_xy_points;
 map<string, bool> m_needs_initial_points;
 bool m_all_initial_points_sent;
 double m_found_x; 
 double m_found_y;
 double m_found_id;
 XYPoint m_found_point;
 
 
};

#endif 
