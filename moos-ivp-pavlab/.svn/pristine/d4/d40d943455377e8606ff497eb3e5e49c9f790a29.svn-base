/************************************************************/
/*    NAME: Alex Wunderlich                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenGreedyRescue.h                                       */
/*    DATE: Mar 8th 2022                                    */
/************************************************************/

#ifndef GenGreedyRescue_HEADER
#define GenGreedyRescue_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h" // Allows AppCasting
#include "MBUtils.h"
#include <string>
#include <vector>
#include <set>
#include <map>
#include "XYPoint.h"
#include <cmath>
#include <math.h>
#include "XYSegList.h"
#include <list>

using namespace std;

class GenGreedyRescue : public AppCastingMOOSApp
{
public:
  GenGreedyRescue();
  ~GenGreedyRescue();

protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload
  //    string key, m_visit_point;
  void registerVariables();
  bool buildReport();
  bool m_init_x;
  bool m_init_y;
  bool m_added_point;
  bool m_added_path;
  bool m_first_time;
  bool m_regen_bool;
  double m_x_current;
  double m_y_current;
  XYSegList m_vector;
  XYPoint m_centroid; 
  std::vector<bool> m_vector_remain;
  std::set<string> m_found_list;
  std::map<string, XYPoint> m_swimmer_list;
  double m_nav_x;
  double m_nav_y;
  double visit_radius;
  int m_list_length;
  string m_vname;
  bool m_found;
  double m_contact_x; 
  double m_contact_y; 
  double m_contact_hdg;

private: // Configuration variables
private: // State variables
};

#endif
