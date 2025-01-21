/************************************************************/
/*    NAME: Mikala Molina                                    */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SendPoints.cpp                     */
/*    DATE: June 1 2023                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "SendPoints.h"
#include "XYFormatUtilsPoint.h"
#include "GeomUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

SendPoints::SendPoints()
{
  m_need_to_send = false;
  m_min_dist =10; 
  m_all_initial_points_sent = false;
}

//---------------------------------------------------------
// Destructor

SendPoints::~SendPoints()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool SendPoints::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval  = msg.GetString();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "SWIMMER_REQUEST"){
        m_requesting_vehicle = sval;
        m_need_to_send = true;
        cout << "Swimmer Request Received" << endl;
     }
     else if(key == "AWAKE"){
      string sval  = msg.GetString();
      for (int i=0; i<m_number_of_vehicles; i++){
        if(tolower(sval) == m_vehicle_names[i]){
          m_vehicle_awake[m_vehicle_names[i]] = true;
        }
      }
     }
     else if(key == "NODE_REPORT"){
      cout << "Node Report Received: " << sval << endl;
      NodeReport report = stringToNodeReport(sval);
      string vname = report.getName();
      for (int i=0; i<m_number_of_vehicles; i++){
        if(tolower(vname) == m_vehicle_names[i]){
          m_nav_xy_points[m_vehicle_names[i]].push_back(report.getXYPoint());
        }
      }
     }

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool SendPoints::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SendPoints::Iterate()
{
  AppCastingMOOSApp::Iterate();

//Check if a vehicle is awake. If it is awake, send initial points
  for (int i = 0; i < m_number_of_vehicles; i++){
    if(m_vehicle_awake[m_vehicle_names[i]]==true && m_needs_initial_points[m_vehicle_names[i]]==true){
      string vname = m_vehicle_names[i];
      cout << "Sending Initial Points to " + toupper(vname) << endl;
      for (int i = 0; i < m_initial_points; i++){
        XYPoint point = m_region_fld.generatePoint();
        VisitPoint vpoint; 
        vpoint.initXY(point); 
        vpoint.setVehicle(vname);
        vpoint.setID(intToString(m_id_counter[vname]));
        m_visit_points[vname].push_back(vpoint);
        Notify("SWIMMER_ALERT_"+toupper(vname), vpoint.getSwimmerAlert());
        m_id_counter[vname] = m_id_counter[vname] + 1;
        if(m_id_counter[vname] >= m_initial_points){
          m_needs_initial_points[vname] = false; 
        }
    }
  }
  }

//if all bools in m_initial_points_needed are false, set m_all_initial_points_sent to true
  for (int i = 0; i < m_number_of_vehicles; i++){
    if(m_needs_initial_points[m_vehicle_names[i]] == true){
      m_all_initial_points_sent = false;
      break;
    }
    else m_all_initial_points_sent = true;
  }

//Generate a requested point
if((m_need_to_send)&&(m_all_initial_points_sent)){
  cout << "Sending Requested Point to " + toupper(m_requesting_vehicle) << endl;
  XYPoint point = m_region_fld.generatePoint();

  //check to make sure point is at least m_min_dist away from all other points in m_visit_points
  bool good_point = false; 
  while(!good_point){
    good_point = true;
    for(int i = 0; i < m_number_of_vehicles; i++){
      string vname = m_vehicle_names[i];
      for(unsigned int j = 0; j < m_visit_points[vname].size(); j++){
        double dist = distPointToPoint(point.get_vx(), point.get_vy(), m_visit_points[vname][j].getX(), m_visit_points[vname][j].getY());
        if(dist < m_min_dist){
          good_point = false; 
          point = m_region_fld.generatePoint();
          break;
        }
      }
    }
  }

  VisitPoint vpoint; 
  vpoint.initXY(point); 
  vpoint.setVehicle(m_requesting_vehicle);

  for (int i = 0; i < m_number_of_vehicles; i++){
    if(m_vehicle_names[i] == m_requesting_vehicle){
      m_visit_points[m_vehicle_names[i]].push_back(vpoint);
    }
    else{
      cout << "ERROR: Requesting vehicle not found" << endl;
    }
  }
  
  Notify("SWIMMER_ALERT_"+toupper(m_requesting_vehicle), vpoint.getSwimmerAlert());
  for (int i = 0; i < m_number_of_vehicles; i++){
    if(m_vehicle_names[i] == m_requesting_vehicle){
      m_id_counter[m_vehicle_names[i]] = m_id_counter[m_vehicle_names[i]] + 1;
    }
  
    else{
      cout << "ERROR: Requesting vehicle not found" << endl;
    }
  m_need_to_send = false;
}
}
 
//for each vehicle, check if it has visited its points
for (int i = 0; i < m_number_of_vehicles; i++){
  string vname = m_vehicle_names[i];
  for(unsigned int j = 0; j < m_visit_points[vname].size(); j++){
      double dist = distPointToPoint(m_nav_xy_points[vname][0].get_vx(), m_nav_xy_points[vname][0].get_vy(), m_visit_points[vname][j].getX(), m_visit_points[vname][j].getY());
      if(dist < m_capture_radius){
        Notify("FOUND_SWIMMER_"+toupper(vname), "id=" + m_visit_points[vname][j].getID());
        m_visit_points[vname].erase(m_visit_points[vname].begin()+j);
      }
    }
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}


//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SendPoints::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();
  
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "vnames") {
      m_vehicle_names = parseString(value, ',');
      m_number_of_vehicles = m_vehicle_names.size();
      for (int i = 0; i < m_number_of_vehicles; i++){
        m_vehicle_awake[m_vehicle_names[i]] = false;
        m_id_counter[m_vehicle_names[i]] = 0;
        m_needs_initial_points[m_vehicle_names[i]] = true;
      }
      handled = true;
    }

    else if(param == "region") {
      m_region = value;
      if(m_region == "mit"){
        m_region_poly.add_vertex(-13, -28);
        m_region_poly.add_vertex(38, -5);
        m_region_poly.add_vertex(53, -41);
        m_region_poly.add_vertex(-3, -63);
        // m_region_poly.add_vertex(60,10);
        // m_region_poly.add_vertex(-30.3602,-32.8374);
        // m_region_poly.add_vertex(-4.6578,-87.0535);
        // m_region_poly.add_vertex(85.7024,-44.2161); 
        m_region_fld.addPolygon(m_region_poly);
      }
      handled = true;
    }
    else if(param == "initial_points") {
      m_initial_points = stoi(value);
      handled = true;
      cout << "Initial Points from parameters: " << m_initial_points << endl;
    }
    else if(param == "capture_radius"){
      m_capture_radius = stoi(value);
      handled = true;
      cout << "Capture Radius from parameters: " << m_capture_radius << endl;
    }
    else if(param == "min_distance"){
      m_min_dist = stoi(value);
      handled = true; 
      cout << "minimum distance enforced between points (default 10): " << m_min_dist << endl; 
    }
    else if(param == "region") {
      m_region = value;
      if(m_region == "mit"){
        m_region_poly.add_vertex(-13, -28);
        m_region_poly.add_vertex(38, -5);
        m_region_poly.add_vertex(53, -41);
        m_region_poly.add_vertex(-3, -63);
        // m_region_poly.add_vertex(60,10);
        // m_region_poly.add_vertex(-30.3602,-32.8374);
        // m_region_poly.add_vertex(-4.6578,-87.0535);
        // m_region_poly.add_vertex(85.7024,-44.2161); 
        m_region_fld.addPolygon(m_region_poly);
      }
      handled = true;
    }
    if(!handled)
      reportUnhandledConfigWarning(orig);
    
  }
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void SendPoints::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("SWIMMER_REQUEST", 0);
  Register("AWAKE", 0);
  Register("NODE_REPORT", 0); 
}


//------------------------------------------------------------
// Procedure: buildReport()

bool SendPoints::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: SendPoints.cpp " << endl;
  m_msgs << "============================================" << endl;
 //generate a table of the vehicle names, if they are awake, if they need initial points, and how many points they have received
  ACTable actab(4);
  actab << "Vehicle" << "Awake" << "Needs Initial Points" << "Points Received";
  for (int i = 0; i < m_number_of_vehicles; i++){
    string vname = m_vehicle_names[i];
    actab << vname << boolToString(m_vehicle_awake[vname]) << boolToString(m_needs_initial_points[vname]) << intToString(m_id_counter[vname]);
  }
  m_msgs << actab.getFormattedString() << endl;

  m_msgs << "============================================" << endl;


   

  return(true);
}
//-----------------------------------------------------------------



