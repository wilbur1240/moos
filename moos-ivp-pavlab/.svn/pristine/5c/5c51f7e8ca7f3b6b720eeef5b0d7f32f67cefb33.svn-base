/************************************************************/
/*    NAME: Alex Wunderlich and Mikala Molina               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenRescue.cpp                                     */
/*    DATE: Mar 8th 2022                                    */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "GenGreedyRescue.h"
#include "ACTable.h"
#include "math.h"
#include <vector>
#include <algorithm>
#include <vector>
#include "XYPoint.h"
#include <cmath>
#include "XYSegList.h"
#include "XYFormatUtilsPoint.h"
#include <list>
#include "PathUtils.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"

using namespace std;
//---------------------------------------------------------
// Constructor

GenGreedyRescue::GenGreedyRescue()
{
  visit_radius = 5;
  m_first_time = true;
  m_regen_bool = false;
  m_added_path = false;
  m_found = false;
}
//---------------------------------------------------------
// Destructor
GenGreedyRescue::~GenGreedyRescue()
{
}
//---------------------------------------------------------
// Procedure: OnNewMail

bool GenGreedyRescue::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);
  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    double dval = msg.GetDouble();

    if (key == "NAV_X")
    {
      m_x_current = msg.GetDouble();
      m_init_x = true;
    }
    else if (key == "NAV_Y")
    {
      m_y_current = msg.GetDouble();
    }
    else if (key == "FOUND_SWIMMER")
    {
      string visit_swimmer = msg.GetString();
      string found_id = tokStringParse(visit_swimmer, "id", ',', '='); // parses  "id= #" from found message
      m_found_list.insert(found_id); //creates a set of all found ID's. Sets do not include duplicates like vectors
      m_swimmer_list.erase(found_id); //removes ID's that have been found from the map of swimmers in the water
    }

    else if (key == "SWIMMER_ALERT")
    {
      string visit_point = msg.GetString(); 
      string swimmer_id = tokStringParse(visit_point, "id", ',', '='); // parses "id= #" from swimmer alert
      
      //Compares swimmer ID to list of found ID's
      bool found_it = false;
      for (set<string>::iterator it = m_found_list.begin(); it != m_found_list.end(); it++)
      {
        if (swimmer_id == *it)
        {
          found_it = true;
          break;
        }
      }
      if (!found_it) //if the ID is not in the set of found ID's, the ID and XYPoint are added to map of swimmers in the water
      {
        m_swimmer_list[swimmer_id] = string2Point(visit_point);
      }
    }
    else if (key == "NODE_REPORT"){
      string sval = msg.GetString();
      string type = tokStringParse(sval, "TYPE", ',', '='); 
      NodeRecord node_report = string2NodeRecord(sval);
      if(type == "KAYAK"){
        m_contact_x = node_report.getX(); 
        m_contact_y = node_report.getY(); 
        m_contact_hdg = node_report.getHeading(); 
      }
    }
    else if (key == "GENPATH_REGENERATE")
    {
      m_regen_bool = true;
    }
    else if (key != "APPCAST_REQ") // handled by appcastingmoosap
    {
      reportRunWarning("Unhandled Mail:" + key);
    }
  }
  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer
bool GenGreedyRescue::OnConnectToServer()
{
  registerVariables();
  return (true);
}
//---------------------------------------------------------

// Procedure: Iterate()
bool GenGreedyRescue::Iterate()
{
  AppCastingMOOSApp::Iterate();
  AppCastingMOOSApp::PostReport();
  Notify("AWAKE", m_vname);

  if (m_swimmer_list.size() != 0)
  {
    string update_visit_point = "points =";
    XYSegList visit_seglist;
    //add XYPoints from map of swimmers in the water to an XYSegList
    for (map<string, XYPoint>::iterator it = m_swimmer_list.begin(); it != m_swimmer_list.end(); ++it)
    {
      visit_seglist.add_vertex(it->second, it->first);
    }
    visit_seglist = greedyPath(visit_seglist, m_x_current, m_y_current);
    update_visit_point += visit_seglist.get_spec();
    Notify("SURVEY_UPDATE", update_visit_point);
    Notify("SWIMMERS_REMAINING", m_swimmer_list.size());
    
  }
    // if ((m_swimmer_list.size() == 0))
    // {
    //   Notify("RETURN", "true");
    // }
  
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool GenGreedyRescue::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (m_MissionReader.GetConfiguration(GetAppName(), sParams))
  {
    list<string>::iterator p;
    for (p = sParams.begin(); p != sParams.end(); p++)
    {
      string line = *p;
      string param = tolower(biteStringX(line, '='));
      string value = line;

      if (param == "vname")
      {
        m_vname = value;
      }
      else if (param == "bar")
      {
        // handled
      }
    }
  }
  registerVariables();
  return (true);
}
//---------------------------------------------------------
// Procedure: registerVariables
void GenGreedyRescue::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VISIT_POINT", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("GENPATH_REGENERATE", 0);
  Register("GENRESCUE_REGENERATE", 0);
  Register("SWIMMER_ALERT", 0);       
  Register("FOUND_SWIMMER", 0); 
  Register("NODE_REPORT", 0);      
  Notify("READY_TO_RECIEVE", "true"); // added
}

//------------------------------------------------------------
// Procedure: BuildReport()
bool GenGreedyRescue::buildReport()
{
  m_msgs << "============================================ \n";
  m_msgs << "File: GenRescue.cpp                        \n";
                   
  m_msgs << "============================================ \n";

  return (true);
}
