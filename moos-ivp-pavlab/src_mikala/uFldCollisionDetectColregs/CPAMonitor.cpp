/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: CPAMonitor.cpp                                       */
/*    DATE: Dec 20th 2015                                        */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/

#include <iostream>
#include <cstdlib>
#include <cmath>
#include "MBUtils.h"
#include "AngleUtils.h"
#include "CPAMonitor.h"
#include "NodeRecordUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor

CPAMonitor::CPAMonitor()
{
  m_ignore_range = 100;     // meters
  m_report_range = 50;      // meters
  m_swing_range  = 1;       // meters
  m_verbose      = false;

  m_closest_range = -1;
  m_closest_range_ever = -1;
}

//---------------------------------------------------------
// Procedure: setIgnoreRange
//      Note: Enforce report_range <= ignore_range

void CPAMonitor::setIgnoreRange(double val)
{
  m_ignore_range = val;
  if(m_report_range > m_ignore_range)
    m_report_range = m_ignore_range;
}


//---------------------------------------------------------
// Procedure: setReportRange
//      Note: Enforce ignore_range >= reportRange

void CPAMonitor::setReportRange(double val)
{
  m_report_range = val;
  if(m_ignore_range < m_report_range)
    m_ignore_range = m_report_range;
}


//---------------------------------------------------------
// Procedure: setSwingRange
//      Note: The amount of noted changed needed before declaring
//            a swing from opening to closing mode. To avoid
//            spurious events due to jitter in range measurements.

void CPAMonitor::setSwingRange(double val)
{
  m_swing_range = val;
  if(m_swing_range < 0)
    m_swing_range = 0;
}


//---------------------------------------------------------
// Procedure: addIgnoreGroup()

bool CPAMonitor::addIgnoreGroup(string grp_name)
{
  if(strContainsWhite(grp_name))
    return(false);
  
  grp_name = tolower(grp_name);
  if(vectorContains(m_ignore_groups, grp_name))
    return(false);

  m_ignore_groups.push_back(grp_name);
  return(true);
}


//---------------------------------------------------------
// Procedure: addRejectGroup()

bool CPAMonitor::addRejectGroup(string grp_name)
{
  if(strContainsWhite(grp_name))
    return(false);

  grp_name = tolower(grp_name);
  if(vectorContains(m_reject_groups, grp_name))
    return(false);

  m_reject_groups.push_back(grp_name);
  return(true);
}


//---------------------------------------------------------
// Procedure: getIgnoreGroups()

string CPAMonitor::getIgnoreGroups() const
{
  string rval;
  for(unsigned int i=0; i<m_ignore_groups.size(); i++) {
    if(i != 0)
      rval += ",";
    rval += m_ignore_groups[i];
  }

  return(rval);
}

//---------------------------------------------------------
// Procedure: getRejectGroups()

string CPAMonitor::getRejectGroups() const
{
  string rval;
  for(unsigned int i=0; i<m_reject_groups.size(); i++) {
    if(i != 0)
      rval += ",";
    rval += m_reject_groups[i];
  }

  return(rval);
}


//---------------------------------------------------------
// Procedure: getEvent()

CPAEvent CPAMonitor::getEvent(unsigned int ix) const
{
  CPAEvent null_event;
  if(ix >= m_events.size())
    return(null_event);
  return(m_events[ix]);
}

//---------------------------------------------------------
// Procedure: handleNodeReport()
//   Purpose: Handle a single node report, presumably within a
//            round of multiple incoming updated reports.
//    Return: false if syntactically invalid node report
//            true otherwise

bool CPAMonitor::handleNodeReport(string node_report)
{
  NodeRecord record = string2NodeRecord(node_report);
  if(!record.valid())
    return(false);
  string vname = record.getName();
  string group = tolower(record.getGroup());
  if(vectorContains(m_reject_groups, group))
    return(true);

  m_map_vgroup[vname] = group;
  
  // Part 1: Update the node record list for this vehicle
  m_map_vrecords[vname].push_front(record);
  if(m_map_vrecords[vname].size() > 2)
    m_map_vrecords[vname].pop_back();
  m_map_updated[vname] = true;

  return(true);
}

//---------------------------------------------------------
// Procedure: examineAndReport()

bool CPAMonitor::examineAndReport()
{
  m_closest_range = -1;

  map<string, bool>::iterator p;
  for(p=m_map_updated.begin(); p!=m_map_updated.end(); p++) {
    string vname = p->first;
    if(m_map_updated[vname]) {
      examineAndReport(vname);
    }
  }
      
  return(true);
}

//---------------------------------------------------------
// Procedure: examineAndReport(vname)

bool CPAMonitor::examineAndReport(string vname)
{
  map<string, bool>::iterator p;
  for(p=m_map_updated.begin(); p!=m_map_updated.end(); p++) {
    string contact = p->first;
    if(vname != contact) {
      examineAndReport(vname, contact);
    }
  }
      
  return(true);
}

//---------------------------------------------------------
// Procedure: examineAndReport(vname, contact)

bool CPAMonitor::examineAndReport(string vname, string contact)
{
  // Part 1: Sanity check
  if(!m_map_vrecords.count(vname) || !m_map_vrecords.count(contact))
    return(false);

  // Part 1B: Check ignore groups. If both vehicles have a group on
  // the list of ignore groups, then just consider ourselves done now.  
  string vname_group   = m_map_vgroup[vname];
  string contact_group = m_map_vgroup[contact];
  if(vectorContains(m_ignore_groups, vname_group) &&
     vectorContains(m_ignore_groups, contact_group))
    return(true);

  
  // Part 2: Create a unique "pairtag" so an event betweeen two
  //         vehicles is singular and not treated twice
  string tag = pairTag(vname, contact);
  if(m_verbose) {
    cout << "Examining: " << vname << " and " << contact <<
      "(" << tag << ")" << "  [" << m_iteration << "]" << endl;
  }
  if(m_map_pair_examined[tag])
    return(true);


  // Part 3: Update range and rate
  double prev_dist    = m_map_pair_dist[tag];
  bool   prev_closing = m_map_pair_closing[tag];
  bool   prev_valid   = m_map_pair_valid[tag];

  updatePairRangeAndRate(vname, contact);
  updatePairSituation(vname, contact);

  bool now_closing = m_map_pair_closing[tag];
  bool now_valid   = m_map_pair_valid[tag];

  if(m_verbose) {
    cout << "  prev_dist:    " << prev_dist << endl;
    cout << "  prev_closing: " << boolToString(prev_closing) << endl;
    cout << "  prev_valid:   " << boolToString(prev_valid) << endl;  
    cout << "  NOW closing: " << now_closing << endl;
    cout << "  NOW valid:   " << boolToString(now_valid)   << endl;
  }
  
  if(!prev_valid || !now_valid)
    return(true);

  // If transitioned from closing to opening
  if(prev_closing && !now_closing) {
    if(m_verbose)
      cout << " *********** POSTING ************* " << endl;
    double cpa_dist = m_map_pair_min_dist_running[tag];
    if(cpa_dist <= m_report_range) {
      CPAEvent event(vname, contact, cpa_dist);
      double midx = m_map_pair_midx[tag];
      double midy = m_map_pair_midy[tag];
      double beta = relBng(vname, contact);
      double alpha = relBng(contact, vname);

      event.setX(midx);
      event.setY(midy);
      event.setAlpha(alpha);
      event.setBeta(beta);
      event.setCOLREGSsituation(m_map_pair_situation[tag]);
      m_events.push_back(event);
    }
  }
  
  return(true);
}

//---------------------------------------------------------
// Procedure: updatePairRangeAndRate(vname, contact)

bool CPAMonitor::updatePairRangeAndRate(string vname, string contact)
{
  // Sanity check - make sure we have node records for each
  if(!m_map_vrecords.count(vname) || !m_map_vrecords.count(contact))
    return(false);

  // Part 1: get the vehicle info and calculate range
  NodeRecord os_record = m_map_vrecords[vname].front();
  NodeRecord cn_record = m_map_vrecords[contact].front();
  double osx  = os_record.getX();
  double osy  = os_record.getY();
  double cnx  = cn_record.getX();
  double cny  = cn_record.getY();
  double midx = (osx + ((cnx-osx)/2));
  double midy = (osy + ((cny-osy)/2));
  double dist = hypot(osx-cnx, osy-cny);
  string tag  = pairTag(vname, contact);

  if((m_closest_range < 0) || (dist < m_closest_range))
    m_closest_range = dist;
  
  if((m_closest_range_ever < 0) || (dist < m_closest_range_ever))
    m_closest_range_ever = dist;
  
  // Note that this pair has been examined on this round. This is cleared
  // for all pairs at the end of a round.
  m_map_pair_examined[tag] = true; 
  
  // If the distance is really large (greater than the ignore_range) then
  // remove all data for this tag and return. 
  if(dist > m_ignore_range) {
    m_map_pair_dist.erase(tag);
    m_map_pair_closing[tag] = false;
    m_map_pair_valid[tag] = false;
    m_map_pair_midx[tag] = 0;
    m_map_pair_midy[tag] = 0;
    return(true);
  }
  
  // Handle case where this is the first distance noted for this pair
  if(m_map_pair_dist.count(tag) == 0) {
    m_map_pair_dist[tag] = dist;
    m_map_pair_closing[tag] = false;
    m_map_pair_valid[tag] = false;
    return(true);
  }
  
  double dist_prev = m_map_pair_dist[tag];
  bool   closing_prev = m_map_pair_closing[tag];
  
  // Simple case: If distance hasn't changed, then no updates to the 
  // closing and valid flags either. 
  if(dist_prev == dist) 
    return(true);

  // Handle the case where we are technically closing
  if(dist_prev > dist) {
    // Handle case where may be transitioning from opening to closing
    if(!closing_prev) {
      // Check if range has "swung" sufficiently to warrant a change
      if((m_map_pair_max_dist_running[tag] - dist) > m_swing_range) {
	m_map_pair_closing[tag] = true;
      }
    }
    m_map_pair_min_dist_running[tag] = dist;
  }
  else {  // Handle case where we are technically opening
    // Handle case where may be transitioning from closing to opening 
    if(closing_prev) {
      // Check if range has "swung" sufficiently to warrant a change
      if((dist - m_map_pair_min_dist_running[tag]) > m_swing_range) {
	m_map_pair_closing[tag] = false;
      }
    }
    m_map_pair_max_dist_running[tag] = dist;
  }    

  m_map_pair_dist[tag] = dist;
  m_map_pair_valid[tag] = true;
  m_map_pair_midx[tag] = midx;
  m_map_pair_midy[tag] = midy;
  
  return(true);
}

//---------------------------------------------------------
// Procedure: clear()
//   Purpose: At the end of a round, clear data.

void CPAMonitor::clear()
{
  map<string, bool>::iterator p;
  for(p=m_map_updated.begin(); p!=m_map_updated.end(); p++)
    p->second = false;
  
  for(p=m_map_pair_examined.begin(); p!=m_map_pair_examined.end(); p++)
    p->second = false;

  m_events.clear();
}

//---------------------------------------------------------
// Procedure: pairTag()

string CPAMonitor::pairTag(string a, string b)
{
  if(a < b)
    return(a + "#" + b);
  return(b + "#" + a);
}


//---------------------------------------------------------
// Procedure: relBng()

double CPAMonitor::relBng(string vname1, string vname2)
{
  // Sanity Check - Need position information for both vehicles
  if((m_map_vrecords.count(vname1) == 0) ||
     (m_map_vrecords.count(vname2) == 0))
    return(0);
  
  NodeRecord record1 = m_map_vrecords[vname1].front();
  NodeRecord record2 = m_map_vrecords[vname2].front();

  double osx = record1.getX();
  double osy = record1.getY();
  double osh = record1.getHeading();

  double cnx = record2.getX();
  double cny = record2.getY();

  return(relBearing(osx, osy, osh, cnx, cny));
}


//---------------------------------------------------------
// Procedure: getVNames()

set<string> CPAMonitor::getVNames() const
{
  set<string> vnames;

  map<string, string>::const_iterator p;
  for(p=m_map_vgroup.begin(); p!=m_map_vgroup.end(); p++)
    vnames.insert(p->first);

  return(vnames);
}

//---------------------------------------------------------
// Procedure: getVRecord()

NodeRecord CPAMonitor::getVRecord(string vname)
{
  NodeRecord null_record;
  
  if(m_map_vrecords.count(vname) == 0)
    return(null_record);

  if(m_map_vrecords[vname].size() == 0)
    return(null_record);

  return(m_map_vrecords[vname].back());
}


//---------------------------------------------------------
// Procedure: getContactDensity()

unsigned int CPAMonitor::getContactDensity(string vname, double range) const
{
  // Sanity checks
  if((vname == "") || (range <= 0))
    return(0);

  if(m_map_vrecords.count(vname) == 0)
    return(0);
  if(m_map_vrecords.at(vname).size() == 0)
    return(0);
  
  // Part 1: Get ownship position
  NodeRecord os_record = m_map_vrecords.at(vname).front();
  double osx = os_record.getX();
  double osy = os_record.getY();
  
  // Part 2: Examine contact ranges, maybe increment counter
  unsigned int counter = 0;

  map<string, list<NodeRecord> >::const_iterator p;
  for(p=m_map_vrecords.begin(); p!=m_map_vrecords.end(); p++) {
    string contact = p->first;
    NodeRecord record = p->second.back();

    if(vname != contact) {
      double cnx = record.getX();
      double cny = record.getY();
      double cn_range = hypot(osx-cnx, osy-cny);
      if(cn_range <= range)
	counter++;
    }
  }
      
  return(counter);
}


//--------------------------------------------------------
// Procedure: updatePairSituation(vname, contact)
//   Purpose: Update the COLREGS situation for a pair of vehicles
bool CPAMonitor::updatePairSituation(string vname, string contact)
{
  // Sanity check - make sure we have node records for each
  if(!m_map_vrecords.count(vname) || !m_map_vrecords.count(contact))
    return(false);

  // Part 1: get the vehicle positions information and determine the situation 
    NodeRecord os_record = m_map_vrecords[vname].front();
    NodeRecord cn_record = m_map_vrecords[contact].front();
    string tag  = pairTag(vname, contact);
    

    double osh = os_record.getHeading();
    double cnh = cn_record.getHeading();

    double osrh = osh + 180; //ownship reciprocal heading
    if (osrh > 360)
      osrh = osrh - 360;

    double cnrh = cnh + 180; //contact reciprocal heading
    if (cnrh > 360)
      cnrh = cnrh - 360;

    double rel_bng = relBng(vname, contact);
    double targ_ang = targAng(vname, contact);

    //keep track that the pair's COLREGS situation has been evaluated
    m_map_pair_situation_examined[tag] = true;

    //check for overtaking
    // rule 13 contact must be 22.5 deg abaft the beam
    // target angle of ownship should be fwd of the contacts beam 
     if ((rel_bng > 112.5 && rel_bng < 247.5) && (targ_ang > 270 || targ_ang < 90))
     {
       m_map_pair_situation[tag] = "overtaking";
       return(true);
     }

    //check for head on 
    //compare courses
     double compare1 = abs(osh - cnrh);
     double compare2 = abs(cnh - osrh);

    if (compare1 < 10 || compare2 < 10)
    {
      m_map_pair_situation[tag] = "headon";
      return(true);
    }
    //check for crossing
    // forward of the sector for overtaking, and not close enought to reciprocal
    //course that it would be considered "head on"
    if ((rel_bng < 112.5 || rel_bng > 247.5) && (compare1 > 10 && compare2 > 10))
    {
      m_map_pair_situation[tag] = "crossing";
      return(true); 
    }
  else {
    m_map_pair_situation[tag] = "not_a_situation";
    return(false);
  }
}


//---------------------------------------------------------
//---------------------------------------------------------
// Procedure: targAng
//returns the target angle of vehicle 1 from vehicle 2 (ownship from contact)
//"where the contact should aim thier guns to hit us"

double CPAMonitor::targAng(string vname1, string vname2)
{
  // Sanity Check - Need position information for both vehicles
  if((m_map_vrecords.count(vname1) == 0) ||
     (m_map_vrecords.count(vname2) == 0))
    return(0);
  
  NodeRecord record1 = m_map_vrecords[vname1].front();
  NodeRecord record2 = m_map_vrecords[vname2].front();

  double osx = record1.getX();
  double osy = record1.getY();

  double cnx = record2.getX();
  double cny = record2.getY();
  double cnh = record2.getHeading();

  return(relBearing(cnx, cny, cnh, osx, osy));
}

//---------------------------------------------------------