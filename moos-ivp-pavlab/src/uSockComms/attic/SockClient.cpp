/*****************************************************************/
/*    NAME: Michael Benjamin (deriv from Alon Yaari)             */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: SockClient.cpp                                       */
/*    DATE: Mar 17th 2020                                        */
/*****************************************************************/

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <fcntl.h>
#include "MBUtils.h"
#include "SockClient.h"

using namespace std;

#define BUF_SIZE 1024

//---------------------------------------------------------
// Constructor

SockClient::SockClient(int port)
{
  m_port      = port;
  m_sockfd    = 0;
  m_connected = false;

  m_total_rcvd = 0;
  m_total_sent = 0;
  m_last_rcvd  = 0;
  m_last_sent  = 0;
  m_curr_time  = -1;
    
  m_max_list_size = 1000;
}

//---------------------------------------------------------
// Procedure: setupConnection()

bool SockClient::setupConnection()
{
  sockaddr_in serv_addr;
  if((m_sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    string whyfail(strerror(errno));
    addWarning("Socket creation error " + whyfail);
    return(false);
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(m_port);

  // Convert IPv4 and IPv6 addresses from text to binary form  
  if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) {
    addWarning("Invalid address or Address not supported");
    return(false);
  }

  if(connect(m_sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    string whyfail(strerror(errno));
    addWarning("Connection Failed" + whyfail);
    return(false);
  }

  addEvent("Established connection on port: " + intToString(m_port));
  addEvent("m_sockfd:" + intToString(m_sockfd));

  m_connected = true;
  return(true);
}

//---------------------------------------------------------
// Procedure: readFromServer()

bool SockClient::readFromServer()
{
  if(!m_connected)
    return(false);
  
  // ==========================================================
  // Part 1: Grab any chars that may have arrived at the socket
  // ==========================================================
  char incoming[BUF_SIZE];
  
  ssize_t num_bytes = read(m_sockfd, incoming, BUF_SIZE);
  string whyfail(strerror(errno));
  if((num_bytes == 0) || (errno == EWOULDBLOCK)) {
    addWarning("Nothing to read");
    return(false);
  }
  if((num_bytes < 0) && (errno != EWOULDBLOCK)) {
    m_connected = false;
    addWarning("Error in reading from socket: " + whyfail);
    return(false);
  }

  // If the new data fits, move onto the input buffer
  if(strlen(m_inBuffer) + num_bytes < MAX_BUF_SIZE)
    strncat(m_inBuffer, incoming, num_bytes);

  string inbuff = m_inBuffer;

  // If there is no $ char, then no NMEA to parse. Leave the buffer
  // alone and wait for more content to arrive.
  if(!strContains(inbuff, '$'))
    return(true);


  // ==========================================================
  // Part 2: Handle and Parse the input buffer
  // ==========================================================

  // Clear the character array buffer
  memset(m_inBuffer, 0, MAX_BUF_SIZE);

  // Remove any content preceding the first $ char, and then replace the $
  biteStringX(inbuff, '$');
  inbuff = "$" + inbuff;

  // Parse string into a vector of lines
  findReplace(inbuff, '\r', '\n');
  vector<string> lines = parseString(inbuff, '\n');
  if(lines.size() == 0)
    return(true);

  // If last item is not a valide NMEA sentence, keep it for next time
  string last = lines[lines.size() - 1];
  if(!isValidNMEA(last)) {
    strcpy(m_inBuffer, last.c_str());
    lines.pop_back();
  }
    
  // handle lines that are valid NMEA messages w.r.t. checksum
  if(lines.size() > 0) 
    m_last_rcvd = m_curr_time;
  for(unsigned int i=0; i<lines.size(); i++) {
    m_total_rcvd++;
    string line = lines[i];
    if(isValidNMEA(line))
      m_rcvd_nmea_msgs.push_front(line);
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: sendToServer()
//   Purpose: Sends a string over TCP to the attached client.
//   Returns: TRUE if successful or if blank string (which is not sent)
//            FALSE if error on writing to the TCP port

bool SockClient::sendToServer(string str)
{
  if(str.length() == 0)
    return(true);
  
  int num_bytes = write(m_sockfd, str.c_str(), str.length());
  string whyfail(strerror(errno));
  if((num_bytes == 0) || (errno == EWOULDBLOCK)) {
    addWarning("Nothing to write: " + whyfail);
    return(false);
  }
  if(num_bytes < 0) {
    m_connected = false;
    addWarning("Error writing to socket: " + whyfail);
    return(false);
  }

  m_total_sent++;
  m_last_sent = m_curr_time;
  
  return(true);
}

//---------------------------------------------------------
// Procedure: addMessage()

void SockClient::addMessage(string msg)
{
  m_rcvd_nmea_msgs.push_front(msg);
  if(m_rcvd_nmea_msgs.size() > m_max_list_size)
    m_rcvd_nmea_msgs.pop_back();
}

//---------------------------------------------------------
// Procedure: addWarning()

void SockClient::addWarning(string warning)
{
  m_warnings.push_front(warning);
  if(m_warnings.size() > m_max_list_size)
    m_warnings.pop_back();
}

//---------------------------------------------------------
// Procedure: addEvent()

void SockClient::addEvent(string event)
{
  m_events.push_front(event);
  if(m_events.size() > m_max_list_size)
    m_events.pop_back();
}

//---------------------------------------------------------
// Procedure: getMessages()

list<string> SockClient::getMessages()
{
  list<string> messages = m_rcvd_nmea_msgs;
  m_rcvd_nmea_msgs.clear();

  return(messages);  
}

//---------------------------------------------------------
// Procedure: getWarnings()

list<string> SockClient::getWarnings()
{
  list<string> warnings = m_warnings;
  m_warnings.clear();

  return(warnings);  
}

//---------------------------------------------------------
// Procedure: getEvents()

list<string> SockClient::getEvents()
{
  list<string> events = m_events;
  m_events.clear();

  return(events);  
}


//---------------------------------------------------------
// Procedure: isValidNMEA()

bool SockClient::isValidNMEA(string str)
{
  if((str.length()==0) || (str[0]!='$') || !strContains(str, '*'))
    return(false);

  str = str.substr(1); // chop off the leading dollar sign

  string tail = rbiteString(str, '*');
  if(tail.length() < 2)
    return(false);
  string hexstr1 = tail.substr(0,2);
  string hexstr2 = checksumHexStr(str);

  if(hexstr1 != hexstr2)
    return(false);
    
  return(true);
}
