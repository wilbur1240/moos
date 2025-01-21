/*****************************************************************/
/*    NAME: Michael Benjamin (deriv from Alon Yaari)             */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: SockServer.cpp                                       */
/*    DATE: Mar 17th 2020                                        */
/*****************************************************************/

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <unistd.h>
#include <netinet/in.h>
#include <cstdio>
#include <fcntl.h>
#include "MBUtils.h"
#include "SockServer.h"

using namespace std;

//---------------------------------------------------------
// Constructor

SockServer::SockServer(int port)
{
  m_port        = port;
  m_sockfd_lis  = 0;
  m_sockfd_acc  = 0;
  m_listening   = false;
  m_connected   = false;

  m_total_rcvd  = 0;
  m_total_sent  = 0;
  m_last_rcvd   = 0;
  m_last_sent   = 0;
  m_curr_time   = -1;
  
  m_max_list_size = 1000;
}

//---------------------------------------------------------
// Procedure: setupListening()

bool SockServer::setupListening()
{
  // Part 1: Create the Socket
  m_sockfd_lis = socket(AF_INET, SOCK_STREAM, 0);
  string whyfail1(strerror(errno));

  int good_sock = (m_sockfd_lis >= 0);
  if(!good_sock) {
    addWarning("Error creating TCP port: " + whyfail1);
    return(false);
  }
  addEvent("Created socket connection on port: " + intToString(m_port));
  addEvent("m_sockfd_lis:" + intToString(m_sockfd_lis));


  // Make this socket non-blocking
  int res = fcntl(m_sockfd_lis, F_SETFL,
		  fcntl(m_sockfd_lis, F_GETFL, 0) | O_NONBLOCK);
  if(res == -1){
    addWarning("calling fcntl");
    return(false);
  }
  
  
  bzero((char *) &m_serv_addr, sizeof(m_serv_addr));
  m_serv_addr.sin_family = AF_INET;
  m_serv_addr.sin_addr.s_addr = INADDR_ANY;
  m_serv_addr.sin_port = htons(m_port);
  
  // Part 2: Bind the Socket
  int bind_res = bind(m_sockfd_lis, (sockaddr*) &m_serv_addr, sizeof(m_serv_addr));
  good_sock = (bind_res >= 0);
  string whyfail2(strerror(errno));

  if(!good_sock) {
    addWarning("Error binding TCP socket. " + whyfail2);
    return(false);
  }

  // Part 3: set up listening
  int heard = listen(m_sockfd_lis, 2);
  string whyfail3(strerror(errno));

  if(heard) {
    addEvent("Error configuring listening on TCP socket. " + whyfail3);
    return(false);
  }

  addEvent("Listening for connection on port: " + intToString(m_port));

  m_listening = true;

  return(true);
}

//---------------------------------------------------------
// Procedure: setupConnection()

bool SockServer::setupConnection()
{
  if(!m_listening)
    return(false);

  // Look for incoming connections to accept
  socklen_t client_len = sizeof(m_cli_addr);
  m_sockfd_acc = accept(m_sockfd_lis, (struct sockaddr*) &m_cli_addr, &client_len);
  string whyfail(strerror(errno));
    
  m_connected = (m_sockfd_acc >= 0);

  if(!m_connected) {
    addWarning("Error accepting client to TCP socket: " + whyfail);
    return(false);
  }
  
  addEvent("Accepted connection on port: " + intToString(m_port));
  addEvent("m_sockfd_acc:" + intToString(m_sockfd_acc));

  return(m_connected);
}

//---------------------------------------------------------
// Procedure: readFromCLient()

bool SockServer::readFromClient()
{
  if(!m_connected)
    return(false);

  // ==========================================================
  // Part 1: Grab any chars that may have arrived at the socket
  // ==========================================================
  char incoming[BUF_SIZE];
  
  ssize_t num_bytes = read(m_sockfd_acc, incoming, BUF_SIZE);
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
// Procedure: sendToClient()
//   Purpose: Sends a string over TCP to the attached client.
//   Returns: TRUE if successful or if blank string (which is not sent)
//            FALSE if error on writing to the TCP port

bool SockServer::sendToClient(string str)
{
  if(str.length() == 0)
    return(true);
  
  int num_bytes = write(m_sockfd_acc, str.c_str(), str.length());
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

void SockServer::addMessage(string msg)
{
  m_rcvd_nmea_msgs.push_front(msg);
  if(m_rcvd_nmea_msgs.size() > m_max_list_size)
    m_rcvd_nmea_msgs.pop_back();
}

//---------------------------------------------------------
// Procedure: addWarning()

void SockServer::addWarning(string warning)
{
  m_warnings.push_front(warning);
  if(m_warnings.size() > m_max_list_size)
    m_warnings.pop_back();
}

//---------------------------------------------------------
// Procedure: addEvent()

void SockServer::addEvent(string event)
{
  m_events.push_front(event);
  if(m_events.size() > m_max_list_size)
    m_events.pop_back();
}

//---------------------------------------------------------
// Procedure: getMessages()

list<string> SockServer::getMessages()
{
  list<string> messages = m_rcvd_nmea_msgs;
  m_rcvd_nmea_msgs.clear();

  return(messages);  
}

//---------------------------------------------------------
// Procedure: getWarnings()

list<string> SockServer::getWarnings()
{
  list<string> warnings = m_warnings;
  m_warnings.clear();

  return(warnings);  
}

//---------------------------------------------------------
// Procedure: getEvents()

list<string> SockServer::getEvents()
{
  list<string> events = m_events;
  m_events.clear();

  return(events);  
}


//---------------------------------------------------------
// Procedure: isValidNMEA()

bool SockServer::isValidNMEA(string str)
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
