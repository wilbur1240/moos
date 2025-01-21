/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: SockClient.h                                         */
/*    DATE: Mar 17th 2020                                        */
/*****************************************************************/

#ifndef SOCK_CLIENT_HEADER
#define SOCK_CLIENT_HEADER

#include <sys/socket.h>
#include <netdb.h>
#include <string>
#include <list> 

#define MAX_BUF_SIZE 16384

class SockClient {
public:
  SockClient(int port=29500);
  ~SockClient() {};

  void  setCurrTime(double v) {m_curr_time=v;}

  unsigned int getTotalRcvd() const {return(m_total_rcvd);}
  unsigned int getTotalSent() const {return(m_total_sent);}

  double getLastRcvd() const {return(m_last_rcvd);}
  double getLastSent() const {return(m_last_sent);}
  
  bool  setupConnection();

  bool  isConnected() const   {return(m_connected);}

  bool  readFromServer();
  bool  sendToServer(std::string str);
  
  std::list<std::string> getMessages();
  std::list<std::string> getWarnings();
  std::list<std::string> getEvents();
  
protected:
  void  addMessage(std::string);
  void  addWarning(std::string);
  void  addEvent(std::string);
  bool  isValidNMEA(std::string);
  
protected:
  int   m_port;
  
  bool  m_connected;

  int   m_sockfd;
 
  char  m_inBuffer[MAX_BUF_SIZE];

  unsigned int m_max_list_size;


  unsigned int m_total_rcvd;
  unsigned int m_total_sent;
  double       m_last_rcvd;
  double       m_last_sent;
  double       m_curr_time;
  
  std::list<std::string>  m_rcvd_nmea_msgs;
  std::list<std::string>  m_warnings;
  std::list<std::string>  m_events;
};

#endif
