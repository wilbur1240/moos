/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: SockServer.h                                         */
/*    DATE: Mar 17th 2020                                        */
/*****************************************************************/

#ifndef SOCK_SERVER_HEADER
#define SOCK_SERVER_HEADER

#include <sys/socket.h>
#include <netdb.h>
#include <string>
#include <list> 

#define BUF_SIZE     1024
#define MAX_BUF_SIZE 16384

class SockServer {
public:
  SockServer(int port=29500);
  ~SockServer() {};

  void  setCurrTime(double v) {m_curr_time=v;}

  unsigned int getTotalRcvd() const {return(m_total_rcvd);}
  unsigned int getTotalSent() const {return(m_total_sent);}

  double getLastRcvd() const {return(m_last_rcvd);}
  double getLastSent() const {return(m_last_sent);}
  
  std::list<std::string> getMessages();
  std::list<std::string> getWarnings();
  std::list<std::string> getEvents();


  bool  setupListening();
  bool  setupConnection();

  bool  isListening() const   {return(m_listening);}
  bool  isConnected() const   {return(m_connected);}

  bool  readFromClient();
  bool  sendToClient(std::string str);
  
  
protected:
  void  addMessage(std::string);
  void  addWarning(std::string);
  void  addEvent(std::string);
  bool  isValidNMEA(std::string);
  
protected:
  int   m_port;
  
  bool  m_listening;
  bool  m_connected;

  int   m_sockfd_lis;
  int   m_sockfd_acc;
 
  char  m_inBuffer[MAX_BUF_SIZE];

  sockaddr_in m_serv_addr;
  sockaddr_in m_cli_addr;

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
