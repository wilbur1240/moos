/*****************************************************************/
/*    Name: Raymond Turrisi                                      */
/*    Origin: Dept of Mechanical Engineering, MIT, Cambridge MA  */
/*    File: EvalEngineCore.h                                     */
/*    Developed: Summer 2023                                     */
/*    LastEd: June 5th, 2023                                     */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#pragma once

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <cstdarg> //va_list, va_start, va_end
#include <unordered_set>
#include "MBUtils.h"
#include "ACTable.h"
#include "EvalEngineMessage.h"
#include "EvalEngineKernelCore.h"

//TODO: It should be considered that an AppCastingMOOSApp, has a 'has-a' relationship with an EvalEngine, instead of an 'is-a' relationship

class EvalEngineCore : public AppCastingMOOSApp
{
public:
  EvalEngineCore(std::string eval_collection_type);
  ~EvalEngineCore();

  void showReleaseInfoAndExit();
  void showExampleConfigAndExit();
  void showHelpAndExit();
  void showInterfaceAndExit();

protected: // Standard MOOSApp functions to overload
  std::map<std::string, EvalEngineKernel*> m_eval_kernels;
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();
  void registerVariables();
  
protected:
  void insertKernel(EvalEngineKernel *kernel);
  bool dbg_print(const char *format, ...);

  /*
    TODO: 
      - Develop a state manager for managing the EvalEngine when not connected to the MOOSDB, and instead receives data from an alogfile
      - The kernels should update their state such a way that they are 1) invariant to the order which they are executed, and 2) invariant to 
        a real time analysis or post processng analysis
      - We will either be copying and rewriting an alog file, or returning their final states for a mission. Our publications will be ignored and rewritten to.
        The objective is that we can either apply an EvalEngine to an alogfile and obtain properties about it after the fact (it wasn't runnning initially), or
        we can change parameters/definitions of a kernel and obtain new metrics based on what we are looking for. Variables which we publish to and ignore, must also
        match our process name in MOOS based on the configuration file, since there are options such as VIEW_SEGLIST, VIEW_POINT, which other processes also write to. 
        We consider this, since in post processing/analysis, we may want to indicate seglists (i.e. display coursesness of sampling points for odometry), pulses (i.e. definitions of a near miss), 
        or persistent points with labeled time steps (i.e. labeling occurences/incidents) 
      - It should be considered that we can embed these features into a more atomic class which has analogous function calls to a MOOS App, which can be used in a separate process, rather than 
        include it all in one app
  */

private: // Configuration variables
  std::unordered_set<std::string> m_subscriptions_all;
  std::map<std::string, std::set<std::string> > m_subscriptions_to_kernels;
  std::vector<std::string> m_kernel_identities;
  std::map<std::string, EEMessage> m_info_buffer;
  std::map<std::string, std::string> m_parameters_all;
  std::map<std::string, std::string> m_updates_all;

private: // State variables
  std::string m_eval_collection_type;
  bool m_debug;
  FILE *m_debug_stream;
  static const uint16_t m_fname_buff_size = 255;
  char m_fname[m_fname_buff_size];
};