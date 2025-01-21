/*****************************************************************/
/*    Name: Raymond Turrisi                                      */
/*    Origin: Dept of Mechanical Engineering, MIT, Cambridge MA  */
/*    File: EvalEngineCore.cpp                                   */
/*    Developed: Summer 2023                                     */
/*    LastEd: June 5th, 2023                                     */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "EvalEngineCore.h"
#include "EvalEngineMessage.h"
#include <time.h>
using namespace std;

//---------------------------------------------------------
// Constructor()

// TODO: Should have a better name than collection type
EvalEngineCore::EvalEngineCore(std::string eval_collection_type)
{
  m_eval_collection_type = eval_collection_type;
}

//---------------------------------------------------------
// Destructor

EvalEngineCore::~EvalEngineCore()
{
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    delete kernel_it->second;
  }
  m_eval_kernels.clear();
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool EvalEngineCore::OnNewMail(MOOSMSG_LIST &NewMail)
{
  /*
    Here, when we receive new mail, we accumulate an updates buffer and do nothing with it yet
  */
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    // TODO: This should always be true
    if (m_subscriptions_all.count(key))
    {
      m_info_buffer[key] = EEMessage(msg);
    }

    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool EvalEngineCore::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool EvalEngineCore::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Let all the kernels receive new mail, do error checking, and update their variables
  // for all the updates we have, we go through the names of the topics which have new updates
  // Each kernel should be designed to handle one piece of mail at a time, in order to enable high-level error checking

  m_info_buffer["MOOSTime"] = EEMessage("MOOSTime", MOOSTime(), MOOSTime());

  std::map<std::string, EEMessage>::iterator it_info;
  for (it_info = m_info_buffer.begin(); it_info != m_info_buffer.end(); ++it_info)
  {
    // For each topic, we go through all the kernels which have subscribed to this topic
    std::set<std::string>::iterator kernel_it;
    for (kernel_it = m_subscriptions_to_kernels[it_info->first].begin(); kernel_it != m_subscriptions_to_kernels[it_info->first].end(); ++kernel_it)
    {
      dbg_print("Updating: %s with topic %s\n", (*kernel_it).c_str(), it_info->first.c_str());
      bool successful_update = m_eval_kernels[*kernel_it]->UpdateVariable(m_info_buffer[it_info->first]);
      if (successful_update == false)
      {
        dbg_print("Unsuccessful update\n");
        dbg_print("Unhandled Mail: Kernel <%s> -> Msg<key=%s,val=%s>\n", (*kernel_it).c_str(), it_info->first.c_str(), it_info->second.msg().c_str());
        reportRunWarning("Unhandled Mail: Kernel <" + *kernel_it + "> -> Msg<key=" + it_info->first + ",val=" + it_info->second.msg() + ">");
      }
    }
  }

  // Clear all the new mail that we've received
  m_info_buffer.clear();

  // Now go through all the kernels and let them provide their updates
  std::map<std::string, std::string> updates;
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    if ((kernel_it->second->Evaluate(updates)) == false)
    {
      // TODO: Get rid of only a bool evaluation and let the kernel generate a warning message/runtime error which gets displayed at this level
      // i.e. !it_kernel->second.Evaluate(&updates, &warning_msg) -> print the warning message
      reportRunWarning("Error in Evaluation: Kernel <" + kernel_it->first + ">");
    }
  }

  // All the kernels would have completed a single evaluation, now we post all their updates
  std::map<std::string, std::string>::iterator msg_it;
  for (msg_it = updates.begin(); msg_it != updates.end(); ++msg_it)
  {
    Notify(msg_it->first, msg_it->second);
  }

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool EvalEngineCore::OnStartUp()
{
  /*
    On Startup, we are going to collect all the parameters which are included in the configuration block. We also initially populate the time in the buffer
  */
  AppCastingMOOSApp::OnStartUp();

  m_info_buffer["MOOSTime"] = EEMessage("MOOSTime", MOOSTime(), MOOSTime());
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;

  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    if (param == "debug")
    {
      m_debug = (value == tolower("true")) ? true : false;

      if (m_debug)
      {
        time_t rawtime;
        struct tm *timeinfo;
        memset(m_fname, m_fname_buff_size, '\0');
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char fmt[m_fname_buff_size];
        memset(fmt, m_fname_buff_size, '\0');
        strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                 m_eval_collection_type.c_str(), fmt);
        std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
        for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
          kernel_it->second->set_debug_stream(m_fname);

        dbg_print("Opened file\n");
      }
    }
    else if (m_parameters_all.count(param))
    {
      // If it matches a parameter one of the kernels will use, then we assume that it is valid and will be handled by the kernel
      m_parameters_all[param] = value;
    }
    else
    {
      // If it doesn't match a parameter that a kernel will use, then it is invalid
      reportUnhandledConfigWarning(orig);
    }
  }

  // Now that we have all the parameters, simply let the kernels update themselves
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    kernel_it->second->SetParams(m_parameters_all);
  }

  // We don't register for variables if we are processing an alogfile that already exists
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void EvalEngineCore::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  unordered_set<string>::iterator subscription_it;
  for (subscription_it = m_subscriptions_all.begin(); subscription_it != m_subscriptions_all.end(); ++subscription_it)
    Register(*subscription_it, 0);
}

//---------------------------------------------------------
// Procedure: insertKernel()

void EvalEngineCore::insertKernel(EvalEngineKernel *kernel)
{
  // We can traverse and navigate our collection of kernels with iterators and by looking them up
  // by their identity

  std::map<std::string, std::string> kernel_parameters = kernel->get_parameters();
  std::map<std::string, std::string>::iterator param_it;
  for (param_it = kernel_parameters.begin(); param_it != kernel_parameters.end(); param_it++)
    m_parameters_all[param_it->first] = param_it->second;

  m_eval_kernels[kernel->m_identity] = kernel;

  std::unordered_set<std::string> kernel_subscriptions = kernel->get_subscriptions();

  // We update our master list of all our subscriptions
  // We also construct a map from a topic to all the kernels which are subscribed - we do this so when we receive new mail, we only update those kernels
  std::unordered_set<std::string>::iterator sub_it;
  for (sub_it = kernel_subscriptions.begin(); sub_it != kernel_subscriptions.end(); sub_it++)
  {
    m_subscriptions_all.insert(*sub_it);
    m_subscriptions_to_kernels[*sub_it].insert(kernel->m_identity);
  }
}

// TODO
void EvalEngineCore::showReleaseInfoAndExit()
{
  std::cout << "showReleaseInfoAndExit\n";
  exit(0);
}
void EvalEngineCore::showExampleConfigAndExit()
{

  // TODO: Make it so the user can order the display of the parameters

  // Print the high level MOOS App configuration parameters
  cout << "//--------------------------------------------------------" << endl;
  cout << "// " << m_eval_collection_type << " Example MOOS Configuration" << endl
       << endl;
  cout << "ProcessConfig = " << m_eval_collection_type << endl;
  cout << "{" << endl
       << endl;
  cout << "//Engine Configuration Parameters" << endl;
  cout << "      AppTick = 10" << endl;
  cout << "    CommsTick = 10" << endl
       << endl;
  cout << "        debug = false" << endl
       << endl;
  cout << "//Kernel-specific configuration parameters" << endl
       << endl;

  // For each kernel which is included is built into this EvalEngine, generate a configuration file template with default values
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    // Kernel name
    cout << "//    > " << kernel_it->second->m_identity.c_str() << " (default values as shown)" << endl;
    // First find the longest parameter name, and format the width of each line accordinly so everything is nicely aligned
    std::map<std::string, std::string> parameters = kernel_it->second->get_parameters();
    uint16_t longest_name = 0;
    std::map<std::string, std::string>::iterator param_it;
    for (param_it = parameters.begin(); param_it != parameters.end(); ++param_it)
      if (param_it->first.size() > longest_name)
        longest_name = param_it->first.size();

    // If there are no parameters for a kernel, display such
    if (parameters.size() == 0)
      cout << "//   ( no parameters)" << endl;

    // For each parameter if they exist, print out the parameter name and the default value
    for (param_it = parameters.begin(); param_it != parameters.end(); ++param_it)
      cout << "        " << std::right << std::setw(longest_name) << param_it->first << " = " << param_it->second << endl;
    cout << endl;
  }
  cout << "}" << endl;

  exit(0);
}

// TODO
void EvalEngineCore::showHelpAndExit()
{
  std::cout << "showHelpAndExit\n";
  exit(0);
}

// TODO
void EvalEngineCore::showInterfaceAndExit()
{
  std::cout << "showInterfaceAndExit\n";
  exit(0);
}

//---------------------------------------------------------
// dbg print

bool EvalEngineCore::dbg_print(const char *format, ...)
{
  if (m_debug == true)
  {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_fname, "a");
    if (m_debug_stream != nullptr)
    {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      return true;
    }
    else
    {
      reportRunWarning("Debug mode is enabled and file could not be opened\n");
      return false;
    }
  }
  return false;
}

//------------------------------------------------------------
// Procedure: buildReport()

bool EvalEngineCore::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(2);
  // TODO: Can this format be improved?
  actab << "Kernel | Report";
  actab.addHeaderLines();
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    kernel_it->second->AppCastingNotice(actab);
  }
  m_msgs << actab.getFormattedString();
  return (true);
}
