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
#include "EvalEngineCore.h"
#include "std_ee_kernels.h"
#include <sstream>
#include <time.h>
using namespace std;

//---------------------------------------------------------
// Constructor()

EvalEngineCore::EvalEngineCore()
{
  m_eval_collection_type = "EvalEngine";
}

//---------------------------------------------------------
// Destructor()

EvalEngineCore::~EvalEngineCore()
{
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    delete kernel_it->second;
  }
  m_eval_kernels.clear();
}

// TODO: Should have a better name than collection type
bool EvalEngineCore::setIdentity(std::string identity)
{
  m_eval_collection_type = identity;
}

//---------------------------------------------------------
// Procedure: Initialize()

bool EvalEngineCore::Initialize()
{
  // Now that we have all the parameters, simply let the kernels update themselves
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    kernel_it->second->SetParams(m_parameters_all);
  }
  // We don't register for variables if we are processing an alogfile that already exists
  return (true);
}

//---------------------------------------------------------
// Procedure: UpdateVariables()

bool EvalEngineCore::UpdateVariables(std::map<std::string, EEMessage> msg_buffer)
{

  std::map<std::string, EEMessage>::iterator p;

  // Unpack the collection of MOOS Messages within this timeframe of receiving new mail
  for (p = msg_buffer.begin(); p != msg_buffer.end(); p++)
  {
    string key = p->second.getVarName();
    m_info_buffer[key] = p->second;
  }

  // Now that we have all the mail since the last cycle, we distribute this mail to the kernels
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
        // reportRunWarning("Unhandled Mail: Kernel <" + *kernel_it + "> -> Msg<key=" + it_info->first + ",val=" + it_info->second.msg() + ">");
      }
    }
  }

  // Clear all the new mail that we've received
  m_info_buffer.clear();

  return (true);
}

//---------------------------------------------------------
// Procedure: ComputeEvaluations()
//            happens AppTick times per second

std::vector<std::pair<std::string, std::string> > EvalEngineCore::ComputeEvaluations()
{
  // Let all the kernels receive new mail, do error checking, and update their variables
  // for all the updates we have, we go through the names of the topics which have new updates
  // Each kernel should be designed to handle one piece of mail at a time, in order to enable high-level error checking
  std::vector<std::pair<std::string, std::string> > updates;

  // Now go through all the kernels and let them provide their updates
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    if ((kernel_it->second->Evaluate(updates)) == false)
    {
      // TODO: Get rid of only a bool evaluation and let the kernel generate a warning message/runtime error which gets displayed at this level
      // i.e. !it_kernel->second.Evaluate(&updates, &warning_msg) -> print the warning message
      // reportRunWarning("Error in Evaluation: Kernel <" + kernel_it->first + ">");
    }
  }
  return updates;
}

bool EvalEngineCore::setTime(double moos_time)
{
  m_info_buffer["MOOSTime"] = EEMessage("MOOSTime", moos_time, moos_time);
  return true;
}

//---------------------------------------------------------
// Procedure: setParameter()
bool EvalEngineCore::setParameter(std::string param, std::string value)
{
  // TODO: Maybe there is an opportunity for error checking here
  m_parameters_all[param] = value;
  return true;
}

//---------------------------------------------------------
// Procedure: getParameterKeys()
std::unordered_set<std::string> EvalEngineCore::getParameterKeys()
{
  std::unordered_set<std::string> param_keys;
  std::map<std::string, std::string>::iterator it;
  for (it = m_parameters_all.begin(); it != m_parameters_all.end(); ++it)
    param_keys.insert(it->first);
  return param_keys;
}

//---------------------------------------------------------
// Procedure: getSubscriptions()

std::unordered_set<std::string> EvalEngineCore::getSubscriptions()
{
  return m_subscriptions_all;
}

//---------------------------------------------------------
// Procedure: getPublications()

std::unordered_set<std::string> EvalEngineCore::getPublications()
{
  return m_publications_all;
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
std::string EvalEngineCore::genEngineReleaseInfo()
{
  return "showReleaseInfoAndExit\n";
}

std::string EvalEngineCore::genEngineExampleConfig()
{

  // TODO: Make it so the user can order the display of the parameters

  // Print the high level MOOS App configuration parameters
  // cout << "//--------------------------------------------------------" << endl;
  // cout << "// " << m_eval_collection_type << " Example MOOS Configuration" << endl
  //      << endl;
  // cout << "ProcessConfig = " << m_eval_collection_type << endl;
  // cout << "{" << endl
  //      << endl;
  // cout << "//Engine Configuration Parameters" << endl;
  // cout << "      AppTick = 10" << endl;
  // cout << "    CommsTick = 10" << endl
  //      << endl;
  // cout << "        debug = false" << endl
  //      << endl;

  stringstream ecp; // engine configuration parameters
  ecp << "//Kernel-specific configuration parameters" << endl
      << endl;

  // For each kernel which is included is built into this EvalEngine, generate a configuration file template with default values
  std::map<std::string, EvalEngineKernel *>::iterator kernel_it;
  for (kernel_it = m_eval_kernels.begin(); kernel_it != m_eval_kernels.end(); ++kernel_it)
  {
    // Kernel name
    ecp << "//    > " << kernel_it->second->m_identity.c_str() << " (default values as shown)" << endl;
    // First find the longest parameter name, and format the width of each line accordinly so everything is nicely aligned
    std::map<std::string, std::string> parameters = kernel_it->second->get_parameters();
    uint16_t longest_name = 0;
    std::map<std::string, std::string>::iterator param_it;
    for (param_it = parameters.begin(); param_it != parameters.end(); ++param_it)
      if (param_it->first.size() > longest_name)
        longest_name = param_it->first.size();

    // If there are no parameters for a kernel, display such
    if (parameters.size() == 0)
      ecp << "//   ( no parameters)" << endl;

    // For each parameter if they exist, print out the parameter name and the default value
    for (param_it = parameters.begin(); param_it != parameters.end(); ++param_it)
      ecp << "        " << std::right << std::setw(longest_name) << param_it->first << " = " << param_it->second << endl;
    ecp << endl;
  }
  // ecp << "}" << endl;
  return ecp.str();
}

// TODO
std::string EvalEngineCore::genEngineInterface()
{
  return "showInterfaceAndExit\n";
}

//---------------------------------------------------------
// set_debug_stream

void EvalEngineCore::set_debug_stream(char debug_stream_name[])
{
  m_debug_stream_name = debug_stream_name;
}

//---------------------------------------------------------
// dbg print

bool EvalEngineCore::dbg_print(const char *format, ...)
{
  if (m_debug_stream_name != nullptr)
  {
    va_list args;
    va_start(args, format);
    m_debug_stream = fopen(m_debug_stream_name, "a");
    if (m_debug_stream != nullptr)
    {
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
      return true;
    }
    else
    {
      return false;
    }
  }
  return false;
}

//------------------------------------------------------------
// Procedure: buildReport()

bool EvalEngineCore::genAppCastingReport(std::stringstream &m_msgs)
{

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
