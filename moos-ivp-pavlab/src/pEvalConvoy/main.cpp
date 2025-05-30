/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: main.cpp, Cambridge MA                               */
/*    DATE: July 11th, 2021                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <string>
#include "MBUtils.h"
#include "ColorParse.h"
#include "EvalConvoy.h"
#include "EvalConvoy_Info.h"

using namespace std;

int main(int argc, char *argv[])
{
  string mission_file;
  string run_command = argv[0];

  for(int i=1; i<argc; i++) {
    string argi = argv[i];
    if((argi=="-v") || (argi=="--version") || (argi=="-version"))
      showReleaseInfoAndExit();
    else if((argi=="-e") || (argi=="--example") || (argi=="-example"))
      showExampleConfigAndExit();
    else if((argi == "-h") || (argi == "--help") || (argi=="-help"))
      showHelpAndExit();
    else if((argi == "-i") || (argi == "--interface"))
      showInterfaceAndExit();
    else if(strEnds(argi, ".moos") || strEnds(argi, ".moos++"))
      mission_file = argv[i];
    else if(strBegins(argi, "--alias="))
      run_command = argi.substr(8);
    else if(i==2)
      run_command = argi;
  }
  
  if(mission_file == "")
    showHelpAndExit();

  cout << termColor("green");
  cout << "pEvalConvoy launching as " << run_command << endl;
  cout << termColor() << endl;

  EvalConvoy EvalConvoy;

  EvalConvoy.Run(run_command.c_str(), mission_file.c_str());
  
  return(0);
}



