#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#include <iostream>
#include <vector>
#include <map>

#include <tod/detecting/Loader.h>
#include <tod/detecting/Recognizer.h>
#include <tod/test/ExtendedObjectInfo.h>
#include <tod/test/Calculator.h>

#define foreach BOOST_FOREACH

using namespace cv;
using namespace tod;
using namespace std;
namespace po = boost::program_options;

int options(int ac, char ** av, CalculatorParameters& params)
{
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "Produce help message.");
  desc.add_options()("log,l", po::value<string>(&params.logFilename), "The name of the log file");
  desc.add_options()("verbose,V", po::value<int>(&params.verbose)->default_value(1), "Verbosity level");

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  if (!vm.count("log"))
  {
    cout << "Must supply log file path." << "\n";
    cout << desc << endl;
    return 1;
  }
  return 0;
}

int main(int argc, char* argv[])
{
  CalculatorParameters params;
  if (options(argc, argv, params))
    return 1;

  Calculator calc(params);
  calc.calculate();
  return 0;
}
