/*
 * main.cpp
 *
 *  Created on: Jan 15, 2011
 *      Author: Alexander Shishkov
 */
#include <boost/program_options.hpp>
#include <iostream>

#include <tod/converter/CameraConverter.h>
#include <tod/converter/Converter.h>

using namespace std;
using namespace tod;

namespace po = boost::program_options;

namespace
{
struct Options
{
  string base_directory;
  string dst_directory;
};
}

int options(int ac, char ** av, Options& opts)
{

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "Produce help message.")
      ("base,b", po::value<string>(&opts.base_directory), "Base directory. Required.")
      ("dst,d",  po::value<string>(&opts.dst_directory)->default_value("./"),
                "Directory for converted base.");
  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << endl;
    return 1;
  }

  if (!vm.count("base"))
  {
    cout << "Must supply a base directory." << "\n";
    cout << desc << endl;
    return 1;
  }
  return 0;
}


int main(int argc, char* argv[])
{
  Options opts;
  if (options(argc, argv, opts))
    return 1;

  CameraConverter camConverter(opts.base_directory, opts.dst_directory);
  camConverter.convert();

  Converter converter(opts.base_directory, opts.dst_directory);
  converter.convert();

  return 0;
}

