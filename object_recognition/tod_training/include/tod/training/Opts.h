/*
 * Opts.hpp
 *
 *  Created on: Jan 10, 2011
 *      Author: erublee
 */

#ifndef OPTS_HPP_
#define OPTS_HPP_

#include <boost/program_options.hpp>
#include <tod/core/Camera.h>
#include <string>
#include <list>
namespace tod
{

struct CommonOptions
{
  std::string directory;
  bool verbose;
  int n_threads;

  boost::program_options::options_description desc;

  CommonOptions();

  void check(const  boost::program_options::variables_map& vm) const throw ( std::runtime_error );
};

struct CameraOptions
{
  std::string camera_file;
  int camera_format;
  boost::program_options::options_description desc;

  CameraOptions();
  Camera loadCamera(const CommonOptions& opts) const;

  void check(const  boost::program_options::variables_map& vm) const throw ( std::runtime_error );
};

struct ImagesOptions
{
  std::string image_list;
  std::string extension;

  boost::program_options::options_description desc;

  ImagesOptions();
  std::list<std::string> loadImageNames(const CommonOptions& opts) const;
};

struct PCDOptions
{
  std::string pcd_list;

  boost::program_options::options_description desc;

  PCDOptions();
  std::list<std::string> loadPCDNames(const CommonOptions& opts) const;
};

}

#endif /* OPTS_HPP_ */
