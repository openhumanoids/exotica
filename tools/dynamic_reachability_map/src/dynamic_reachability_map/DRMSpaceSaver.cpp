/*
 * DRMSpaceSaver.cpp
 *
 *  Created on: 20 Sep 2015
 *      Author: yiming
 */

#include "dynamic_reachability_map/DRMSpaceSaver.h"
#include <ctime>
namespace dynamic_reachability_map
{
  DRMSpaceSaver::DRMSpaceSaver()
  {

  }

  DRMSpaceSaver::~DRMSpaceSaver()
  {

  }

  bool DRMSpaceSaver::saveSpace(const std::string &path, DRMSpace_ptr &space,
      std::string &savepath)
  {
    struct stat st = { 0 };
    std::string folderpath = path + "/" + space->ps_->getRobotModel()->getName()
        + "_" + std::to_string(space->sample_size_) + "_"
        + std::to_string(space->resolution_) + "_"
        + std::to_string(ros::Time::now().sec);
    if (stat(folderpath.c_str(), &st) == -1
        && mkdir(folderpath.c_str(), 0700) != 0)
    {
      ROS_ERROR_STREAM("Cannot create saving directory to "<<path);
      return false;
    }
    ROS_INFO_STREAM("Saving sampling result to "<<folderpath);

    savepath = folderpath;
    ROS_INFO("Saving samples");
    std::ofstream samples_file(folderpath + "/samples.bin",
        std::ios_base::binary);
    for (int i = 0; i < space->sample_size_; i++)
    {
      samples_file.write((char *) &space->samples_[i].eff_index,
          sizeof(unsigned int));
      float pose[7] = { (float) space->samples_[i].effpose.position.x,
          (float) space->samples_[i].effpose.position.y,
          (float) space->samples_[i].effpose.position.z,
          (float) space->samples_[i].effpose.orientation.x,
          (float) space->samples_[i].effpose.orientation.y,
          (float) space->samples_[i].effpose.orientation.z,
          (float) space->samples_[i].effpose.orientation.w };
      for (int j = 0; j < 7; j++)
        samples_file.write((char *) &pose[j], sizeof(float));
      for (int j = 0; j < space->dimension_; j++)
        samples_file.write((char*) &space->samples_[i].q[j], sizeof(float));
    }
    samples_file.close();

    ROS_INFO("Saving space occupation");
    mkdir((folderpath + "/space_occupation").c_str(), 0700);
    for (int t = 0; t < space->thread_size_; t++)
    {
      unsigned int tmp_space_size = space->space_size_ / space->thread_size_;
      std::ofstream space_occupied_file(
          folderpath + "/space_occupation/space_occupation_" + std::to_string(t)
              + ".bin", std::ios_base::binary);
      for (unsigned int i = t * tmp_space_size;
          i
              < ((t == space->thread_size_ - 1) ?
                  space->space_size_ : (t + 1) * tmp_space_size); i++)
      {
        if (space->at(i).occup_samples.size() > 0
            && space->at(i).occup_samples.size() < space->sample_size_)
        {
          space_occupied_file.write((char *) &i, sizeof(unsigned int));
          unsigned long int tmp_size = space->at(i).occup_samples.size();
          if (tmp_size < space->sample_size_)
          {
            space_occupied_file.write((char *) &tmp_size,
                sizeof(unsigned long int));
            space_occupied_file.write(
                reinterpret_cast<const char*>(&space->at(i).occup_samples[0]),
                tmp_size * sizeof(unsigned long int));
          }
          else
          {
            unsigned int tmp = 2;
            unsigned long int tmp_index = 0;
            space_occupied_file.write((char *) &tmp, sizeof(unsigned int));
            space_occupied_file.write((char *) &tmp_index,
                sizeof(unsigned long int));
            space_occupied_file.write((char *) &tmp_index,
                sizeof(unsigned long int));
          }
        }
      }
      space_occupied_file.close();
    }

    ROS_INFO("Saving space reachability");
    std::ofstream reach_file;
    reach_file.open(folderpath + "/reach.txt");
    for (int i = 0; i < space->space_size_; i++)
    {
      for (int j = 0; j < space->at(i).reach_samples.size(); j++)
        reach_file << space->at(i).reach_samples[j] << " ";
      reach_file << std::endl;
    }
    reach_file.close();

    ROS_INFO("Saving information");
    std::ofstream info_file;
    info_file.open(folderpath + "/info.xml");

    info_file << "<?xml version=\"1.0\" ?>" << std::endl;
    time_t now = time(0);
    char* dt = ctime(&now);
    info_file
        << "<!-- This file is generated by Dynamic Reachability Map Space Saver -->\n<!-- UCT time "
        << dt << " -->" << std::endl;
    info_file << "<DynamicReachabilityMap>" << std::endl;
    info_file << "  <Robot name=\"" << space->ps_->getRobotModel()->getName()
        << "\"/>" << std::endl;
    info_file << "  <SpaceBounds>" << std::endl;
    info_file << "    <XLower value=\"" << space->space_bounds_.x_low << "\"/>"
        << std::endl;
    info_file << "    <XUpper value=\"" << space->space_bounds_.x_upper
        << "\"/>" << std::endl;
    info_file << "    <YLower value=\"" << space->space_bounds_.y_low << "\"/>"
        << std::endl;
    info_file << "    <YUpper value=\"" << space->space_bounds_.y_upper
        << "\"/>" << std::endl;
    info_file << "    <ZLower value=\"" << space->space_bounds_.z_low << "\"/>"
        << std::endl;
    info_file << "    <ZUpper value=\"" << space->space_bounds_.z_upper
        << "\"/>" << std::endl;
    info_file << "  </SpaceBounds>" << std::endl;
    info_file << "  <SpaceSize value=\"" << space->space_size_ << "\"/>"
        << std::endl;

    info_file << "  <VolumeResolution value=\"" << space->resolution_ << "\"/>"
        << std::endl;
    info_file << "  <PlanningGroup name=\"" << space->group_->getName() << "\">"
        << std::endl;
    for (int i = 0; i < space->group_->getVariableNames().size(); i++)
      info_file << "    <Joint name=\"" << space->group_->getVariableNames()[i]
          << "\"/>" << std::endl;
    info_file << "    <EndEffector name=\"" << space->eff_ << "\"/>"
        << std::endl;
    info_file << "  </PlanningGroup>" << std::endl;
    info_file << "  <SampleSize value=\"" << space->sample_size_ << "\"/>"
        << std::endl;
    info_file
        << "  <!-- !! You need to use the same number of threads to load occupation information !! -->"
        << std::endl;
    info_file << "  <OccupationSavingThreads value=\"" << space->thread_size_
        << "\"/>" << std::endl;
    info_file << "</DynamicReachabilityMap>" << std::endl;
    info_file.close();
    createCellURDF(folderpath, space);
    return true;
  }

  bool DRMSpaceSaver::createCellURDF(const std::string & path,
      DRMSpace_ptr &space)
  {
    struct stat st = { 0 };
    if (stat(path.c_str(), &st) == -1 && mkdir(path.c_str(), 0700) != 0)
    {
      ROS_ERROR("Cannot create directory");
      return false;
    }
    ROS_INFO("Creating DynamicReachabilityMapCell URDF");
    std::ofstream urdf;
    urdf.open(path + "/DRMCells.urdf");
    urdf << "<?xml version=\"1.0\" ?>" << std::endl;
    urdf << "<robot name=\"DRMCells\">" << std::endl;
    urdf << "<material name=\"Black\">" << std::endl;
    urdf << "\t<color rgba=\"0 0 0 .5\"/>" << std::endl;
    urdf << "</material>" << std::endl;
    urdf << "<link name=\"base\">" << std::endl;
    urdf << "</link>" << std::endl;
    for (int i = 0; i < space->space_size_; i++)
    {
      urdf << "<link name=\"cell_" << i << "\">" << std::endl;
      urdf << "\t<visual>" << std::endl;
      urdf << "\t\t<origin rpy=\"0 0 0\" xyz=\"" << space->at(i).center.x << " "
          << space->at(i).center.y << " " << space->at(i).center.z << "\"/>"
          << std::endl;
      urdf << "\t\t<geometry>" << std::endl;
      urdf << "\t\t\t<box size=\"" << space->resolution_ << " "
          << space->resolution_ << " " << space->resolution_ << "\"/>"
          << std::endl;
      urdf << "\t\t</geometry>" << std::endl;
      urdf << "\t\t<material name=\"Black\"/>" << std::endl;
      urdf << "\t</visual>" << std::endl;

      urdf << "\t<collision>" << std::endl;
      urdf << "\t\t<origin rpy=\"0 0 0\" xyz=\"" << space->at(i).center.x << " "
          << space->at(i).center.y << " " << space->at(i).center.z << "\"/>"
          << std::endl;
      urdf << "\t\t<geometry>" << std::endl;
      urdf << "\t\t\t<box size=\"" << space->resolution_ << " "
          << space->resolution_ << " " << space->resolution_ << "\"/>"
          << std::endl;
      urdf << "\t\t</geometry>" << std::endl;
      urdf << "\t</collision>" << std::endl;
      urdf << "</link>" << std::endl;
    }
    for (int i = 0; i < space->space_size_; i++)
    {
      urdf << "<joint name=\"joint_" << i << "\" type=\"fixed\">" << std::endl;
      urdf << "\t<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" << std::endl;
      urdf << "\t<parent link=\"base\"/>" << std::endl;
      urdf << "\t<child link=\"cell_" << i << "\"/>" << std::endl;
      urdf << "</joint>" << std::endl;
    }
    urdf << "</robot>" << std::endl;
    return true;
  }
}

