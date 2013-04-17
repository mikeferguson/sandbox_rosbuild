/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012 Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.h>
#include "audio_nodelet.h"

using namespace std;
using namespace openni_audio_wrapper;
namespace openni_audio
{

PLUGINLIB_DECLARE_CLASS (openni_audio, AudioNodelet, openni_audio::AudioNodelet, nodelet::Nodelet);

AudioNodelet::~AudioNodelet ()
{
//  device_->stopAudioStream ();
}

void AudioNodelet::onInit ()
{
  // NOTE: This version (mistakenly) creates a new NodeHandle independent of nodelet::getNodeHandle().
  // It uses the default ROS callback queue instead of the nodelet manager's callback queue. On the
  // bright side, this happened to avoid various threading issues.
  ros::NodeHandle comm_nh(getNodeHandle ().resolveName ("audio_in")); // for topics, services
  ros::NodeHandle param_nh = getPrivateNodeHandle (); // for parameters

  //updateModeMaps ();      // registering mapping from config modes to XnModes and vice versa
  device_ = boost::shared_ptr<openni_audio_wrapper::OpenNIAudioDevice > ((openni_audio_wrapper::OpenNIAudioDevice*)NULL);
  setupDevice (param_nh); // will change config_ to default values or user given values from param server

  ros::SubscriberStatusCallback subscriberChanged = boost::bind(&AudioNodelet::subscriberChangedEvent, this);
  pub_audio_ = comm_nh.advertise<audio_common_msgs::AudioData> ("data", 5, subscriberChanged, subscriberChanged);
}

void AudioNodelet::setupDevice (ros::NodeHandle& param_nh)
{
  // Initialize the openni device
  OpenNIAudioDriver& driver = OpenNIAudioDriver::getInstance ();

  string device_id;
  int channel;
  do {
    // Don't require sigkill when waiting for device.
    if (!ros::ok())
      exit(-1);

    driver.updateDeviceList ();

    if (driver.getNumberDevices () == 0)
    {
      NODELET_INFO ("[%s] No devices connected.... waiting for devices to be connected", getName ().c_str ());
      sleep(1);
      continue;
    }

    NODELET_INFO ("[%s] Number devices connected: %d", getName ().c_str (), driver.getNumberDevices ());
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      NODELET_INFO ("[%s] %u. device on bus %03u:%02u is a %s (%03x) from %s (%03x) with serial id \'%s\'"
                , getName ().c_str (), deviceIdx + 1, driver.getBus (deviceIdx), driver.getAddress (deviceIdx)
                , driver.getProductName (deviceIdx), driver.getProductID (deviceIdx), driver.getVendorName (deviceIdx)
                , driver.getVendorID (deviceIdx), driver.getSerialNumber (deviceIdx));
    }

    param_nh.param ("device_id", device_id, std::string ());
    param_nh.param ("channel", channel, 0);

    try {
      if (device_id.empty ())
      {
        NODELET_WARN ("[%s] device_id is not set! Using first device.", getName ().c_str ());
        device_ = driver.getDeviceByIndex (0, channel);
      }
      else if (device_id.find ('@') != string::npos)
      {
        size_t pos = device_id.find ('@');
        unsigned bus = atoi (device_id.substr (0, pos).c_str ());
        unsigned address = atoi (device_id.substr (pos + 1, device_id.length () - pos - 1).c_str ());
        NODELET_INFO ("[%s] searching for device with bus@address = %d@%d", getName ().c_str (), bus, address);
        device_ = driver.getDeviceByAddress (bus, address, channel);
      }
      else if (device_id[0] == '#')
      {
        unsigned index = atoi (device_id.c_str () + 1);
        NODELET_INFO ("[%s] searching for device with index = %d", getName ().c_str (), index);
        device_ = driver.getDeviceByIndex (index - 1, channel);
      }
      else
      {
        NODELET_INFO ("[%s] searching for device with serial number = %s", getName ().c_str (), device_id.c_str ());
        device_ = driver.getDeviceBySerialNumber (device_id, channel);
      }
    }
    catch (const OpenNIException& exception)
    {
      if (!device_)
      {
        NODELET_INFO ("[%s] No matching device found.... waiting for devices. Reason: %s", getName ().c_str (), exception.what ());
        sleep (1);
        continue;
      }
      else
      {
        NODELET_ERROR ("[%s] could not retrieve device. Reason %s", getName ().c_str (), exception.what ());
        exit (-1);
      }
    }
  } while (!device_);

  NODELET_INFO ("[%s] Opened '%s' on bus %d:%d with serial number '%s'", getName ().c_str (),
            device_->getProductName (), device_->getBus (), device_->getAddress (), device_->getSerialNumber ());

  boost::shared_ptr<xn::AudioMetaData> audio_data (new xn::AudioMetaData);
  device_->audio_generator_.GetMetaData (*audio_data);
  NODELET_INFO ("[%s] Opened channel %d, with %d hz sample rate and %d bits per sample.", getName().c_str (), channel, audio_data->SampleRate(), audio_data->BitsPerSample());

  device_->registerAudioCallback (&AudioNodelet::audioCallback, *this);
}

void AudioNodelet::audioCallback (boost::shared_ptr<openni_audio_wrapper::AudioBuffer> buffer, void* cookie)
{
  audio_common_msgs::AudioData msg;
  msg.data.resize( buffer->size() );
  memcpy( &msg.data[0], &((*buffer)[0]), buffer->size());
  pub_audio_.publish(msg);
}

void AudioNodelet::subscriberChangedEvent ()
{
  if (pub_audio_.getNumSubscribers() > 0 && !device_->isAudioStreamRunning ())
  {
    device_->startAudioStream ();
  }
  else if (pub_audio_.getNumSubscribers() == 0 && device_->isAudioStreamRunning ())
  {
    device_->stopAudioStream ();
  }
}

}
