/*
 * Software License Agreement (BSD License)
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

#include <openni_camera/openni_driver.h>
#include "openni_audio/audio_device.h"
#include <iostream>
#include <limits>
#include <sstream>
#include <map>
#include <vector>

using namespace std;
using namespace boost;

namespace openni_audio_wrapper
{

OpenNIAudioDevice::OpenNIAudioDevice (xn::Context& context, const xn::NodeInfo& device_node, const xn::NodeInfo& audio_node) throw (OpenNIException)
  : context_ (context)
  , device_node_info_ (device_node)
{
  // create the production nodes
  XnStatus status = context_.CreateProductionTree (const_cast<xn::NodeInfo&>(audio_node));
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating audio generator failed. Reason: %s", xnGetStatusString (status));

  // get production node instances
  status = audio_node.GetInstance (audio_generator_);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("creating audio generator instance failed. Reason: %s", xnGetStatusString (status));

  audio_generator_.RegisterToNewDataAvailable ((xn::StateChangedHandler)NewAudioDataAvailable, this, audio_callback_handle_);

  Init ();
}

// For ONI Player devices
OpenNIAudioDevice::OpenNIAudioDevice (xn::Context& context) throw (OpenNIException)
  : context_ (context)
  , device_node_info_ (0)
{
}

OpenNIAudioDevice::~OpenNIAudioDevice () throw ()
{
  shutdown ();
}

void OpenNIAudioDevice::shutdown ()
{
  {
    // Lock out all the XxxDataThreadFunction threads
    boost::lock_guard<boost::mutex> audio_guard(audio_mutex_);
    
    // Stop streams
    if (audio_generator_.IsValid() && audio_generator_.IsGenerating ())
      audio_generator_.StopGenerating ();

    // On wakeup, each data thread will check quit_ and exit
    quit_ = true;
  }

  // Wake up, time to die
  audio_condition_.notify_all ();
  data_threads_.join_all ();
}

void OpenNIAudioDevice::Init () throw (OpenNIException)
{
  quit_ = false;

  if (hasAudioStream ())
  {
    unique_lock<mutex> audio_lock (audio_mutex_);
    data_threads_.create_thread (boost::bind (&OpenNIAudioDevice::AudioDataThreadFunction, this));
  }
}

void OpenNIAudioDevice::startAudioStream () throw (OpenNIException)
{
  if (hasAudioStream ())
  {
    lock_guard<mutex> audio_lock (audio_mutex_);
    if (!audio_generator_.IsGenerating ())
    {    
      XnStatus status = audio_generator_.StartGenerating ();
      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("starting audio stream failed. Reason: %s", xnGetStatusString (status));
    }
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide an audio stream");
}

void OpenNIAudioDevice::stopAudioStream () throw (OpenNIException)
{
  if (hasAudioStream ())
  {
    lock_guard<mutex> audio_lock (audio_mutex_);
    if (audio_generator_.IsGenerating ())
    {
      XnStatus status = audio_generator_.StopGenerating ();
      if (status != XN_STATUS_OK)
        THROW_OPENNI_EXCEPTION ("stopping audio stream failed. Reason: %s", xnGetStatusString (status));
    }
  }
  else
    THROW_OPENNI_EXCEPTION ("Device does not provide an audio stream");
}


bool OpenNIAudioDevice::isAudioStreamRunning () const throw (OpenNIException)
{
  lock_guard<mutex> audio_lock (audio_mutex_);
  return ( audio_generator_.IsValid () && audio_generator_.IsGenerating ());
}

bool OpenNIAudioDevice::hasAudioStream () const throw ()
{
  lock_guard<mutex> lock (audio_mutex_);
  return audio_generator_.IsValid ();
}

/*bool OpenNIAudioDevice::setChannel(unsigned channel) throw()
{
  if (channel > device_context_[index].audio_nodes.size()){
      THROW_OPENNI_EXCEPTION ("audio channel does not exist.");
    }
    device->audio_generator_ = device_context_[index].audio_nodes[channel];
}*/

void OpenNIAudioDevice::AudioDataThreadFunction () throw (OpenNIException)
{
  while (true)
  {
    // lock before checking running flag
    unique_lock<mutex> audio_lock (audio_mutex_);
    if (quit_)
      return;
    audio_condition_.wait (audio_lock);
    if (quit_)
      return;

    audio_generator_.WaitAndUpdateData ();
    boost::shared_ptr<xn::AudioMetaData> audio_data (new xn::AudioMetaData);
    audio_generator_.GetMetaData (*audio_data);
    audio_lock.unlock ();

    boost::shared_ptr<AudioBuffer> data (new AudioBuffer);
    data->resize(audio_data->DataSize());
    memcpy( &(*data)[0], audio_data->Data(), audio_data->DataSize());
    
    for (map< OpenNIAudioDevice::CallbackHandle, ActualAudioCallbackFunction >::iterator callbackIt = audio_callback_.begin (); callbackIt != audio_callback_.end (); ++callbackIt)
    {
      callbackIt->second.operator()(data);
    }
  }
}

void __stdcall OpenNIAudioDevice::NewAudioDataAvailable (xn::ProductionNode& node, void* cookie) throw ()
{
  OpenNIAudioDevice* device = reinterpret_cast<OpenNIAudioDevice*>(cookie);
  device->audio_condition_.notify_all ();
}

OpenNIAudioDevice::CallbackHandle OpenNIAudioDevice::registerAudioCallback (const AudioCallbackFunction& callback, void* custom_data) throw ()
{
  if (!hasAudioStream ())
    THROW_OPENNI_EXCEPTION ("Device does not provide an audio stream");

  audio_callback_[audio_callback_handle_counter_] = boost::bind (callback, _1, custom_data);
  return audio_callback_handle_counter_++;
}

bool OpenNIAudioDevice::unregisterAudioCallback (const OpenNIAudioDevice::CallbackHandle& callbackHandle) throw ()
{
  if (!hasAudioStream ())
    THROW_OPENNI_EXCEPTION ("Device does not provide an audio stream");

  return (audio_callback_.erase (callbackHandle) != 0);
}

const char* OpenNIAudioDevice::getSerialNumber () const throw ()
{
  return device_node_info_.GetInstanceName ();
}

const char* OpenNIAudioDevice::getConnectionString () const throw ()
{
  return device_node_info_.GetCreationInfo ();
}

unsigned short OpenNIAudioDevice::getVendorID () const throw ()
{
  unsigned short vendor_id;
  unsigned short product_id;
  
#ifndef _WIN32
  unsigned char bus;
  unsigned char address;
  
  sscanf (device_node_info_.GetCreationInfo(), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);

#else
  OpenNIDriver::getDeviceType (device_node_info_.GetCreationInfo(), vendor_id, product_id);
#endif
  return vendor_id;
}

unsigned short OpenNIAudioDevice::getProductID () const throw ()
{
  unsigned short vendor_id;
  unsigned short product_id;
#ifndef _WIN32
  unsigned char bus;
  unsigned char address;
  sscanf (device_node_info_.GetCreationInfo(), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);

#else
  OpenNIDriver::getDeviceType (device_node_info_.GetCreationInfo(), vendor_id, product_id);
#endif
  return product_id;
}

unsigned char OpenNIAudioDevice::getBus () const throw ()
{
  unsigned char bus = 0;
#ifndef _WIN32
  unsigned short vendor_id;
  unsigned short product_id;
  unsigned char address;
  sscanf (device_node_info_.GetCreationInfo(), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);
#endif
  return bus;
}

unsigned char OpenNIAudioDevice::getAddress () const throw ()
{
  unsigned char address = 0;
#ifndef _WIN32
  unsigned short vendor_id;
  unsigned short product_id;
  unsigned char bus;
  sscanf (device_node_info_.GetCreationInfo(), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);
#endif
  return address;
}

const char* OpenNIAudioDevice::getVendorName () const throw ()
{
  XnProductionNodeDescription& description = const_cast<XnProductionNodeDescription&>(device_node_info_.GetDescription ());
  return description.strVendor;
}

const char* OpenNIAudioDevice::getProductName () const throw ()
{
  XnProductionNodeDescription& description = const_cast<XnProductionNodeDescription&>(device_node_info_.GetDescription ());
  return description.strName;
}

} // namespace
