/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012 Willow Garage, Inc.
 *
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

#ifndef __AUDIO_DEVICE_H__
#define __AUDIO_DEVICE_H__

#include <vector>
#include <openni_camera/openni_exception.h>
#include <XnCppWrapper.h>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

using namespace openni_wrapper;

namespace openni_audio_wrapper
{

typedef std::vector<unsigned char> AudioBuffer;

/**
 * @brief Class representing an audio device for Primesense devices.
 */
class OpenNIAudioDevice : public boost::noncopyable
{
public:
  typedef boost::function<void(boost::shared_ptr<AudioBuffer>, void* cookie) > AudioCallbackFunction;
  typedef unsigned CallbackHandle;

public:
  virtual ~OpenNIAudioDevice () throw ();
  void shutdown ();

  virtual void startAudioStream () throw (OpenNIException);
  virtual void stopAudioStream () throw (OpenNIException);

  bool hasAudioStream () const throw ();
  //bool setChannel(unsigned channel) throw ();

  virtual bool isAudioStreamRunning () const throw (OpenNIException);

  CallbackHandle registerAudioCallback (const AudioCallbackFunction& callback, void* cookie = NULL) throw ();
  template<typename T> CallbackHandle registerAudioCallback (void (T::*callback)(boost::shared_ptr<AudioBuffer>, void* cookie), T& instance, void* cookie = NULL) throw ();
  bool unregisterAudioCallback (const CallbackHandle& callbackHandle) throw ();

  /** \brief returns the serial number for device.
   *  \attention This might be an empty string!!!
   */
  const char* getSerialNumber () const throw ();
  /** \brief returns the connectionstring for current device, which has following format vendorID/productID\@BusID/DeviceID */
  const char* getConnectionString () const throw ();

  const char* getVendorName () const throw ();
  const char* getProductName () const throw ();
  unsigned short getVendorID () const throw ();
  unsigned short getProductID () const throw ();
  unsigned char  getBus () const throw ();
  unsigned char  getAddress () const throw ();
  /** \brief Audio generator object. */
  xn::AudioGenerator audio_generator_;

  OpenNIAudioDevice (xn::Context& context, const xn::NodeInfo& device_node, const xn::NodeInfo& audio_node) throw (OpenNIException); // was protected, but is no longer a derived class
protected:
  typedef boost::function<void(boost::shared_ptr<AudioBuffer>) > ActualAudioCallbackFunction;

  OpenNIAudioDevice (xn::Context& context) throw (OpenNIException);
  static void /*__stdcall*/ NewAudioDataAvailable (xn::ProductionNode& node, void* cookie) throw ();

  // This is a workaround, since in the NewDepthDataAvailable function WaitAndUpdateData leads to a dead-lock behaviour
  // and retrieving image data without WaitAndUpdateData leads to incomplete images!!!
  void AudioDataThreadFunction () throw (OpenNIException);

  //virtual boost::shared_ptr<Image> getCurrentImage (boost::shared_ptr<xn::ImageMetaData> image_data) const throw () = 0;

  void Init () throw (OpenNIException); 
  // holds the callback functions together with custom data
  // since same callback function can be registered multiple times with e.g. different custom data
  // we use a map structure with a handle as the key
  std::map< CallbackHandle, ActualAudioCallbackFunction > audio_callback_;

  /** \brief context to OpenNI driver*/
  xn::Context& context_;
  /** \brief node object for current device */
  xn::NodeInfo device_node_info_;

  XnCallbackHandle audio_callback_handle_;

  OpenNIAudioDevice::CallbackHandle audio_callback_handle_counter_;

  bool quit_;
  mutable boost::mutex audio_mutex_;
  boost::condition_variable audio_condition_;
  boost::thread_group data_threads_;
};

template<typename T> OpenNIAudioDevice::CallbackHandle OpenNIAudioDevice::registerAudioCallback (void (T::*callback)(boost::shared_ptr<AudioBuffer>, void* cookie), T& instance, void* custom_data) throw ()
{
  audio_callback_[audio_callback_handle_counter_] = boost::bind (callback, boost::ref (instance), _1, custom_data);
  return audio_callback_handle_counter_++;
}

}
#endif // __AUDIO_DEVICE_H__
