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
#ifndef OPENNI_AUDIO_NODELET_H_
#define OPENNI_AUDIO_NODELET_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "openni_audio/audio_driver.h"
#include "audio_common_msgs/AudioData.h"
#include <boost/shared_ptr.hpp>
#include <string>

namespace openni_audio
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  class AudioNodelet : public nodelet::Nodelet
  {
    public:
      virtual ~AudioNodelet ();
    private:
      /** \brief Nodelet initialization routine. */
      virtual void onInit ();
      void setupDevice (ros::NodeHandle& param_nh);

      // callback methods
      void audioCallback (boost::shared_ptr<openni_audio_wrapper::AudioBuffer> image, void* cookie);
      void subscriberChangedEvent ();

      ros::Publisher pub_audio_;

      /** \brief the actual openni device*/
      boost::shared_ptr<openni_audio_wrapper::OpenNIAudioDevice> device_;
  };

}

#endif  //#ifndef OPENNI_AUDIO_NODELET_H_
