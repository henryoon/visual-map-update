/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */

#pragma once

#include <pcl/pcl_config.h>
#include <pcl/memory.h>
#ifdef HAVE_OPENNI

#include "openni_device.h"
#include "openni_driver.h"
#include "openni_image_bayer_grbg.h"

namespace openni_wrapper
{

  /**
   * @brief Concrete implementation of the interface OpenNIDevice for a MS Kinect device.
   * @author Suat Gedikli
   * @date 02.january 2011
   * @ingroup io
   */
  class DeviceKinect : public OpenNIDevice
  {
    friend class OpenNIDriver;
  public:
    DeviceKinect (xn::Context& context, const xn::NodeInfo& device_node, const xn::NodeInfo& image_node, const xn::NodeInfo& depth_node, const xn::NodeInfo& ir_node);
    ~DeviceKinect () noexcept override;

    inline void setDebayeringMethod (const ImageBayerGRBG::DebayeringMethod& debayering_method) noexcept;
    inline const ImageBayerGRBG::DebayeringMethod& getDebayeringMethod () const noexcept;

    bool isSynchronizationSupported () const noexcept override;

  protected:
    Image::Ptr getCurrentImage (pcl::shared_ptr<xn::ImageMetaData> image_meta_data) const noexcept override;
    void enumAvailableModes () noexcept;
    bool isImageResizeSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const noexcept override;
    ImageBayerGRBG::DebayeringMethod debayering_method_{ImageBayerGRBG::EdgeAwareWeighted};
  } ;

  void
  DeviceKinect::setDebayeringMethod (const ImageBayerGRBG::DebayeringMethod& debayering_method) noexcept
  {
    debayering_method_ = debayering_method;
  }

  const ImageBayerGRBG::DebayeringMethod&
  DeviceKinect::getDebayeringMethod () const noexcept
  {
    return debayering_method_;
  }
} // namespace

#endif
