/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "visensor.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace visensor {

class ViSensorNodelet : public nodelet::Nodelet {
  public:
    virtual void onInit();
  protected:
    boost::shared_ptr<ViSensor> vi_sensor;
};

void ViSensorNodelet::onInit()
{

  vi_sensor = boost::make_shared<ViSensor>(getName(), getNodeHandle(), getPrivateNodeHandle());

  //Initalize params from ROS or set to default value
  // TODO: do this with private NodeHandle, but kept this for backwards compatibility
  ros::NodeHandle nh;
  int cam_rate;
  int imu_rate;
  nh.param("imuRate", imu_rate, IMU_FREQUENCY);
  nh.param("camRate", cam_rate, CAMERA_FREQUENCY);
  vi_sensor->startSensors(cam_rate, imu_rate);
}

PLUGINLIB_DECLARE_CLASS(visensor_node, ViSensorNodelet, visensor::ViSensorNodelet, nodelet::Nodelet);

}
