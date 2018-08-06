#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <stdio.h>

#include <thread>
#include <mutex>
#include <condition_variable>

#include <math.h>
#include <fftw3.h>
#include <complex>

#include <alsa/asoundlib.h>

#include <eigen3/Eigen/Dense>

#include <vib_touch_msgs/VibArray.h>
#include <vib_touch_msgs/Spectrum.h>

using namespace std;
using namespace Eigen;

#include <alsa_haptic/alsa_haptic.hpp>

namespace vib_touch_sensor_ns
{
class VibTouchSensor : public nodelet::Nodelet
{
protected:
  string inputDeviceName;
  double inputGain;
  int numVib;
  int freqCap, freqVib;
  int nStepIn;
  int nPeriodsIn;
  snd_pcm_uframes_t nAlsaBufferIn;

  long unsigned int periodSizeIn;
  long unsigned int bufferSizeIn;
  int periodSizeDir;
  long unsigned int startThresholdIn;
 
  AlsaHaptic alsaIn;
 
  vib_touch_msgs::VibArray vibarrayMsgRaw;
 
  ros::Publisher vibarrayPubRaw;
 
  float sensorGain;

  ros::Time stamp; 
  int seq = 0;

  /// dummy vib data
  double theta[5] = {};
  int freq[5] = {20, 30, 50, 60, 70};
  int amp[5] = {400000, 200000, 300000, 200000, 300000};

  std::thread adThread;

  // Initialize ALSA hardware
  void initialize_input()
  {
    alsaIn.open(inputDeviceName.c_str(), SND_PCM_STREAM_CAPTURE); // SND_PCM_STREAM_CAPTURE or SND_PCM_STREAM_PLAYBACK
    
    alsaIn.hwopen();
    alsaIn.sethw(snd_pcm_hw_params_set_access, SND_PCM_ACCESS_RW_INTERLEAVED);
    alsaIn.sethw(snd_pcm_hw_params_set_format, SND_PCM_FORMAT_S24_LE);
    alsaIn.sethw(snd_pcm_hw_params_set_rate, freqVib, 0);
    alsaIn.sethw(snd_pcm_hw_params_set_channels, numVib); 
    alsaIn.sethw(snd_pcm_hw_params_set_periods, nPeriodsIn, 0);
    alsaIn.sethw(snd_pcm_hw_params_set_buffer_size_near, &nAlsaBufferIn);
    //alsaIn.gethw(snd_pcm_hw_params_get_period_size, &periodSizeIn, &periodSizeDir);
    //alsaIn.gethw(snd_pcm_hw_params_get_buffer_size, &bufferSizeIn);
    alsaIn.hwclose();
    alsaIn.prepare();
  }

public:
    VibTouchSensor(){
    }
    ~VibTouchSensor(){
    }

    virtual void onInit(){

      // initialize ros node
      ros::NodeHandle n = getNodeHandle();
      ros::NodeHandle& private_n = getPrivateNodeHandle();

      // get parameter values from launch file
      private_n.getParam("input_device_name", inputDeviceName); // audio interface
      private_n.getParam("vib_frequency", freqVib); // frequency of vibdata
      private_n.getParam("cap_frequency", freqCap); // frequency of capture
      private_n.getParam("num_vib", numVib); // 1: monaural input
      private_n.getParam("input_gain", inputGain);
      
      // ALSA input
      ros::param::param<int>("~n_step_in_for_ALSA", nStepIn, freqVib/freqCap);
      ros::param::param<int>("~n_periods_in", nPeriodsIn, 5);
      int tmpIn;
      ros::param::param<int>("~n_alsa_buffer_in", tmpIn, nStepIn*nPeriodsIn);
      nAlsaBufferIn = tmpIn;

      // ROS topic
      vibarrayPubRaw = n.advertise<vib_touch_msgs::VibArray>("vibRaw", 1);
      
      vibarrayMsgRaw.header.frame_id = "vibRaw";
      vibarrayMsgRaw.nch = numVib;
      vibarrayMsgRaw.len = nStepIn;
      vibarrayMsgRaw.vib.resize(vibarrayMsgRaw.len * vibarrayMsgRaw.nch);

      initialize_input();

      // thread for recording       
      adThread = std::thread( 
        [&]() {
          NODELET_INFO_STREAM("\n### AD LOOP START ###\n");
          MatrixXi buf(numVib, nStepIn); // temporary buffer

          while (ros::ok()) {
            
            alsaIn.read(buf.block(0, 0, numVib, nStepIn), numVib*nStepIn); //record one sample (size:numVib*nStepIn) 
            stamp = ros::Time::now();
            seq++;
            
            for (int j = 0; j<nStepIn; j++){
              for (int i = 0; i<numVib; i++){
                
                /// dummy sine data for Debug
                /*
                vibarrayMsgRaw.vib[j*numVib+i] = amp[0]*sin(theta[0]);
                for (int k=1;k<5;k++){
                  vibarrayMsgRaw.vib[j*numVib+i] += amp[k]*sin(theta[k]);
                }
                */
                
                vibarrayMsgRaw.vib[j*numVib+i] = buf(i,j)*inputGain; // 
              }
              /*
              for (int k=0;k<5;k++){
                theta[k] = theta[k] + 2*M_PI*freq[k]/freqVib;
              }
              */
            }
            vibarrayMsgRaw.header.seq = seq; // count up 
            vibarrayMsgRaw.header.stamp = stamp; //time 
            vibarrayPubRaw.publish(vibarrayMsgRaw);

          }
        }
      );
    }
};
} // namespace vib_touch_snesor_ns

PLUGINLIB_EXPORT_CLASS(vib_touch_sensor_ns::VibTouchSensor, nodelet::Nodelet)
