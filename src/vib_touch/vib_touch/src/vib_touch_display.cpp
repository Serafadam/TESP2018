#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <stdio.h>

#include <thread>
#include <mutex>
#include <condition_variable>

#include <math.h>

#include <alsa/asoundlib.h>

#include <eigen3/Eigen/Dense>

#include <vib_touch_msgs/VibArray.h>

using namespace std;
using namespace Eigen;

#include <alsa_haptic/alsa_haptic.hpp>

namespace vib_touch_display_ns
{
class VibTouchDisplay : public nodelet::Nodelet
{
protected:
  
  int numVib;
  int freqCap, freqVib;

  AlsaHaptic alsaOut;
  string outputDeviceName;
  int nStepOut;
  int nPeriodsOut;
  snd_pcm_uframes_t nAlsaBufferOut;
  long unsigned int periodSizeOut;
  long unsigned int bufferSizeOut;
  int periodSizeDir;
  long unsigned int startThresholdOut;

  int vibCnt = 0;

  double outputGain = 0.01;  
  double GUIGain = 1.0;
  double AMGain = 1.0;
  int vibSubRawQueue = 4;
  
  int carrierFreq = 300;
  
  vib_touch_msgs::VibArray vibarrayMsgAM;
  
  ros::Publisher vibarrayPubAM;
  
  ros::Subscriber vibarraySubRaw;
  
  MatrixXf vibRaw, vibAM, vibBufAM;
  MatrixXf upEnv, lowEnv;
  
  double amp, offset;
  double theta = 0.0;
  
  int outputMode = 0;
  MatrixXi frame;
  

  void initializeOutput()
	{
		// initialize snd_pcm_hw  
		alsaOut.open(outputDeviceName.c_str(), SND_PCM_STREAM_PLAYBACK);
		alsaOut.hwopen();
		alsaOut.sethw(snd_pcm_hw_params_set_access, SND_PCM_ACCESS_RW_INTERLEAVED);
		alsaOut.sethw(snd_pcm_hw_params_set_format, SND_PCM_FORMAT_S16_LE);
		alsaOut.sethw(snd_pcm_hw_params_set_rate, freqVib, 0);
		alsaOut.sethw(snd_pcm_hw_params_set_channels, 2);
		alsaOut.sethw(snd_pcm_hw_params_set_periods, nPeriodsOut, 0);
		alsaOut.sethw(snd_pcm_hw_params_set_buffer_size_near, &nAlsaBufferOut);
		alsaOut.gethw(snd_pcm_hw_params_get_period_size, &periodSizeOut, &periodSizeDir);
		alsaOut.gethw(snd_pcm_hw_params_get_buffer_size, &bufferSizeOut);
		alsaOut.hwclose();
		//alsaOut.swopen();
		//alsaOut.setsw(snd_pcm_sw_params_set_start_threshold, bufferSizeOut - periodSizeOut);
		//alsaOut.swclose();
		//alsaOut.getsw(snd_pcm_sw_params_get_start_threshold, &startThresholdOut);
		alsaOut.prepare();
	}

public:
    VibTouchDisplay(){
    }
    ~VibTouchDisplay(){
    }

    virtual void onInit(){

      // initialize ros node
      ros::NodeHandle n = getNodeHandle();
      ros::NodeHandle& private_n = getPrivateNodeHandle();

      private_n.getParam("output_device_name", outputDeviceName);
      private_n.getParam("vib_frequency", freqVib);
      private_n.getParam("cap_frequency", freqCap);
      private_n.getParam("num_vib", numVib);
     
      private_n.getParam("output_gain", outputGain);
      
      private_n.getParam("output_mode", outputMode);
      
      ///// ALSA output
      ros::param::param<int>("~n_step_out_for_ALSA", nStepOut,  freqVib/freqCap);
      ros::param::param<int>("~n_periods_out", nPeriodsOut, 5);
      int tmpOut;
      ros::param::param<int>("~n_alsa_buffer_out", tmpOut, nStepOut*3);
      nAlsaBufferOut = tmpOut;
      //////

      vibarrayPubAM = n.advertise<vib_touch_msgs::VibArray>("vibAM", 1);
        
      // print output parameters
      NODELET_INFO_STREAM("### OUTPUT DEVICE ###");
      NODELET_INFO_STREAM("Output device name: "       << outputDeviceName);
      NODELET_INFO_STREAM("Step length for output: "       << nStepOut);
      NODELET_INFO_STREAM("# of periods for output: "      << nPeriodsOut);
      NODELET_INFO_STREAM("# of alsa buffer for output: " << nAlsaBufferOut);
  
      vibarrayMsgAM.header.frame_id = "vibAM";
      vibarrayMsgAM.nch = numVib;
      vibarrayMsgAM.len = nStepOut;
      vibarrayMsgAM.vib.resize(vibarrayMsgAM.len * vibarrayMsgAM.nch);

      vibRaw = MatrixXf::Zero(numVib, nStepOut); 
      vibAM = MatrixXf::Zero(numVib, nStepOut);
      vibBufAM = MatrixXf::Zero(numVib, nStepOut*2); 

      initializeOutput();
      
      vibarraySubRaw = n.subscribe("vibRaw", vibSubRawQueue, &VibTouchDisplay::vibSubCallback, this);
      
    }

    void vibSubCallback(const vib_touch_msgs::VibArray::ConstPtr &vibarrayRaw)
    {      
      if(nStepOut != vibarrayRaw->len)NODELET_ERROR_STREAM("Data size is different"); 
      
      vibBufAM.block(0, 0, numVib, nStepOut) = vibBufAM.block(0, nStepOut, numVib, nStepOut);
      
      for (int i=0;i<numVib;i++)
        for (int j=0;j<nStepOut;j++){
        {
          vibRaw(i,j) = vibarrayRaw->vib[j*numVib+i];
        }
      }

      vibBufAM.block(0, nStepOut, numVib, nStepOut) = vibRaw.block(0, 0, numVib, nStepOut);

      upEnv = MatrixXf::Zero(numVib, nStepOut*2);
      lowEnv = MatrixXf::Zero(numVib, nStepOut*2);

      int upCntPre[numVib] = {};
      int upCntTmp[numVib] = {};
      int lowCntPre[numVib] = {};
      int lowCntTmp[numVib] = {};
      
      for (int j=0; j< numVib; j++){
        for (int i=1; i< nStepOut*2-1; i++){
          if (vibBufAM(j,i) > vibBufAM(j,i-1) && vibBufAM(j,i) > vibBufAM(j,i+1)){
            upEnv(j,i) = vibBufAM(j,i);
            upCntTmp[j] = i;
            if((upCntPre[j]-upCntTmp[j])!=1){
              for (int upCnt = upCntPre[j]+1; upCnt<upCntTmp[j]; upCnt++){
                upEnv(j,upCnt) = (vibBufAM(j,upCntPre[j])*(upCntTmp[j]-upCnt) + vibBufAM(j,upCntTmp[j])*(upCnt-upCntPre[j]) ) / (upCntTmp[j]-upCntPre[j]);
              }
            }
            upCntPre[j] = i;
          }else if (vibBufAM(j,i) < vibBufAM(j,i-1) && vibBufAM(j,i) < vibBufAM(j,i+1)){
            lowEnv(j,i) = vibBufAM(j,i);
            lowCntTmp[j] = i;
            if((lowCntPre[j]-lowCntTmp[j])!=1){
              for (int lowCnt = lowCntPre[j]+1; lowCnt<lowCntTmp[j]; lowCnt++){
                lowEnv(j,lowCnt) = (vibBufAM(j,lowCntPre[j])*(lowCntTmp[j]-lowCnt) + vibBufAM(j,lowCntTmp[j])*(lowCnt-lowCntPre[j]) ) / (lowCntTmp[j]-lowCntPre[j]);
              }
            }
            lowCntPre[j] = i;
          }
        }
        for(int i2=upCntTmp[j]+1; i2<nStepOut*2; i2++){
          upEnv(j,i2) = vibBufAM(j,upCntTmp[j]);
        }
        for(int i3=lowCntTmp[j]+1; i3<nStepOut*2; i3++){
          lowEnv(j,i3) = vibBufAM(j,lowCntTmp[j]);
        }
      }
      
      // output is AM signal
      for (int i=0; i< nStepOut; i++){
        for (int j=0; j<numVib; j++){
          amp = (upEnv(j,nStepOut/2+i) - lowEnv(j,nStepOut/2+i)) / 2;
          offset = (upEnv(j,nStepOut/2+i) + lowEnv(j,nStepOut/2+i)) / 2;
          vibAM(j,i) = amp * sin(theta) + offset;
          
        }  
        theta = theta + 2*M_PI*carrierFreq/freqVib;
      }


      vibAM = vibAM*AMGain;
      for (int i=0; i<numVib; i++){
        for (int j=0; j< nStepOut; j++){
          vibarrayMsgAM.vib[i+numVib*j] = GUIGain * vibAM(i,j);
        }
      }
      vibarrayMsgAM.header.seq   = vibarrayRaw->header.seq;
      vibarrayMsgAM.header.stamp = vibarrayRaw->header.stamp;

      vibAM = vibAM*outputGain;
      vibRaw = vibRaw*outputGain;
      
      //// threshold ////
      frame = MatrixXi::Zero(1, nStepOut);
      for (int i=0; i<nStepOut; i++){
        int left_vib;
        int right_vib;
        
        if (outputMode == 0){
          left_vib = static_cast<int>(vibAM(0, i));
          right_vib = static_cast<int>(vibRaw(0, i));
        }else{
          left_vib = static_cast<int>(vibRaw(0, i));
          right_vib = static_cast<int>(vibAM(0, i));
        }

        if ( (-10 < left_vib) && (left_vib < 10) ) left_vib=0;
        if ( (-10 < right_vib) && (right_vib < 10) ) right_vib=0;
        left_vib = left_vib * 2;
        right_vib = right_vib * 2;

        if ( (static_cast<int>(vibAM(0, i) < -32768)) || (static_cast<int>(vibAM(0, i) > 32768))) ROS_ERROR_STREAM("\n // Saturated // \n");
        
        frame(0, i) = ((right_vib << 16)&0xFFFF0000) | (left_vib&0x0000FFFF) ; // -32768 ～ 32767
      }
      vibarrayPubAM.publish(vibarrayMsgAM);
      alsaOut.write(frame, nStepOut);
      
      
    }

};
} // namespace vib_touch_display_ns

PLUGINLIB_EXPORT_CLASS(vib_touch_display_ns::VibTouchDisplay, nodelet::Nodelet)
