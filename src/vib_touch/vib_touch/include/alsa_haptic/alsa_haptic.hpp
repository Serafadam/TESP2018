class AlsaHaptic
{
  snd_pcm_t *handle;
  snd_pcm_hw_params_t *hw_params;
  snd_pcm_sw_params_t *sw_params;

  int error(int err)
  {
    
    if (err < 0) {
      string str_error = snd_strerror(err);
      ROS_ERROR_STREAM("[ASLA ERROR]: " << str_error << " (Code: " << err << ")");
      //snd_pcm_recover(handle, err, 0));
    }

    assert(err >= 0);

    return err;
  }

public:
  AlsaHaptic() {}

  void open(string dev_name, snd_pcm_stream_t mode)
  {
    error(snd_pcm_open(&handle, dev_name.c_str(), mode, 0));    
  }

  void hwopen()
  {
    error(snd_pcm_hw_params_malloc(&hw_params));
    error(snd_pcm_hw_params_any(handle, hw_params));
  }
  void hwclose()
  {
    error(snd_pcm_hw_params(handle, hw_params));  
  }
  
  void swopen()
  {
    error(snd_pcm_sw_params_malloc(&sw_params));
    error(snd_pcm_sw_params_current(handle, sw_params));
  }
  void swclose()
  {
    error(snd_pcm_sw_params(handle, sw_params));
  }

  template<typename Op, typename Arg1>
  int sethw(Op op, Arg1 arg1) { return error(op(handle, hw_params, arg1)); }

  template<typename Op, typename Arg1, typename Arg2>
  int sethw(Op op, Arg1 arg1, Arg2 arg2) { return error(op(handle, hw_params, arg1, arg2)); }
  
  template<typename Op, typename Arg1>
  int gethw(Op op, Arg1 arg1) { return error(op(hw_params, arg1)); }
  
  template<typename Op, typename Arg1, typename Arg2>
  int gethw(Op op, Arg1 arg1, Arg2 arg2) { return error(op(hw_params, arg1, arg2)); }

  template<typename Op, typename Arg1>
  int setsw(Op op, Arg1 arg1) { return error(op(handle, sw_params, arg1)); }

  template<typename Op, typename Arg1, typename Arg2>
  int setsw(Op op, Arg1 arg1, Arg2 arg2) { return error(op(handle, sw_params, arg1, arg2)); }
  
  template<typename Op, typename Arg1>
  int getsw(Op op, Arg1 arg1) { return error(op(sw_params, arg1)); }
  
  template<typename Op, typename Arg1, typename Arg2>
  int getsw(Op op, Arg1 arg1, Arg2 arg2) { return error(op(sw_params, arg1, arg2)); }

  void prepare()
  {
    error(snd_pcm_prepare(handle));
  }
 
  template<typename Any>
  int read(const DenseBase<Any> &buf, snd_pcm_uframes_t nbuf)
  { 
    //return error(snd_pcm_readi(handle, const_cast<DenseBase<Any>&>(buf).derived().data(), nbuf));
    int readn;
    while((readn = snd_pcm_readi(handle, const_cast<DenseBase<Any>&>(buf).derived().data(), nbuf))<0){
      //snd_pcm_prepare(handle);
      snd_pcm_recover(handle, readn, 0);
  
      //fprintf(stderr, "<<<<<<<<<<<<<<< Input Buffer Overrun >>>>>>>>>>>>>>>\n");
    }
  }

  template<typename Any>
  int write(const DenseBase<Any> &buf, snd_pcm_uframes_t nbuf)
  {


    int writen = snd_pcm_writei(handle, buf.derived().data(), nbuf);
    if( writen < 0 ) {
      fprintf(stderr, "<<<<<<<<<<<<<<< Output Buffer Underrun >>>>>>>>>>>>>>>\n");
      snd_pcm_recover(handle, writen, 0);

    }

  }

  void close()
  {
    if (!handle) {
      snd_pcm_hw_params_free(hw_params);
      snd_pcm_close(handle);
    }
  }

  ~AlsaHaptic() {
    close();
  }
};
