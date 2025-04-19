#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <cstdint>


class DriverBase{
  public:
   virtual void init() = 0;
   virtual void setPWM(float d_a, float d_b, float d_c) = 0;
  //  virtual void forward(uint16_t) = 0;
  //  virtual void reverse(uint16_t) = 0;
  //  virtual void stop() = 0;
  //  virtual void brake() = 0;
  private:

};


#ifdef __cplusplus
}
#endif
