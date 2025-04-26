#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <cstdint>


class CurrentBase{
  public:
   virtual void init(void) = 0;
   virtual void getCurrent(float *Ia, float *Ib, float *Ic) = 0;
   virtual float getIa(void) = 0;
   virtual float getIb(void) = 0;
   virtual float getIc(void) = 0;
   virtual void update(uint32_t frequency ) = 0;
  protected:

   float _Ia = 0;
   float _Ib = 0;
   float _Ic = 0;
  private:

};


#ifdef __cplusplus
}
#endif
