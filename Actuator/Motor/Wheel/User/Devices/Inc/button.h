#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <cstdint>


class ButtonBase{
  public:
    virtual void init() = 0;
    // virtual int64_t getCount() = 0;
    virtual void update(uint32_t frequency ) = 0;
  private:

};


#ifdef __cplusplus
}
#endif
