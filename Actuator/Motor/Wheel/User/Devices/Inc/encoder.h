#pragma once


#ifdef __cplusplus
extern "C" {
#endif

#include <cstdint>

class EncoderBase {
public:
   // explicit EncoderBase() = default;

    virtual void init() = 0;
    // virtual int64_t getCount() = 0;
    virtual float getAngle() = 0;
    virtual float getRadian() = 0;
    virtual float getVelocity() = 0;
    virtual void update(uint32_t frequency ) = 0;
    // virtual void setCount(int64_t value) = 0;
    // virtual void cleaCount() = 0;
};


#ifdef __cplusplus
}
#endif
