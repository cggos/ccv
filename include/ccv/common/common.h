//
// Created by cg on 9/16/19.
//

#ifndef VIKIT_CG_COMMON_H
#define VIKIT_CG_COMMON_H

namespace cg {

#ifndef _MSC_VER
#include <stdint.h>
#else
    typedef __int8            int8_t;
    typedef __int16           int16_t;
    typedef __int32           int32_t;
    typedef __int64           int64_t;
    typedef unsigned __int8   uint8_t;
    typedef unsigned __int16  uint16_t;
    typedef unsigned __int32  uint32_t;
    typedef unsigned __int64  uint64_t;
#endif

#ifdef __USER_DEBUG__
    #if __cplusplus < 201103L
#define DEBUG_LOG(format,...) printf("File: %s, Line: %05d: "format"\n", __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define DEBUG_LOG(format,...) printf("File: %s, Line: %05d: " format "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#endif
#else
#define DEBUG_LOG(format,...)
#endif

#define DELETE_NEW_OBJ(obj) if(nullptr!=obj){delete obj;obj=nullptr;}

#define endll std::endl << std::endl // double end line definition

    typedef double FLOAT; // double precision, float for single precision

    const FLOAT EPS = 1e-20;

    enum {
        RET_FAILED = -1,
        RET_SUCESS = 0
    };

    enum INTERPOLATION_TYPE {
        INTERPOLATION_NEAREST,
        INTERPOLATION_BILINEAR
    };
}

#endif //VIKIT_CG_COMMON_H
