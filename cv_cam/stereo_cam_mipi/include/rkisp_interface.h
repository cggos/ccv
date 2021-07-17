#ifndef RKISP_X3A_INTERFACE
#define RKISP_X3A_INTERFACE

#ifdef __cplusplus
extern "C" {
#endif

enum HAL_WB_MODE {
  HAL_WB_INVAL = -1,
  HAL_WB_INCANDESCENT = 0,
  HAL_WB_FLUORESCENT,
  HAL_WB_DAYLIGHT,
  HAL_WB_CLOUDY_DAYLIGHT,
  HAL_WB_AUTO,
  HAL_WB_SUNSET,
  HAL_WB_MANUAL_CCT,
  HAL_WB_BEACH,
  HAL_WB_SNOW,
  HAL_WB_CANDLE,
  HAL_WB_MAX
};

enum HAL_AE_OPERATION_MODE {
  HAL_AE_OPERATION_MODE_AUTO,
  HAL_AE_OPERATION_MODE_MANUAL,
  HAL_AE_OPERATION_MODE_LONG_EXPOSURE,
  HAL_AE_OPERATION_MODE_ACTION,
  HAL_AE_OPERATION_MODE_VIDEO_CONFERENCE,
  HAL_AE_OPERATION_MODE_PRODUCT_TEST,
  HAL_AE_OPERATION_MODE_ULL,
  HAL_AE_OPERATION_MODE_FIREWORKS,
};

enum HAL_AE_FLK_MODE {
  HAL_AE_FLK_OFF = 0,
  HAL_AE_FLK_50,
  HAL_AE_FLK_60,
  HAL_AE_FLK_AUTO,
};

enum HAL_AE_METERING_MODE {
  HAL_AE_METERING_MODE_AVERAGE,
  HAL_AE_METERING_MODE_CENTER,
  HAL_AE_METERING_MODE_SPOT,
  HAL_AE_METERING_MODE_MATRIX,
  HAL_AE_METERING_MODE_USER,
};

enum HAL_AF_MODE {
  HAL_AF_MODE_NOT_SET = -1,
  HAL_AF_MODE_AUTO,
  HAL_AF_MODE_MACRO,
  HAL_AF_MODE_INFINITY,
  HAL_AF_MODE_FIXED,
  HAL_AF_MODE_EDOF,
  HAL_AF_MODE_CONTINUOUS_VIDEO,
  HAL_AF_MODE_CONTINUOUS_PICTURE,
};

enum HAL_MODE_e {
  HAL_MODE_OFF,
  HAL_MODE_AUTO,
  HAL_MODE_MANUAL
};

enum HAL_3A_LOCKS {
  HAL_3A_LOCKS_NONE  = 0,
  HAL_3A_LOCKS_FOCUS   = 0x1,
  HAL_3A_LOCKS_EXPOSURE    = 0x2,
  HAL_3A_LOCKS_WB    = 0x4,
  HAL_3A_LOCKS_ALL   = HAL_3A_LOCKS_FOCUS
                       | HAL_3A_LOCKS_EXPOSURE | HAL_3A_LOCKS_WB,
};

enum HAL_FLT_DENOISE_LEVEL_e {
  HAL_FLT_DENOISE_LEVEL_0,
  HAL_FLT_DENOISE_LEVEL_1,
  HAL_FLT_DENOISE_LEVEL_2,
  HAL_FLT_DENOISE_LEVEL_3,
  HAL_FLT_DENOISE_LEVEL_4,
  HAL_FLT_DENOISE_LEVEL_5,
  HAL_FLT_DENOISE_LEVEL_6,
  HAL_FLT_DENOISE_LEVEL_7,
  HAL_FLT_DENOISE_LEVEL_8,
  HAL_FLT_DENOISE_LEVEL_9,
  HAL_FLT_DENOISE_LEVEL_10
};

enum HAL_FLT_SHARPENING_LEVEL_e {
  HAL_FLT_SHARPENING_LEVEL_0,
  HAL_FLT_SHARPENING_LEVEL_1,
  HAL_FLT_SHARPENING_LEVEL_2,
  HAL_FLT_SHARPENING_LEVEL_3,
  HAL_FLT_SHARPENING_LEVEL_4,
  HAL_FLT_SHARPENING_LEVEL_5,
  HAL_FLT_SHARPENING_LEVEL_6,
  HAL_FLT_SHARPENING_LEVEL_7,
  HAL_FLT_SHARPENING_LEVEL_8,
  HAL_FLT_SHARPENING_LEVEL_9,
  HAL_FLT_SHARPENING_LEVEL_10
};

enum HAL_DAYNIGHT_MODE {
  HAL_DAYNIGHT_AUTO,
  HAL_DAYNIGHT_DAY,
  HAL_DAYNIGHT_NIGHT
};

typedef struct HAL_FPS_INFO_s {
  unsigned int numerator;
  unsigned int denominator;
} HAL_FPS_INFO_t;

typedef struct HAL_Window {
  int left_hoff;
  int top_voff;
  int right_width;
  int bottom_height;
} HAL_Window_t;

enum CAMISP_CTRL_MODE {
  CAMISP_CTRL_MASTER = 0,
  CAMISP_CTRL_SLAVE = 1
};

/* -------- CamIsp10CtrItf interface -----------*/
//int rkisp_start(void* &engine, int vidFd, const char* ispNode, const char* tuningFile);
int rkisp_start(void* &engine, int vidFd, const char* ispNode, const char* tuningFile, enum CAMISP_CTRL_MODE ctrl_mode);

int rkisp_stop(void* &engine);

int rkisp_getAeTime(float &time);
int rkisp_getAeGain(float &gain);
int rkisp_getAeMaxExposureTime(float &time);
int rkisp_getAeMaxExposureGain(float &gain);
int rkisp_setAeMaxExposureTime(float time);
int rkisp_setAeMaxExposureGain(float gain);
int rkisp_getAeMeanLuma(int &meanLuma);
int rkisp_setWhiteBalance(HAL_WB_MODE wbMode);
int rkisp_setAeMode(enum HAL_AE_OPERATION_MODE aeMode);
int rkisp_setManualGainAndTime(float hal_gain, float hal_time);
int rkisp_setAntiBandMode(enum HAL_AE_FLK_MODE flkMode);
int rkisp_setAeBias(int aeBias);
int rkisp_setFps(HAL_FPS_INFO_t fps);
int rkisp_setAeWindow(int left_hoff, int top_voff, int right_width, int bottom_height);
int rkisp_getAeWindow(int &left_hoff, int &top_voff, int &right_width, int &bottom_height);
int rkisp_setExposureMeterMode(enum HAL_AE_METERING_MODE aeMeterMode);
int rkisp_getExposureMeterMode(enum HAL_AE_METERING_MODE& aeMeterMode);
int rkisp_setExposureMeterCoeff(unsigned char meter_coeff[]);
int rkisp_getExposureMeterCoeff(unsigned char meter_coeff[]);
int rkisp_setAeSetPoint(float set_point);
int rkisp_getAeSetPoint(float &set_point);
int rkisp_set3ALocks(int locks);
int rkisp_get3ALocks(int& curLocks);
int rkisp_setFocusMode(enum HAL_AF_MODE fcMode);
int rkisp_getFocusMode(enum HAL_AF_MODE& fcMode);
int rkisp_setFocusWin(HAL_Window_t afwin);
int rkisp_getFocusWin(HAL_Window_t& afwin);
int rkisp_trigggerAf(bool trigger);
int rkisp_getBrightness(int& brightVal);
int rkisp_getContrast(int& contrast);
int rkisp_getSaturation(int& sat);
int rkisp_getHue(int& hue);
int rkisp_setBrightness(int brightVal);
int rkisp_setContrast(int contrast);
int rkisp_setSaturation(int sat);
int rkisp_setHue(int hue);
int rkisp_setFilterLevel(enum HAL_MODE_e mode,
        enum HAL_FLT_DENOISE_LEVEL_e denoise, enum HAL_FLT_SHARPENING_LEVEL_e sharp);
int rkisp_getFilterLevel(enum HAL_MODE_e& mode,
        enum HAL_FLT_DENOISE_LEVEL_e& denoise, enum HAL_FLT_SHARPENING_LEVEL_e& sharp);
int rkisp_setNightMode(bool night_mode);
int rkisp_getNightMode(bool& night_mode);
int rkisp_setDayNightSwitch(enum HAL_DAYNIGHT_MODE sw);
int rkisp_getDayNightSwitch(enum HAL_DAYNIGHT_MODE& sw);

#ifdef __cplusplus
}
#endif

#endif
