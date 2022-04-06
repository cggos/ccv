#include "com_ndk_test_NEONTest.h"

#include <vector>
#include <random>
#include <chrono>
#include <fstream>
#include <assert.h>
#include <arm_neon.h>

#undef  LOG
#define LOG_TAG	"com_ndk_test_NEONTest"

#include "Tools/log.h"

decltype(std::chrono::high_resolution_clock::now()) start;
decltype(start) end;
long long ticks;

extern "C" {
void neon_asm_convert(uint8_t *dest, uint8_t *src, int n);
}

int compare(uint8_t *a, uint8_t* b, int n)
    {
        for(int i=0;i<n;i++){
            if(a[i]!=b[i]){
                return -1;
            }
        }
        return 0;
    }

void normal_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n)
    {
        int i;
        for (i=0; i<n; i++)
        {
            int r = *src++; // load red
            int g = *src++; // load green
            int b = *src++; // load blue

            // build weighted average:
            int y = (r*77)+(g*151)+(b*28);

            // undo the scale by 256 and write to memory:
            *dest++ = (y>>8);
        }
    }

void neon_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n)
    {
        int i;
        uint8x8_t rfac = vdup_n_u8 (77);
        uint8x8_t gfac = vdup_n_u8 (151);
        uint8x8_t bfac = vdup_n_u8 (28);
        n/=8;

        for (i=0; i<n; i++)
        {
            uint16x8_t  temp;
            uint8x8x3_t rgb  = vld3_u8 (src);
            uint8x8_t result;

            temp = vmull_u8 (rgb.val[0],      rfac);
            temp = vmlal_u8 (temp,rgb.val[1], gfac);
            temp = vmlal_u8 (temp,rgb.val[2], bfac);

            result = vshrn_n_u16 (temp, 8);
            vst1_u8 (dest, result);
            src  += 8*3;
            dest += 8;
        }
    }

void test1()
{
    //准备一张图片，使用软件模拟生成，格式为rgb rgb ..
    uint32_t const array_size = 2048*2048;
    uint8_t * rgb = new uint8_t[array_size*3];
    for(int i=0;i<array_size;i++){
        rgb[i*3]=234;
        rgb[i*3+1]=94;
        rgb[i*3+2]=23;
    }
    //灰度图大小为rgb的1/3
    uint8_t * gray_cpu = new uint8_t[array_size];
    uint8_t * gray_neon = new uint8_t[array_size];
    uint8_t * gray_neon_asm = new uint8_t[array_size];

    struct timeval tv1,tv2;
    gettimeofday(&tv1,NULL);
    normal_convert(gray_cpu,rgb,array_size);
    gettimeofday(&tv2,NULL);
    LOGI("pure cpu cost time:%ld",(tv2.tv_sec-tv1.tv_sec)*1000000+(tv2.tv_usec-tv1.tv_usec));

    gettimeofday(&tv1,NULL);
    neon_convert(gray_neon,rgb,array_size);
    gettimeofday(&tv2,NULL);
    bool result = compare(gray_cpu,gray_neon,array_size);
    LOGI("neon c cost time:%ld,result is %d",(tv2.tv_sec-tv1.tv_sec)*1000000+(tv2.tv_usec-tv1.tv_usec),result);

//    gettimeofday(&tv1,NULL);
//    neon_asm_convert(gray_neon_asm,rgb,array_size);
//    gettimeofday(&tv2,NULL);
//    result = compare(gray_cpu,gray_neon_asm,array_size);
//    LOGI("neon asm cost time:%ld,result is %d",(tv2.tv_sec-tv1.tv_sec)*1000000+(tv2.tv_usec-tv1.tv_usec),result);

    delete[] rgb;
    delete[] gray_cpu;
    delete[] gray_neon;
    delete[] gray_neon_asm;
}

static void fill_random_value(std::vector<float>& vec_data)
{
    std::uniform_real_distribution<float> distribution(std::numeric_limits<float>::min(), std::numeric_limits<float>::max());
    std::default_random_engine generator;
    std::generate(vec_data.begin(), vec_data.end(), [&]() { return distribution(generator); });
}

static bool is_equals_vector(const std::vector<float>& vec_a, const std::vector<float>& vec_b)
{
    if (vec_a.size() != vec_b.size())
    {
        return false;
    }
    for (size_t i = 0; i < vec_a.size(); i++)
    {
        if (vec_a[i] != vec_b[i])
        {
            return false;
        }
    }
    return true;
}

static void normal_vector_mul(const std::vector<float>& vec_a, const std::vector<float>& vec_b, std::vector<float>& vec_result)
{
    assert(vec_a.size() == vec_b.size());
    assert(vec_a.size() == vec_result.size());
    //compiler may optimized auto tree vectorize (test this diabled -ftree-vectorize)
    for (size_t i = 0; i < vec_result.size();i++)
    {
        vec_result[i] = vec_a[i] * vec_b[i];
    }
}

static void neon_vector_mul(const std::vector<float>& vec_a, const std::vector<float>& vec_b, std::vector<float>& vec_result)
{
    assert(vec_a.size() == vec_b.size());
    assert(vec_a.size() == vec_result.size());
    int i = 0;
    //neon process
    for (; i < (int)vec_result.size() - 3 ; i+=4)
    {
        const auto data_a = vld1q_f32(&vec_a[i]);
        const auto data_b = vld1q_f32(&vec_b[i]);
        float* dst_ptr = &vec_result[i];
        const auto data_res = vmulq_f32(data_a, data_b);
        vst1q_f32(dst_ptr, data_res);
    }
    //normal process
    for (; i < (int)vec_result.size(); i++)
    {
        vec_result[i] = vec_a[i] * vec_b[i];
    }
}

static int test_neon()
{
    const int test_round = 1000;
    const int data_len = 100000;
    std::vector<float> vec_a(data_len);
    std::vector<float> vec_b(data_len);
    std::vector<float> vec_result(data_len);
    std::vector<float> vec_result2(data_len);
    //fill random value in vecA & vecB
    fill_random_value(vec_a);
    fill_random_value(vec_b);
    //check the result is same
    {
        normal_vector_mul(vec_a, vec_b, vec_result);
        neon_vector_mul(vec_a, vec_b, vec_result2);
        if (!is_equals_vector(vec_result,vec_result2))
        {
            LOGI("result vector is not equals!");
            return -1;
        }
    }

    //test normal_vector_mul
    start = std::chrono::high_resolution_clock::now();
    {
        for (int i = 0; i < test_round;i++)
        {
            normal_vector_mul(vec_a, vec_b, vec_result);
        }
    }
    end = std::chrono::high_resolution_clock::now();
    ticks = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    LOGI("normal_vector_mul: %lld microseconds", ticks);

    //test neon_vector_mul
    start = std::chrono::high_resolution_clock::now();
    {
        for (int i = 0; i < test_round; i++)
        {
            neon_vector_mul(vec_a, vec_b, vec_result2);
        }
    }
    end = std::chrono::high_resolution_clock::now();
    ticks = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    LOGI("neon_vector_mul: %lld microseconds", ticks);

    return 0;
}

static void test_file() {
    std::ifstream f;
    f.open("/sdcard/Download/SLAM/Calibration/mi6.yaml");
    if(!f.is_open()) {
        LOGE("Failed to open yaml file");
    } else {
        LOGI("Open yaml file successful");
        f.close();
    }
}

/*
 * Class:     com_ndk_test_NEONTest
 * Method:    test
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_ndk_test_NEONTest_test
  (JNIEnv *, jclass){

    test_file();

    test1();

	test_neon();
}

