#pragma once
#include <immintrin.h>

#if !defined(SH_USE_AVX) && !defined(SH_USE_SSE) && !defined(SH_USE_SCALAR)
    #define SH_USE_AVX
#endif

// Type selection: float vs double
#ifdef SH_USE_FLOAT
    using SHReal = float;
#else
    using SHReal = double;
#endif

// AVX 256-bit implementation
#if defined(SH_USE_AVX)

    #ifdef SH_USE_FLOAT
        using SHVec = __m256;
        #define SH_LANES 8

        #define SH_SETZERO()        _mm256_setzero_ps()
        #define SH_LOAD(ptr)        _mm256_loadu_ps(ptr)
        #define SH_STORE(ptr, v)    _mm256_storeu_ps(ptr, v)
        #define SH_ADD(a,b)         _mm256_add_ps(a,b)
        #define SH_SUB(a,b)         _mm256_sub_ps(a,b)
        #define SH_MUL(a,b)         _mm256_mul_ps(a,b)
        #define SH_SET1(x)          _mm256_set1_ps(x)
    #else
        using SHVec = __m256d;
        #define SH_LANES 4

        #define SH_SETZERO()        _mm256_setzero_pd()
        #define SH_LOAD(ptr)        _mm256_loadu_pd(ptr)
        #define SH_STORE(ptr, v)    _mm256_storeu_pd(ptr, v)
        #define SH_ADD(a,b)         _mm256_add_pd(a,b)
        #define SH_SUB(a,b)         _mm256_sub_pd(a,b)
        #define SH_MUL(a,b)         _mm256_mul_pd(a,b)
        #define SH_SET1(x)          _mm256_set1_pd(x)
    #endif

// SSE 128-bit implementation 
#elif defined(SH_USE_SSE)

    #ifdef SH_USE_FLOAT
        using SHVec = __m128;
        #define SH_LANES 4

        #define SH_SETZERO()        _mm_setzero_ps()
        #define SH_LOAD(ptr)        _mm_loadu_ps(ptr)
        #define SH_STORE(ptr, v)    _mm_storeu_ps(ptr, v)
        #define SH_ADD(a,b)         _mm_add_ps(a,b)
        #define SH_SUB(a,b)         _mm_sub_ps(a,b)
        #define SH_MUL(a,b)         _mm_mul_ps(a,b)
        #define SH_SET1(x)          _mm_set1_ps(x)
    #else
        using SHVec = __m128d;
        #define SH_LANES 2

        #define SH_SETZERO()        _mm_setzero_pd()
        #define SH_LOAD(ptr)        _mm_loadu_pd(ptr)
        #define SH_STORE(ptr, v)    _mm_storeu_pd(ptr, v)
        #define SH_ADD(a,b)         _mm_add_pd(a,b)
        #define SH_SUB(a,b)         _mm_sub_pd(a,b)
        #define SH_MUL(a,b)         _mm_mul_pd(a,b)
        #define SH_SET1(x)          _mm_set1_pd(x)
    #endif

// No SIMD
#elif defined(SH_USE_SCALAR)

    using SHVec = SHReal;
    #define SH_LANES 1

    inline SHVec SH_SETZERO() { return static_cast<SHReal>(0); }
    inline SHVec SH_LOAD(const SHReal* ptr) { return *ptr; }
    inline void  SH_STORE(SHReal* ptr, SHVec v) { *ptr = v; }
    inline SHVec SH_ADD(SHVec a, SHVec b) { return a + b; }
    inline SHVec SH_SUB(SHVec a, SHVec b) { return a - b; }
    inline SHVec SH_MUL(SHVec a, SHVec b) { return a * b; }
    inline SHVec SH_SET1(SHReal x) { return x; }

#else
    #error "Must define one of SH_USE_AVX, SH_USE_SSE, or SH_USE_SCALAR"
#endif