#include "mex.h"
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_1_PI
#define M_1_PI 0.31830988618379067154
#endif

// Got from SLEEF
static inline float mlaf(float x, float y, float z) {
#if defined(__FMA__) || defined(__FMA)
    return std::fma(x, y, z);
#else
    return x * y + z;
#endif
}

static inline int rintfk(float x) {
    return static_cast<int>(std::round(x));
}

static inline float fabsfk(float x) {
    return std::fabs(x);
}

static inline float xfastsinf_u3500(float d) {
    int q;
    float u, s;

    q = rintfk(d * (float)M_1_PI);
    d = mlaf((float)q, -(float)M_PI, d);

    s = d * d;

    u = -0.1881748176e-3f;
    u = mlaf(u, s, +0.8323502727e-2f);
    u = mlaf(u, s, -0.1666651368e+0f);
    u = mlaf(s * d, u, d);

    if ((q & 1) != 0) u = -u;

    return u;
}

static inline float xfastcosf_u3500(float d) {
    int q;
    float u, s;

    q = rintfk(mlaf(d, (float)M_1_PI, -0.5f));
    d = mlaf((float)q, -(float)M_PI, d - (float)M_PI * 0.5f);

    s = d * d;

    u = -0.1881748176e-3f;
    u = mlaf(u, s, +0.8323502727e-2f);
    u = mlaf(u, s, -0.1666651368e+0f);
    u = mlaf(s * d, u, d);

    if ((q & 1) == 0) u = -u;

    return u;
}

static inline void fast_sincos(double phase, double &s, double &c) {
    float pf = static_cast<float>(phase);
    float sf = xfastsinf_u3500(pf);
    float cf = xfastcosf_u3500(pf);
    s = static_cast<double>(sf);
    c = static_cast<double>(cf);
}

static inline void wrap_phase(double &phase) {
    const double TWO_PI = 2.0 * M_PI;
    double k = std::floor((phase + M_PI) / TWO_PI);
    phase -= k * TWO_PI;
    if (phase <= -M_PI) phase += TWO_PI;
    else if (phase > M_PI) phase -= TWO_PI;
}

static inline void decideQpsk(double yr, double yi, double &dr, double &di)
{
    const double invSqrt2 = 1.0 / std::sqrt(2.0);
    double reHat = (yr >= 0.0) ? 1.0 : -1.0;
    double imHat = (yi >= 0.0) ? 1.0 : -1.0;

    dr = reHat * invSqrt2;
    di = imHat * invSqrt2;
}

static inline void buildQamLevels(int M, int &L, std::vector<double> &levels,
                                  double &EsAvg, double &scaleQam, double &idxOffset)
{
    double Ld = std::sqrt(static_cast<double>(M));
    L = static_cast<int>(std::round(Ld)); // Level
    if (std::fabs(Ld - L) > 1e-9) {
        mexErrMsgIdAndTxt("decisionDirectedCarrierSyncMex:InvalidM",
                          "For M>4 only square QAM (M = L^2) is supported.");
    }

    levels.resize(L);
    int halfIdx = (L - 1) / 2;
    for (int i = 0; i < L; ++i) {
        int m = i - halfIdx;
        levels[i] = 2.0 * static_cast<double>(m);   // step 2 -> [-3 -1 1 3]
    }

    // Average energy
    EsAvg = 0.0;
    for (int i = 0; i < L; ++i) {
        EsAvg += levels[i] * levels[i];
    }
    EsAvg /= static_cast<double>(L);

    // Overall scaling
    scaleQam  = 1.0 / std::sqrt(2.0 * EsAvg);
    idxOffset = static_cast<double>(halfIdx);
}

// Changing from near level problem to an indexing problem
static inline double quantizeQamScalar(double x,
                                       const std::vector<double> &levels,
                                       int L,
                                       double idxOffset)
{
    // Ideal index is roughly x/2 + (L-1)/2.
    double kd = std::round(x / 2.0 + idxOffset);
    int    k  = static_cast<int>(kd);

    if (k < 0) {
        k = 0;
    } else if (k > (L - 1)) {
        k = L - 1;
    }
    return levels[k];
}

// Entry point
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    if (nrhs != 6) {
        mexErrMsgIdAndTxt("decisionDirectedCarrierSyncMex:InvalidNumInputs",
            "Expected 6 inputs: x, M, Kp, Ki, phase0, freq0.");
    }
    if (nlhs != 3) {
        mexErrMsgIdAndTxt("decisionDirectedCarrierSyncMex:InvalidNumOutputs",
            "Expected 3 outputs: y, phaseOut, freqOut.");
    }

    // Input signal x (in form of symbol)
    const mxArray *x_in = prhs[0];
    if (!mxIsDouble(x_in) || !mxIsComplex(x_in)) {
        mexErrMsgIdAndTxt("decisionDirectedCarrierSyncMex:InvalidX",
            "Input x must be complex double.");
    }

    mxComplexDouble *xData = mxGetComplexDoubles(x_in);
    mwSize N = mxGetNumberOfElements(x_in);

    // Loop parameters
    int    M     = static_cast<int>(mxGetScalar(prhs[1]));
    double Kp    = mxGetScalar(prhs[2]);
    double Ki    = mxGetScalar(prhs[3]);
    double phase = mxGetScalar(prhs[4]);
    double freq  = mxGetScalar(prhs[5]);

    if (M <= 0) {
        mexErrMsgIdAndTxt("decisionDirectedCarrierSyncMex:InvalidM",
                          "M must be positive.");
    }

    // Output y has the same shape as x
    mwSize nDims       = mxGetNumberOfDimensions(x_in);
    const mwSize *dims = mxGetDimensions(x_in);

    plhs[0] = mxCreateNumericArray(nDims, dims, mxDOUBLE_CLASS, mxCOMPLEX);
    mxComplexDouble *yData = mxGetComplexDoubles(plhs[0]);

    // QPSK vs square QAM setup
    bool isQpsk = (M == 4);
    int L = 0;
    std::vector<double> levels;
    double EsAvg     = 1.0;
    double scaleQam  = 1.0;
    double idxOffset = 0.0;

    if (!isQpsk) {
        // The M should be L ^ 2
        buildQamLevels(M, L, levels, EsAvg, scaleQam, idxOffset);
    }

    // Main decision-directed PLL loop
    for (mwSize k = 0; k < N; ++k) {
        double xr = xData[k].real;
        double xi = xData[k].imag;

        // Fast approximation of sincos
        double c, s;
        fast_sincos(phase, s, c);

        // Apply rotation: y = x * exp(-j*phase)
        double yr = xr * c + xi * s;
        double yi = xi * c - xr * s;

        yData[k].real = yr;
        yData[k].imag = yi;

        // Decide nearest constellation point
        double dr, di;
        if (isQpsk) {
            decideQpsk(yr, yi, dr, di);
        } else {
            double reHat = quantizeQamScalar(yr, levels, L, idxOffset);
            double imHat = quantizeQamScalar(yi, levels, L, idxOffset);
            dr = reHat * scaleQam;
            di = imHat * scaleQam;
        }

        // Calculate phase error and skip atan
        double ei = yi * dr - yr * di;   // imag( y * conj(d) )
        double e  = ei;

        // 2nd-order PLL update
        freq  += Ki * e;
        phase += freq + Kp * e;

        // Wrapping the phase so the approximations dont see a huge numbers
        wrap_phase(phase);
    }

    plhs[1] = mxCreateDoubleScalar(phase);
    plhs[2] = mxCreateDoubleScalar(freq);
}
