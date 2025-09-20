#ifndef BUCHLA_LPG_H
#define BUCHLA_LPG_H

#include <cstddef>

namespace sfdsp {

enum LPGMode {
    LPG_MODE_FULL,      // Classic Buchla LPG behavior (default)
    LPG_MODE_VCA,       // VCA only (no filtering)
    LPG_MODE_FILTER     // Filter only (no VCA)
};

class BuchlaLPG {
public:
    void Init(float samplerate);
    
    void ProcessBlock(float* cv_in, const float* in, float* out, size_t size);
    void ProcessCurrent(const float* vc_in, float* vc_out, size_t size);
    void ProcessAudio(const float* vc_in, const float* in, float* out, size_t size);
    
    // Mode control
    void SetMode(LPGMode mode) { mode_ = mode; }
    LPGMode GetMode() const { return mode_; }
    
    // Optional: Add parameter setters if needed
    void SetNonLinear(bool non_linear) { non_lin_ = non_linear; }
    bool GetNonLinear() const { return non_lin_; }

private:
    float samplerate_;
    float dt_;
    float f_;
    float f_inv_;
    LPGMode mode_ = LPG_MODE_FULL;  // Default to full LPG mode
    
    // State variables
    float prev_current_ = 0.f;
    int8_t prev_dir_ = 0;
    float wc_ = 0.f;
    float yc_ = 0.f;
    float sc_ = 0.f;
    
    // Audio processing state
    float sx_ = 0.f;
    float so_ = 0.f;
    float sd_ = 0.f;
    
    bool non_lin_ = false;
};

// Utility functions
float GetCurrent(float vc, float offset, bool smoothed);
void GetCurrent(const float* vc_in, float* vc_out, size_t size, bool smoothed);

} // namespace sfdsp

#endif // BUCHLA_LPG_H