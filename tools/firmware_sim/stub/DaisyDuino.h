// Host stub of DaisyDuino: the real DaisySP, a fake DAISY / QSPI / PersistentStorage.
#pragma once
#include "Arduino.h"
#include "utility/DaisySP/daisysp.h"
enum DaisyDuinoDevice { DAISY_SEED };
enum DaisyDuinoSampleRate { AUDIO_SR_48K };
typedef void (*DaisyDuinoCallback)(float**, float**, size_t);
struct DaisyHardware { void ResetToBootloader() {} };
namespace sim { extern DaisyDuinoCallback audioCb; }
struct AudioClass {
  DaisyHardware init(DaisyDuinoDevice, DaisyDuinoSampleRate) { return DaisyHardware(); }
  float get_samplerate() { return 48000.0f; }
  void SetAudioBlockSize(size_t) {}
  void begin(DaisyDuinoCallback cb) { sim::audioCb = cb; }
};
extern AudioClass DAISY;
enum { DSY_GPIOF, DSY_GPIOG };
struct dsy_gpio_pin { int port, pin; };
namespace daisy {
struct QSPIHandle {
  enum Result { OK, ERR };
  struct Config {
    enum Device { IS25LP064A } device;
    enum Mode { MEMORY_MAPPED } mode;
    struct { dsy_gpio_pin io0, io1, io2, io3, clk, ncs; } pin_config;
  };
  Result Init(const Config&) { return ERR; }  // no QSPI: editor runs on defaults
};
template <typename T> class PersistentStorage {
  T s_{};
public:
  enum class State { UNKNOWN, FACTORY, USER };
  PersistentStorage(QSPIHandle&) {}
  void Init(const T& d, uint32_t) { s_ = d; }
  T& GetSettings() { return s_; }
  void Save() {}
  void RestoreDefaults() {}
  State GetState() const { return State::FACTORY; }
};
}
