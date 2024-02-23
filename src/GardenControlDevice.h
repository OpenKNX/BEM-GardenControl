#pragma once

#include "OpenKNX.h"

class GardenControlDevice : public OpenKNX::Module
{
  public:
    GardenControlDevice();
    ~GardenControlDevice();

    // void processReadRequests();
    void processInputKo(GroupObject &iKo);
    // void showHelp() override;
    // bool processCommand(const std::string cmd, bool debugKo) override;
    void setup();
    void loop();
    // void readFlash(const uint8_t* iBuffer, const uint16_t iSize) override;
    // void writeFlash() override;
    // uint16_t flashSize() override;
    const std::string name() override;
    const std::string version() override;

  private:
    void waitStartupLoop();
};
 extern GardenControlDevice openknxGardenControlModule;