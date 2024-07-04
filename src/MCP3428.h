/****************************************************************************
/*
        Distributed with a free-will license.
        Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
        MCP3428
        This code is designed to work with the MCP3428_I2CADC I2C Mini Module available from ControlEverything.com.
        https://www.controleverything.com/content/Analog-Digital-Converters?sku=MCP3428_I2CADC#tabs-0-product_tabset-2
*/
/****************************************************************************/

#include <Wire.h>
#include <math.h>

class MCP3428
{
    protected:
        long raw_adc;
        uint8_t SPS;
        bool MODE;
        uint8_t i;
        uint8_t testvar;
        uint8_t config;
        uint8_t GAIN;
        uint8_t no_of_bytes;
        uint8_t data[3];
    
    public:
        MCP3428(uint8_t i2cAddress, TwoWire *wire);
        ~MCP3428();
        bool testConnection(void);
        void SetConfiguration(uint8_t channel, uint8_t resolution, bool mode, uint8_t PGA);
        bool CheckConversion();
        bool CheckforResult();
        long readADC();
        int isAdcI2cBusy_TOP_MCP3428();

    private:
        TwoWire*  _wire;
        uint8_t _address;                                    // address of port this class is supporting
};