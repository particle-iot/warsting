#pragma once

#include "application.h"

// scan result taken from http://e2e.ti.com/support/wireless_connectivity/f/851/t/166684.aspx?pi310978=4
struct WifiScanResults_t {
    uint32_t networks;
    uint32_t status;
    uint8_t valid:1;
    uint8_t rssi:7;
    uint8_t security:2;  // 0 - open 
    uint8_t ssidlen:6;
    uint16_t time;
    uint8_t ssid[32];
    uint8_t bssid[6];
};


class WifiScan {
    public:
        int startScan();
        bool next(WifiScanResults_t& result);        
};

