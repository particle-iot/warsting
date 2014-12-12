
#include "WifiScan.h"
#include "application.h"
#include "wlan.h"

int WifiScan::startScan() {
   const unsigned long intervalTime[16] = { 2000, 2000, 2000, 2000,  2000,
        2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000 };

    wlan_ioctl_set_scan_params(4000, 20, 100, 5, 0x7FF, -120, 0, 300, 
        (unsigned long * ) &intervalTime);

    return 0;
}

/**
 * Determine if a given region of memory is empty - i.e. all zeros.
 */
bool is_empty(void* data, unsigned len) {
    for (unsigned i=0; i<len; i++)
        if (((uint8_t*)data)[i])
            return false;
    return true;
}

bool WifiScan::next(WifiScanResults_t& scanResult) {
    // clear the result
    memset(&scanResult, 0, sizeof(scanResult));
    long err = wlan_ioctl_get_scan_results(0, (uint8_t* ) &scanResult);
    return (!err && scanResult.valid && !is_empty(scanResult.bssid, sizeof(scanResult.bssid)));
}

