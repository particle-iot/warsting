#pragma SPARK_NO_PREPROCESSOR
#include "WifiScan.h"
#undef min
#undef max
#include <vector>
#include <algorithm>
#include "hw_config.h"

// connect to the cloud when we have hopped on to a open network
SYSTEM_MODE(MANUAL);

//
// Various pin definitions
//
const int SOUND_PIN = A1;           // trigger the sword sound
const int MOTION_PIN = A4;          // detect when the sword has been moved - only works when the button is pressed
const int BUTTON_PIN = A6;          // detect when the button has been pressed
const int GROUND1_PIN = A7;         // ground pins - set to low
const int GROUND2_PIN = A0;
const int LED1_PIN = D0;            // The sword has 2 leds - this is one of them
const int LED2_PIN = D1;            // and this is the other 

int buttonChange = 0;               // incremented on rising signal edge from the button
int motionChange = 0;               // incremented on rising signal edge from the motion sensor

int openNetworkCount = 0;           // the current number of known open networks yet to be vanquished!
WifiScanResults_t strongest;   // the current strongest network that's not been seen before
std::vector<String> seen;      // SSIDs of networks we've seen before, so they can be hopped over
std::vector<String> scanned;   // The raw list of SSIDs from this scan (includes already seen networks)
// This is used to know when the scan has "looped" around when the SSID has already been scanned previously

void buttonToggle() {           // interrupt function when button line rises
    buttonChange++;
}

void motionToggle() {           // interrupt function when motion line rises
    motionChange++;
}

/**
 * Set a pin to output mode and set the initial state.
 * @param pin   The pin to set as output
 * @param state The initial state for the pin - {@code HIGH} or {@code LOW}
 */
void output(int pin, int state) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, state);
}

/**
 * Set up the pins and change interrupts.
 */
void setup() {
    memset(&strongest, 0, sizeof(strongest));  // erase
    
    output(GROUND1_PIN, LOW);
    output(GROUND2_PIN, LOW);
    output(SOUND_PIN, HIGH);
    output(LED1_PIN, LOW);
    output(LED2_PIN, LOW);
    output(D0, LOW);
    
    // set sensors as inputs and invoke a function when they change
    pinMode(MOTION_PIN, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP); 
    attachInterrupt(BUTTON_PIN, buttonToggle, RISING);
    attachInterrupt(MOTION_PIN, motionToggle, RISING);
    WiFi.on();   // WiFi must be on so we can scan available networks
}

/**
 * Fetch the SSID as a string from the scan result. The SSID is not null terminated
 * in the result, and so is not readily usable as a string.
 * @param result  The scan result to fetch the SSID of.
 * @return The SSID as a String
 */
String ssid(WifiScanResults_t& result) {
    char buf[33];
    memcpy(buf, result.ssid, 32);
    buf[result.ssidlen] = 0;
    return buf;
}

/**
 * Determines if a vector contains a string.
 * @param v     The vector to check
 * @param value The value to look for
 * @return {@code true} if the string was found in the vector.
 */
bool contains(std::vector<String>& v, String& value) {
    return std::find(v.begin(), v.end(), value)!=v.end();
}

/**
 * Determines if this network has been seen before.
 */
bool is_seen(WifiScanResults_t& result) {
    String name = ssid(result);
    return contains(seen, name);
}

/**
 * Determines if this network is open/unsecured.
 */
bool is_open(WifiScanResults_t& result) {
    return result.security==0; 
}

void togglePin(int pin) {
    digitalWrite(pin, !digitalRead(pin));
}

/**
 * Determines if a new candidate network has a stringer RSSI than the current
 * strongest network.
 */
bool is_stronger(WifiScanResults_t& candidate, WifiScanResults_t& strongest) {
    return strongest.rssi == 0 || (candidate.rssi < strongest.rssi);
}

void soundfx() {
    togglePin(SOUND_PIN);
}

/**
 * Sets the swords light to the given brightness, and also set the core's LED.
 * @param b The brightness to set: 0 is off up to 255 maximum brightness
 */
void setLight(int b)
{
    analogWrite(LED1_PIN, b);
    analogWrite(LED2_PIN, b);
    RGB.color(0,0,255);
    RGB.brightness(b);
    RGB.control(true);
}

int lastUpdate = 0;     // the time in millis the sword status was last updated
void updateStatus(int openCount, WifiScanResults_t& strongest) 
{    
    setLight(0);
    delay(400);
    
    // brightness proportional to strength of WiFi signal
    int brightness = openCount ? 255-(strongest.rssi*2) : 0;
    // flash the number of networks found
    for (int i=0; i<openCount-1; i++) {
        setLight(brightness);
        delay(200);
        setLight(0);
        delay(400);
    }
    setLight(brightness);
    lastUpdate = millis();
}


bool is_scanned(String& name) {
    bool result = contains(scanned, name);
    if (!result)
        scanned.push_back(name);
    return result;
}

/**
 * Waits for the cloud connection to be in a certain state (connected or disconnected.)
 * @param state {@code true} to wait for the cloud to be connected, else wait for the cloud
 *  to be disconnected.
 * @param timeout   The timeout in millis to wait.
 * @return {@code true} if the cloud is in the state required
 */
bool waitForCloud(bool state, unsigned timeout) {
    unsigned start = millis();
    while (Spark.connected()!=state && (millis()-start)<timeout) {
        SPARK_WLAN_Loop();
        delay(100);
        soundfx();
    }
    return Spark.connected()==state;
}

/**
 * Toggle the sound line with a delay to cause the sword's PCB to make 
 * battle sounds.
 */
void makeSound() {
    soundfx();
    delay(50);
    soundfx();
}

/**
 * Battles to make a connection via WiFi.
 * @param timeout The timeout in millis
 * @return {@code true} if the wifi was connected, false if not.
 */
bool waitForWifi(unsigned timeout) {
    unsigned start = millis();
    while (!WiFi.ready() && (millis()-start)<timeout) {
        makeSound();
        SPARK_WLAN_Loop();
        delay(1000);
    }
    return WiFi.ready();
}

/**
 * Slay and pillage an unprotected network! ...oh, we can't do that? 
 * Ok, let's publish an event instead, to a backdrop of flashing lights and
 * clashing of metal on metal.
 */
void vanquishOpenNetwork() {
    if (!openNetworkCount || !strongest.rssi)
        return;
    RGB.control(false);     // allow the system to control the LED so we can see cloud connection progress
    makeSound();
    String name = ssid(strongest);
    WiFi.on();
    WiFi.setCredentials(name.c_str());
    makeSound();
    WiFi.connect();
    if (waitForWifi(20000)) {    // wait for the DHCP to complete
        Spark.connect();         // start connecting to the cloud
        if (waitForCloud(true, 30000))  // wait for connection
        {
            Spark.publish("vanquished",name);  // Feel the Wrath!            
            makeSound();

            for (int i=0; i<50; i++) {         // flash the LED faster and faster
                setLight(240);
                int d = (60-i)*(60-i)/20;
                delay(d);
                soundfx();
                setLight(0);
                delay(d);
            }
            
            openNetworkCount--;                // one less unsecured network!
            seen.push_back(name);              // remember that we've dealt with this one
            strongest.rssi = 0;
            Spark.disconnect();
            WiFi.disconnect();
        }
    }
    RGB.control(true);
}

bool start_scan = true;                 // when set to true, scan() will start a new scan if not already scanning)

/**
 * Start or continue an existing WiFi scan.
 */
void scan() {
    static bool scanning = false;
    static WifiScan scanner;    
    if (!scanning && start_scan) {
        memset(&strongest, 0, sizeof(strongest));
        openNetworkCount = 0;
        scanning = true;
        start_scan = false;
        scanned.clear();
        scanner.startScan();
    }
    else if (scanning) {
        WifiScanResults_t result;
        scanning = scanner.next(result);
        String name = ssid(result);        
        if (scanning && name.length() && is_scanned(name)) {
            // already seen this name, so stop scanning
            scanning = false;
            updateStatus(openNetworkCount, strongest);
        }
        if (scanning && is_open(result) && !is_seen(result)) {  
            // a new open network
            openNetworkCount++;
            if (is_stronger(result,strongest)) {
                memcpy(&strongest, &result, sizeof(result));
            }
        }
    }
}

void loop() {
    static int lastScan = 0;    
    if ((millis()-lastScan)>30000 || start_scan ) {  // scan every 30s or if requested
        setLight(0);
        start_scan = true;
        RGB.color(255,32,0);
        RGB.brightness(64);
        RGB.control(true);
        lastScan = millis();
    }
    scan();   // start or continue scan
    
    // flash the open network count every 5s
    if ((millis()-lastUpdate)>5000) {
        updateStatus(openNetworkCount, strongest);
    }
    
    if (motionChange) {             // sword was swung - commence battle
        vanquishOpenNetwork();
        start_scan = true;        
    }
    if (buttonChange)              // request a new scan
        start_scan = true;
    
    buttonChange = 0;
    motionChange = 0;
}
    
