
#include <Arduino.h>
#include <NimBLEDevice.h>

#include "HIDKeys.h"

// The remote service we wish to connect to.
static NimBLEUUID serviceUUID((uint16_t)0x1812);
// Characteristic of report map
static NimBLEUUID reportUUID((uint16_t)0x2a4b);
// The characteristic of the remote service we are interested in.
static NimBLEUUID charUUID((uint16_t)0x2a4d);

static uint8_t LED = 2;

static uint32_t scanTime = 30;  // * 1000;
static bool doConnect = false;
static bool connected = false;
static bool doScan = false;
static NimBLERemoteCharacteristic *pRemoteCharacteristic;
static NimBLEAdvertisedDevice *myDevice;

void flashLED();

/** Notification / Indication receiving handler callback */
void notifyCallback(NimBLERemoteCharacteristic *pRemoteCharacteristic, uint8_t *pData,
                    size_t length, bool isNotify) {
	Serial.print("Notify callback for characteristic: ");
	Serial.print(pRemoteCharacteristic->getUUID().toString().c_str());
	Serial.printf(", of data length: %d", length);
	Serial.printf(", handle: 0x%02X\n", pRemoteCharacteristic->getHandle());
	Serial.print("data: ");
	for (size_t i = 0; i < length; i++) {
		Serial.printf("0x%02X ", pData[i]);
	}
	Serial.println("");

	// FIXME: This isn't any good. Standard data length for key press should be
	// 8, but Logi MX Keys Mac seems to send 7 (skipping the reserved 2nd byte)
	// but the EpoMaker sends all 8 bytes, plus some notifications have 2 - this
	// appears to be related to the volume + / - / mute
	if (length == 7) {
		Serial.print("KeyCodes: ");
		for (size_t i = 1; i < 7; i++)
			Serial.printf("%c ", pData[0] == 2 ? shift_keys[pData[i]] : keys[pData[i]]);
		Serial.println();

		flashLED();
	}
}

class MyClientCallback : public NimBLEClientCallbacks {
	void onConnect(NimBLEClient *pClient) {
		connected = true;
		Serial.println(" - Device connected");
		Serial.printf(" - Bonded device count: %d\n", NimBLEDevice::getNumBonds());
		for (int i = 0; i < NimBLEDevice::getNumBonds(); i++) {
			Serial.printf(" - Bonded device: %s\n",
			              NimBLEDevice::getBondedAddress(i).toString().c_str());
		}

		pClient->updateConnParams(120, 120, 0, 60);
	}

	void onDisconnect(NimBLEClient *pClient) {  //, int reason) {
		connected = false;
		Serial.println(" - Device disconnected");
		Serial.printf(" - Bonded device count: %d\n", NimBLEDevice::getNumBonds());
		for (int i = 0; i < NimBLEDevice::getNumBonds(); i++) {
			Serial.printf(" - Bonded device: %s\n",
			              NimBLEDevice::getBondedAddress(i).toString().c_str());
		}
	}

	void onAuthenticationComplete(ble_gap_conn_desc *desc) {
		Serial.println(" - Authentication complete");
	}
};

/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToServer() {
	Serial.print("Forming a connection to ");
	Serial.println(myDevice->getAddress().toString().c_str());

	NimBLEClient *pClient = NimBLEDevice::createClient();
	Serial.println(" - Created client");

	pClient->setClientCallbacks(new MyClientCallback());

	// Connect to the remote BLE Server.
	if (!pClient->connect(myDevice)) {
		// if you pass BLEAdvertisedDevice instead of address, it will be recognized
		// type of peer device address (public or private)
		return false;
	}
	Serial.println(" - Connected to server");

	// Obtain a reference to the service we are after in the remote BLE server.
	NimBLERemoteService *pRemoteService = pClient->getService(serviceUUID);
	if (pRemoteService == nullptr) {
		Serial.print("Failed to find our service UUID: ");
		Serial.println(serviceUUID.toString().c_str());
		pClient->disconnect();
		return false;
	}
	Serial.println(" - Found our service");

	if (NimBLEDevice::isBonded(pClient->getPeerAddress())) {
		Serial.println(" - Device is bonded");
	} else {
		Serial.println(" - Device not bonded");
	}
	// We could read the report characteristic to trigger pairing, but this is quicker
	NimBLEDevice::startSecurity(pClient->getConnId());

	std::vector<NimBLERemoteCharacteristic *> *charvector;
	charvector = pRemoteService->getCharacteristics(true);
	for (auto &it : *charvector) {
		if (it->getUUID() == NimBLEUUID(charUUID)) {
			if (it->canNotify()) {
				Serial.printf(" - %s", it->toString().c_str());
				if (!it->subscribe(true, notifyCallback)) {
					Serial.println(" - subscribe failed");
					pClient->disconnect();
					return false;
				} else {
					Serial.println(" - subscribe succeeded");
				}
			}
		}
	}

	Serial.println(" - Device up.");
	return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are
 * looking for.
 */
class MyScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
	// This changes to NimBLEScanCallBacks in v2.0.0
	/**
	 * Called for each advertising BLE server.
	 */
	void onResult(NimBLEAdvertisedDevice *advertisedDevice) {
		Serial.print("BLE Advertised Device found: ");
		Serial.println(advertisedDevice->toString().c_str());

		// 	// We have found a device, let us now see if it contains the service we are
		// looking for.
		if (NimBLEDevice::isBonded(advertisedDevice->getAddress()) ||
		    advertisedDevice->isAdvertisingService(serviceUUID)) {
			Serial.println("HID service found - will attempt to connect");
			NimBLEDevice::getScan()->stop();
			myDevice = advertisedDevice;
			doConnect = true;
			doScan = true;
		}  // Found our server
	}  // onResult
};  // MyScanCallbacks

void flashLED() {
	digitalWrite(LED, HIGH);
	delay(100);
	digitalWrite(LED, LOW);
}

void setup() {
	pinMode(LED, OUTPUT);

	Serial.begin(115200);
	Serial.println("Starting Arduino BLE Client application...");
	/** Initialize NimBLE, no device name spcified as we are not advertising */
	NimBLEDevice::init("");

	// One off if we need to:
	// Serial.println("Deleting all bonds - if you see this and you are not the dev,
	// kick their arse!"); NimBLEDevice::deleteAllBonds();

	/** Set the IO capabilities of the device, each option will trigger a different
	 * pairing method. BLE_HS_IO_KEYBOARD_ONLY    - Passkey pairing
	 *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
	 *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
	 */
	// NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
	// NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison
	// NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

	/** 2 different ways to set security - both calls achieve the same result.
	 *  no bonding, no man in the middle protection, secure connections.
	 *
	 *  These are the default values, only shown here for demonstration.
	 */
	// NimBLEDevice::setSecurityAuth(false, false, true);
	// NimBLEDevice::setSecurityAuth(true, false, true);
	NimBLEDevice::setSecurityAuth(
		BLE_SM_PAIR_AUTHREQ_BOND |
		/* BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);
	// NimBLEDevice::setSecurityInitKey(3);

	/** Optional: set the transmit power, default is 3db */
#ifdef ESP_PLATFORM
	NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
	NimBLEDevice::setPower(9); /** +9db */
#endif

	/** Optional: set any devices you don't want to get advertisments from */
	// Ignore NVidia Shield
	NimBLEDevice::addIgnored(NimBLEAddress("48:b0:2d:19:50:12"));

	/** create new scan */
	NimBLEScan *pBLEScan = NimBLEDevice::getScan();

	/** create a callback that gets called when advertisers are found */
	pBLEScan->setAdvertisedDeviceCallbacks(new MyScanCallbacks());

	/** Set scan interval (how often) and window (how long) in milliseconds */
	// pBLEScan->setInterval(45);
	// pBLEScan->setWindow(15);
	pBLEScan->setInterval(1349);
	pBLEScan->setWindow(449);

	/** Active scan will gather scan response data from advertisers
	 *  but will use more energy from both devices
	 */
	// pBLEScan->setActiveScan(true);
	/** Start scanning for advertisers for the scan time specified (in seconds) 0 =
	 * forever Optional callback for when scanning stops.
	 */
	// pScan->start(scanTime, scanEndedCB);
	pBLEScan->start(scanTime, false);
}

// This is the Arduino main loop function.
void loop() {
	// If the flag "doConnect" is true then we have scanned for and found the desired
	// BLE Server with which we wish to connect.  Now we connect to it.  Once we are
	// connected we set the connected flag to be true.
	if (doConnect == true) {
		if (connectToServer()) {
			Serial.println("We are now connected to the BLE Server.");
		} else {
			Serial.println(
				"We have failed to connect to the server; there is nothin more we will "
				"do.");
		}
		doConnect = false;
	}

	if (connected) {
		// do nothing, all data is handled in notifications callback
	} else if (doScan) {
		NimBLEDevice::getScan()->start(scanTime, false);
	}

	delay(1000);  // Delay a second between loops.
}