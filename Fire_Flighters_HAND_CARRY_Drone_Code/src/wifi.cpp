/***
 * Copyright 2022, The Pennsylvania State University, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * Pennsylvania State University Unmanned Aerial System Research Laboratory (PURL)
 * Department of Aerospace Engineering
 * 229 Hammond
 * The Pennsylvania State University
 * University Park, PA 16802
 * http://purl.psu.edu
 *
 * Contact Information:
 * Dr. Thanakorn Khamvilai (thanakorn.khamvilai@ttu.edu)
 * Dr. Vitor T. Valente (vitor.valente@psu.edu)
 *
 * EndCopyright
 ***/

#include "../include/wifi.h"
#include "../include/rc_pilot.h"

wifi::wifi(){
  this->ssid = "ECORE_001_MOCAP"; // your network SSID (name)
  this->pass = "ecore_001_mocap"; // your network password (use for WPA, or use as key for WEP)
  //this->ssid = "ECORE_448_3DPRINTERS"; // your network SSID (name)
  //this->pass = "ecore_448_3dprinters"; // your network password (use for WPA, or use as key for WEP)

  this->keyIndex = 0;
  this->localPortGCS = 10001;  // this need to match target port from GCS
  this->localPortGPS = 9001;   // this need to match target port from GPS/Mocap
}

wifi::~wifi() {};

void wifi::init() {
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(this->ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    this->status = WiFi.begin(this->ssid, this->pass);

    // wait 2 seconds for connection:
    delay(2000);
  }
  Serial.println("Connected to WiFi");

  this->print_status();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  this->UdpGPS.begin(this->localPortGPS);
  delay(100);
  this->UdpGCS.begin(this->localPortGCS);
}

void wifi::print_status() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}