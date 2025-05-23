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

#ifndef AERSP_WIFI
#define AERSP_WIFI

#include <WiFiS3.h>
#include <Arduino.h>

#define BUFFERSIZE 1024

class wifi
{
public:
    wifi();
    ~wifi();

    unsigned char buffer[BUFFERSIZE];
    WiFiUDP UdpGCS; /* local port to listen to GCS */
    WiFiUDP UdpGPS; /* local port to listen to GPS/Optitrack */

    void init();
    void print_status();
private:
    char *ssid; // your network SSID (name)
    char *pass; // your network password (use for WPA, or use as key for WEP)
    int status;
    int keyIndex;
    unsigned int localPortGPS;
    unsigned int localPortGCS;
};

#endif