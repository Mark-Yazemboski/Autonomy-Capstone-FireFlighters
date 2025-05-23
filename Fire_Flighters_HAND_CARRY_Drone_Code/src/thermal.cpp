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
 * Dr. Vitor T. Valente (vitor.valente@psu.edu)
 *
 * EndCopyright
 ***/

 #include "../include/thermal.h"
 uint16_t mlx90640Frame[834]; 
 
 Thermal::Thermal() {}
 
 Thermal::~Thermal() {}
 
 void Thermal::init() {
     Wire1.begin();
     Wire1.setClock(400000); // Increase I2C clock speed to 400kHz
     delay(100);
 
     if (!this->is_connected()) {
         Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
         while (1);
     }
 
     MLX90640_SetRefreshRate(MLX90640_address, 0x02);
     Serial.println("MLX90640 online!");
 }
 
 void Thermal::update() {
       // Now local
     //float mlx90640To[768];         // Now local
 
     for (byte x = 0; x < 2; x++) {
         int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
 
         //MLX90640_CalculateTo(mlx90640Frame, &mlx90640Params, emissivity, reflectedTemp, mlx90640To);
 
         if (status < 0) {
             Serial.print("GetFrame Error: ");
             Serial.println(status);
         }
     }
 
 }
 
 void Thermal::print() {
     for (int h = 0; h < 24; h++) {
         for (int w = 0; w < 32; w++) {
            float raw = mlx90640Frame[h * 32 + w];
            float t = scaleRawToCelsius(raw);
             if (t > 90) {
                Serial.print("1");
             }else{
                Serial.print("0");
             }
             
             //Serial.print(" ");
         }
         Serial.println();
     }
     for (int q = 0; q < 5; q++) {
        Serial.println(" ");
    }
 }
 
 bool Thermal::is_connected() {
     Wire1.beginTransmission((uint8_t)MLX90640_address);
     return Wire1.endTransmission() == 0;
 }
 
 bool Thermal::Find_Fire(float Drone_Height, float Phi_FOV, float Theta_FOV, float* Fire_Position) {
     Phi_FOV = (Phi_FOV * PI) / 180;
     Theta_FOV = (Theta_FOV * PI) / 180;
     float W = 2 * Drone_Height * tan(Phi_FOV / 2);
     float L = 2 * Drone_Height * tan(Theta_FOV / 2);
 
     float Ft_Per_Pixle_W = W / 32;
     float Ft_Per_Pixle_L = L / 24;
 
     float Temp_Threshold = 100;
 
     int Count = 0;
     float X_Avg = 0;
     float Y_Avg = 0;
 
     for (int i = 0; i < 24; i++) {
         for (int j = 0; j < 32; j++) {
             float raw = mlx90640Frame[i * 32 + j];
             float t = scaleRawToCelsius(raw);
             if (t > Temp_Threshold) {
                 Count++;
                 Y_Avg += ((32 - j) * Ft_Per_Pixle_W) + (Ft_Per_Pixle_W / 2) - (W / 2);
                 X_Avg += (i * Ft_Per_Pixle_L) + (Ft_Per_Pixle_L / 2) - (L / 2);
             }
         }
     }
 
     if (Count >= 20) {
         Fire_Position[0] = X_Avg / Count;
         Fire_Position[1] = Y_Avg / Count;
         Fire_Position[2] = Drone_Height;
         return true;
     } else {
         Fire_Position[0] = 0;
         Fire_Position[1] = 0;
         Fire_Position[2] = 0;
         return false;
     }
 }

 
 float scaleRawToCelsius(uint16_t rawVal) {
    // Map raw value range (e.g., 25,000 to 65,000) to Celsius (e.g., 20°C to 100°C)
    //float tempC = (rawVal - 25000) * (80.0 / (65000 - 25000)) + 20;
    float tempC = (rawVal - 65400);
    return tempC;
}