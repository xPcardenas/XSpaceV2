/*
  XSpaceV2 v1.1 beta 091620221618

  Version   :  1.1
  Autor     :  Pablo Cardenas
  Fecha     :  16/10/2022
  Hora      :  9:27am

*/

#ifndef XSPACEV2_H
#define XSPACEV2_H

#include <Arduino.h>
#include <stdint.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define IN1 32
#define IN2 33
#define nSLEEP 25

#define encoder_CHA 34
#define encoder_CHB 35

#define DEGREES_PER_SECOND 1
#define RADS_PER_SECOND 2

#define DEGREES 1
#define RADS 2

extern volatile double Periodo;

class XSpaceV2Board{
    private:
        double vel_ant = 0;
        bool XSpace_info = false;

        double _resolution;

    public:
        void init(int freq, double resolution);
        void DRV8837_Sleep();
        void DRV8837_Wake();
        void DRV8837_Voltage(double vp);
        
        double GetEncoderSpeed(int modo);
        double GetEncoderPosition(int modo);

        void SerialInfo(bool mode);
        void Wifi_init(const char* ssid, const char* password);
        void Mqtt_init(const char* mqtt_server, uint16_t mqtt_port);
        void Mqtt_Connect(const char *clientId, const char *mqtt_user, const char *mqtt_pass);
        void Mqtt_Publish(const char* topic, const char* payload);
        void Mqtt_Suscribe(const char* topic);
        void Mqtt_KeepAlive();
        

        bool Mqtt_IsConnected();
};






#define FORWARD_EULER 1
#define TUSTIN 2

class XSpaceControl{
  private:
    double _e_1 = 0;
    double _u_1 = 0;
  public:
    double control_law(double vel, double vel_ref, double Kp, double Ki,double Ts, int aproximation);

};

#endif