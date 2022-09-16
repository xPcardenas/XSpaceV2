//XSpaceV2 v1.1 beta 091620221618

#include <XSpaceV2.h>

volatile double T = 0;
volatile double Periodo = 1000000;
volatile double Tant = 0;
volatile double counter = 0;

WiFiClient XSpaceV2WifiClient;
PubSubClient XSpaceV2MQTT(XSpaceV2WifiClient);
void Mqtt_Callback(char* topic, byte* payload, unsigned int length);

void IRAM_ATTR ISR_encoder()
{
    T = micros();

    Periodo = T - Tant;
    Tant = T;

    if(digitalRead(encoder_CHB) == LOW){
        Periodo = (-1)*Periodo;
        counter--;
    }else counter++;
    
}


void XSpaceV2Board::init(int freq, double resolution){
    pinMode(nSLEEP,OUTPUT);

    ledcAttachPin(IN1, 1);
    ledcAttachPin(IN2, 2);

    ledcSetup(1, freq, 10);
    ledcSetup(2, freq, 10);

    pinMode(encoder_CHA,INPUT_PULLDOWN);
    pinMode(encoder_CHB,INPUT_PULLDOWN);
    attachInterrupt(encoder_CHA, ISR_encoder, HIGH);

    this->_resolution = resolution;

}

void XSpaceV2Board::DRV8837_Sleep(){
    digitalWrite(nSLEEP,LOW);
}

void XSpaceV2Board::DRV8837_Wake(){
    digitalWrite(nSLEEP,HIGH);
}

void XSpaceV2Board::DRV8837_Voltage(double vp){

    double vm = 5;

    if(vp>5) vp = 5;
    if(vp<-5) vp = -5;
    int Duty = (int) ( (1 - abs(vp)/vm) * 1024.0);


    if(vp<0){
        ledcWrite(1, Duty);
        ledcWrite(2, 1024);
    }else{
        ledcWrite(1, 1024);
        ledcWrite(2, Duty);
    }
}

double XSpaceV2Board::GetEncoderSpeed(int modo){
    double vel=0;

    switch (modo)
    {
    case DEGREES_PER_SECOND:
        vel = 360000000.0/(this->_resolution*Periodo);
        if(abs(vel)>800)vel=vel_ant;
        if(abs(vel-vel_ant)>100)vel=vel_ant;
        break;
    case RADS_PER_SECOND:
        vel = 6283185.30717/(this->_resolution*Periodo);
        break;
    
    default:
        break;
    }

    vel_ant = vel;

    return vel;
}
double XSpaceV2Board::GetEncoderPosition(int modo){
    double pos=0;

    switch (modo)
    {
    case DEGREES:
        pos = counter/this->_resolution*360.0;
        break;
    case RADS:
        pos = counter/this->_resolution*2*PI;
        break;
    
    default:
        break;
    }

    return pos;
}

/* Metodos reescritos para un mejor entendimiento */

void XSpaceV2Board::Wifi_init(const char* ssid, const char* password){
	
    if(this->XSpace_info){
        Serial.println();
	    Serial.print("Conectando a ssid: ");
	    Serial.println(ssid);
    }

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		if(this->XSpace_info) Serial.print(".");
	}

    if(this->XSpace_info){
	    Serial.println("");
	    Serial.println("Conectado!!");
	    Serial.println("Dirección IP: ");
	    Serial.println(WiFi.localIP());
    }
    WiFi.setSleep(false);

}

void XSpaceV2Board::Mqtt_Connect(const char *clientId, const char *mqtt_user, const char *mqtt_pass){

	while (!XSpaceV2MQTT.connected()) {
		
		
		// Intentamos conectar
		if (XSpaceV2MQTT.connect(clientId,mqtt_user,mqtt_pass)) {
				if(this->XSpace_info) Serial.println("Conectado al Broker!");
                
		}else {
			if(this->XSpace_info){
                Serial.print("falló :( con error -> ");
			    Serial.print(XSpaceV2MQTT.state());
			    Serial.println(" Intentamos de nuevo en 5 segundos");
			    delay(5000);
            }
		}
	}

}

void XSpaceV2Board::Mqtt_init(const char *mqtt_server, uint16_t mqtt_port){
    XSpaceV2MQTT.setServer(mqtt_server, mqtt_port);
    XSpaceV2MQTT.setCallback([this] (char* topic, byte* payload, unsigned int length) { Mqtt_Callback(topic, payload, length); });
}

bool XSpaceV2Board::Mqtt_IsConnected(){
    return XSpaceV2MQTT.connected();
}

void XSpaceV2Board::Mqtt_Publish(const char* topic, const char* payload){
    XSpaceV2MQTT.publish(topic,payload);
}

void XSpaceV2Board::Mqtt_Suscribe(const char* topic){
    if(this->Mqtt_IsConnected()){
         XSpaceV2MQTT.subscribe(topic);
         if(this->XSpace_info){
             Serial.print("Suscrito a: ");
             Serial.println(topic);
         }else{
             Serial.println("Subscripcion fallida, no se tiene conexion con el broker");
         }
    }
   

}

void XSpaceV2Board::Mqtt_KeepAlive(){
    XSpaceV2MQTT.loop();
}

void XSpaceV2Board::SerialInfo(bool mode){
    this->XSpace_info = mode;
}


/***************************************************/

double XSpaceControl::control_law(double var_real, double var_ref, double Kp, double Ki,double Ts, int aproximation){
    double u=0;

    double e = var_ref - var_real; 

    switch (aproximation)
    {
    case FORWARD_EULER:
        u = 0;
        break;
    case TUSTIN:
        u = (Kp+Ts/2*Ki)*e + (Ts/2*Ki-Kp)*this->_e_1 + this->_u_1;
        this->_e_1 = e;
        this->_u_1 = u;
        break;
    
    default:
        break;
    }

    return u;
    
}