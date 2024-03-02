// SENSOR IR

// #define S1 23
// #define S2 22
// #define S3 21
// #define S4 19
#define S1 4
#define S2 2
#define S3 27
#define S4 26
#define S5 33
#define S6 32
#define S7 35
#define S8 34

// SENSOR LATERAL 

#define Sensor_L 25
#define Sensor_R 18

// PONTE H 

#define PWMA 14
#define PWMB 13

//ENCODER 

// #define ENCODERA_L 23
// #define ENCODERB_L 22
// #define ENCODERA_R 21
// #define ENCODERB_R 19

// PWM

#define DEFAULT_LEDC_FREQ 800
#define _resolution 10
#define A_channel 0
#define B_channel 1

double KP = 0;
double KD = 0;
double KI = 0;
double VEL = 100;

bool iniciar = LOW;
bool calibrar = LOW;

int contadorR = 0;
bool anteriorreadR = LOW;