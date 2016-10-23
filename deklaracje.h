#ifndef DEKLARACJE_H
#define DEKLARACJE_H

/* -----------------------------------------------------------------------------------------
 * Wszystkie zmienne globalne oraz funkcje deklarowane w innych plikach
 * -----------------------------------------------------------------------------------------
 */



void InitSensors (void);
void InitTimer1(void);
void InitADC(void);
void InitUSART (void);
void InitLED(void);
void InitMotors(void);
void InitADC(void);
void InitTurbine(void);

void ReadSensors(void);
void CountErr (void);
void CountAdjustment(void);
int GetCharFromUSART (void);
void ReadFromUSART(void);
void MotorP(int start, int force);
void MotorL(int start, int force);
void MotorsControl(void);
void Calibrate(void);
void CheckBattery(void);
void InitTimer8(void);

extern int weight[14];
extern int sensor[14];
extern int memory[256];
extern volatile uint16_t ADCBuffer[15];


extern double err_i;
extern double err_d;
extern double prev_time;
extern volatile int cycle;

extern int speed_err;
extern int lost;
extern int prev_lost;
extern int number;
extern int prev_err ;
extern int err ;
extern int adjustment;
extern int Kp;
extern int Ki;
extern int Kd;
extern int Kl;
extern int Tp;
extern double force_prescaler;
extern int remote_control;
extern int border;
extern int turbine_power;

extern int id_Kp;
extern int id_Ki;
extern int id_Kd;
extern int id_Kl;
extern int id_force_prescaler;
extern int id_turbine_power;
extern int id_border;
extern int id_weight;
extern int id_mem;

int led_on_time;
int led_on;
void LedSendConfirm(void);


//irda
extern volatile int irbuf[34];
extern volatile int iridx;
extern const int MAX_IR;
extern const int IRREADY;
extern int ir_addr;
extern int ir_data;
extern int ir_valid;
void IrReceive(void);
void init_EXTI (void);
int id_ir_data;
int id_ir_addr;
void IrControl(void);

extern volatile uint8_t ids;
extern volatile uint8_t idr;
extern volatile int send;
extern char tbl[256];

#endif
