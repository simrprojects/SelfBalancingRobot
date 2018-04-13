/**\file
 * \brief Modul dostarcza podstawowych dynamicznych blokow liniowych, 
 * jak obiekt inercyjny, calkujacy, rozniczkujacy czy PID
 */

#ifndef LINEARMODULES_H_
#define LINEARMODULES_H_

/**\brief Modu�z� dostarcza podstawowych blok�z�w dynamicznych liniowych, jak obbiekt inercyjny,
 * ca�z�kuj�z�cy r�z�niczkuj�z�cy czy PID
 */

typedef struct pidParameters_s{

  float kp;
  float ti;
  float td;
  float t1;
  
}tPidParameters;

typedef struct inerty_s{

  signed long lastOutput;
  unsigned char time;
  
}tInerty;

typedef struct loHiPassIir_s{

    float y1;
    float x1;


}loHiPassIir_t;

typedef struct {
    float x;
    float y;
    float a;
    float b;
}tDelay;

#define tRealDiv tDelay 

typedef struct integrator_s{
  float x;
  float preOutput;
  float output;
  float wspA;
  signed short feadBack;
}tIntegrator;

typedef struct{
    tIntegrator integrator;
    tDelay delay;
    float limit;
}tSpecialIntegrator;

typedef struct{
    float lastIn;
    float tp;
}tDiv;

typedef struct{

  float lastInput[2];
  float lastOutput[2];
  float wspA;
  float wspB;
  float wspC;
  float wspE;
  float wspF;
  
}tPidControler;

#define NCoef 4

typedef struct {
    float x[NCoef+1];
    float y[NCoef+1];
}loHiOrderPassIir_t;

typedef struct{
    char enable;
    float curentValue;
    float step;
}tSlope;

typedef struct{
    float a;
    float lim;
    float y;
    float x;
}tSmoothRamp;

typedef struct{
	float kp;
	float ki;
	tIntegrator integrator;
}tSimplePID;

//signed short CF_Execute(tControlerData* controler, signed short gyroValue, signed short accValue);
void Inerty_SetTime(tInerty* inerty, unsigned short value);
signed short Inerty_Execute(tInerty* inerty, signed short in);
void Integrator_SetTime(tIntegrator* integrator,float tp);
float Integrator_Execute(tIntegrator* integrator, signed char feedBack, float in);
void Integrator_Reset(tIntegrator* integrator);
void Integrator_SetFeadBackValue(tIntegrator* integrator, unsigned short val);
void Integrator_SetOutputState(tIntegrator* integrator,float in);
void Delay_Init(tDelay * delay,float time,float tp);
float xDelay_Execute(tDelay * delay,float in);
void xDelay_Reset(tDelay * delay);
void xDelay_SetState(tDelay * delay, float outValue);
void Div_Init(tDiv * d,float tp);
float Div_Execute(tDiv * d,float in);
void RealDiv_Init(tRealDiv * rDiv,float tD,float tI,float tp);
float RealDiv_Execute(tRealDiv * rDiv,float in);
void RealDiv_SetState(tRealDiv * rDiv,float y,float x);

void SimplePID_Init(tSimplePID *pid,float kp,float ki,float dt);
float SimplePID_Update(tSimplePID *pid,float e,signed int saturation);

void PidControler_Init(tPidParameters* params, tPidControler* pidControler);
float PidControler_Execute(tPidControler* pidControler, float in);

void InitHiLoPassIIRFilter(loHiPassIir_t * filtr);
float ExecuteHiPassIIRFilter(loHiPassIir_t * filtr, float inp);
float ExecuteLoPassIIRFilter(loHiPassIir_t * filtr, float inp);
signed long ExecuteHiPassFIIR(loHiPassIir_t * filtr, signed long inp);

void InitLoPassIIRFilterHiOrder(loHiOrderPassIir_t * filtr);
float ExecuteLoPassIIRFilterHiOrder(loHiOrderPassIir_t * filtr, float inp);

void Line(int X1, int Y1, int X2, int Y2, float*a_, float*b_);

void Slope_Reset(tSlope *slope,float slopeValue,float initValue,float tp);
float Slope_Update(tSlope *slope);

void SmoothRamp_Init(tSmoothRamp *smr,float ramp,float delay,float tp);
float SmoothRamp_Update(tSmoothRamp *smr,float in);
float SmoothRamp_UpdateWithSaturation(tSmoothRamp *smr,float in,signed int *saturation);
void SmoothRamp_UpdateParam(tSmoothRamp *smr,float ramp);


void SpecialIntegrator_Init(tSpecialIntegrator *integrator, float timeDelay, float integratorOffThreshold, float tp);
float SpecialIntegrator_Execute(tSpecialIntegrator *integrator, float value,char feedBack);

#endif
