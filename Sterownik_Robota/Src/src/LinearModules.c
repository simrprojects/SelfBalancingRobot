/**\file
 * \brief Modul dostarcza podstawowych dynamicznych blokow liniowych, 
 * jak obiekt inercyjny, calkujacy, rozniczkujacy czy PID
 */

#include "LinearModules.h"
#include "math.h"

#define TP 1

/**\brief Funkcja ustawia odpowiednie parametry dla obiektu inercyjnego
 * wywolanie tej funkcji resetuje wewnetrzne parametry modulu
 * /param inerty struktura przechowujaca 2 zmienne dla filtru: lastOutput i time,
            zadeklarowana w pliku LinearModules.h
 * /param value szybkosc reakcji filtru, opoznienie; im wiekszy, tym wolniej           
 */
void Inerty_SetTime(tInerty* inerty, unsigned short value){

  value++;
  inerty->time = 255 / (char)value;  
  inerty->lastOutput = 0;  
}

/**\brief Funkcja wykonuje pojedynczy cykl inercji - musi znajdowac sie w petli
 * /param inerty struktura przechowujaca 2 zmienne dla filtru: lastOutput i time,
            zadeklarowana w pliku LinearModules.h
 * /param in wartosc do odfiltrowania
 * /return wartosc odfiltrowana (f-cja musi byc wykonana wiele razy - 
            stad koniecznosc wywolywania jej w petli)
 */
signed short Inerty_Execute(tInerty* inerty, signed short in){
  
  signed short tmp=0;
  signed long tmp2=0;
  tmp = in - *(signed short*)((char*)&inerty->lastOutput+1);
  tmp2 = (signed long)tmp*inerty->time;
  inerty->lastOutput = inerty->lastOutput + tmp2;
  return *(signed short*)((char*)&inerty->lastOutput + 1);
}

void RealDiv_Init(tRealDiv * rDiv,float tD,float tI,float tp){
    rDiv->x=0.0;
    rDiv->y=0.0;
    rDiv->a = (2*tI-tp)/(2*tI+tp);
    rDiv->b = 2*tD/(2*tI+tp);
}
float RealDiv_Execute(tRealDiv * rDiv,float in){
    rDiv->y = rDiv->a*rDiv->y+rDiv->b*(in-rDiv->x);
    rDiv->x=in;
    return rDiv->y;
}
void RealDiv_SetState(tRealDiv * rDiv,float y,float x){
    rDiv->x=x;
    rDiv->y=y;
}
/**\brief Funkcja inicjuje strukture delay. Delay to dyskretna realizacja
 * obiektu inercyjnego pierwszego rzedu.
 * \param delay wskaznik do dyskretnej realizacji obiektu inercyjnego pierwszego rzedu
 * \param time czas wyrazony jest w sekundach
 */
void xDelay_Init(tDelay * delay,float time,float tp){
    delay->x=0.0;
    delay->y=0.0;
    delay->a = (2*time-tp)/(2*time+tp);
    delay->b = tp/(2*time+tp);
}

/**\brief Wykonanie pojedynczego kroku obliczeniowego funkcji Delay
 * \param delay wskaznik do dyskretnej realizacji obiektu inercyjnego pierwszego rzedu
 * \param in wartosc wejsciowa
 * \return wartosc po obliczeniach
 */
float xDelay_Execute(tDelay * delay,float in){
    delay->y = delay->a*delay->y+delay->b*(delay->x+in);
    delay->x=in;
    return delay->y;
}
/**\brief Funkcja resetuje wewnetrzny stan obiektu
 */
void xDelay_Reset(tDelay * delay){
    delay->x=0.0;
    delay->y=0.0;
}
/**\brief Uaktualnienie stanu wewnetrznego delay do zadanej wartosci
 */
void xDelay_SetState(tDelay * delay, float outValue){
    delay->y = outValue;
}
/**\brief Funkcja konfigurujaca obiekt calkujacy. Funkcja resetuje 
 * wewnetrzny stan urzadzenia
 * \param integrator wskaznik do struktury obiektu calkujacego
 * \param time czas
 */
void Integrator_SetTime(tIntegrator* integrator,float tp){
  integrator->wspA = 2.0f/tp;
  integrator->preOutput = 0;
  integrator->x = 0;  
}

/**\brief Funkcja wykonuje pojedynczy cykl calkujacy. 
 * \param integrator wskaznik do struktury obiektu calkujacego
 * \param feedBack flaga informujaca o sprzezeniu zwrotnym
 * \param in wartosc wejsciowa
 * \return wartosc po obliczeniach
 */
float Integrator_Execute(tIntegrator* integrator, signed char feedBack, float in){
  if(feedBack==1){
    if(in>0){
      return integrator->output;  
    }
  }
  if(feedBack==-1){
    if(in<0){
      return integrator->output; 
    }
  }
  integrator->output += ((in+integrator->x)/integrator->wspA);
  integrator->x = in;
  return  integrator->output;
}
/**\brief Funcja ustawia stan werwntrzny integratora tak, by przy wywolaniu funkcji Execute z wartoscia
 * wejsioiwa 0, funkcja zwrocila wartosc in
 */
void Integrator_SetOutputState(tIntegrator* integrator,float in){

    integrator->preOutput = in*integrator->wspA;
    integrator->x=0;
    integrator->output = in;
}
/**\brief Sprzezenie zwrotne od wyjscia regulatora
 * \param integrator wskaznik do struktury obiektu calkujacego
 * \param val flaga informujaca o sprzezeniu zwrotnym
 */ 
void Integrator_SetFeadBackValue(tIntegrator* integrator, unsigned short val){
  integrator->feadBack = val;
}

/**\brief Funkcja resetuje wewnetrzny stan integratora, 
 * tak by zaczal on liczyc od poczatku
 * \param integrator wskaznik do struktury obiektu calkujacego
 */

void Integrator_Reset(tIntegrator* integrator){
    integrator->preOutput = 0;
    integrator->x = 0;
    integrator->output=0;
}
/**\brief Funkcja inicjuje integrator specjalnego rodzaju
 */
void SpecialIntegrator_Init(tSpecialIntegrator *integrator, float timeDelay, float integratorOffThreshold, float tp){
    Integrator_SetTime(&integrator->integrator,tp);
    xDelay_Init(&integrator->delay,timeDelay,tp);
    integrator->limit=fabs(integratorOffThreshold);
}

float SpecialIntegrator_Execute(tSpecialIntegrator *integrator, float value,char feedBack){
    float d;
    d=xDelay_Execute(&integrator->delay,value);
    if(fabs(d)<integrator->limit){
        //znajduje sie w obszaze resetowania integratora
        Integrator_Reset(&integrator->integrator);
        //zwracam sygnal z delay
        return d;
    }else{
        //znajduje sie w obszaze calkowania
        return Integrator_Execute(&integrator->integrator,feedBack,value)*d;
    }
}

/**\Inicjacja rozniczkowania
 * \param d wskaznik do struktury obiektu rozniczkujacego
 */
void Div_Init(tDiv * d,float tp){
    d->lastIn = 0.0;
    d->tp = tp;
}

/**\brief Wykonanie pojedynczego cyklu rozniczkowania
 * \param d wskaznik do struktury obiektu rozniczkujacego
 * \param in wartosc wejsciowa
 * \return wynik po obliczeniach
 */
float Div_Execute(tDiv * d,float in){
    float tmp;
    tmp = (in-d->lastIn)/d->tp;
    d->lastIn = in;
    return tmp;
}
/**\funkcja Slope realiuje funkcje liniowo opadajcej/narastajacej do zera rampy
 * Funkcja inicjowana jest za pomoca procedury Reset w kt�rej poszczegolne parametry maja nastepujace znaczenie
 * slope - wskaznik do struktury
 * slopeValue - okresla stromosc rampy, o ile ma zmniejszyc sie wartosc wyjsciowa po okresie 1 sekundy
 * initValue - poczatkowa wartosc ktora bedzie stopniowo malala
 * tp - okres wykonywania funkcji update
 */
void Slope_Reset(tSlope *slope,float slopeValue,float initValue,float tp){
    slope->enable=1;
    slope->curentValue=initValue;
    if(initValue>0){
        slope->step=tp*slopeValue;
    }else{
        slope->step=-tp*slopeValue;
    }
}
/**\brief Funkcja uaktualnia rampe. zwraca jej aktualna wartosc
 */
float Slope_Update(tSlope *slope){
    if(slope->enable==0){
        //zakonczony proces
        return 0.0;
    }else{
        if(fabs(slope->curentValue)>fabs(slope->step)){
            slope->curentValue-=slope->step;
            return slope->curentValue;
        }else{
            slope->enable=0;
            return 0.0;
        }
    }
}
/**\ ModuB SmoothRamp jest polaczeniem bloku inercyjnego z blokiem liniowo narastajacej rampy
 * W module okreslone sa maksymalne przyrosty wartosci wyznaczanej przez funkcje Update.
 * Jest to zatem funkcja liniowo narastajacej rampy. Dzieki dodaniu bloku inercyjnego
 * jednoczesnie uzyskuje sie gladkie whodzenie i wychodzenie z obszaru liniowego narastania sygnalu
 * Funkcja inicjuje modul
 */
void SmoothRamp_Init(tSmoothRamp *smr,float ramp,float delay,float tp){
    smr->a=0.5f*tp/delay;
    smr->lim=ramp*tp;
    smr->y=0;
    smr->x=0;
}
/**\brief Funkcjaz uaktualnia stan rampy
 */
float SmoothRamp_Update(tSmoothRamp *smr,float in){
    float x=in-smr->y;
    in = smr->a*(smr->x+x);
    if(in>smr->lim){
        in=smr->lim;
    }else if(in<-smr->lim){
        in = -smr->lim;
    }
    smr->y+=in;
    smr->x=x;
    return smr->y;
}
/**\brief Funkcjaz uaktualnia stan rampy oraz dodatkowo informuje o osiągnięciu limitu
 */
float SmoothRamp_UpdateWithSaturation(tSmoothRamp *smr,float in,signed int *saturation){
    float x=in-smr->y;
    in = smr->a*(smr->x+x);
    if(in>smr->lim){
				*saturation=1;
        in=smr->lim;
    }else if(in<-smr->lim){
        in = -smr->lim;
				*saturation=-1;
    }
    smr->y+=in;
    smr->x=x;
    return smr->y;
}
/**\brief Funkcja uaktualnia parametr okreslajacy szybkosc narastania rampy
 * Przy okazji funkcja resetuje wewnetrzny stan rampy
 */
void SmoothRamp_UpdateParam(tSmoothRamp *smr,float ramp){
}

/**
  * @brief  Funkcja inicjuje prosty obiekt regulatora PI
  * @param[in]  None
  * @retval None
  */
void SimplePID_Init(tSimplePID *pid,float kp,float ki,float dt){
	pid->ki = ki;
	pid->kp = kp;
	Integrator_SetTime(&pid->integrator,dt);

}
/**
  * @brief  Funkcja uaktualnia sygnał sterujący na podstawie sygnału błędu
  * @param[in]  None
  * @retval None
  */
float SimplePID_Update(tSimplePID *pid,float e,signed int saturation){
	return e*pid->kp + Integrator_Execute(&pid->integrator,saturation,e)*pid->ki;
}

/**\brief Funkcja konfigurujaca regulator PID. Funkcja resetuje 
 * wewnetrzny stan urzadzenia
 * \param params wskaznik do parametrow PIDa
 * \param pidControler wskaznik do parametrow regulatora
 */
void PidControler_Init(tPidParameters* params, tPidControler* pidControler){

  float wspD,wspX,wspY;
  pidControler->lastInput[0]=0;
  pidControler->lastInput[1]=0;
  pidControler->lastOutput[0] = 0;
  pidControler->lastOutput[1] = 0;
  wspD = 2*params->t1+TP;
  wspX = 2*(params->t1+params->td);
  wspY = (TP+2*params->t1)/(2*params->ti);
  pidControler->wspA = params->kp*(wspX+wspY+TP)/wspD;
  pidControler->wspB = params->kp*(-2*wspX+TP*TP/params->ti)/wspD;
  pidControler->wspC = params->kp*(wspX-TP+(TP-2*params->t1)/2/params->ti)/wspD;
  pidControler->wspE = -4*params->t1/wspD;
  pidControler->wspF = (2*params->t1-TP)/wspD;
}

/**\brief Funkcja wykonuje pojedynczy cykl akcji PID
 * \param pidControler wskaznik do parametrow regulatora
 * \param in wartosc wejsciowa
 * \return wartosc po obliczeniach
 */
float PidControler_Execute(tPidControler* pidControler, float in){

  float tempOut;
  
  tempOut = pidControler->lastOutput[0];
  pidControler->lastOutput[0] = -pidControler->wspE*pidControler->lastOutput[0]-pidControler->wspF*pidControler->lastOutput[1]; 
  pidControler->lastOutput[0] += pidControler->wspA*in; 
  pidControler->lastOutput[0] += pidControler->wspB*pidControler->lastInput[0];
  pidControler->lastOutput[0] += pidControler->wspC*pidControler->lastInput[1];
  pidControler->lastOutput[1] = tempOut;
  pidControler->lastInput[1] = pidControler->lastInput[0];
  pidControler->lastInput[0] = in;
  return pidControler->lastOutput[0];
  
}

/**\brief Funkcja inicjuje parametry filtru
 * W filtrze tym nie ma mozliwosci ustawienia stalych czasowych.
 * Parametr ten ustawia sie poprzez zmiane wspolczynnikow definicji
 * \param filtr wskaznik do filtra
 */
void InitHiLoPassIIRFilter(loHiPassIir_t * filtr){

    filtr->y1 = 0;
    filtr->x1 = 0;
}

/**\brief Funkcja wyznacza odpowiedz dla filtru gorno przepustowego
 * \param filtr wskaznik do filtra
 * \param inp wartosc wejsciowa
 * \return wartosc po obliczeniach
 * typu IIR
 *      (2T-Tp)          2
 * y(k)=-------y(k-1)+ -------(x(k)-x(k-1))
 *      (2T+Tp)        (2T+Tp)
 *
 *         ah             fh
 */
float ExecuteHiPassIIRFilter(loHiPassIir_t * filtr, float inp){
#define ah 0.5f
#define bh 0.f
#define ch 0.f
#define dh 0.f
#define eh 1.f
#define fh 0.32491969625421685f  //Tp=0.005 f=30
#define gh 0.f
#define hh 0.f
    filtr->y1 = fh*filtr->y1+ah*(inp - filtr->x1);
    filtr->x1 = inp;
    return filtr->y1;
    
}

/**\brief Funkcja wyznacza odpowiedz dla filtru dolno przepustowego typu IIR
 * \param filtr wskaznik do filtra
 * \param inp wartosc wejsciowa
 * \return wartosc po obliczeniach
 */
float ExecuteLoPassIIRFilter(loHiPassIir_t * filtr, float inp){
#define al 0.00196165100094654930f
#define bl 0.f
#define cl 0.f
#define dl 0.f
#define el 1.f
#define fl 0.99686333183343800000f
#define gl 0.f
#define hl 0.f
    //filtr->x4 = filtr->x3;
    //filtr->x3 = filtr->x2;
    //filtr->x2 = filtr->x1;
    
    //filtr->y4 = filtr->y3;
    //filtr->y3 = filtr->y2;
    filtr->y1 = fl*filtr->y1+al*(inp+filtr->x1);
    filtr->x1 = inp;
    return filtr->y1;
    
}

/**\brief Funkcja filtracji gornoprzepustowej
 * \param filtr wskaznik do filtra
 * \param inp wartosc wejsciowa
 * \return wartosc po obliczeniach
 */
signed long ExecuteHiPassFIIR(loHiPassIir_t * filtr, signed long inp){
#define ahf 0.8687f
#define bhf -0.8687f
#define chf 0.7374f

    //filtr->x4 = filtr->x3;
    //filtr->x3 = filtr->x2;
    //filtr->x2 = filtr->x1;
    //filtr->y4 = filtr->y3;
    //filtr->y3 = filtr->y2;
    //filtr->y2 = filtr->y1;
    filtr->y1 = chf*filtr->y1+ahf*(float)inp+bhf*filtr->x1;
    filtr->x1 = (float)inp;
    return (signed long)filtr->y1;
    
}

/**\brief Funkcja wyznacza odpowiedz dla filtru dolno przepustowego typu IIR
 * \param filtr wskaznik do filtra
 * \param inp wartosc wejsciowa
 * \return wartosc po obliczeniach
 */
float ExecuteLoPassIIRFilterHiOrder(loHiOrderPassIir_t * filtr, float inp){
    static const float ACoef[NCoef+1] = {
        0.00038924323486576795,
        0.00155697293946307180,
        0.00233545940919460770,
        0.00155697293946307180,
        0.00038924323486576795
    };

    static const float BCoef[NCoef+1] = {
        1.00000000000000000000,
        -3.06594882157537720000,
        3.57378745285616970000,
        -1.87441156447673760000,
        0.37279467617297274000
    };

    float * y; //output samples
    float * x; //input samples
    int n;
    y = &filtr->y[0];
    x = &filtr->x[0];
    //shift the old samples
    for(n=(NCoef); n>0; n--) {
       x[n] = x[n-1];
       y[n] = y[n-1];
    }

    //Calculate the new output
    x[0] = inp;
    y[0] = ACoef[0] * x[0];
    for(n=1; n<=NCoef; n++)
        y[0] += ACoef[n] * x[n] - BCoef[n] * y[n];
    
    return y[0];
    
}

/**\brief Funkcja inicjujaca filtr dolnoprzepustowy typu IIR
 * \param filtr wskaznik do filtra
 */
void InitLoPassIIRFilterHiOrder(loHiOrderPassIir_t * filtr){

    int i;
    for(i=0;i<=NCoef;i++){
        filtr->y[i]=0.0;
        filtr->x[i]=0.0;
    }
}

/**\brief Funkcja obliczajaca wzor funkcji liniowej na podstawie wspolrzednych 2 punktow
 * \param X1 wspolrzedna X pierwszego punktu
 * \param Y1 wspolrzedna Y pierwszego punktu
 * \param X2 wspolrzedna X drugiego punktu
 * \param Y2 wspolrzedna Y drugiego punktu
 * \param a_ wskaznik zmiennej typu float - wynik: wspolczynnik a
 * \param b_ wskaznik zmiennej typu float - wynik: wspolczynnik b
 */
void Line(int X1, int Y1, int X2, int Y2, float*a_, float*b_){

    if((X1-X2) != 0){
        float a1 = (Y1-Y2);
        float a2 = (X1-X2);
        float a =  (a1/a2);
        float b = Y2-(a*X2);
    
        *a_=a;
        *b_=b;
    }
}
