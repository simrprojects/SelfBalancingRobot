/*
 * Loger.h
 *
 *  Created on: 04.04.2018
 *      Author: Przemek
 */

#ifndef INC_LOGER_H_
#define INC_LOGER_H_

typedef void* tLogerHandler;
typedef struct{
	int memoryPoolSize;/*wielkośc lokalnego bufora danych na logowane parametry
	 	 	 	 	 	 rozmiar tego bufora ma szczególne znaczenie w przypadku zapisu na nośniki, mające
	 	 	 	 	 	 tendencje do zatykania się jak karta SD*/
	int maxNumberOfParams;/*<maksymalna liczba logowanych parametrów*/
	int dtms;/*<okres wykonywania logu wyrażony w ms*/
}tLogerCfg;
typedef enum{eParamTypeU8,eParamTypeI8,eParamTypeU16,eParamTypeI16,eParamTypeU32,eParamTypeI32,eParamTypeSGL}tLogerParamType;

typedef enum{eLR_OpenSesjon=0,eLR_CloseSesjon}tLogerRequest;

typedef struct{
	void* src;
	char* name;
	tLogerParamType type;
}tParamDescriptor;

typedef struct{
	tParamDescriptor *paramsArray;
	int numberOfParams;
	void* logerStreamerHandler;
}tLogDescriptor;

typedef int(*tOpenLogerSesion)(tLogDescriptor*);
typedef int(*tCloseLogerSesion)(tLogDescriptor*);
typedef int(*tUpdateLogerSesion)(tLogDescriptor*);

typedef struct{
	void* logerStreamerHandler;
	tOpenLogerSesion open;
	tCloseLogerSesion close;
	tUpdateLogerSesion update;
}tLogerStreamerDriver;

int Loger_Create(tLogerHandler *h,tLogerCfg *cfg,tLogerStreamerDriver *drv);
int Loger_AddParams(tLogerHandler h,void* paramRef,char* paramName,tLogerParamType paramType);
int Loger_OpenSesion(tLogerHandler h);
int Loger_CloseSesion(tLogerHandler h);
int Loger_Convert2Text(char* buf,tLogerParamType type,void* data);
float Loger_Convert2Float(tLogerParamType type,void* data);

#endif /* INC_LOGER_H_ */
