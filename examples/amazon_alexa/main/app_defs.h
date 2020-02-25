#ifndef __APP_DEFS_H__
#define __APP_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
	Jace. 191230. For CTC H/W Rev01. 
*/
#define CTC_REV01

/* 
	Jace. 191230. For audio preprocessor CS48L32.
*/
#define CTC_CS48L32

/*
	Jace. 200109. Ready or Disable Sensory's trigger function with CS48L32.
*/
#define CTC_CS48L32_SENSORY

/*
	Jace. 200117. Pre CS48L32's F/W 'SCSH_COOKE_MonoSCBI_dsp1_20190826.wmfw'.
*/
#define CTC_CS48L32_WMFW_20190826x

/*
	Jace. 200117. New CS48L32's F/W 'SCSH_COOKE_MonoSCBI_dsp1_3.4.11_12062019.wmfw'.
*/
#define CTC_CS48L32_WMFW_12062019

/*
	Jace. 200120. CS48L32's FLL source change from MCLK1 to ASP1_BCLK.
*/
#define CTC_CS48L32_FLL_ASP1_BCLK

/*
	Jace. 200206. Add Json parser for ASK.
*/
#define CTC_CS48L32_ASK

/*
	Jace. 200210. Add blynk's App. function.
*/
#define BLYNK_APPS

/*
	Jace. 200213. Using SENSORY's trigger.
*/
#define CTC_CS48L32_SENSORY_TRIGGER

/*
	Jace. 200225. Apply 1st tunning data for CS48L32.
*/
#define CTC_CS48L32_TUNE_1ST

/*
	Jace. 200225. Check CS48L32's register.
*/
#define CTC_CS48L32_CHECK_REGx

/*
	Jace. 200225. Only trigger test.
*/
#define CTC_TRIGGER_TEST

#ifdef __cplusplus
}
#endif

#endif
