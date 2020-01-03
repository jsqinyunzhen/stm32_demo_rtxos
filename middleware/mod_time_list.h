#include <string.h>

//#include "include.h"

//#include "MKL_ISR.h"


//#include "c20_board_config.h"


#ifndef _MOD_TIME_LIST_H
#define _MOD_TIME_LIST_H


#include "mod_date_time.h"

/* Exported Types ---------------------------------------------------------- */

/* Common unsigned types */
#ifndef DEFINED_u8
#define DEFINED_u8
typedef unsigned char  u8;
#endif

#ifndef DEFINED_u16
#define DEFINED_u16
typedef unsigned short u16;
#endif

#ifndef DEFINED_u32
#define DEFINED_u32
typedef unsigned int   u32;
#endif

/* Common signed types */
#ifndef DEFINED_s8
#define DEFINED_s8
typedef signed char  s8;
#endif

#ifndef DEFINED_s16
#define DEFINED_s16
typedef signed short s16;
#endif

#ifndef DEFINED_s32
#define DEFINED_s32
typedef signed int   s32;
#endif

typedef unsigned int app_error_t;

#if(S_SW_PERIOD_RECORD_DEBUG == 1)
#define   TIME_UNIT_MAX       (7)        //ʱ�䵥Ԫ�����ֵ 
#else
#define   TIME_UNIT_MAX       (6)        //ʱ�䵥Ԫ�����ֵ 
#endif
//#define   TIME_UNIT_MAX       (5)        //ʱ�䵥Ԫ�����ֵ 

#define     TIME_CTR2_SECOND    (1000)
#define     TIME_CTR2_MINUTE    (60*TIME_CTR2_SECOND)

#ifdef TIMER_LOW_MEM
//Program Size: Code=15116 RO-data=940 RW-data=116 ZI-data=1372  

//typedef void (*FunType)(int ); //��. ����һ������ָ������FunType,��ٺ�������һ��

typedef void (*FunType)( void );
typedef struct time_unit_s
{
  //  void( * pfun )( void );
    FunType pfun; 
    u8             is_register; //ע���ʶ
    u8             is_start;  //��Ч��ʶ
    u8             time_id;  //��ʱ��ʶ
    u8             priority;
    u32             interval;
    u32             curvalue;
    time_type_t     type;  //�����Ƕ�ʱ��ʱ
    time_run_mode_t run_mode;
} time_unit_t;



app_error_t     time_list_init( void );   // ��ʱ�б��ʼ��
//app_error_t     time_register( void* pfun, u32 uinterval, u8 priority,time_type_t type, time_run_mode_t fun_run_mode,u8* id );  //ע��һ����ʱ��Ԫ
app_error_t time_register(  FunType pfun, u32  uinterval,u8 priority, time_type_t       type,time_run_mode_t   fun_run_mode, u8* id );
app_error_t     time_start( u8 id );  //ʹ��һ����ʱ��Ԫ
app_error_t     time_stop( u8 id );  //ʹ����һ����ʱ��Ԫ
//app_error_t     time_restart( u16 id );
app_error_t time_restart( u8 id,u32 new_period);
unsigned char time_status_get( u8 id);
unsigned int time_period_get_by_id( u8 id);

app_error_t     time_isr_query( void );
app_error_t     time_task_query( void ); 
app_error_t     time_release( u8 id );
app_error_t     time_get_id( void* pfun, u8* id );
#else
//Program Size: Code=15116 RO-data=940 RW-data=124 ZI-data=1396  
typedef struct time_unit_s
{
    void( * pfun )( void );
    u16             is_register; //ע���ʶ
    u16             is_start;  //��Ч��ʶ
    u16             time_id;  //��ʱ��ʶ
    u16             priority;
    u32             interval;
    u32             curvalue;
    time_type_t     type;  //�����Ƕ�ʱ��ʱ
    time_run_mode_t run_mode;
} time_unit_t;



app_error_t     time_list_init( void );   // ��ʱ�б��ʼ��
app_error_t     time_register( void* pfun, u32 uinterval, u16 priority,
                               time_type_t type, time_run_mode_t fun_run_mode,
                               u16* id );  //ע��һ����ʱ��Ԫ
app_error_t     time_start( u16 id );  //ʹ��һ����ʱ��Ԫ
app_error_t     time_stop( u16 id );  //ʹ����һ����ʱ��Ԫ
//app_error_t     time_restart( u16 id );
app_error_t time_restart( u16 id,u32 new_period);
app_error_t     time_isr_query( void );
app_error_t     time_task_query( void ); 
app_error_t     time_release( u16 id );
app_error_t     time_get_id( void* pfun, u16* id );
#endif
void gxos_interrupt_enable(void);
void  gxos_interrupt_disable(void);

#endif

