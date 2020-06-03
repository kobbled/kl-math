/PROG TEST_LS_MATH
/ATTR
COMMENT = "test karel math";
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP = *,*,*,*,*;
/MN
 : ! ***************** ;
 : ! Test karel cross-over math ;
 : ! functions ;
 : ! ======= ;
 : ! ***************** ;
 :  ;
 :  ;
 : ! absolute value test ;
 : R[290:dummy_var1]=(-3.5) ;
 : CALL MTH_ABS(R[290:dummy_var1],290) ;
 :  ;
 : ! exponent test ;
 : R[291:dummy_var2]=1 ;
 : CALL MTH_EXP(R[291:dummy_var2],291) ;
 :  ;
 : ! power test ;
 : R[292:dummy_var3]=2 ;
 : R[293:dummy_var4]=8 ;
 : CALL MTH_POW(R[292:dummy_var3],R[293:dummy_var4],292) ;
 :  ;
 : ! sqrt test ;
 : R[294:dummy_var5]=64 ;
 : CALL MTH_SQRT(R[294:dummy_var5],294) ;
 :  ;
 : LBL[101:end] ;
/END
