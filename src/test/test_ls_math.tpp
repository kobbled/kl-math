# *****************
# Test karel cross-over math functions
# =======
# *****************

TP_GROUPMASK = "*,*,*,*,*"
TP_COMMENT = "test karel math"

#absolute value test
dummy_var1 = -3.5
MTH_ABS(dummy_var1, &dummy_var1)

#exponent test
dummy_var2 = 1
MTH_EXP(dummy_var2, &dummy_var2)

#power test
dummy_var3 = 2
dummy_var4 = 8
MTH_POW(dummy_var3, dummy_var4, &dummy_var3)

#sqrt test
dummy_var5 = 64
MTH_SQRT(dummy_var5, &dummy_var5)


@end