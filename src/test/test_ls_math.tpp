# *****************
# Test karel cross-over math functions
# =======
# *****************

TP_GROUPMASK = "*,*,*,*,*"
TP_COMMENT = "test karel math"

#absolute value test
Dummy_1 = -3.5
MTH_ABS(Dummy_1, &Dummy_1)

#exponent test
Dummy_2 = 1
MTH_EXP(Dummy_2, &Dummy_2)

#power test
Dummy_3 = 2
Dummy_4 = 8
MTH_POW(Dummy_3, Dummy_4, &Dummy_3)

#sqrt test
Dummy_5 = 64
MTH_SQRT(Dummy_5, &Dummy_5)


@end