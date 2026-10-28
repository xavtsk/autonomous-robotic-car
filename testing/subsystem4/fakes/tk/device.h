#ifndef S4_TEST_DEVICE_H
#define S4_TEST_DEVICE_H
typedef int SZ;
#define TD_UPDATE 3
ER tk_swri_dev(ID id, int start, const void *buffer, SZ length, SZ *actual);
ID tk_opn_dev(UB *name, unsigned mode);
#endif
