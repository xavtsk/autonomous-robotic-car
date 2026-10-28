#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H
#include "../app.h"
#include "s5_api.h"
#define S5_PRI_MAIN 20
ER subsystem5_init(void);
ER subsystem5_start(void);
#if APP_TEST
void subsystem5_test(void);
#endif
#endif
