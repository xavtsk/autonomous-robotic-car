void sys_timeout(unsigned ms, void (*callback)(void *), void *arg);
void sys_untimeout(void (*callback)(void *), void *arg);
