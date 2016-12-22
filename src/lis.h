// LIS3DH MEMS motion sensor

#ifndef LIS3DH_H__
#define LIS3DH_H__

void s_lis_measure(void *s);
bool s_lis_init();
bool s_lis_term();
void s_lis_poll(void *g);

#endif // LIS3DH_H__
