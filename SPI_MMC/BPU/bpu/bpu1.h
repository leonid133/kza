#ifndef BPU1H
#define BPU1H

#define ToGrad 57.2957795130823
#define M_PI   3.14159265358979323846
#define D_PI 	6.28318530717958647692
#define M_PI_2 1,57079632679489661

extern bit flNoKoord, flCommand;
extern bdata unsigned char Dat37, Dat38;

extern xdata unsigned long LatFly, LonFly; 
extern xdata int koors, H_zad_buf;
extern xdata unsigned char SteckPoint, n_;
extern xdata char dataRSTSRC;
extern xdata float  H_filtr, Vz, V_dat, Vy_filtr, Vy_zad_buf, kren_dat, delta_e, delta_v, delta_g, delta_n, delta_z, delta_z_zad, Vy_zad_ogr_nab, nSU, Vtop, V_zad;

extern xdata char kren_zad, kren_zad_buf, Vz_zad, KrenKam_zad, UgolKam_zad;
extern xdata char RegimeKren, RegimeVy, RegimeV, RegimeSU, RegimeStart;
#endif