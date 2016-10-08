#ifndef TWIDDLE_H
#define TWIDDLE_H

#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))
#define TST(x,y) (x&(1<<y))
#define TOG(x,y) (x^=(1<<y))

#endif