#include <stdio.h>
/* 
 * gcc Source/expgen.c -o expgen && ./expgen 
 */
int main(int argc, char** argv){
  double rate = 1.08;
  double value = 0.1;
  int i;
  for(i=0; i<128; ++i){
    printf("%d, ", (int)value);
    value *= rate;
  }
}
