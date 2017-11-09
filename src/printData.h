#include "yocto_math.h"
using namespace std;

void printFrame(const ym::frame<float,3>& M){
  printf("%.6g,%.6g,%.6g;\n %.6g,%.6g,%.6g;\n %.6g,%.6g,%.6g;\n %.6g,%.6g,%.6g;\n\n",
         M.x.x,M.y.x,M.z.x,
         M.x.y,M.y.y,M.z.y,
         M.x.z,M.y.z,M.z.z,
         M.o.z,M.o.z,M.o.z);
}

template <typename T, int N, int M>
void printMatrix(ym::mat<T, N, M> m){
  for(int i=0; i<M; i++) {
    for (int j = 0; j < N*M; j=j+N) {
      printf(" %.6g;    ", m.operator[](i+j));
    }
    printf("\n");
  }
  printf("\n");
}

template <typename T, int N>
void printVector(ym::vec<T, N> v){
  for(int i=0; i<N; i++) {
    printf(" %.6g;    ", v.operator[](i));
  }
  printf("\n");
}
template <typename T, int N>
void printVectorInt(ym::vec<T, N> v){
  for(int i=0; i<N; i++) {
    printf(" %i;    ", v.operator[](i));
  }
  printf("\n");
}
#ifndef GRAPHICS17B_HW01_PRINTDATA_H
#define GRAPHICS17B_HW01_PRINTDATA_H

#endif //GRAPHICS17B_HW01_PRINTDATA_H