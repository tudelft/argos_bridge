#ifndef ENTM_MEMORY_H
#define ENTM_MEMORY_H

#include <vector>

class ENTMMemory {

public:

   ENTMMemory(const int vector_size);

   void write(std::vector<double> write_vector, double write_interpolation,
              double jump, double shift);

   std::vector<double> read();

private:

   const int VECTOR_SIZE_M;
   const int MIN_SIMILARITY_THRESHOLD;

   int head;

   std::vector<std::vector<double>> memory;

   std::vector<double> interpolate(std::vector<double> write_vector,
                                   double write_interpolation);
   void jumpHead(std::vector<double> write_vector);
   double vectorSimilarity(std::vector<double> v1, std::vector<double> v2);

   void printMemory();

};

#endif
