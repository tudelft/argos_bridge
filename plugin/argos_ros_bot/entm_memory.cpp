#include "entm_memory.h"

#include <iostream>

//Initialise memory
ENTMMemory::ENTMMemory(const int vector_size) : VECTOR_SIZE_M(vector_size),
                                                MIN_SIMILARITY_THRESHOLD(2.0) {

   memory = std::vector<std::vector<double>>(1, std::vector<double>(VECTOR_SIZE_M, 0.5));
   head = 0;

   std::cout << "Memory created" << std::endl;

}

std::vector<double> ENTMMemory::read() {
   // std::cout << head << std::endl;
   return memory[head];

}

void ENTMMemory::write(std::vector<double> write_vector, double write_interpolation,
                       double jump, double shift) {

   //Create new content jump default mem location
   if (head == memory.size()-1) memory.push_back(std::vector<double>(VECTOR_SIZE_M, 0.5));

   //Interpolate and write vector
   memory[head] = interpolate(write_vector, write_interpolation);

   //Jump head to most similar vector if content jump > 0.5
   if (jump > 0.5) jumpHead(write_vector);

   //Shift the head according the the shift input
   if (shift < 0.33) {

      head -= 1;

      if (head < 0) head = memory.size() - 1;

   } else if (shift >= 0.66) {

      head += 1;

      if (head >= memory.size()) head = 0;

   }

   printMemory();
   std::cout << "----------" << std::endl;

}

std::vector<double> ENTMMemory::interpolate(std::vector<double> write_vector,
                                            double write_interpolation) {

   std::vector<double> interpolation_vector(VECTOR_SIZE_M, write_interpolation);

   std::vector<double> old_blend(VECTOR_SIZE_M);
   std::vector<double> new_blend(VECTOR_SIZE_M);

   for(int i = 0; i < VECTOR_SIZE_M; i++) {

      old_blend[i] = memory[head][i] * (1 - interpolation_vector[i]);
      new_blend[i] = write_vector[i] * interpolation_vector[i];

   }

   std::vector<double> interpolated_memory_vector(VECTOR_SIZE_M);

   for(int i = 0; i < VECTOR_SIZE_M; i++) {

      interpolated_memory_vector[i] = old_blend[i] + new_blend[i];

   }

   return interpolated_memory_vector;

}

void ENTMMemory::jumpHead(std::vector<double> write_vector) {

   //Check for most similar vector

   int most_similar_index = 0;
   double largest_similarity = 0.0;

   for(int i = 0; i < memory.size(); i++) {

      double current_similarity = vectorSimilarity(write_vector, memory[i]);

      if(current_similarity > largest_similarity) {

         most_similar_index = i;
         largest_similarity = current_similarity;

      }

   }

   if (largest_similarity < MIN_SIMILARITY_THRESHOLD) {

      head = most_similar_index;

   } else {

      head = VECTOR_SIZE_M - 1;

   }



}

double ENTMMemory::vectorSimilarity(std::vector<double> v1, std::vector<double> v2) {

   double cumm_similarity = 0;

   for(int i = 0; i < VECTOR_SIZE_M; i++) {

      cumm_similarity += abs(v1[i] - v2[i]);

   }

   return cumm_similarity / VECTOR_SIZE_M;

}

void ENTMMemory::printMemory() {

   for(int i = 0; i < memory.size(); i++) {

      for(int j = 0; j < VECTOR_SIZE_M; j++) {

         std::cout << memory[i][j] << " ";

      }

      std::cout << std::endl;

   }

}
