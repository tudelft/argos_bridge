#include "entm_memory.h"

#include <iostream>
#include <algorithm>

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
                       double jump, double shiftl, double shiftn, double shiftr) {

   //Create new content jump default mem location
   if (head == memory.size()-1) memory.push_back(std::vector<double>(VECTOR_SIZE_M, 0.5));

   //Interpolate and write vector
   memory[head] = interpolate(write_vector, write_interpolation);

   //Jump head to most similar vector if content jump > 0.5
   if (jump > 0.5) jumpHead(write_vector);

   std::vector<double> shift_vector = {shiftl, shiftn, shiftr};
   std::vector<double>::iterator largest_shift_it = std::max_element(shift_vector.begin(), shift_vector.end());
   int largest_shift_pos = std::distance(shift_vector.begin(), largest_shift_it);

   if (largest_shift_pos == 0) {

      head -= 1;

      if (head < 0) head = memory.size() - 1;

   } else if (largest_shift_pos == 2) {

      head += 1;

      if (head >= memory.size()) head = 0;

   }

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
   std::vector<double> similarities(memory.size());

   for(int i = 0; i < memory.size(); i++) {

      similarities[i] = vectorSimilarity(write_vector, memory[i]);

   }

   std::vector<double>::iterator largest_sim_it = std::min_element(similarities.begin(), similarities.end());
   int largest_sim_pos = std::distance(similarities.begin(), largest_sim_it);

   //std::cout << *largest_sim_it << " " << MIN_SIMILARITY_THRESHOLD << std::endl;

   if (*largest_sim_it < MIN_SIMILARITY_THRESHOLD) {

      head = largest_sim_pos;

   } else {
      //std::cout << "Does not meet similarity threshold!" << std::endl;
      head = memory.size() - 1;

   }



}

double ENTMMemory::vectorSimilarity(std::vector<double> v1, std::vector<double> v2) {
   //std::cout << "Vec sim: " << v1[0] << " " << v2[0] << std::endl;
   double cumm_similarity = 0;

   for(int i = 0; i < VECTOR_SIZE_M; i++) {
      //std::cout << "Diff: " << fabs(v1[i] - v2[i]) << std::endl;

      cumm_similarity += fabs(v1[i] - v2[i]);

   }

   //std::cout << "Cumm sim: " << cumm_similarity << std::endl;

   return cumm_similarity / VECTOR_SIZE_M;

}

void ENTMMemory::printMemory() {

   for(int i = 0; i < memory.size(); i++) {

      for(int j = 0; j < VECTOR_SIZE_M; j++) {

         std::cout << memory[i][j] << " ";

      }

      std::cout << std::endl;

   }

   std::cout << "HEAD: " << head << std::endl;
   std::cout << "----------" << std::endl;

}
