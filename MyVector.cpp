#include "MyVector.h"

MyVector::MyVector(int vec_length) {
//  if (vec_length > MAX_VEC_LEN) vec_len = MAX_VEC_LEN;
//  else vec_len = vec_length;
  vec_len = vec_length;
  float * vec_array = new float[vec_length];
  
  for (unsigned int i = 0; i<vec_len; ++i) {
    vec_array[i] = 0;
  }
}

MyVector::MyVector(int vec_length, float * vec_in) {
//  if (vec_length > MAX_VEC_LEN) vec_len = MAX_VEC_LEN;
//  else vec_len = vec_length;
  vec_len = vec_length;
  float * vec_array = new float[vec_length];

  for (unsigned int i = 0; i<vec_len; ++i) {
    vec_array[i] = vec_in[i];
  }
}

float MyVector::get_element(unsigned int rel_idx) {
  return vec_array[(idx+rel_idx)%vec_len];
}

float MyVector::get_avg() {
  float sum = 0;
  for (unsigned int i = 0; i<vec_len; ++i) {
    sum += vec_array[i];
  }
  return sum/vec_len;
}

float MyVector::get_avg_diff() {
  return float(get_element(vec_len-1)-get_element(0))/float(vec_len-1);
}

void MyVector::add_element(float val) {
  vec_array[idx] = val;
  ++idx;
  if (idx >= vec_len) idx = 0;
}

void MyVector::fill_with(float val) {
  for (unsigned int i = 0; i<vec_len; ++i) {
    vec_array[i] = val;
  }
  idx = 0;
}

