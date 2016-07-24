#include "MyVector.h"

MyVector::MyVector() {
//  if (vec_length > MAX_VEC_LEN) vec_len = MAX_VEC_LEN;
//  else vec_len = vec_length;
  vec_len = 25;
  float * vec_array = new float[MAX_VEC_LEN];
  
  for (unsigned int i = 0; i<vec_len; ++i) {
    vec_array[i] = 0;
  }
}

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

float MyVector::get_by_id(int rel_idx) {
  int index = (int(idx)+rel_idx)%int(vec_len);
  if (index<0) return vec_array[index+int(vec_len)];
  return vec_array[index];
}

float MyVector::get_avg() {
  float sum = 0;
  for (unsigned int i = 0; i<vec_len; ++i) {
    sum += vec_array[i];
  }
  return sum/(float)vec_len;
}

float MyVector::get_avg_diff() {
  return float(get_by_id(vec_len-1)-get_by_id(0))/float(vec_len-1);
}

void MyVector::push(float val) {
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

