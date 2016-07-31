#ifndef MYVECTOR
#define MYVECTOR

#define MAX_VEC_LEN   15

class MyVector {
  public:
    MyVector();
    MyVector(int vec_length);
    MyVector(int vec_length, float * vec_in);
    float get_by_id(int rel_idx);
    float get_avg();
    float get_avg_diff();
    void push(float val);
    void fill_with(float val);

  private:
    float vec_array[MAX_VEC_LEN];
    unsigned int idx = 0;
    unsigned int vec_len;
};

#endif
