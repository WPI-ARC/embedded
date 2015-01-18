void cap(float min, float* val, float max) {
    if(*val < min)
       *val = min;
    if(*val > max)
       *val = max;
}