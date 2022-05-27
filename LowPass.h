// A simple low-pass filter that takes the average of the past N samples.
template<typename T, int N>
struct LowPass {
    T buf[N];
    int i;
    T err;

    LowPass() : buf{0}, i(0), err(0) {
    }

    // Updates the filter with a new sample, returning the new average.
    T update(T val) {
        buf[i++] = val;
        i %= N;

        T result = err;
        for (int j = 0; j < N; j++) {
            T sample = (buf[j] + err);
            T scaled = sample / N;
            result += scaled;
            err = sample - (scaled * N);
        }

        return result;
    }
};
