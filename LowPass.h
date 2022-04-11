template<typename T, int N>
struct LowPass {
    T buf[N];
    int i;
    T err;

    LowPass() : buf{0}, i(0), err(0) {
    }

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
