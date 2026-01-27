#pragma once
#include <Arduino.h>


class Utils{
    public:
        int clampInt(int v, int lo, int hi);
        int map100To255(int v);

    private:

};