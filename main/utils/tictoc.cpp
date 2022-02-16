#include "tictoc.h"
#include "echo.h"
#include <stdio.h>

std::chrono::_V2::system_clock::time_point t;

void tic() {
    t = std::chrono::high_resolution_clock::now();
}

void toc(const char *msg) {
    const auto dt = std::chrono::high_resolution_clock::now() - t;
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(dt);
    echo("%s took %.3f ms", msg, 0.001 * us.count());
}
