#pragma once

#include <iostream>
#include <fstream>
#include <string>

typedef enum {
    THR_OK = 0,
    THR_FILE_ERROR,
    THR_INVALID_ARGUMENT,
    THR_EOF,
    THR_INVALID_LINE
} ThetaRho_Error_t;

class ThetaRhoParser {
public:
    ThetaRhoParser(std::string path);
    ~ThetaRhoParser();

    ThetaRho_Error_t isOpen();
    ThetaRho_Error_t getNextCommand(float *theta, float *rho);
    uint32_t getTotalLines();
    uint32_t getCurrentLine();

private:
    ThetaRho_Error_t countLines();
    std::ifstream thrFile;
    uint32_t totalLines;
    uint32_t currentLine;
    const uint8_t LINE_BUFFER_LENGTH = 64;
};
