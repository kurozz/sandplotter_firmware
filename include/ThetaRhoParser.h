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

typedef enum {
    THR_DIR_NORMAL = 0,
    THR_DIR_REVERSE
} ThetaRho_Direction_t;

class ThetaRhoParser {
public:
    ThetaRhoParser(std::string path);
    ThetaRhoParser(std::string path, ThetaRho_Direction_t reverse);
    ~ThetaRhoParser();

    ThetaRho_Error_t isOpen();
    ThetaRho_Error_t getNextCommand(float *theta, float *rho);
    uint32_t getTotalLines();
    uint32_t getCurrentLine();

private:
    ThetaRho_Error_t countLines();
    bool getLineReverse(char *line, uint32_t n);

    std::ifstream thrFile;
    ThetaRho_Direction_t direction;
    uint32_t totalLines;
    uint32_t currentLine;
    int32_t fileLength;
    const uint8_t LINE_BUFFER_LENGTH = 64;
};
