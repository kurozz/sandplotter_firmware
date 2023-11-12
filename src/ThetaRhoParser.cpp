#include "ThetaRhoParser.h"
#include "esp_log.h"

ThetaRhoParser::ThetaRhoParser(std::string path) {
    thrFile.open(path, std::ifstream::in);

    totalLines = 0;
    currentLine = 0;

    if (countLines() == THR_FILE_ERROR) {
        ESP_LOGE(__FUNCTION__, "File not open");
    }
}

ThetaRhoParser::~ThetaRhoParser() {
    if (isOpen() == THR_OK) {
        thrFile.close();
    } else {
        ESP_LOGD(__FUNCTION__, "File not open");
    }
}

ThetaRho_Error_t ThetaRhoParser::isOpen() {
    if (thrFile.is_open()) {
        return THR_OK;
    }

    return THR_FILE_ERROR;
}

ThetaRho_Error_t ThetaRhoParser::countLines() {
    if (isOpen() != THR_OK) {
        ESP_LOGD(__FUNCTION__, "File not open");
        return THR_FILE_ERROR;
    }

    char line[LINE_BUFFER_LENGTH];

    while (thrFile.getline(line, LINE_BUFFER_LENGTH)) {
        totalLines++;
    }

    thrFile.clear();
    thrFile.seekg(0);

    return THR_OK;
}

uint32_t ThetaRhoParser::getTotalLines() {
    return totalLines;
}

uint32_t ThetaRhoParser::getCurrentLine() {
    return currentLine;
}

ThetaRho_Error_t ThetaRhoParser::getNextCommand(float *theta, float *rho) {
    ThetaRho_Error_t error;

    error = isOpen();
    if (error != THR_OK) {
        ESP_LOGD(__FUNCTION__, "File not open");
        return error;
    }

    if (theta == nullptr || rho == nullptr) {
        ESP_LOGD(__FUNCTION__, "Invalid argument");
        return THR_INVALID_ARGUMENT;
    }

    char line[LINE_BUFFER_LENGTH];
    while ( thrFile.getline(line, LINE_BUFFER_LENGTH) ) {
        currentLine++;

        char* strThetaEnd;
        char* strRhoEnd;

        *theta = strtof(line, &strThetaEnd);
        *rho = strtof(strThetaEnd, &strRhoEnd);

        if (strThetaEnd != line && strRhoEnd != strThetaEnd) {
            return THR_OK;
        }

        ESP_LOGD(__FUNCTION__, "[%ld/%ld] Invalid line", currentLine, totalLines);
    }
    
    ESP_LOGD(__FUNCTION__, "End of file");
    return THR_EOF;
}
