#include "ThetaRhoParser.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#define END_OF_FILE (-1)
#define TRUNCATED (-2)
#define FILE_NOT_OPEN (-3)

ThetaRhoParser::ThetaRhoParser(std::string path) {
    thrFile.open(path, std::ifstream::in);

    totalLines = 0;
    currentLine = 0;
    direction = THR_DIR_NORMAL;

    if (countLines() == THR_FILE_ERROR) {
        ESP_LOGE(__FUNCTION__, "File not open");
    }

    thrFile.seekg(0, thrFile.end);
    fileLength = thrFile.tellg();
    thrFile.seekg(0, thrFile.beg);
}

ThetaRhoParser::ThetaRhoParser(std::string path, ThetaRho_Direction_t reverse) {
    thrFile.open(path, std::ifstream::in);

    totalLines = 0;
    direction = reverse;

    if (countLines() == THR_FILE_ERROR) {
        ESP_LOGE(__FUNCTION__, "File not open");
    }

    thrFile.seekg(0, thrFile.end);
    fileLength = thrFile.tellg();

    if (direction == THR_DIR_REVERSE) {
        thrFile.seekg(-1, thrFile.end);
        currentLine = totalLines;
    } else {
        currentLine = 0;
        thrFile.seekg(0, thrFile.beg);
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
    if (direction == THR_DIR_NORMAL) {
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
    } else {
        while (getLineReverse(line, LINE_BUFFER_LENGTH)) {
            currentLine--;

            char* strThetaEnd;
            char* strRhoEnd;

            *theta = strtof(line, &strThetaEnd);
            *rho = strtof(strThetaEnd, &strRhoEnd);

            if (strThetaEnd != line && strRhoEnd != strThetaEnd) {
                return THR_OK;
            }

            ESP_LOGD(__FUNCTION__, "[%ld/%ld] Invalid line", currentLine, totalLines);
        }
    }
    
    ESP_LOGD(__FUNCTION__, "End of file");
    return THR_EOF;
}

bool ThetaRhoParser::getLineReverse(char *line, uint32_t n) {
    if (thrFile.tellg() == 0) {
        return false;
    }

    int size = 0;
    char buffer[n];
    while (thrFile.tellg() >= 1) {
        char ch = (char) thrFile.get();
        thrFile.seekg(-2, std::ios_base::cur);

        if (ch == '\n') {
            buffer[size] = '\0';
            break;
        }

        buffer[size] = ch;
        size++;

        if (size > (n - 1)) {
            line[0] = '\0';
            return false;
        }
    }

    for (unsigned int i = 0; i < size; i++) {
        line[i] = buffer[(size - 1) - i];
    }
    line[size] = '\0';

    return true;
}
