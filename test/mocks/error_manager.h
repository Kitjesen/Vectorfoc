#ifndef TEST_MOCK_ERROR_MANAGER_H
#define TEST_MOCK_ERROR_MANAGER_H

#include <stdint.h>

void ErrorManager_Report(uint32_t error_code, const char *message);
void ErrorManager_ReportFull(uint32_t error_code, const char *message,
                             const char *file, uint32_t line);

#define ERROR_REPORT(code, msg) ((void)(code), (void)(msg))

#endif
