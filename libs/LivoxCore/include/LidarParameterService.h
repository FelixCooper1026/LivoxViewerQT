#ifndef LIVOXCORE_LIDARPARAMETERSERVICE_H
#define LIVOXCORE_LIDARPARAMETERSERVICE_H

#include <QString>

#include <cstdint>

namespace LidarParameterService {

QString formatValue(uint16_t key, uint8_t* value, uint16_t length, uint8_t deviceType = 0);
bool isOptionSupported(uint8_t deviceType, uint16_t key, int option);

} // namespace LidarParameterService

#endif // LIVOXCORE_LIDARPARAMETERSERVICE_H
