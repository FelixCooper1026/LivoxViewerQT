#ifndef LIVOXCORE_LIDARPARAMETERSERVICE_H
#define LIVOXCORE_LIDARPARAMETERSERVICE_H

#include <QString>

#include <cstdint>

namespace LidarParameterService {

QString formatValue(uint16_t key, uint8_t* value, uint16_t length);

} // namespace LidarParameterService

#endif // LIVOXCORE_LIDARPARAMETERSERVICE_H
