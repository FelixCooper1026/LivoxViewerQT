#ifndef LIVOXCORE_LIDARTEXTUTILS_H
#define LIVOXCORE_LIDARTEXTUTILS_H

#include <QString>

namespace LivoxCore {

inline QString fixedLatin1String(const char* data, int size)
{
    int length = 0;
    while (length < size && data[length] != '\0') {
        ++length;
    }
    return QString::fromLatin1(data, length).trimmed();
}

} // namespace LivoxCore

#endif // LIVOXCORE_LIDARTEXTUTILS_H
