#include "helpers/PacketCaptureProtocol.h"

#include <pcap.h>

#include <csignal>
#include <cstdio>
#include <unistd.h>

namespace {

volatile std::sig_atomic_t stopRequested = 0;

void handleSignal(int)
{
    stopRequested = 1;
}

bool writeAll(const void* data, std::size_t size)
{
    const auto* bytes = static_cast<const unsigned char*>(data);
    while (size > 0) {
        const ssize_t written = write(STDOUT_FILENO, bytes, size);
        if (written <= 0) {
            return false;
        }
        bytes += written;
        size -= std::size_t(written);
    }
    return true;
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::fputs("Missing capture interface.\n", stderr);
        return 1;
    }

    std::signal(SIGTERM, handleSignal);
    std::signal(SIGINT, handleSignal);

    char errorBuffer[PCAP_ERRBUF_SIZE]{};
    pcap_t* capture = pcap_create(argv[1], errorBuffer);
    if (!capture) {
        std::fprintf(stderr, "%s\n", errorBuffer);
        return 2;
    }

    pcap_set_snaplen(capture, 65535);
    pcap_set_promisc(capture, 1);
    pcap_set_timeout(capture, 250);
    const int activateResult = pcap_activate(capture);
    if (activateResult < 0) {
        std::fprintf(stderr, "%s\n", pcap_geterr(capture));
        pcap_close(capture);
        return 3;
    }

    const PacketCaptureProtocol::StreamHeader streamHeader{
        PacketCaptureProtocol::kStreamMagic,
        pcap_datalink(capture)
    };
    if (!writeAll(&streamHeader, sizeof(streamHeader))) {
        pcap_close(capture);
        return 4;
    }

    while (!stopRequested) {
        pcap_pkthdr* header = nullptr;
        const unsigned char* data = nullptr;
        const int result = pcap_next_ex(capture, &header, &data);
        if (result == 0) {
            continue;
        }
        if (result < 0) {
            if (result == -1) {
                std::fprintf(stderr, "%s\n", pcap_geterr(capture));
            }
            pcap_close(capture);
            return result == -1 ? 5 : 0;
        }

        const PacketCaptureProtocol::PacketHeader packetHeader{
            header->ts.tv_sec,
            header->ts.tv_usec,
            header->caplen,
            header->len
        };
        if (!writeAll(&packetHeader, sizeof(packetHeader)) ||
            !writeAll(data, packetHeader.capturedLength)) {
            pcap_close(capture);
            return 0;
        }
    }

    pcap_close(capture);
    return 0;
}
