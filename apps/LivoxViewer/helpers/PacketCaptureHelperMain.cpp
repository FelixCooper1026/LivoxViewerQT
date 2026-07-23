#include "helpers/PacketCaptureProtocol.h"

#include <pcap.h>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <poll.h>
#include <string>
#include <unistd.h>

namespace {

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

bool readCommand(std::string& command)
{
    command.clear();
    char character = 0;
    while (read(STDIN_FILENO, &character, 1) == 1) {
        if (character == '\n') {
            return true;
        }
        command.push_back(character);
    }
    return false;
}

bool writeCaptureStopped()
{
    const PacketCaptureProtocol::PacketHeader stoppedHeader{
        0,
        0,
        PacketCaptureProtocol::kCaptureStopped,
        0
    };
    return writeAll(&stoppedHeader, sizeof(stoppedHeader));
}

bool writeCaptureStartFailed()
{
    const PacketCaptureProtocol::StreamHeader streamHeader{
        PacketCaptureProtocol::kStreamMagic,
        -1
    };
    return writeAll(&streamHeader, sizeof(streamHeader));
}

bool runCapture(const std::string& deviceName, bool& quitRequested)
{
    char errorBuffer[PCAP_ERRBUF_SIZE]{};
    pcap_t* capture = pcap_create(deviceName.c_str(), errorBuffer);
    if (!capture) {
        std::fprintf(stderr, "%s\n", errorBuffer);
        return writeCaptureStartFailed();
    }

    pcap_set_snaplen(capture, 65535);
    pcap_set_promisc(capture, 1);
    pcap_set_timeout(capture, 50);
    const int activateResult = pcap_activate(capture);
    if (activateResult < 0) {
        std::fprintf(stderr, "%s\n", pcap_geterr(capture));
        pcap_close(capture);
        return writeCaptureStartFailed();
    }
    pcap_setnonblock(capture, 1, errorBuffer);

    const PacketCaptureProtocol::StreamHeader streamHeader{
        PacketCaptureProtocol::kStreamMagic,
        pcap_datalink(capture)
    };
    if (!writeAll(&streamHeader, sizeof(streamHeader))) {
        pcap_close(capture);
        return false;
    }

    const int captureFd = pcap_get_selectable_fd(capture);
    while (true) {
        pollfd descriptors[2] = {
            {STDIN_FILENO, POLLIN, 0},
            {captureFd, POLLIN, 0}
        };
        const nfds_t descriptorCount = captureFd >= 0 ? 2 : 1;
        const int pollResult = poll(descriptors, descriptorCount, captureFd >= 0 ? -1 : 25);
        if (pollResult < 0 && errno != EINTR) {
            pcap_close(capture);
            return false;
        }

        if (descriptors[0].revents & (POLLIN | POLLHUP)) {
            std::string command;
            if (!readCommand(command)) {
                pcap_close(capture);
                return false;
            }
            if (command == "quit") {
                quitRequested = true;
            }
            if (command == "stop" || quitRequested) {
                break;
            }
        }

        if (captureFd < 0 || (descriptors[1].revents & POLLIN)) {
            while (true) {
                pcap_pkthdr* header = nullptr;
                const unsigned char* data = nullptr;
                const int result = pcap_next_ex(capture, &header, &data);
                if (result == 0) {
                    break;
                }
                if (result < 0) {
                    if (result == -1) {
                        std::fprintf(stderr, "%s\n", pcap_geterr(capture));
                    }
                    pcap_close(capture);
                    return writeCaptureStopped();
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
                    return false;
                }
                break;
            }
        }
    }

    pcap_close(capture);
    return writeCaptureStopped();
}

} // namespace

int main()
{
    std::signal(SIGPIPE, SIG_IGN);
    std::string command;
    while (readCommand(command)) {
        if (command == "quit") {
            return 0;
        }
        constexpr char startPrefix[] = "start ";
        if (command.rfind(startPrefix, 0) != 0) {
            continue;
        }
        bool quitRequested = false;
        if (!runCapture(command.substr(sizeof(startPrefix) - 1), quitRequested)) {
            return 1;
        }
        if (quitRequested) {
            return 0;
        }
    }
    return 0;
}
