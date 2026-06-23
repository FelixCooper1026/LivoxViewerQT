#include "PointCloud/PointCloudColorizer.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace PointCloudColorizer {

namespace {

constexpr int kColorByReflectivity = 0;
constexpr int kColorByDistance = 1;
constexpr int kColorByElevation = 2;
constexpr int kColorSolid = 3;
constexpr int kColorByLine = 4;
constexpr int kReflectivityScaleCount = ReflectivityInferno + 1;
constexpr int kReflectivityValueCount = 256;

struct Rgb {
    float r;
    float g;
    float b;
};

struct ColorScaleStop {
    Rgb color;
    float position;
};

struct ColorScaleDefinition {
    std::array<ColorScaleStop, 15> stops;
    int stopCount;
};

constexpr Rgb rgb(int r, int g, int b)
{
    return {float(r) / 255.0f, float(g) / 255.0f, float(b) / 255.0f};
}

constexpr ColorScaleStop stop(int r, int g, int b, float position)
{
    return {rgb(r, g, b), position};
}

constexpr std::array<ColorScaleDefinition, kReflectivityScaleCount> kReflectivityColorScales = {{
    {std::array<ColorScaleStop, 15>{stop(0, 0, 255, 0.0f), stop(0, 255, 0, 0.333333333333f), stop(255, 255, 0, 0.666666666667f), stop(255, 0, 0, 1.0f)}, 4},
    {std::array<ColorScaleStop, 15>{stop(0, 0, 255, 0.0f), stop(0, 255, 255, 0.25f), stop(0, 255, 0, 0.5f), stop(255, 255, 0, 0.75f), stop(255, 0, 0, 1.0f)}, 5},
    {std::array<ColorScaleStop, 15>{stop(68, 1, 84, 0.0f), stop(59, 82, 139, 0.25f), stop(33, 145, 140, 0.5f), stop(94, 201, 98, 0.75f), stop(253, 231, 37, 1.0f)}, 5},
    {std::array<ColorScaleStop, 15>{stop(48, 18, 59, 0.0f), stop(70, 100, 215, 0.2f), stop(26, 228, 182, 0.4f), stop(164, 252, 60, 0.6f), stop(250, 186, 57, 0.8f), stop(196, 0, 0, 1.0f)}, 6},
    {std::array<ColorScaleStop, 15>{stop(0, 32, 76, 0.0f), stop(49, 68, 107, 0.25f), stop(102, 104, 112, 0.5f), stop(166, 157, 117, 0.75f), stop(255, 233, 69, 1.0f)}, 5},
    {std::array<ColorScaleStop, 15>{stop(170, 255, 255, 0.0f), stop(158, 158, 158, 0.01f), stop(0, 0, 127, 0.02f), stop(0, 255, 0, 0.04f), stop(0, 85, 0, 0.08f), stop(255, 255, 0, 0.16f), stop(255, 0, 0, 0.32f), stop(135, 0, 0, 0.5f), stop(232, 232, 232, 1.0f)}, 9},
    {std::array<ColorScaleStop, 15>{stop(0, 0, 0, 0.0f), stop(255, 255, 255, 1.0f)}, 2},
    {std::array<ColorScaleStop, 15>{stop(13, 8, 135, 0.0f), stop(53, 4, 152, 0.071429f), stop(83, 2, 163, 0.142857f), stop(111, 0, 168, 0.214286f), stop(139, 10, 165, 0.285714f), stop(163, 30, 154, 0.357143f), stop(184, 50, 137, 0.428571f), stop(204, 71, 120, 0.5f), stop(219, 92, 104, 0.571429f), stop(233, 113, 88, 0.642857f), stop(244, 136, 73, 0.714286f), stop(251, 162, 56, 0.785714f), stop(254, 189, 42, 0.857143f), stop(250, 218, 36, 0.928571f), stop(240, 249, 33, 1.0f)}, 15},
    {std::array<ColorScaleStop, 15>{stop(51, 51, 153, 0.0f), stop(27, 99, 201, 0.071429f), stop(3, 147, 249, 0.142857f), stop(0, 184, 160, 0.214286f), stop(37, 211, 109, 0.285714f), stop(109, 226, 124, 0.357143f), stop(181, 240, 138, 0.428571f), stop(254, 254, 152, 0.5f), stop(218, 208, 133, 0.571429f), stop(182, 162, 114, 0.642857f), stop(146, 115, 94, 0.714286f), stop(147, 117, 110, 0.785714f), stop(183, 163, 159, 0.857143f), stop(219, 209, 207, 0.928571f), stop(255, 255, 255, 1.0f)}, 15},
    {std::array<ColorScaleStop, 15>{stop(0, 0, 4, 0.0f), stop(13, 8, 41, 0.071429f), stop(40, 11, 83, 0.142857f), stop(71, 11, 106, 0.214286f), stop(101, 21, 110, 0.285714f), stop(130, 32, 108, 0.357143f), stop(159, 42, 99, 0.428571f), stop(188, 55, 84, 0.5f), stop(212, 72, 66, 0.571429f), stop(232, 96, 45, 0.642857f), stop(245, 125, 21, 0.714286f), stop(252, 159, 7, 0.785714f), stop(250, 194, 40, 0.857143f), stop(243, 229, 93, 0.928571f), stop(252, 255, 164, 1.0f)}, 15}
}};

constexpr const char* kViridisScaleHex =
    "44015444025544035745055845065A45085B46095C460B5E460C5F460E61470F62471163471265471466471567471669"
    "47186A48196B481A6C481C6E481D6F481E70482071482172482273482374472575472676472777472878472A79472B7A"
    "472C7B462D7C462F7C46307D46317E45327F45347F453580453681443781443982433A83433B83433C84423D84423E85"
    "4240854141864142864043874044873F45873F47883E48883E49893D4A893D4B893D4C893C4D8A3C4E8A3B508A3B518A"
    "3A528B3A538B39548B39558B38568B38578C37588C37598C365A8C365B8C355C8C355D8C345E8D345F8D33608D33618D"
    "32628D32638D31648D31658D31668D30678D30688D2F698D2F6A8D2E6B8E2E6C8E2E6D8E2D6E8E2D6F8E2C708E2C718E"
    "2C728E2B738E2B748E2A758E2A768E2A778E29788E29798E287A8E287A8E287B8E277C8E277D8E277E8E267F8E26808E"
    "26818E25828E25838D24848D24858D24868D23878D23888D23898D22898D228A8D228B8D218C8D218D8C218E8C208F8C"
    "20908C20918C1F928C1F938B1F948B1F958B1F968B1E978A1E988A1E998A1E998A1E9A891E9B891E9C891E9D881E9E88"
    "1E9F881EA0871FA1871FA2861FA38620A48520A58521A68521A78422A78423A88323A98224AA8225AB8126AC8127AD80"
    "28AE7F29AF7F2AB07E2BB17D2CB17D2EB27C2FB37B30B47A32B57A33B67935B77836B87738B97639B9763BBA753DBB74"
    "3EBC7340BD7242BE7144BE7045BF6F47C06E49C16D4BC26C4DC26B4FC36951C46853C56755C66657C66559C7645BC862"
    "5EC96160C96062CA5F64CB5D67CC5C69CC5B6BCD596DCE5870CE5672CF5574D05477D05279D1517CD24F7ED24E81D34C"
    "83D34B86D44988D5478BD5468DD64490D64392D74195D73F97D83E9AD83C9DD93A9FD938A2DA37A5DA35A7DB33AADB32"
    "ADDC30AFDC2EB2DD2CB5DD2BB7DD29BADE27BDDE26BFDF24C2DF22C5DF21C7E01FCAE01ECDE01DCFE11CD2E11BD4E11A"
    "D7E219DAE218DCE218DFE318E1E318E4E318E7E419E9E419ECE41AEEE51BF1E51CF3E51EF6E61FF8E621FAE622FDE724";

constexpr const char* kCividisScaleHex =
    "00204D00214E00225000225200235300245500255700255800265A00275C00275E002860002961002A63002A65002B67"
    "002C69002C6A002D6C002E6E002E6F002F6F002F6F00306F00306F00316F00326F00336F00336F00346F00356E01366E"
    "06366E0B376E0F386E12386D15396D183A6D1A3B6D1D3B6D1F3C6D213D6D233E6C243E6C263F6C28406C2A406C2B416C"
    "2D426C2E436C30436C31446B32456B34456B35466B36476B38486B39486B3A496B3B4A6B3D4A6B3E4B6B3F4C6B404D6B"
    "414D6B424E6B434F6B444F6B46506B47516B48526B49526B4A536B4B546C4C546C4D556C4E566C4F576C50576C51586C"
    "52596C53596C545A6C555B6D565C6D575C6D585D6D595E6D595F6D5A5F6D5B606E5C616E5D616E5E626E5F636E60646F"
    "61646F62656F63666F64666F646770656870666970676970686A71696B716A6C716B6C716C6D726C6E726D6E726E6F73"
    "6F70737071737171747272747273747374757474757575757676767777767877777878777979777A7A787B7A787C7B78"
    "7D7C787E7D787F7D78807E79817F798280798380798481798482798583798683798784798885798986798A87798B8779"
    "8C88798D89798E8A798F8A79908B79918C78928D78938E78948E78958F789690789791789892789992789A93779B9477"
    "9C95779D96779E96779F9777A09877A19976A29A76A39A76A49B76A59C76A69D75A89E75A99F75AA9F75ABA074ACA174"
    "ADA274AEA374AFA473B0A473B1A573B2A672B3A772B4A872B5A971B6A971B7AA71B8AB70B9AC70BAAD70BBAE6FBCAF6F"
    "BEAF6FBFB06EC0B16EC1B26DC2B36DC3B46DC4B56CC5B56CC6B66BC7B76BC8B86AC9B96ACBBA69CCBB69CDBC68CEBC68"
    "CFBD67D0BE67D1BF66D2C066D3C165D4C264D6C364D7C463D8C563D9C562DAC661DBC761DCC860DDC95FDECA5FE0CB5E"
    "E1CC5DE2CD5CE3CE5CE4CF5BE5D05AE6D159E8D259E9D358EAD357EBD456ECD555EDD654EFD753F0D852F1D951F2DA50"
    "F3DB4FF4DC4EF6DD4DF7DE4CF8DF4BF9E04AFAE149FBE248FDE346FEE445FFE544FFE642FFE742FFE843FFE944FFEA46";

const char* exactColorScaleHex(int scale)
{
    switch (scale) {
    case ReflectivityViridis:
        return kViridisScaleHex;
    case ReflectivityCividis:
        return kCividisScaleHex;
    default:
        return nullptr;
    }
}

int hexValue(char value)
{
    return value <= '9' ? value - '0' : value - 'A' + 10;
}

Rgb colorFromHexTable(const char* colors, int index)
{
    const char* color = colors + index * 6;
    return rgb(hexValue(color[0]) * 16 + hexValue(color[1]),
               hexValue(color[2]) * 16 + hexValue(color[3]),
               hexValue(color[4]) * 16 + hexValue(color[5]));
}

Rgb interpolateColor(const ColorScaleDefinition& scale, float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    int index = 0;
    while (index + 1 < scale.stopCount && t > scale.stops[size_t(index + 1)].position) {
        ++index;
    }
    index = std::min(index, scale.stopCount - 2);
    const ColorScaleStop& aStop = scale.stops[size_t(index)];
    const ColorScaleStop& bStop = scale.stops[size_t(index + 1)];
    const float localT = (t - aStop.position) / (bStop.position - aStop.position);
    const Rgb& a = aStop.color;
    const Rgb& b = bStop.color;
    return {a.r + (b.r - a.r) * localT,
            a.g + (b.g - a.g) * localT,
            a.b + (b.b - a.b) * localT};
}

std::array<std::array<Rgb, kReflectivityValueCount>, kReflectivityScaleCount> buildReflectivityLookupTables()
{
    std::array<std::array<Rgb, kReflectivityValueCount>, kReflectivityScaleCount> tables;
    for (int scale = 0; scale < kReflectivityScaleCount; ++scale) {
        const char* exactColors = exactColorScaleHex(scale);
        for (int reflectivity = 0; reflectivity < kReflectivityValueCount; ++reflectivity) {
            tables[size_t(scale)][size_t(reflectivity)] =
                exactColors ? colorFromHexTable(exactColors, reflectivity)
                            : interpolateColor(kReflectivityColorScales[size_t(scale)], float(reflectivity) / 255.0f);
        }
    }
    return tables;
}

const std::array<std::array<Rgb, kReflectivityValueCount>, kReflectivityScaleCount>& reflectivityLookupTables()
{
    static const std::array<std::array<Rgb, kReflectivityValueCount>, kReflectivityScaleCount> tables =
        buildReflectivityLookupTables();
    return tables;
}

void calculateReflectivityColor(uint8_t reflectivity, int scale, float& r, float& g, float& b)
{
    const Rgb& color = reflectivityLookupTables()[size_t(scale)][size_t(reflectivity)];
    r = color.r;
    g = color.g;
    b = color.b;
}

} // namespace

QVector<QColor> reflectivityColorScaleStops(int scale)
{
    QVector<QColor> colors;
    colors.reserve(kReflectivityValueCount);
    for (const Rgb& color : reflectivityLookupTables()[size_t(scale)]) {
        colors.append(QColor::fromRgbF(color.r, color.g, color.b));
    }
    return colors;
}

PointCloudPipelineLegend apply(QVector<PointCloudPoint>& points, const Config& config)
{
    PointCloudPipelineLegend legend;
    legend.mode = config.mode;
    legend.minValue = 0.0f;
    legend.maxValue = 1.0f;
    legend.visible = (config.mode != kColorSolid);
    if (config.mode == kColorByLine) {
        legend.lineColors = config.lineColors;
        legend.lineNumbers.reserve(config.lineColors.size());
        for (int i = 0; i < config.lineColors.size(); ++i) {
            legend.lineNumbers.append(i);
        }
    }

    if (points.isEmpty()) {
        return legend;
    }

    if (config.mode == kColorByReflectivity) {
        for (PointCloudPoint& point : points) {
            calculateReflectivityColor(point.reflectivity, config.reflectivityColorScale, point.r, point.g, point.b);
        }
        legend.minValue = 0.0f;
        legend.maxValue = 255.0f;
        legend.visible = true;
    } else if (config.mode == kColorByDistance) {
        const float minD = config.distanceColorMin;
        const float maxD = config.distanceColorMax;
        for (PointCloudPoint& point : points) {
            const float d = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            float t = (d - minD) / (maxD - minD);
            t = std::clamp(t, 0.0f, 1.0f);
            if (t < 0.25f) {
                point.r = 0.0f;
                point.g = t / 0.25f;
                point.b = 1.0f;
            } else if (t < 0.5f) {
                point.r = 0.0f;
                point.g = 1.0f;
                point.b = 1.0f - (t - 0.25f) / 0.25f;
            } else if (t < 0.75f) {
                point.r = (t - 0.5f) / 0.25f;
                point.g = 1.0f;
                point.b = 0.0f;
            } else {
                point.r = 1.0f;
                point.g = 1.0f - (t - 0.75f) / 0.25f;
                point.b = 0.0f;
            }
        }
        legend.minValue = minD;
        legend.maxValue = maxD;
        legend.visible = true;
    } else if (config.mode == kColorByElevation) {
        const float minZ = config.elevationColorMin;
        const float maxZ = config.elevationColorMax;
        for (PointCloudPoint& point : points) {
            float t = (point.z - minZ) / (maxZ - minZ);
            t = std::clamp(t, 0.0f, 1.0f);
            point.r = t;
            point.g = 0.0f;
            point.b = 1.0f - t;
        }
        legend.minValue = minZ;
        legend.maxValue = maxZ;
        legend.visible = true;
    } else if (config.mode == kColorSolid) {
        for (PointCloudPoint& point : points) {
            point.r = config.solidColor.redF();
            point.g = config.solidColor.greenF();
            point.b = config.solidColor.blueF();
        }
        legend.visible = false;
    } else if (config.mode == kColorByLine) {
        QVector<int> usedLines;
        for (PointCloudPoint& point : points) {
            const int line = int(point.line);
            const QColor color = config.lineColors.at(line % config.lineColors.size());
            point.r = color.redF();
            point.g = color.greenF();
            point.b = color.blueF();
            if (!usedLines.contains(line)) {
                usedLines.append(line);
            }
        }
        std::sort(usedLines.begin(), usedLines.end());
        legend.lineNumbers = usedLines;
        legend.lineColors.clear();
        for (int line : usedLines) {
            legend.lineColors.append(config.lineColors.at(line % config.lineColors.size()));
        }
        legend.visible = true;
    }

    return legend;
}

} // namespace PointCloudColorizer
