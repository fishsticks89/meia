#include "bitset"
#include "vector"
#include <stdio.h>
#include "main.h"
#include <cstdint>
namespace meia {
    std::vector<std::pair<std::pair<uint16_t, uint8_t>, bool>> Console::white() {
    return std::vector<std::pair<std::pair<uint16_t, uint8_t>, bool>> {{{1, 1}, false}, {{122, 90}, true}, {{122, 117}, false}, {{123, 90}, true}, {{123, 119}, false}, {{124, 90}, true}, {{124, 121}, false}, {{125, 90}, true}, {{125, 122}, false}, {{126, 90}, true}, {{126, 122}, false}, {{127, 90}, true}, {{127, 123}, false}, {{128, 90}, true}, {{128, 123}, false}, {{129, 90}, true}, {{129, 123}, false}, {{130, 90}, true}, {{130, 124}, false}, {{131, 90}, true}, {{131, 124}, false}, {{132, 90}, true}, {{132, 124}, false}, {{133, 90}, true}, {{133, 124}, false}, {{134, 111}, true}, {{134, 124}, false}, {{135, 112}, true}, {{135, 124}, false}, {{136, 112}, true}, {{136, 124}, false}, {{137, 112}, true}, {{137, 124}, false}, {{138, 112}, true}, {{138, 124}, false}, {{139, 112}, true}, {{139, 124}, false}, {{140, 112}, true}, {{140, 124}, false}, {{141, 112}, true}, {{141, 124}, false}, {{142, 112}, true}, {{142, 124}, false}, {{143, 112}, true}, {{143, 124}, false}, {{144, 112}, true}, {{144, 124}, false}, {{145, 112}, true}, {{145, 124}, false}, {{146, 112}, true}, {{146, 124}, false}, {{147, 112}, true}, {{147, 124}, false}, {{148, 112}, true}, {{148, 124}, false}, {{149, 112}, true}, {{149, 124}, false}, {{150, 112}, true}, {{150, 124}, false}, {{151, 112}, true}, {{151, 124}, false}, {{152, 112}, true}, {{152, 124}, false}, {{153, 112}, true}, {{153, 124}, false}, {{154, 112}, true}, {{154, 124}, false}, {{155, 112}, true}, {{155, 124}, false}, {{156, 112}, true}, {{156, 124}, false}, {{157, 112}, true}, {{157, 124}, false}, {{158, 112}, true}, {{158, 124}, false}, {{159, 90}, true}, {{159, 147}, false}, {{160, 90}, true}, {{160, 147}, false}, {{161, 90}, true}, {{161, 147}, false}, {{162, 90}, true}, {{162, 147}, false}, {{163, 90}, true}, {{163, 147}, false}, {{164, 90}, true}, {{164, 147}, false}, {{165, 90}, true}, {{165, 147}, false}, {{166, 90}, true}, {{166, 147}, false}, {{167, 90}, true}, {{167, 147}, false}, {{168, 90}, true}, {{168, 147}, false}, {{169, 90}, true}, {{169, 147}, false}, {{170, 90}, true}, {{170, 147}, false}, {{183, 99}, true}, {{183, 101}, false}, {{183, 112}, true}, {{183, 137}, false}, {{184, 96}, true}, {{184, 102}, false}, {{184, 112}, true}, {{184, 141}, false}, {{185, 94}, true}, {{185, 102}, false}, {{185, 112}, true}, {{185, 142}, false}, {{186, 92}, true}, {{186, 102}, false}, {{186, 112}, true}, {{186, 144}, false}, {{187, 92}, true}, {{187, 102}, false}, {{187, 112}, true}, {{187, 145}, false}, {{188, 91}, true}, {{188, 102}, false}, {{188, 112}, true}, {{188, 145}, false}, {{189, 90}, true}, {{189, 102}, false}, {{189, 112}, true}, {{189, 146}, false}, {{190, 90}, true}, {{190, 102}, false}, {{190, 112}, true}, {{190, 146}, false}, {{191, 90}, true}, {{191, 102}, false}, {{191, 112}, true}, {{191, 146}, false}, {{192, 90}, true}, {{192, 102}, false}, {{192, 112}, true}, {{192, 146}, false}, {{193, 90}, true}, {{193, 102}, false}, {{193, 112}, true}, {{193, 146}, false}, {{194, 90}, true}, {{194, 102}, false}, {{194, 112}, true}, {{194, 146}, false}, {{195, 90}, true}, {{195, 102}, false}, {{195, 112}, true}, {{195, 125}, false}, {{195, 133}, true}, {{195, 147}, false}, {{196, 90}, true}, {{196, 102}, false}, {{196, 112}, true}, {{196, 124}, false}, {{196, 134}, true}, {{196, 147}, false}, {{197, 90}, true}, {{197, 102}, false}, {{197, 112}, true}, {{197, 124}, false}, {{197, 134}, true}, {{197, 147}, false}, {{198, 90}, true}, {{198, 102}, false}, {{198, 112}, true}, {{198, 124}, false}, {{198, 134}, true}, {{198, 147}, false}, {{199, 90}, true}, {{199, 102}, false}, {{199, 112}, true}, {{199, 124}, false}, {{199, 134}, true}, {{199, 147}, false}, {{200, 90}, true}, {{200, 102}, false}, {{200, 112}, true}, {{200, 124}, false}, {{200, 134}, true}, {{200, 147}, false}, {{201, 90}, true}, {{201, 102}, false}, {{201, 112}, true}, {{201, 124}, false}, {{201, 134}, true}, {{201, 147}, false}, {{202, 90}, true}, {{202, 102}, false}, {{202, 112}, true}, {{202, 124}, false}, {{202, 134}, true}, {{202, 147}, false}, {{203, 90}, true}, {{203, 102}, false}, {{203, 112}, true}, {{203, 124}, false}, {{203, 134}, true}, {{203, 147}, false}, {{204, 90}, true}, {{204, 102}, false}, {{204, 112}, true}, {{204, 124}, false}, {{204, 134}, true}, {{204, 147}, false}, {{205, 90}, true}, {{205, 102}, false}, {{205, 112}, true}, {{205, 124}, false}, {{205, 134}, true}, {{205, 147}, false}, {{206, 90}, true}, {{206, 102}, false}, {{206, 112}, true}, {{206, 124}, false}, {{206, 134}, true}, {{206, 147}, false}, {{207, 90}, true}, {{207, 102}, false}, {{207, 112}, true}, {{207, 124}, false}, {{207, 134}, true}, {{207, 147}, false}, {{208, 90}, true}, {{208, 102}, false}, {{208, 112}, true}, {{208, 124}, false}, {{208, 134}, true}, {{208, 147}, false}, {{209, 90}, true}, {{209, 102}, false}, {{209, 112}, true}, {{209, 124}, false}, {{209, 134}, true}, {{209, 147}, false}, {{210, 90}, true}, {{210, 102}, false}, {{210, 112}, true}, {{210, 124}, false}, {{210, 134}, true}, {{210, 147}, false}, {{211, 90}, true}, {{211, 102}, false}, {{211, 112}, true}, {{211, 124}, false}, {{211, 134}, true}, {{211, 147}, false}, {{212, 90}, true}, {{212, 102}, false}, {{212, 112}, true}, {{212, 124}, false}, {{212, 134}, true}, {{212, 147}, false}, {{213, 90}, true}, {{213, 102}, false}, {{213, 112}, true}, {{213, 124}, false}, {{213, 134}, true}, {{213, 147}, false}, {{214, 90}, true}, {{214, 102}, false}, {{214, 112}, true}, {{214, 124}, false}, {{214, 134}, true}, {{214, 147}, false}, {{215, 90}, true}, {{215, 102}, false}, {{215, 112}, true}, {{215, 124}, false}, {{215, 134}, true}, {{215, 147}, false}, {{216, 90}, true}, {{216, 102}, false}, {{216, 112}, true}, {{216, 124}, false}, {{216, 134}, true}, {{216, 147}, false}, {{217, 90}, true}, {{217, 102}, false}, {{217, 112}, true}, {{217, 124}, false}, {{217, 134}, true}, {{217, 147}, false}, {{218, 90}, true}, {{218, 102}, false}, {{218, 112}, true}, {{218, 124}, false}, {{218, 134}, true}, {{218, 147}, false}, {{219, 90}, true}, {{219, 102}, false}, {{219, 112}, true}, {{219, 125}, false}, {{219, 134}, true}, {{219, 147}, false}, {{220, 90}, true}, {{220, 102}, false}, {{220, 112}, true}, {{220, 147}, false}, {{221, 90}, true}, {{221, 102}, false}, {{221, 112}, true}, {{221, 146}, false}, {{222, 90}, true}, {{222, 102}, false}, {{222, 112}, true}, {{222, 146}, false}, {{223, 90}, true}, {{223, 102}, false}, {{223, 112}, true}, {{223, 146}, false}, {{224, 113}, true}, {{224, 146}, false}, {{225, 113}, true}, {{225, 146}, false}, {{226, 113}, true}, {{226, 145}, false}, {{227, 114}, true}, {{227, 145}, false}, {{228, 114}, true}, {{228, 144}, false}, {{229, 115}, true}, {{229, 144}, false}, {{230, 117}, true}, {{230, 142}, false}, {{231, 119}, true}, {{231, 140}, false}, {{232, 124}, true}, {{232, 135}, false}, {{245, 90}, true}, {{245, 102}, false}, {{245, 120}, true}, {{245, 147}, false}, {{246, 90}, true}, {{246, 102}, false}, {{246, 117}, true}, {{246, 147}, false}, {{247, 90}, true}, {{247, 102}, false}, {{247, 115}, true}, {{247, 147}, false}, {{248, 90}, true}, {{248, 102}, false}, {{248, 115}, true}, {{248, 147}, false}, {{249, 90}, true}, {{249, 102}, false}, {{249, 114}, true}, {{249, 147}, false}, {{250, 90}, true}, {{250, 102}, false}, {{250, 113}, true}, {{250, 147}, false}, {{251, 90}, true}, {{251, 102}, false}, {{251, 113}, true}, {{251, 147}, false}, {{252, 90}, true}, {{252, 102}, false}, {{252, 113}, true}, {{252, 147}, false}, {{253, 90}, true}, {{253, 102}, false}, {{253, 112}, true}, {{253, 147}, false}, {{254, 90}, true}, {{254, 102}, false}, {{254, 112}, true}, {{254, 147}, false}, {{255, 90}, true}, {{255, 102}, false}, {{255, 112}, true}, {{255, 147}, false}, {{256, 90}, true}, {{256, 102}, false}, {{256, 112}, true}, {{256, 147}, false}, {{257, 90}, true}, {{257, 102}, false}, {{257, 112}, true}, {{257, 125}, false}, {{257, 134}, true}, {{257, 147}, false}, {{258, 90}, true}, {{258, 102}, false}, {{258, 112}, true}, {{258, 124}, false}, {{258, 134}, true}, {{258, 147}, false}, {{259, 90}, true}, {{259, 102}, false}, {{259, 112}, true}, {{259, 124}, false}, {{259, 134}, true}, {{259, 147}, false}, {{260, 90}, true}, {{260, 102}, false}, {{260, 112}, true}, {{260, 124}, false}, {{260, 134}, true}, {{260, 147}, false}, {{261, 90}, true}, {{261, 102}, false}, {{261, 112}, true}, {{261, 124}, false}, {{261, 134}, true}, {{261, 147}, false}, {{262, 90}, true}, {{262, 102}, false}, {{262, 112}, true}, {{262, 124}, false}, {{262, 134}, true}, {{262, 147}, false}, {{263, 90}, true}, {{263, 102}, false}, {{263, 112}, true}, {{263, 124}, false}, {{263, 134}, true}, {{263, 147}, false}, {{264, 90}, true}, {{264, 102}, false}, {{264, 112}, true}, {{264, 124}, false}, {{264, 134}, true}, {{264, 147}, false}, {{265, 90}, true}, {{265, 102}, false}, {{265, 112}, true}, {{265, 124}, false}, {{265, 134}, true}, {{265, 147}, false}, {{266, 90}, true}, {{266, 102}, false}, {{266, 112}, true}, {{266, 124}, false}, {{266, 134}, true}, {{266, 147}, false}, {{267, 90}, true}, {{267, 102}, false}, {{267, 112}, true}, {{267, 124}, false}, {{267, 134}, true}, {{267, 147}, false}, {{268, 90}, true}, {{268, 102}, false}, {{268, 112}, true}, {{268, 124}, false}, {{268, 134}, true}, {{268, 147}, false}, {{269, 90}, true}, {{269, 102}, false}, {{269, 112}, true}, {{269, 124}, false}, {{269, 134}, true}, {{269, 147}, false}, {{270, 90}, true}, {{270, 102}, false}, {{270, 112}, true}, {{270, 124}, false}, {{270, 134}, true}, {{270, 147}, false}, {{271, 90}, true}, {{271, 102}, false}, {{271, 112}, true}, {{271, 124}, false}, {{271, 134}, true}, {{271, 147}, false}, {{272, 90}, true}, {{272, 102}, false}, {{272, 112}, true}, {{272, 124}, false}, {{272, 134}, true}, {{272, 147}, false}, {{273, 90}, true}, {{273, 102}, false}, {{273, 112}, true}, {{273, 124}, false}, {{273, 134}, true}, {{273, 147}, false}, {{274, 90}, true}, {{274, 102}, false}, {{274, 112}, true}, {{274, 124}, false}, {{274, 134}, true}, {{274, 147}, false}, {{275, 90}, true}, {{275, 102}, false}, {{275, 112}, true}, {{275, 124}, false}, {{275, 134}, true}, {{275, 147}, false}, {{276, 90}, true}, {{276, 102}, false}, {{276, 112}, true}, {{276, 124}, false}, {{276, 134}, true}, {{276, 147}, false}, {{277, 90}, true}, {{277, 102}, false}, {{277, 112}, true}, {{277, 124}, false}, {{277, 134}, true}, {{277, 147}, false}, {{278, 90}, true}, {{278, 102}, false}, {{278, 112}, true}, {{278, 124}, false}, {{278, 134}, true}, {{278, 147}, false}, {{279, 90}, true}, {{279, 102}, false}, {{279, 112}, true}, {{279, 124}, false}, {{279, 134}, true}, {{279, 147}, false}, {{280, 90}, true}, {{280, 102}, false}, {{280, 112}, true}, {{280, 124}, false}, {{280, 134}, true}, {{280, 147}, false}, {{281, 90}, true}, {{281, 103}, false}, {{281, 111}, true}, {{281, 124}, false}, {{281, 134}, true}, {{281, 147}, false}, {{282, 90}, true}, {{282, 124}, false}, {{282, 134}, true}, {{282, 147}, false}, {{283, 90}, true}, {{283, 124}, false}, {{283, 134}, true}, {{283, 147}, false}, {{284, 90}, true}, {{284, 124}, false}, {{284, 134}, true}, {{284, 147}, false}, {{285, 90}, true}, {{285, 124}, false}, {{285, 134}, true}, {{285, 147}, false}, {{286, 90}, true}, {{286, 123}, false}, {{286, 134}, true}, {{286, 147}, false}, {{287, 90}, true}, {{287, 123}, false}, {{287, 134}, true}, {{287, 147}, false}, {{288, 91}, true}, {{288, 123}, false}, {{288, 134}, true}, {{288, 147}, false}, {{289, 91}, true}, {{289, 122}, false}, {{289, 134}, true}, {{289, 147}, false}, {{290, 92}, true}, {{290, 121}, false}, {{290, 134}, true}, {{290, 147}, false}, {{291, 94}, true}, {{291, 120}, false}, {{291, 134}, true}, {{291, 147}, false}, {{292, 95}, true}, {{292, 119}, false}, {{292, 134}, true}, {{292, 147}, false}, {{293, 98}, true}, {{293, 116}, false}, {{293, 135}, true}, {{293, 146}, false}, {{306, 99}, true}, {{306, 101}, false}, {{306, 112}, true}, {{306, 146}, false}, {{307, 95}, true}, {{307, 102}, false}, {{307, 112}, true}, {{307, 147}, false}, {{308, 94}, true}, {{308, 102}, false}, {{308, 112}, true}, {{308, 147}, false}, {{309, 93}, true}, {{309, 102}, false}, {{309, 112}, true}, {{309, 147}, false}, {{310, 92}, true}, {{310, 102}, false}, {{310, 112}, true}, {{310, 147}, false}, {{311, 91}, true}, {{311, 102}, false}, {{311, 112}, true}, {{311, 147}, false}, {{312, 90}, true}, {{312, 102}, false}, {{312, 112}, true}, {{312, 147}, false}, {{313, 90}, true}, {{313, 102}, false}, {{313, 112}, true}, {{313, 147}, false}, {{314, 90}, true}, {{314, 102}, false}, {{314, 112}, true}, {{314, 147}, false}, {{315, 90}, true}, {{315, 102}, false}, {{315, 112}, true}, {{315, 147}, false}, {{316, 90}, true}, {{316, 102}, false}, {{316, 112}, true}, {{316, 147}, false}, {{317, 90}, true}, {{317, 102}, false}, {{317, 112}, true}, {{317, 147}, false}, {{318, 90}, true}, {{318, 102}, false}, {{318, 112}, true}, {{318, 124}, false}, {{319, 90}, true}, {{319, 102}, false}, {{319, 112}, true}, {{319, 124}, false}, {{320, 90}, true}, {{320, 102}, false}, {{320, 112}, true}, {{320, 124}, false}, {{321, 90}, true}, {{321, 102}, false}, {{321, 112}, true}, {{321, 124}, false}, {{322, 90}, true}, {{322, 102}, false}, {{322, 112}, true}, {{322, 124}, false}, {{323, 90}, true}, {{323, 102}, false}, {{323, 112}, true}, {{323, 124}, false}, {{324, 90}, true}, {{324, 102}, false}, {{324, 112}, true}, {{324, 124}, false}, {{325, 90}, true}, {{325, 102}, false}, {{325, 112}, true}, {{325, 124}, false}, {{326, 90}, true}, {{326, 102}, false}, {{326, 112}, true}, {{326, 124}, false}, {{327, 90}, true}, {{327, 102}, false}, {{327, 112}, true}, {{327, 124}, false}, {{328, 90}, true}, {{328, 102}, false}, {{328, 112}, true}, {{328, 124}, false}, {{329, 90}, true}, {{329, 102}, false}, {{329, 112}, true}, {{329, 124}, false}, {{330, 90}, true}, {{330, 102}, false}, {{330, 112}, true}, {{330, 124}, false}, {{331, 90}, true}, {{331, 102}, false}, {{331, 112}, true}, {{331, 124}, false}, {{332, 90}, true}, {{332, 102}, false}, {{332, 112}, true}, {{332, 124}, false}, {{333, 90}, true}, {{333, 102}, false}, {{333, 112}, true}, {{333, 124}, false}, {{334, 90}, true}, {{334, 102}, false}, {{334, 112}, true}, {{334, 124}, false}, {{335, 90}, true}, {{335, 102}, false}, {{335, 112}, true}, {{335, 124}, false}, {{336, 90}, true}, {{336, 102}, false}, {{336, 112}, true}, {{336, 124}, false}, {{337, 90}, true}, {{337, 102}, false}, {{337, 112}, true}, {{337, 124}, false}, {{338, 90}, true}, {{338, 102}, false}, {{338, 112}, true}, {{338, 124}, false}, {{339, 90}, true}, {{339, 102}, false}, {{339, 112}, true}, {{339, 124}, false}, {{340, 90}, true}, {{340, 102}, false}, {{340, 112}, true}, {{340, 124}, false}, {{341, 90}, true}, {{341, 102}, false}, {{341, 112}, true}, {{341, 124}, false}, {{342, 90}, true}, {{342, 102}, false}, {{342, 112}, true}, {{342, 124}, false}, {{343, 90}, true}, {{343, 102}, false}, {{343, 112}, true}, {{343, 124}, false}, {{344, 90}, true}, {{344, 102}, false}, {{344, 112}, true}, {{344, 124}, false}, {{345, 90}, true}, {{345, 102}, false}, {{345, 112}, true}, {{345, 124}, false}, {{346, 90}, true}, {{346, 102}, false}, {{346, 112}, true}, {{346, 124}, false}, {{347, 90}, true}, {{347, 102}, false}, {{347, 112}, true}, {{347, 124}, false}, {{348, 90}, true}, {{348, 102}, false}, {{348, 112}, true}, {{348, 124}, false}, {{349, 90}, true}, {{349, 102}, false}, {{349, 112}, true}, {{349, 124}, false}, {{350, 90}, true}, {{350, 102}, false}, {{350, 112}, true}, {{350, 124}, false}, {{351, 90}, true}, {{351, 146}, false}, {{352, 90}, true}, {{352, 147}, false}, {{353, 90}, true}, {{353, 147}, false}, {{354, 90}, true}, {{354, 147}, false}, {{355, 90}, true}, {{355, 147}, false}, {{356, 90}, true}, {{356, 147}, false}, {{357, 91}, true}, {{357, 147}, false}, {{358, 91}, true}, {{358, 147}, false}, {{359, 92}, true}, {{359, 147}, false}, {{360, 93}, true}, {{360, 147}, false}, {{361, 94}, true}, {{361, 147}, false}, {{362, 96}, true}, {{362, 147}, false}};
}
std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> Console::red() {
    return std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> {{{1, 1}, false}, {{1, 172}, true}, {{1, 187}, false}, {{1, 203}, true}, {{1, 217}, false}, {{2, 173}, true}, {{2, 188}, false}, {{2, 204}, true}, {{2, 218}, false}, {{3, 174}, true}, {{3, 189}, false}, {{3, 204}, true}, {{3, 219}, false}, {{4, 175}, true}, {{4, 190}, false}, {{4, 206}, true}, {{4, 220}, false}, {{5, 176}, true}, {{5, 191}, false}, {{5, 207}, true}, {{5, 221}, false}, {{6, 177}, true}, {{6, 192}, false}, {{6, 208}, true}, {{6, 222}, false}, {{7, 178}, true}, {{7, 193}, false}, {{7, 208}, true}, {{7, 223}, false}, {{8, 179}, true}, {{8, 194}, false}, {{8, 210}, true}, {{8, 224}, false}, {{9, 180}, true}, {{9, 195}, false}, {{9, 211}, true}, {{9, 225}, false}, {{10, 181}, true}, {{10, 196}, false}, {{10, 212}, true}, {{10, 226}, false}, {{11, 182}, true}, {{11, 197}, false}, {{11, 212}, true}, {{11, 227}, false}, {{12, 183}, true}, {{12, 198}, false}, {{12, 214}, true}, {{12, 228}, false}, {{13, 184}, true}, {{13, 199}, false}, {{13, 215}, true}, {{13, 229}, false}, {{14, 185}, true}, {{14, 200}, false}, {{14, 216}, true}, {{14, 230}, false}, {{15, 186}, true}, {{15, 201}, false}, {{15, 216}, true}, {{15, 231}, false}, {{16, 187}, true}, {{16, 202}, false}, {{16, 218}, true}, {{16, 232}, false}, {{17, 188}, true}, {{17, 203}, false}, {{17, 219}, true}, {{17, 233}, false}, {{18, 189}, true}, {{18, 204}, false}, {{18, 220}, true}, {{18, 234}, false}, {{19, 190}, true}, {{19, 205}, false}, {{19, 220}, true}, {{19, 235}, false}, {{20, 191}, true}, {{20, 206}, false}, {{20, 222}, true}, {{20, 236}, false}, {{21, 192}, true}, {{21, 207}, false}, {{21, 223}, true}, {{21, 237}, false}, {{22, 193}, true}, {{22, 208}, false}, {{22, 224}, true}, {{22, 238}, false}, {{23, 194}, true}, {{23, 209}, false}, {{23, 224}, true}, {{23, 239}, false}, {{24, 195}, true}, {{24, 210}, false}, {{24, 226}, true}, {{24, 240}, false}, {{25, 196}, true}, {{25, 211}, false}, {{25, 227}, true}, {{25, 240}, false}, {{26, 197}, true}, {{26, 212}, false}, {{26, 228}, true}, {{26, 240}, false}, {{27, 198}, true}, {{27, 213}, false}, {{27, 228}, true}, {{27, 240}, false}, {{28, 199}, true}, {{28, 214}, false}, {{28, 230}, true}, {{28, 240}, false}, {{29, 200}, true}, {{29, 215}, false}, {{29, 231}, true}, {{29, 240}, false}, {{30, 201}, true}, {{30, 216}, false}, {{30, 232}, true}, {{30, 240}, false}, {{31, 202}, true}, {{31, 217}, false}, {{31, 232}, true}, {{31, 240}, false}, {{32, 203}, true}, {{32, 218}, false}, {{32, 234}, true}, {{32, 240}, false}, {{33, 204}, true}, {{33, 219}, false}, {{33, 235}, true}, {{33, 240}, false}, {{34, 205}, true}, {{34, 220}, false}, {{34, 236}, true}, {{34, 240}, false}, {{35, 206}, true}, {{35, 221}, false}, {{35, 236}, true}, {{35, 240}, false}, {{36, 207}, true}, {{36, 222}, false}, {{36, 238}, true}, {{36, 240}, false}, {{37, 208}, true}, {{37, 223}, false}, {{37, 239}, true}, {{37, 240}, false}, {{38, 209}, true}, {{38, 224}, false}, {{39, 210}, true}, {{39, 225}, false}, {{40, 211}, true}, {{40, 226}, false}, {{41, 212}, true}, {{41, 227}, false}, {{42, 213}, true}, {{42, 228}, false}, {{43, 214}, true}, {{43, 229}, false}, {{44, 215}, true}, {{44, 230}, false}, {{45, 216}, true}, {{45, 231}, false}, {{46, 217}, true}, {{46, 232}, false}, {{47, 218}, true}, {{47, 233}, false}, {{48, 219}, true}, {{48, 234}, false}, {{49, 220}, true}, {{49, 235}, false}, {{50, 221}, true}, {{50, 236}, false}, {{51, 222}, true}, {{51, 237}, false}, {{52, 223}, true}, {{52, 238}, false}, {{53, 224}, true}, {{53, 239}, false}, {{54, 225}, true}, {{54, 240}, false}, {{55, 226}, true}, {{55, 240}, false}, {{56, 227}, true}, {{56, 240}, false}, {{57, 228}, true}, {{57, 240}, false}, {{58, 229}, true}, {{58, 240}, false}, {{59, 230}, true}, {{59, 240}, false}, {{60, 231}, true}, {{60, 240}, false}, {{61, 232}, true}, {{61, 240}, false}, {{62, 233}, true}, {{62, 240}, false}, {{63, 234}, true}, {{63, 240}, false}, {{64, 235}, true}, {{64, 240}, false}, {{65, 236}, true}, {{65, 240}, false}, {{66, 237}, true}, {{66, 240}, false}, {{67, 238}, true}, {{67, 240}, false}, {{68, 239}, true}, {{68, 240}, false}};
}
}