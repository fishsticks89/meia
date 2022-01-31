#include "bitset"
#include "vector"
#include <stdio.h>
inline std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> white() {
    return std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> {{{1, 1}, false}, {{82, 89}, true}, {{82, 158}, false}, {{83, 89}, true}, {{83, 158}, false}, {{84, 89}, true}, {{84, 158}, false}, {{85, 89}, true}, {{85, 158}, false}, {{86, 89}, true}, {{86, 158}, false}, {{87, 89}, true}, {{87, 158}, false}, {{88, 89}, true}, {{88, 158}, false}, {{89, 89}, true}, {{89, 158}, false}, {{90, 89}, true}, {{90, 158}, false}, {{91, 89}, true}, {{91, 158}, false}, {{92, 89}, true}, {{92, 158}, false}, {{93, 89}, true}, {{93, 158}, false}, {{94, 89}, true}, {{94, 158}, false}, {{95, 89}, true}, {{95, 158}, false}, {{96, 89}, true}, {{96, 158}, false}, {{97, 89}, true}, {{97, 158}, false}, {{98, 89}, true}, {{98, 158}, false}, {{99, 89}, true}, {{99, 158}, false}, {{100, 89}, true}, {{100, 158}, false}, {{101, 96}, true}, {{101, 158}, false}, {{102, 95}, true}, {{102, 119}, false}, {{103, 94}, true}, {{103, 115}, false}, {{104, 93}, true}, {{104, 112}, false}, {{105, 93}, true}, {{105, 111}, false}, {{106, 92}, true}, {{106, 110}, false}, {{107, 91}, true}, {{107, 109}, false}, {{108, 91}, true}, {{108, 108}, false}, {{109, 90}, true}, {{109, 107}, false}, {{110, 90}, true}, {{110, 106}, false}, {{111, 89}, true}, {{111, 106}, false}, {{112, 89}, true}, {{112, 106}, false}, {{113, 89}, true}, {{113, 105}, false}, {{114, 89}, true}, {{114, 105}, false}, {{115, 88}, true}, {{115, 105}, false}, {{116, 88}, true}, {{116, 105}, false}, {{117, 88}, true}, {{117, 105}, false}, {{118, 88}, true}, {{118, 105}, false}, {{119, 88}, true}, {{119, 105}, false}, {{120, 88}, true}, {{120, 105}, false}, {{121, 87}, true}, {{121, 105}, false}, {{122, 87}, true}, {{122, 106}, false}, {{123, 87}, true}, {{123, 106}, false}, {{124, 87}, true}, {{124, 107}, false}, {{125, 88}, true}, {{125, 107}, false}, {{126, 88}, true}, {{126, 108}, false}, {{127, 88}, true}, {{127, 109}, false}, {{128, 88}, true}, {{128, 111}, false}, {{129, 88}, true}, {{129, 114}, false}, {{130, 88}, true}, {{130, 157}, false}, {{131, 89}, true}, {{131, 158}, false}, {{132, 89}, true}, {{132, 158}, false}, {{133, 89}, true}, {{133, 158}, false}, {{134, 90}, true}, {{134, 158}, false}, {{135, 90}, true}, {{135, 158}, false}, {{136, 91}, true}, {{136, 158}, false}, {{137, 91}, true}, {{137, 158}, false}, {{138, 92}, true}, {{138, 158}, false}, {{139, 92}, true}, {{139, 158}, false}, {{140, 93}, true}, {{140, 158}, false}, {{141, 94}, true}, {{141, 158}, false}, {{142, 95}, true}, {{142, 158}, false}, {{143, 96}, true}, {{143, 158}, false}, {{144, 97}, true}, {{144, 158}, false}, {{145, 98}, true}, {{145, 158}, false}, {{146, 98}, true}, {{146, 158}, false}, {{147, 97}, true}, {{147, 158}, false}, {{148, 96}, true}, {{148, 158}, false}, {{149, 95}, true}, {{149, 158}, false}, {{150, 94}, true}, {{150, 157}, false}, {{151, 93}, true}, {{151, 117}, false}, {{152, 93}, true}, {{152, 114}, false}, {{153, 92}, true}, {{153, 112}, false}, {{154, 92}, true}, {{154, 110}, false}, {{155, 91}, true}, {{155, 109}, false}, {{156, 91}, true}, {{156, 108}, false}, {{157, 90}, true}, {{157, 107}, false}, {{158, 90}, true}, {{158, 107}, false}, {{159, 89}, true}, {{159, 106}, false}, {{160, 89}, true}, {{160, 106}, false}, {{161, 89}, true}, {{161, 106}, false}, {{162, 88}, true}, {{162, 105}, false}, {{163, 88}, true}, {{163, 105}, false}, {{164, 88}, true}, {{164, 105}, false}, {{165, 88}, true}, {{165, 105}, false}, {{166, 88}, true}, {{166, 105}, false}, {{167, 88}, true}, {{167, 105}, false}, {{168, 88}, true}, {{168, 105}, false}, {{169, 87}, true}, {{169, 105}, false}, {{170, 87}, true}, {{170, 105}, false}, {{171, 87}, true}, {{171, 106}, false}, {{172, 87}, true}, {{172, 106}, false}, {{173, 88}, true}, {{173, 107}, false}, {{174, 88}, true}, {{174, 108}, false}, {{175, 88}, true}, {{175, 109}, false}, {{176, 88}, true}, {{176, 110}, false}, {{177, 88}, true}, {{177, 112}, false}, {{178, 88}, true}, {{178, 115}, false}, {{179, 88}, true}, {{179, 158}, false}, {{180, 89}, true}, {{180, 158}, false}, {{181, 89}, true}, {{181, 158}, false}, {{182, 89}, true}, {{182, 158}, false}, {{183, 90}, true}, {{183, 158}, false}, {{184, 90}, true}, {{184, 158}, false}, {{185, 91}, true}, {{185, 158}, false}, {{186, 91}, true}, {{186, 158}, false}, {{187, 92}, true}, {{187, 158}, false}, {{188, 92}, true}, {{188, 158}, false}, {{189, 93}, true}, {{189, 158}, false}, {{190, 94}, true}, {{190, 158}, false}, {{191, 95}, true}, {{191, 158}, false}, {{192, 96}, true}, {{192, 158}, false}, {{193, 97}, true}, {{193, 158}, false}, {{194, 99}, true}, {{194, 158}, false}, {{195, 100}, true}, {{195, 158}, false}, {{196, 102}, true}, {{196, 158}, false}, {{197, 105}, true}, {{197, 158}, false}, {{198, 108}, true}, {{198, 158}, false}, {{213, 116}, true}, {{213, 130}, false}, {{214, 112}, true}, {{214, 134}, false}, {{215, 109}, true}, {{215, 137}, false}, {{216, 107}, true}, {{216, 139}, false}, {{217, 105}, true}, {{217, 141}, false}, {{218, 104}, true}, {{218, 142}, false}, {{219, 102}, true}, {{219, 144}, false}, {{220, 101}, true}, {{220, 145}, false}, {{221, 100}, true}, {{221, 146}, false}, {{222, 99}, true}, {{222, 147}, false}, {{223, 98}, true}, {{223, 148}, false}, {{224, 97}, true}, {{224, 149}, false}, {{225, 96}, true}, {{225, 150}, false}, {{226, 95}, true}, {{226, 151}, false}, {{227, 94}, true}, {{227, 151}, false}, {{228, 94}, true}, {{228, 152}, false}, {{229, 93}, true}, {{229, 153}, false}, {{230, 92}, true}, {{230, 153}, false}, {{231, 92}, true}, {{231, 154}, false}, {{232, 91}, true}, {{232, 154}, false}, {{233, 91}, true}, {{233, 115}, false}, {{233, 117}, true}, {{233, 155}, false}, {{234, 90}, true}, {{234, 112}, false}, {{234, 117}, true}, {{234, 129}, false}, {{234, 132}, true}, {{234, 155}, false}, {{235, 90}, true}, {{235, 110}, false}, {{235, 117}, true}, {{235, 129}, false}, {{235, 134}, true}, {{235, 156}, false}, {{236, 90}, true}, {{236, 109}, false}, {{236, 117}, true}, {{236, 129}, false}, {{236, 135}, true}, {{236, 156}, false}, {{237, 89}, true}, {{237, 108}, false}, {{237, 117}, true}, {{237, 129}, false}, {{237, 137}, true}, {{237, 156}, false}, {{238, 89}, true}, {{238, 107}, false}, {{238, 117}, true}, {{238, 129}, false}, {{238, 137}, true}, {{238, 157}, false}, {{239, 89}, true}, {{239, 106}, false}, {{239, 117}, true}, {{239, 129}, false}, {{239, 138}, true}, {{239, 157}, false}, {{240, 89}, true}, {{240, 105}, false}, {{240, 117}, true}, {{240, 129}, false}, {{240, 139}, true}, {{240, 157}, false}, {{241, 88}, true}, {{241, 105}, false}, {{241, 117}, true}, {{241, 129}, false}, {{241, 140}, true}, {{241, 157}, false}, {{242, 88}, true}, {{242, 104}, false}, {{242, 117}, true}, {{242, 129}, false}, {{242, 140}, true}, {{242, 158}, false}, {{243, 88}, true}, {{243, 104}, false}, {{243, 117}, true}, {{243, 129}, false}, {{243, 141}, true}, {{243, 158}, false}, {{244, 88}, true}, {{244, 103}, false}, {{244, 117}, true}, {{244, 129}, false}, {{244, 141}, true}, {{244, 158}, false}, {{245, 88}, true}, {{245, 103}, false}, {{245, 117}, true}, {{245, 129}, false}, {{245, 141}, true}, {{245, 158}, false}, {{246, 88}, true}, {{246, 103}, false}, {{246, 117}, true}, {{246, 129}, false}, {{246, 142}, true}, {{246, 158}, false}, {{247, 88}, true}, {{247, 103}, false}, {{247, 117}, true}, {{247, 129}, false}, {{247, 142}, true}, {{247, 158}, false}, {{248, 87}, true}, {{248, 103}, false}, {{248, 117}, true}, {{248, 129}, false}, {{248, 142}, true}, {{248, 158}, false}, {{249, 87}, true}, {{249, 103}, false}, {{249, 117}, true}, {{249, 129}, false}, {{249, 142}, true}, {{249, 159}, false}, {{250, 87}, true}, {{250, 103}, false}, {{250, 117}, true}, {{250, 129}, false}, {{250, 142}, true}, {{250, 159}, false}, {{251, 87}, true}, {{251, 103}, false}, {{251, 117}, true}, {{251, 129}, false}, {{251, 142}, true}, {{251, 159}, false}, {{252, 88}, true}, {{252, 103}, false}, {{252, 117}, true}, {{252, 129}, false}, {{252, 142}, true}, {{252, 159}, false}, {{253, 88}, true}, {{253, 103}, false}, {{253, 117}, true}, {{253, 129}, false}, {{253, 142}, true}, {{253, 159}, false}, {{254, 88}, true}, {{254, 103}, false}, {{254, 117}, true}, {{254, 129}, false}, {{254, 142}, true}, {{254, 159}, false}, {{255, 88}, true}, {{255, 103}, false}, {{255, 117}, true}, {{255, 129}, false}, {{255, 142}, true}, {{255, 159}, false}, {{256, 88}, true}, {{256, 104}, false}, {{256, 117}, true}, {{256, 129}, false}, {{256, 142}, true}, {{256, 158}, false}, {{257, 88}, true}, {{257, 104}, false}, {{257, 117}, true}, {{257, 129}, false}, {{257, 142}, true}, {{257, 158}, false}, {{258, 88}, true}, {{258, 105}, false}, {{258, 117}, true}, {{258, 129}, false}, {{258, 142}, true}, {{258, 158}, false}, {{259, 88}, true}, {{259, 105}, false}, {{259, 117}, true}, {{259, 129}, false}, {{259, 142}, true}, {{259, 158}, false}, {{260, 89}, true}, {{260, 106}, false}, {{260, 117}, true}, {{260, 129}, false}, {{260, 141}, true}, {{260, 158}, false}, {{261, 89}, true}, {{261, 107}, false}, {{261, 117}, true}, {{261, 129}, false}, {{261, 141}, true}, {{261, 158}, false}, {{262, 89}, true}, {{262, 108}, false}, {{262, 117}, true}, {{262, 129}, false}, {{262, 141}, true}, {{262, 158}, false}, {{263, 90}, true}, {{263, 109}, false}, {{263, 117}, true}, {{263, 129}, false}, {{263, 140}, true}, {{263, 158}, false}, {{264, 90}, true}, {{264, 110}, false}, {{264, 117}, true}, {{264, 129}, false}, {{264, 140}, true}, {{264, 157}, false}, {{265, 90}, true}, {{265, 112}, false}, {{265, 117}, true}, {{265, 129}, false}, {{265, 139}, true}, {{265, 157}, false}, {{266, 91}, true}, {{266, 114}, false}, {{266, 117}, true}, {{266, 129}, false}, {{266, 139}, true}, {{266, 157}, false}, {{267, 91}, true}, {{267, 129}, false}, {{267, 138}, true}, {{267, 156}, false}, {{268, 92}, true}, {{268, 129}, false}, {{268, 137}, true}, {{268, 156}, false}, {{269, 92}, true}, {{269, 129}, false}, {{269, 137}, true}, {{269, 156}, false}, {{270, 93}, true}, {{270, 129}, false}, {{270, 136}, true}, {{270, 155}, false}, {{271, 93}, true}, {{271, 129}, false}, {{271, 137}, true}, {{271, 155}, false}, {{272, 94}, true}, {{272, 129}, false}, {{272, 138}, true}, {{272, 154}, false}, {{273, 95}, true}, {{273, 129}, false}, {{273, 139}, true}, {{273, 154}, false}, {{274, 96}, true}, {{274, 129}, false}, {{274, 140}, true}, {{274, 153}, false}, {{275, 97}, true}, {{275, 129}, false}, {{275, 141}, true}, {{275, 153}, false}, {{276, 97}, true}, {{276, 129}, false}, {{276, 142}, true}, {{276, 152}, false}, {{277, 99}, true}, {{277, 129}, false}, {{277, 143}, true}, {{277, 151}, false}, {{278, 100}, true}, {{278, 129}, false}, {{278, 144}, true}, {{278, 150}, false}, {{279, 101}, true}, {{279, 129}, false}, {{279, 145}, true}, {{279, 149}, false}, {{280, 102}, true}, {{280, 129}, false}, {{280, 146}, true}, {{280, 148}, false}, {{281, 104}, true}, {{281, 129}, false}, {{282, 106}, true}, {{282, 129}, false}, {{283, 108}, true}, {{283, 129}, false}, {{284, 111}, true}, {{284, 129}, false}, {{285, 115}, true}, {{285, 129}, false}, {{300, 94}, true}, {{300, 158}, false}, {{301, 95}, true}, {{301, 158}, false}, {{302, 95}, true}, {{302, 158}, false}, {{303, 96}, true}, {{303, 158}, false}, {{304, 96}, true}, {{304, 158}, false}, {{305, 96}, true}, {{305, 158}, false}, {{306, 96}, true}, {{306, 158}, false}, {{307, 97}, true}, {{307, 158}, false}, {{308, 97}, true}, {{308, 158}, false}, {{309, 97}, true}, {{309, 158}, false}, {{310, 97}, true}, {{310, 158}, false}, {{311, 96}, true}, {{311, 158}, false}, {{312, 96}, true}, {{312, 158}, false}, {{313, 96}, true}, {{313, 158}, false}, {{314, 96}, true}, {{314, 158}, false}, {{315, 95}, true}, {{315, 158}, false}, {{316, 95}, true}, {{316, 158}, false}, {{317, 94}, true}, {{317, 158}, false}, {{318, 93}, true}, {{318, 158}, false}, {{319, 92}, true}, {{319, 158}, false}, {{334, 133}, true}, {{334, 142}, false}, {{335, 130}, true}, {{335, 145}, false}, {{336, 128}, true}, {{336, 147}, false}, {{337, 95}, true}, {{337, 98}, false}, {{337, 126}, true}, {{337, 149}, false}, {{338, 95}, true}, {{338, 100}, false}, {{338, 125}, true}, {{338, 150}, false}, {{339, 94}, true}, {{339, 102}, false}, {{339, 124}, true}, {{339, 151}, false}, {{340, 93}, true}, {{340, 104}, false}, {{340, 123}, true}, {{340, 152}, false}, {{341, 93}, true}, {{341, 106}, false}, {{341, 122}, true}, {{341, 153}, false}, {{342, 92}, true}, {{342, 108}, false}, {{342, 122}, true}, {{342, 154}, false}, {{343, 92}, true}, {{343, 109}, false}, {{343, 121}, true}, {{343, 155}, false}, {{344, 92}, true}, {{344, 109}, false}, {{344, 121}, true}, {{344, 155}, false}, {{345, 91}, true}, {{345, 109}, false}, {{345, 120}, true}, {{345, 156}, false}, {{346, 91}, true}, {{346, 108}, false}, {{346, 120}, true}, {{346, 156}, false}, {{347, 90}, true}, {{347, 107}, false}, {{347, 119}, true}, {{347, 156}, false}, {{348, 90}, true}, {{348, 107}, false}, {{348, 119}, true}, {{348, 157}, false}, {{349, 90}, true}, {{349, 106}, false}, {{349, 119}, true}, {{349, 157}, false}, {{350, 90}, true}, {{350, 106}, false}, {{350, 118}, true}, {{350, 157}, false}, {{351, 89}, true}, {{351, 106}, false}, {{351, 118}, true}, {{351, 158}, false}, {{352, 89}, true}, {{352, 105}, false}, {{352, 118}, true}, {{352, 158}, false}, {{353, 89}, true}, {{353, 105}, false}, {{353, 118}, true}, {{353, 158}, false}, {{354, 89}, true}, {{354, 105}, false}, {{354, 118}, true}, {{354, 133}, false}, {{354, 140}, true}, {{354, 158}, false}, {{355, 88}, true}, {{355, 104}, false}, {{355, 117}, true}, {{355, 132}, false}, {{355, 141}, true}, {{355, 158}, false}, {{356, 88}, true}, {{356, 104}, false}, {{356, 117}, true}, {{356, 131}, false}, {{356, 142}, true}, {{356, 158}, false}, {{357, 88}, true}, {{357, 104}, false}, {{357, 117}, true}, {{357, 130}, false}, {{357, 143}, true}, {{357, 159}, false}, {{358, 88}, true}, {{358, 104}, false}, {{358, 117}, true}, {{358, 130}, false}, {{358, 144}, true}, {{358, 159}, false}, {{359, 88}, true}, {{359, 104}, false}, {{359, 117}, true}, {{359, 130}, false}, {{359, 144}, true}, {{359, 159}, false}, {{360, 88}, true}, {{360, 104}, false}, {{360, 117}, true}, {{360, 129}, false}, {{360, 144}, true}, {{360, 159}, false}, {{361, 88}, true}, {{361, 104}, false}, {{361, 117}, true}, {{361, 129}, false}, {{361, 145}, true}, {{361, 159}, false}, {{362, 88}, true}, {{362, 103}, false}, {{362, 117}, true}, {{362, 129}, false}, {{362, 145}, true}, {{362, 159}, false}, {{363, 88}, true}, {{363, 103}, false}, {{363, 117}, true}, {{363, 129}, false}, {{363, 145}, true}, {{363, 158}, false}, {{364, 87}, true}, {{364, 103}, false}, {{364, 117}, true}, {{364, 129}, false}, {{364, 145}, true}, {{364, 158}, false}, {{365, 87}, true}, {{365, 104}, false}, {{365, 117}, true}, {{365, 129}, false}, {{365, 145}, true}, {{365, 158}, false}, {{366, 87}, true}, {{366, 104}, false}, {{366, 117}, true}, {{366, 129}, false}, {{366, 145}, true}, {{366, 158}, false}, {{367, 87}, true}, {{367, 104}, false}, {{367, 117}, true}, {{367, 129}, false}, {{367, 145}, true}, {{367, 158}, false}, {{368, 87}, true}, {{368, 104}, false}, {{368, 117}, true}, {{368, 129}, false}, {{368, 145}, true}, {{368, 158}, false}, {{369, 88}, true}, {{369, 104}, false}, {{369, 117}, true}, {{369, 129}, false}, {{369, 144}, true}, {{369, 157}, false}, {{370, 88}, true}, {{370, 104}, false}, {{370, 117}, true}, {{370, 129}, false}, {{370, 144}, true}, {{370, 157}, false}, {{371, 88}, true}, {{371, 105}, false}, {{371, 117}, true}, {{371, 129}, false}, {{371, 144}, true}, {{371, 157}, false}, {{372, 88}, true}, {{372, 105}, false}, {{372, 117}, true}, {{372, 129}, false}, {{372, 143}, true}, {{372, 156}, false}, {{373, 88}, true}, {{373, 105}, false}, {{373, 117}, true}, {{373, 129}, false}, {{373, 143}, true}, {{373, 156}, false}, {{374, 88}, true}, {{374, 106}, false}, {{374, 117}, true}, {{374, 129}, false}, {{374, 142}, true}, {{374, 155}, false}, {{375, 88}, true}, {{375, 107}, false}, {{375, 117}, true}, {{375, 129}, false}, {{375, 142}, true}, {{375, 155}, false}, {{376, 88}, true}, {{376, 108}, false}, {{376, 117}, true}, {{376, 129}, false}, {{376, 141}, true}, {{376, 154}, false}, {{377, 89}, true}, {{377, 109}, false}, {{377, 117}, true}, {{377, 129}, false}, {{377, 140}, true}, {{377, 153}, false}, {{378, 89}, true}, {{378, 110}, false}, {{378, 117}, true}, {{378, 129}, false}, {{378, 138}, true}, {{378, 152}, false}, {{379, 89}, true}, {{379, 114}, false}, {{379, 117}, true}, {{379, 129}, false}, {{379, 136}, true}, {{379, 151}, false}, {{380, 89}, true}, {{380, 150}, false}, {{381, 90}, true}, {{381, 158}, false}, {{382, 90}, true}, {{382, 158}, false}, {{383, 90}, true}, {{383, 158}, false}, {{384, 91}, true}, {{384, 158}, false}, {{385, 91}, true}, {{385, 158}, false}, {{386, 92}, true}, {{386, 158}, false}, {{387, 92}, true}, {{387, 158}, false}, {{388, 93}, true}, {{388, 158}, false}, {{389, 94}, true}, {{389, 158}, false}, {{390, 94}, true}, {{390, 158}, false}, {{391, 95}, true}, {{391, 158}, false}, {{392, 96}, true}, {{392, 158}, false}, {{393, 97}, true}, {{393, 158}, false}, {{394, 98}, true}, {{394, 158}, false}, {{395, 100}, true}, {{395, 158}, false}, {{396, 102}, true}, {{396, 158}, false}, {{397, 104}, true}, {{397, 158}, false}, {{398, 107}, true}, {{398, 158}, false}, {{399, 111}, true}, {{399, 158}, false}};
}
inline std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> blue() {
    return std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> {{{1, 1}, false}, {{295, 73}, true}, {{295, 77}, false}, {{296, 70}, true}, {{296, 80}, false}, {{297, 68}, true}, {{297, 81}, false}, {{298, 67}, true}, {{298, 83}, false}, {{299, 66}, true}, {{299, 84}, false}, {{300, 65}, true}, {{300, 84}, false}, {{301, 65}, true}, {{301, 85}, false}, {{302, 64}, true}, {{302, 86}, false}, {{303, 64}, true}, {{303, 86}, false}, {{304, 63}, true}, {{304, 86}, false}, {{305, 63}, true}, {{305, 87}, false}, {{306, 63}, true}, {{306, 87}, false}, {{307, 63}, true}, {{307, 87}, false}, {{308, 63}, true}, {{308, 87}, false}, {{309, 63}, true}, {{309, 87}, false}, {{310, 63}, true}, {{310, 87}, false}, {{311, 63}, true}, {{311, 87}, false}, {{312, 63}, true}, {{312, 87}, false}, {{313, 63}, true}, {{313, 86}, false}, {{314, 64}, true}, {{314, 86}, false}, {{315, 64}, true}, {{315, 86}, false}, {{316, 65}, true}, {{316, 85}, false}, {{317, 65}, true}, {{317, 84}, false}, {{318, 66}, true}, {{318, 84}, false}, {{319, 67}, true}, {{319, 83}, false}, {{320, 68}, true}, {{320, 81}, false}, {{321, 70}, true}, {{321, 80}, false}};
}