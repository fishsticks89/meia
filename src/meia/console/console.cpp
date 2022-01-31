#include "logo.cpp"
#include "main.h"
#include "pros/colors.h"
namespace meia {
    static const int maincolor = RGB2COLOR(255, 255, 255);
    static const int accentcolor = RGB2COLOR(31, 97, 232);
    static const int lineheight = 25;
    static const int breaklineoffset = 10;
    void Console::clear() {
        if (pros::lcd::is_initialized() == true)
            throw "llemu cannot be initalized when using meia lcd";
        pros::screen::set_eraser(RGB2COLOR(0, 0, 0));
        pros::screen::erase_rect(1, 1, 480, 272);
    }
    void drawbutton(int xoffset, int yoffset, int width, int height) {
        if (width < height)
            throw "buttonwidth must be greater than buttonheight"; // height must be less than width
        pros::screen::set_pen(maincolor);
        pros::screen::fill_circle(480/2 + width/2 + xoffset, yoffset, height/2);
        pros::screen::fill_circle(480/2 - width/2 + xoffset, yoffset, height/2);
        pros::screen::fill_rect(
            480/2 - width/2 + xoffset, 
            height/2 + yoffset, 
            480/2 + width/2 + xoffset, 
            -height/2 + yoffset);
    }
    void Console::render() {
        clear();
        int logindex;
        // console stuff
        const int lines = 6;
        for (int i = lines; i >= 0; i--) {
            logindex = (lines - i);
            if (logs.size() > logindex) {
                pros::screen::set_pen(maincolor);
                pros::screen::print(pros::E_TEXT_MEDIUM, lineheight, (i * lineheight) - 7, logs.at(logindex).first.c_str());
                if (logs.at(logindex).second) {
                    pros::screen::set_pen(accentcolor);
                    pros::screen::draw_line(lineheight, (i * lineheight) + breaklineoffset, 480 - lineheight, (i * lineheight) + breaklineoffset);
                }
            }
        }
        drawbutton(0, 195, 60, 20);
        drawbutton(150, 195, 60, 20);
        drawbutton(-150, 195, 60, 20);
    }
    void Console::logo() {
        clear();
        std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> whitels = white();
        std::vector<std::pair<std::pair<int_fast16_t, int_fast16_t>, bool>> bluels = blue();
        int i = 0;
        bool draw = false;
        pros::screen::set_pen(RGB2COLOR(0, 0, 0));
        pros::screen::draw_rect(0, 0, 480, 271);
        pros::screen::set_pen(maincolor);
        for (int x = 0; x <= 480; x++) {
            for (int y = 0; y <= 272; y++) {
                if (std::pair<int_fast16_t, int_fast16_t>{whitels[i].first.first, whitels[i].first.second} == std::pair<int_fast16_t, int_fast16_t>{x, y}) {
                    draw = whitels[i].second;
                    i++;
                }
                if (draw)
                    pros::screen::draw_pixel(x, y);
            }
        }
        i = 0;
        draw = false;
        pros::screen::set_pen(accentcolor);
        for (int x = 0; x <= 480; x++) {
            for (int y = 0; y <= 272; y++) {
                if (std::pair<int_fast16_t, int_fast16_t>{bluels[i].first.first, bluels[i].first.second} == std::pair<int_fast16_t, int_fast16_t>{x, y}) {
                    draw = bluels[i].second;
                    i++;
                }
                if (draw)
                    pros::screen::draw_pixel(x, y);
            }
        }
    }
    int Console::init() {
        logo();
        return 1;
    }
    int Console::print() {
        render();
        return 1;
    }
} // namespace meia