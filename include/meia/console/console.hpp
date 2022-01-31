#include "main.h"

namespace meia {
    class Console {
        private:
            std::vector<std::pair<std::string, bool>> logs = {{"1", false}, {"2", true}, {"3", false}};
            void logo();
            void render();
            void clear();
        public:
            Console() {};
            int init();
            int print();
    };
}