#include <stdint.h>
#include <stdbool.h>

void main_board_init(void);
void main_board_loop(void);

int main(void) {
    main_board_init();
    while (1) {
        main_board_loop();
    }
    return 0;
}
