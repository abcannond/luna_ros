#include "luna_linaks/LinakActuator.hpp"

int main()
{
    // test_mode=true: prints register contents instead of writing to CAN
    LinakActuator act("can0", 0xC8, 0x01, true);

    act.set_speed(80);
    act.run_out();
    act.send_command();

    act.run_to_position(150.0f);
    act.send_command();

    act.stop();
    act.send_command();

    return 0;
}
