#include "serial_mi48.hpp"

int main() {
    SerialCommandSender sender;
    if (sender.open_port()) {
        sender.stop_stream();
        sender.send_and_receive_serial_command();
        int camera_type;
        sender.get_senxor_type(camera_type);
    }

    sender.start_stream(false);
    sender.loop_on_read();
    // Port is closed automatically by destructor
    return 0;
}