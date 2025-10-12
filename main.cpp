#include "serial_mi48.hpp"
#include <string>

int main(int argc, char* argv[]) {
    std::string port_path = "/dev/ttyACM0"; // Default port
    if (argc > 1) {
        port_path = argv[1]; // Use provided port path
    } else {
        std::cout << "No port path provided, using default: " << port_path << "\n";
    }

    SerialCommandSender sender;
    if (sender.open_port(port_path)) {
        sender.stop_stream();
        sender.send_and_receive_serial_command();
        int camera_type;
        sender.get_senxor_type(camera_type);
    }

    sender.start_stream(true);
    sender.loop_on_read();
    // Port is closed automatically by destructor
    return 0;
}