#include <chrono>    // For std::chrono::seconds
#include <iostream>  // For std::cout
#include <rerun.hpp>
#include <rerun/components/color.hpp>
#include <rerun/components/position3d.hpp>
#include <rerun/components/radius.hpp>
#include <rerun/datatypes/rgba32.hpp>
#include <rerun/datatypes/vec3d.hpp>
#include <string>
#include <thread>  // For std::this_thread::sleep_for
#include <vector>

int main(int argc, char* argv[]) {
    // Create a new `RecordingStream` which sends data over gRPC to the viewer process.
    const auto rec = rerun::RecordingStream("rerun_example_cpp_test_simplified");

    auto spawn_result = rec.spawn();
    if (spawn_result.is_err()) {
        std::cerr << "ERROR: Failed to spawn Rerun Viewer. ";
        // The exact method to get string from spawn_result.err() is still unconfirmed for this
        // version. std::cerr << spawn_result.err().format() << std::endl; // One of the previous
        // guesses
        std::cerr << "Error details unavailable with current error handling code for spawn."
                  << std::endl;
        return 1;
    }
    std::cout << "Rerun Viewer spawned." << std::endl;

    // Log a startup text message
    rec.log("status", rerun::TextLog("Simplified test application started."));
    std::cout << "Logged initial status message." << std::endl;

    // Try logging a single Position3D component instance by wrapping it in a std::vector
    std::vector<rerun::components::Position3D> single_point_vec = {
        rerun::components::Position3D(1.0f, 2.0f, 3.0f)};
    rec.log("test_point_path/a_single_point", single_point_vec);
    std::cout << "Logged a single Position3D component." << std::endl;

    // Try logging a single Color component instance by wrapping it in a std::vector
    std::vector<rerun::components::Color> single_color_vec = {
        rerun::components::Color(255, 0, 128, 255)};  // RGBA
    rec.log("test_color_path/a_single_color", single_color_vec);
    std::cout << "Logged a single Color component." << std::endl;

    // Log a final text message
    rec.log("status", rerun::TextLog("Finished logging simple components. Application will wait."));
    std::cout << "Finished logging simple components." << std::endl;

    std::cout << "Attempting to flush Rerun stream..." << std::endl;
    rec.flush_blocking();  // Explicitly flush the data
    std::cout << "Flush complete." << std::endl;

    if (rec.is_enabled()) {
        std::cout << "Rerun stream is still enabled after flush." << std::endl;
    } else {
        std::cout << "WARNING: Rerun stream is NOT enabled after flush." << std::endl;
    }

    std::cout << "Application will wait for 30 seconds." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(30));

    std::cout << "Application exiting." << std::endl;
    return 0;
}
