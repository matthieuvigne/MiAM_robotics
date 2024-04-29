// Test solar panel camera
#include <miam_utils/Metronome.h>

#include "common/solar_panel_camera.hpp"


int main (int argc, char *argv[])
{
    vision::SolarPanelCamera camera("/dev/video0");

    Metronome metronome(0.002 * 1e9);
    while (true)
    {
        std::cout << metronome.getElapsedTime() << " camera: " << camera.getSolarPanelOrientation(false) << std::endl;
    }

    return 0;
}