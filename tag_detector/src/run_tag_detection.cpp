#include <iostream>

#include "LibCameraWrapper.h"
#include "TagDetector.h"


int main(int argc, char **argv)
{
    // Parse input
    bool saveImages = false;
    bool verbose = false;
    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == "--save_images")
            saveImages = true;
        else if (std::string(argv[i]) == "--verbose")
            verbose = true;
        else
        {
            std::cout << "Unknown parameter: \"" << argv[i] << "\"" << std::endl;
            exit(0);
        }
    }

    SHMWriter shmWriter;
    if (!shmWriter.init())
    {
        std::cout << "Failed to create shared memory!" << std::endl;
        return -1;
    }
    VisionBuffer buffer;

    int const width = 640;
    int const height = 480;
    int const fx = 330.;
    int const fy = 330.;
    int const cx = fx/2.;
    int const cy = fy/2.;

    LibCameraWrapper camera;
    camera.start(width, height);
    TagDetector detector(width, height, fx, fy, cx, cy);

    for (int i = 0; i < 10; i++)
    {
        cv::Mat img = camera.waitForNextFrame();

        if (saveImages)
        {
            std::cout << "Saving image: imgs/test_" + std::to_string(i) + ".png" << std::endl;
            cv::imwrite("imgs/test_" + std::to_string(i) + ".png", img);
        }
        MarkerList detected_markers = detector.find_markers(img);

        // Update shared memory
        buffer.nMarkers = static_cast<int>(detected_markers.size());
        for (int i = 0; i < std::min(MAX_MARKERS, buffer.nMarkers); i++)
        buffer.markers[i] = detected_markers.at(i);
        shmWriter.update(buffer);

        // Print stats
        if (verbose)
        {
            if(!detected_markers.empty())
            {
                std::cout << "Detected " << detected_markers.size() << " markers." << std::endl;
                for(Marker const& marker : detected_markers)
                {
                    std::stringstream msg;
                    msg << "- marker detected at " << marker.radius << " m" << std::endl;
                    std::cout << msg.str();
                }
            }
            else
            {
                std::cout << "No marker has been detected!" << std::endl;
            }
        }
    }

    return 0;
}