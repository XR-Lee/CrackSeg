#include "PointCloudProcessor.hpp"
#include <iostream>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv) {
    try {
        // Define available options
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help,h", "Produce help message") // Show help information
            ("point_cloud_path,p", po::value<std::string>(), "Path to the point cloud data file")
            ("odometry_path,o", po::value<std::string>(), "Path to odometry data file")
            ("images_folder,i", po::value<std::string>(), "Path to directory containing images")
            ("mask_image_folder,m", po::value<std::string>()->default_value(""), "Path to directory for segmented images")
            ("output_path,t", po::value<std::string>()->default_value("."), "Path to save processed output")
            ("enableMLS,e", po::value<bool>()->default_value(false), "Enable MLS smoothing")
            ("enableNIDOptimize,e", po::value<bool>()->default_value(false), "Enable NID-based camera pose optimization")
            ("enableInitialGuessManual,e", po::value<bool>()->default_value(false), "Enable manual pickup point based camera pose optimization");

        // Store and parse command line arguments
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        // Display help message if needed
        if (vm.count("help")) {
            std::cout << desc << "\n";
            return 1;
        }

        // Ensure required arguments are present
        if (vm.count("point_cloud_path") && vm.count("odometry_path") && vm.count("images_folder")) {
            std::string pointCloudPath = vm["point_cloud_path"].as<std::string>();
            std::string odometryPath = vm["odometry_path"].as<std::string>();
            std::string imagesFolder = vm["images_folder"].as<std::string>();
            
            // Check if mask_image_folder was provided; if not, set to a default or use imagesFolder
            std::string maskImageFolder = vm.count("mask_image_folder") ? vm["mask_image_folder"].as<std::string>() : imagesFolder;
            
            std::string outputPath = vm["output_path"].as<std::string>();
            bool enableMLS = vm["enableMLS"].as<bool>();
            bool enableNIDOptimize = vm["enableNIDOptimize"].as<bool>();
            bool enableInitialGuessManual = vm["enableInitialGuessManual"].as<bool>();

            PointCloudProcessor processor(
                pointCloudPath, 
                odometryPath, 
                imagesFolder, 
                maskImageFolder, 
                outputPath, 
                enableMLS,
                enableNIDOptimize,
                enableInitialGuessManual);

            processor.process();
            std::cout << "Processing completed successfully." << std::endl;
        } else {
            std::cerr << "Error: Missing required arguments." << std::endl;
            std::cerr << desc << std::endl;
            return -1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Unhandled Exception reached the top of main: "
                  << e.what() << ", application will now exit" << std::endl;
        return -2;
    }

    return 0;
}
