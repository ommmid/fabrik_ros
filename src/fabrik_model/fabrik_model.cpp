
#include "fabrik_ros/fabrik_model.h"

#include <iostream>



FabrikModel::FabrikModel(const std::string& urdf_file)
{
    std::string urdf_file_path = "/home/omid/ws_fabrik_ros/src/fabrik_ros/urdf/model.urdf";
    
    if (!urdf_model_.initFile(urdf_file_path)){
        std::cout << "Failed to parse urdf file" << std::endl;
        std::terminate();
    }
}


FabrikModel::FabrikModel(const urdf::Model& urdf_model):
urdf_model_(urdf_model)
{

}
