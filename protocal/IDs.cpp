#ifndef __PACKAGE_ID_CPP__
#define __PACKAGE_ID_CPP__

#include "protocal/IDs.hpp"
#include "protocal/GlobalParam.hpp"
#include "impls/logger.hpp"
#include "utils/Utility.hpp"

PackageID::PackageID(std::string package_id_name){
    GET_PARAM(PackageID::PackageNameIdMapper)->package_name_id_mapper[package_id_name] = this;
}

PackageID::operator ID() const{
    return id;
}

ID PackageID::operator =(ID _id){
    id = _id;
    return _id;
}

bool PackageID::setPacakgeID(std::string package_name, ID value)
{
    auto package = GET_PARAM(PackageID::PackageNameIdMapper)->package_name_id_mapper.find(package_name);
    if (package != GET_PARAM(PackageID::PackageNameIdMapper)->package_name_id_mapper.end()){
        package->second->id = value;
        return true;
    }
    else{
        std::string sim_string;
        double similarity = 0.0;
        for (const auto& it : GET_PARAM(PackageID::PackageNameIdMapper)->package_name_id_mapper)
        {
            double sim = stringSimilarity(package_name, it.first);
            if (sim > similarity){
                similarity = sim;
                sim_string = it.first;
            }
        }
        if(sim_string.empty()){
            std::string error_msg = "package name '" + package_name + "' not found, and no similar name was defined either!";
            LOGERROR(error_msg.c_str());
            throw PORT_EXCEPTION(error_msg);
        }
        else{
            std::string error_msg = "package name '" + package_name + "' not found, did you mean '" + sim_string + "'?";
            LOGERROR(error_msg.c_str());
            throw PORT_EXCEPTION(error_msg);
        }
        return false;
    }
    return false;
}


#endif // __PACKAGE_ID_CPP__