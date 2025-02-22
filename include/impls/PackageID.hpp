#ifndef __DEFINE_PACKAGE_HPP__
#define __DEFINE_PACKAGE_HPP__


#include <memory>
#include <map>
#include <string>
#include <cstdint>
#include <vector>
#include "protocal/GlobalParam.hpp"
#include "impls/logger.hpp"
#include "impls/exception.hpp"

typedef uint64_t ID; // 定义ID类型

int editDistance(const std::string& str1, const std::string& str2);
double stringSimilarity(const std::string& str1, const std::string& str2);

/**
 * @brief ID作为统一各种类型的包的标识，一定是端口ID，组ID和包ID共同组成的。
 *        结构为： [ reserve : 24 | group id : 8 | port id : 8 | device type : 8 | package id : 16 ] 
*/

class PackageID
{
public:
    struct PackageNameIdMapper
    {
        std::map<std::string, PackageID*> package_name_id_mapper;
    };
public:
    ID id;

    // PackageID(): id(0){};
    PackageID() = delete;
    PackageID(ID _id): id(_id){}
    PackageID(std::string package_id_name){
        GET_PARAM(PackageID::PackageNameIdMapper)->package_name_id_mapper[package_id_name] = this;
    }

    operator ID() const{
        return id;
    }
    ID operator =(ID _id){
        this->id = _id;
        return _id;
    }

    static bool setPacakgeID(std::string package_name, ID value){
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
};

#define ADD_PACKAGE(PACKAGE_NAME) \
PackageID PACKAGE_NAME(#PACKAGE_NAME);

#define SET_PACKAGE(PACKAGE_NAME, VALUE) \
PackageID::setPacakgeID(PACKAGE_NAME, VALUE)

#define GET_PACKAGE(PACKAGE_NAME) \
GET_PARAM(PackageID::PackageNameIdMapper)->package_name_id_mapper[PACKAGE_NAME]->id;

/**
 * @brief 这里通过include的方式，引入Package的全局变量。
 */
#include "protocal/IDs.hpp"

// 用来约束IDType可以使用的类型
template<typename T>
concept IDType = std::is_same<T, ID>::value ||
                 std::is_same<T, PackageID>::value;

/**
 * @brief 计算两个字符串之间的编辑距离(edit distance)
 * 
 * @param str1 第一个字符串
 * @param str2 第二个字符串
 * @return int 距离
 */
int editDistance(const std::string& str1, const std::string& str2) {
    size_t len1 = str1.size(), len2 = str2.size();
    if (len1 < len2) std::swap(len1, len2);
    std::vector<int> prevRow(len2 + 1), currRow(len2 + 1);

    for (size_t j = 0; j <= len2; ++j) prevRow[j] = j;

    for (size_t i = 1; i <= len1; ++i) {
        currRow[0] = i;

        for (size_t j = 1; j <= len2; ++j) {
            int cost = (str1[i - 1] == str2[j - 1]) ? 0 : 1;
            currRow[j] = std::min({currRow[j - 1] + 1,
                                   prevRow[j] + 1,
                                   prevRow[j - 1] + cost});
        }

        prevRow.swap(currRow);
    }

    return prevRow[len2];
}

/**
 * @brief 根据编辑距离计算两个字符串之间的相似度，就是用编辑距离除以最长字符串的长度
 * 
 * @param str1 第一个字符串
 * @param str2 第二个字符串
 * @return double 相似度，0为完全不相似，1为相同
 */
double stringSimilarity(const std::string& str1, const std::string& str2) {
    int distance = editDistance(str1, str2);
    size_t maxLength = std::max(str1.size(), str2.size());
    return maxLength > 0 ? (maxLength - distance) / static_cast<double>(maxLength) : 1.0;
}

#endif // __DEFINE_PACKAGE_HPP__
