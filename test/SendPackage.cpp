/**
 * @file SendPackage.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>
#include <map>
#include <regex>
#include <random>
#include <thread>
#include <chrono>

#include <boost/program_options.hpp>

#include <impls/PackageID.hpp>
#include <impls/logger.hpp>
#include <impls/PackageInterface.hpp>
#include <PackageManager.hpp>
#include <PortManager.hpp>

#include <pkg/Gimbal.hpp>
#include <pkg/Shoot.hpp>

/**************************发包流程*********************************** */
using namespace transport;
using SendFunction = std::function<void(PackageID)>;

class Wapper {
public:
    static Wapper& getInstance() {
        static std::once_flag once_flag;
        std::call_once(once_flag, [] {
            instance = std::unique_ptr<Wapper>(new Wapper());
        });
        return *instance;
    }

    std::map<std::string, SendFunction> generator;
    std::shared_ptr<config::Config> config;
    std::shared_ptr<PackageManager> packageManager;
    std::shared_ptr<PortManager> portManager;

private:
    using RegisterFunction = std::function<void()>;
    static std::unique_ptr<Wapper> instance;

    explicit Wapper() {
        config = std::make_shared<config::Config>();
        packageManager = std::make_shared<PackageManager>(config);
        portManager = std::make_shared<PortManager>(config, packageManager);
    }
};

std::unique_ptr<Wapper> Wapper::instance;

template<typename PackageType, typename GenerateFunc>
void register_function(const std::string& type_name, GenerateFunc generate_func) {
    SendFunction send_function = [generate_func](auto id) {
        PackageType pkg = generate_func();
        Wapper& wapper = Wapper::getInstance();
        wapper.packageManager->send(id, pkg);
    };
    Wapper& wapper = Wapper::getInstance();
    wapper.generator[type_name] = send_function;
}

bool use_default_constructor = true;
template<typename PackageType>
void register_function(const std::string& type_name, bool use_default_constructor) {
    SendFunction send_function = [](auto id) {
        PackageType pkg;
        Wapper& wapper = Wapper::getInstance();
        wapper.packageManager->send(id, pkg);
    };
    Wapper& wapper = Wapper::getInstance();
    wapper.generator[type_name] = send_function;
}

#ifdef ADD_TYPE
#undef ADD_TYPE
#endif // ADD_TYPE
#define ADD_TYPE(type, func) register_function<type>(#type, func);

/*************************注册包*************************************** */

/**
 * @brief 在这里一个一个填上你想要如何生成一个虚拟包，宏定义需要两个参数，第一个是类型，第二个是生成这个包的lambda表达式，
 *        lambda表达式的参数为空，返回值为包类型，其他不限，使用use_default_constructor参数时，使用默认构造函数生成包
 */
void register_all() {
    ADD_TYPE(GimbalPackage, [](){
        for(int i = 0; i < 10; ++i) {
            std::cout << i;
        }
        std::cout << "\n";
        GimbalPackage package;
        package.m_pitch_angle = 10;
        return package;
    });

    ADD_TYPE(ShootPackage, use_default_constructor);

}

/**************************参数解析************************************* */
namespace po = boost::program_options;

struct PackageInfo{
    std::string package_name;
    int id;
    int count;
};

class ParserArgs {
public:
    ParserArgs(int argc, char* argv[]) : desc("Allow Options") {
        desc.add_options()
            ("help,h", "produce help message")
            ("packages,p", po::value<std::string>(), "packages you want to send")
            ("id", po::value<std::string>(), "id you want to send")
            ("count,c", po::value<int>()->default_value(1), "count you want to send, if < 0, continue send, default 1")
            ("type,t", po::value<std::string>()->default_value("random"), "what type you want to send package, random|sequence")
            ("interval,i", po::value<int>()->default_value(1000), "time interval between each package, unit -> ms, default 1000ms");
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
        parser();
    }

    std::vector<PackageInfo> packages;
    std::string send_type;
    int time_interval = 1000;

    PackageInfo& getPackageInfo() {
        if (send_type == "random") {
            return getRandomPackageInfo();
        } else {
            return getSequencePackageInfo();
        }
    }

private:
    po::options_description desc;
    po::variables_map vm;
    int seq_index = -1;

    class RandomNum {
        std::mt19937 gen;
        std::uniform_int_distribution<> dis;

    public:
        RandomNum(int start, int end) : gen(std::random_device{}()), dis(start, end) {}

        int get() {
            return dis(gen);
        }
    };

    std::unique_ptr<RandomNum> p_random;

    void parser() {
        std::vector<std::string> packages;
        std::vector<ID> ids;
        int count = 1;
        if (vm.count("help")) {
            std::cout << desc << "\n";
        }
        if (vm.count("packages")) {
            packages = splitString(vm["packages"].as<std::string>());
        }
        if (vm.count("id")) {
            std::vector<std::string> string_ids = splitString(vm["id"].as<std::string>());
            for (auto& s : string_ids) {
                ID id = std::stol(s, 0, 0);
                ids.push_back(id);
            }
        }
        if (vm.count("count")) {
            count = vm["count"].as<int>();
        }
        if (vm.count("type")) {
            send_type = vm["type"].as<std::string>();
        }
        if (vm.count("interval")) {
            time_interval = vm["interval"].as<int>();
        }

        if (packages.size() != 0 && packages.size() == ids.size()) {
            for (auto i = 0; i < packages.size(); ++i) {
                PackageInfo info;
                info.package_name = packages[i];
                info.id = ids[i];
                info.count = count;
                this->packages.push_back(info);
            }
        } else {
            throw std::logic_error("command line arguments do not meet the requirements");
        }
    }

    PackageInfo& getRandomPackageInfo() {
        if (!p_random) {
            p_random = std::make_unique<RandomNum>(0, packages.size());
        }
        int index = p_random->get();
        index = index % packages.size();
        return packages[index];
    }

    PackageInfo& getSequencePackageInfo() {
        seq_index++;
        seq_index = seq_index % packages.size();
        return packages[seq_index];
    }

    std::vector<std::string> splitString(const std::string& str) {
        std::regex re("[,;| ]");    // 支持使用 ‘,’ ‘;’ ‘|’ ‘ ’四种字符作为分隔符
        std::sregex_token_iterator iter(str.begin(), str.end(), re, -1);  
        std::sregex_token_iterator end;  
        std::vector<std::string> tokens(iter, end); 
        return tokens;
    }
};


int main(int argc, char* argv[]){
    LOGINIT(argv[0]);
    ParserArgs args(argc, argv);
    Wapper& wapper = Wapper::getInstance();
    register_all();
    // wapper.generator[type](CAN_ID_CHASSIS);
    
    while (transport::ok())
    {
        PackageInfo& info = args.getPackageInfo();
        if(wapper.generator.find(info.package_name) == wapper.generator.end()) {
            continue;
        }
        bool all_send = std::all_of(args.packages.begin(), args.packages.end(), [](PackageInfo& info){ return info.count <= 0;});
        if (all_send) 
        {
            std::cout << "all packages send" << std::endl;
            break;
        }
        if(info.count <= 0) {
            if (info.count == 0) {
                std::cout << "package '" << info.package_name << "' count is zero, send finished" << std::endl;
                info.count--;
            }
            continue;
        }
        ID id = info.id;
        for(auto& pkg : wapper.config->m_package_list) {
            if(id == pkg.m_oid) {
                id = pkg.m_id; 
            }
        }
        std::cout << "send package name: " << info.package_name << std::endl;
        wapper.generator[info.package_name](id);
        info.count -= 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(args.time_interval));
    }
    transport::shutdown();
}