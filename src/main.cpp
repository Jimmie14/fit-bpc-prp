#include <rclcpp/rclcpp.hpp>

#include "App.hpp"
#include "Components/ArucoDetectionEngine.hpp"
#include "Components/ButtonsDriver.hpp"
#include "Components/ExplorerEngine.hpp"
#include "Components/ImuDriver.hpp"
#include "Components/LidarDriver.hpp"
#include "Components/LineEngine.hpp"
#include "Components/MapThinningUnit.hpp"
#include "Components/MappingEngine.hpp"
#include "Components/MotorDriver.hpp"
#include "Components/OdometryEngine.hpp"
#include "Components/UserInputDriver.hpp"
#include "Components/DwppNavigatorEngine.hpp"

using namespace std;
using namespace Manhattan;

int main(const int argc, char* argv[])
{
    init(argc, argv);

    const auto app = make_shared<App>();


    app->addComponent<ImuDriver>()->Enable();
    app->addComponent<LidarDriver>()->Enable();
    app->addComponent<MotorDriver>()->Enable();
    app->addComponent<ButtonsDriver>()->Enable();

    app->addComponent<LineEngine>();
    app->addComponent<UserInputDriver>();

    app->addComponent<OdometryEngine>()->Enable();
    app->addComponent<MappingEngine>()->Enable();

    app->addComponent<ArucoDetectionEngine>()->Enable();
    app->addComponent<DwppNavigatorEngine>()->Enable();

    app->addComponent<MapThinningUnit>()->Enable();
    app->addComponent<ExplorerEngine>()->Enable();

    app->run();

    shutdown();
    return 0;
}
