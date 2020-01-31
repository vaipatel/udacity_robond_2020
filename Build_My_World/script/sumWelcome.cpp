#include <iostream>
#include <gazebo/gazebo.hh>

namespace gazebo
{
    class SumWelcomeWorldPlugin : public WorldPlugin
    {
        public:
        SumWelcomeWorldPlugin() : WorldPlugin()
        {
            std::cout << "Hello from Sum World!" << std::endl;            
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {}
    };

    GZ_REGISTER_WORLD_PLUGIN(SumWelcomeWorldPlugin)
}