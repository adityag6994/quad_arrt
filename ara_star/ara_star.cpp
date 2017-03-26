#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <openrave/planningutils.h>

#include <boost/bind.hpp>
#include <boost/algorithm/string/classification.hpp> 
#include <boost/algorithm/string/split.hpp> 

#include <iostream>
#include <string.h>
#include <tuple>
#include <ctime>
#include <cstdio>



#include "ara_star.h"

using namespace OpenRAVE;
using namespace std;

typedef vector<OpenRAVE::dReal> Point;
typedef boost::shared_ptr<Node> NodePtr;
typedef vector< NodePtr >  NodeTree;

//main RRT class
class ARAStarModule : public ModuleBase
{


public:
    //to connect with python
    ARAStarModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        _penv = penv;
        RegisterCommand("MyCommand",boost::bind(&ARAStarModule::MyCommand,this,_1,_2),
                        "This is an example command hello");
    }
    virtual ~ARAStarModule() {}
    
    void araStar(){
    	//main ARA* algorithm

    }
    
    
	private:
		Point _start;
		Point _goal;

		EnvironmentBasePtr _penv;
	    RobotBasePtr _robot;


};



// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtmodule" ) {
        return InterfaceBasePtr(new RRTModule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("RRTModule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}





