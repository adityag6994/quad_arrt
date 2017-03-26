#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <openrave/planningutils.h>

#include <boost/bind.hpp>
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split

#include <iostream>
#include <string.h>
#include <tuple>
#include <ctime>
#include <cstdio>



#include "myplugin.h"

using namespace OpenRAVE;
using namespace std;

size_t MAX_ITERATIONS = 200;//used for smoothing
int RRTNode::CurrentID = 0;
dReal GOAL_BIAS = 0.16;//biasing towards goal 16% OPTIMAL
dReal delta_Q = 0.35;//step size
int flag = 0;//checks if goal is reached


typedef std::vector<OpenRAVE::dReal> Config; //configration for robot
typedef boost::shared_ptr<RRTNode> Node; //node containing configration and parents address
typedef boost::shared_ptr<NodeTree> Tree;//tree to hold vector of nodes

//main RRT class
class RRTModule : public ModuleBase
{


public:
    //to connect with python
    RRTModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        _penv = penv;
        RegisterCommand("MyCommand",boost::bind(&RRTModule::MyCommand,this,_1,_2),
                        "This is an example command hello");
    }
    virtual ~RRTModule() {}
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        cout << "*PLUGIN STARTED" << endl;   
        //setting up clocks
        // clock_t start;
       
        // start = clock();
        //setenvironment and robot
        _penv->GetRobots(_robots);
        _robot = _robots.at(0);
        _robot->GetActiveDOFValues(_startConfig);
        
        // set goal config
         _goalConfig.push_back(0.449);
         _goalConfig.push_back(-0.201);
         _goalConfig.push_back(-0.151);
         _goalConfig.push_back(-0.11);
         _goalConfig.push_back(0);
         _goalConfig.push_back(-0.11);
         _goalConfig.push_back(0);

         //get active dof limits
        _robot->GetActiveDOFLimits(_lowerLimit, _upperLimit);

        //changing the limits of last and second last DOF
        _lowerLimit[4] = -3.14;
        _lowerLimit[6] = -3.14;
        
        _upperLimit[4] = 3.14;
        _upperLimit[6] = 3.14;
        
        /*
        Lower Limits:  Config:  -0.564602 -0.3536 -2.12131 -2.00001 -10000 -2.00001 -10000 
        Upper Limits:  Config:   2.13539   1.2963 -0.15    -0.1      10000 -0.1      10000 
        */
        //weights used while calculating neirest node
        _weight.push_back(3.17104);
        _weight.push_back(2.75674);
        _weight.push_back(2.2325);
        _weight.push_back(2.2325);
        _weight.push_back(1.78948);
        _weight.push_back(0);
        _weight.push_back(0.809013);
        _weight.push_back(0);



        //initialising tree
        _startNode = Node(new RRTNode());
        _startNode->setConfig(_startConfig);
        _startNode->setParent(nullptr);
        Config test = _startNode->getConfig();
        
        //adding start to tree
        _mainTree.setrootNode(_startNode);
        _mainTree.nodeAdd(_startNode);

        _randomConfig = _startConfig;
        //Main Algorithm Starts Here
        count = 0;
        int cc=0;

        //do until goal is reached
        while(count++ < 100000){
            
            
            //------------------------get random node
            _randomConfig = randomnodeGenerator();
            
            //------------------------check for nearest node in tree
            _nearestNode = NNNode();
                       
            
            //------------------------Get the step size
             _stepSize = step();
             
            if(flag){
            //-----------------------if goal is reached, break out of while loop    
                cout << "Goal Found" << endl;
                break;
                
            }else{
            //------------------------else get the next node
                EXTEND(count);
                //--------------------just to get an idea where we stand currently
                if(cc % 1000 == 0){
                    cout << "Searching.." << count << endl;
                }
                cc++;

            }

        }

        //clock stops as, we are out of loop with path searched :)
        // duration = (clock() - start)/(double)CLOCKS_PER_SEC;

        //get the path trajector
        getPath();

        //exectue the path using robots controller
        ExecuteTrajectory();

        cout << "*PLUGIN FINISHED" << endl;   
        
        //draw the path
        Draw();

        //finish the loop 
        return true;

    }

    //getPath from last found goal as nearest neibhour, used to get path from reached tree
    void getPath(){
        //save in _path
        _path.push_back(_nearestNode);

        while(_nearestNode->getParent() != nullptr){
            _nearestNode = _nearestNode->getParent();
            _path.push_back(_nearestNode);
        }


    }

    //Print config function
    void printConfig(string s ,Config config){
        cout << s <<" Config:  " ;
        for(size_t i = 0; i < 7 ; i++){
                cout << config[i] << ' ';
            }
            cout << endl;
    }

    //getStringVector,used to read vector from python as string
    vector<dReal> getStringVector(std::ostream& sout, std::istream& sinput){
        vector<std::string> words;
        string s;
        ostringstream os;
        os<<sinput.rdbuf();
        s=os.str();
        std::vector<dReal> goal_config;
        boost::split(words, s, boost::is_any_of(", "), boost::token_compress_on);
        for(int i=0; i<7; i++){
            dReal num = atof(words.at(i).c_str());
            goal_config.push_back(num);
        }
        return goal_config;
    }

    //check if config are in limits
    bool checkConfigLimits(Config config){
    /*
        Lower Limits:  Config:  -0.564602 -0.3536 -2.12131 -2.00001 -10000 -2.00001 -10000 
        Upper Limits:  Config:   2.13539   1.2963 -0.15    -0.1      10000 -0.1      10000 
    */
        int sum = 0;
        for(size_t i = 0; i<7 ;i++){
            if(config[i] < _upperLimit[i] && config[i] > _lowerLimit[i]){
                sum++;
                // cout << " i : " << i;
            }
        }
        if(sum==7)
            return true;
        else
            return false;

    }

    //generate random node
    Config randomnodeGenerator(){

        //generate random number
        dReal bias= RaveRandomFloat();
       
        Config temp_config = _goalConfig;
        
        if(bias < GOAL_BIAS){
            temp_config = _goalConfig;
        }else{
            do{

                for (size_t i = 0; i < 7; ++i) {

                    temp_config[i] = static_cast<dReal>(bias*(_upperLimit[i] - _lowerLimit[i]) + _lowerLimit[i]);
                    bias = RaveRandomFloat();//change random configration next time <common sense , not so common :'D  :| >

                   }    
                   //check for validity
                }while(CheckCollision(temp_config)||!inLimits(temp_config));   
        }
        return temp_config; 
    
    }

    //nearest node 
   boost::shared_ptr<RRTNode> NNNode(){
    
        dReal temp_val = 0;
        dReal min_val = 1000000;
        dReal diff;
        size_t range = _mainTree.getTreeSize();
        vector< boost::shared_ptr<RRTNode> > temp_tree = _mainTree.getfullPath();
    
        for(size_t i=0; i<range; i++){
    
            Config temp_config = temp_tree[i]->getConfig();

            for(size_t j=0; j<7; j++){

                  temp_val += pow((-temp_config[j] + _randomConfig[j])*_weight[j], 2);
            }

            temp_val = sqrt(temp_val);

            diff = min_val - temp_val;

            if(diff > 0){
                min_val = temp_val;
                _nearestNode = temp_tree[i];
                _nearestNode->setUniqueId(i);
            }
        }

        return _nearestNode;
    }
   
   //difference berween node and goal
   dReal differenceCost(Config config){
        dReal temp_val;
        
        for(size_t j=0; j<7; j++){
                  temp_val += pow((-_goalConfig[j] + config[j]), 2);//*_weight[j]
            }

            temp_val = sqrt(temp_val);
            return temp_val;
   }
    
//difference between node and random config
    dReal differencebetweenTwo(Config config){
        dReal temp_val;
        for(size_t j=0; j<7; j++){
                  temp_val += pow((-_randomConfig[j] + config[j]), 2);//*_weight[j]
            }

            temp_val = sqrt(temp_val);
            return temp_val;    
    }
   

    //find step size as vector in direcion toward random node from nearest node
   Config step(){
        
        Config temp_step = _randomConfig;
        dReal magnitude = 0.0;

        for(size_t i=0 ; i<7 ; i++){
            temp_step[i] = _randomConfig[i] - _nearestNode->getConfig()[i];
        }

        for(size_t i=0; i<7; i++){
            magnitude += pow(temp_step[i], 2.0);
        }
        magnitude = sqrt(magnitude);

        for (size_t i = 0; i < 7; ++i) {
            temp_step[i] = ((temp_step[i]) * delta_Q) / magnitude;
        }

        return temp_step;
   }



   //extend in the direction of nearest node
    void EXTEND(int count){
        string s;
        //flag = 0;
        Config temp_new_config = _nearestNode->getConfig();

        do{
        
            temp_new_config = _nearestNode->getConfig();
            
            for(size_t i=0; i<7 ; i++){
                temp_new_config[i] += _stepSize[i];
        
            }
            //if not in limits start over again    
            if(!inLimits(temp_new_config) || CheckCollision(temp_new_config)){
        
                break;
            }

             _mainTree.nodeAdd(Node(new RRTNode(temp_new_config, _nearestNode)));
             _nearestNode = _mainTree.getLast();       
            _diffConfig = differencebetweenTwo(temp_new_config);
            if(_diffConfig < delta_Q){
                _mainTree.nodeAdd(Node(new RRTNode(_randomConfig, _nearestNode)));
                _diffGoal = differenceCost(_randomConfig);

            if(_diffGoal < delta_Q){
                _mainTree.nodeAdd(Node(new RRTNode(_goalConfig, _nearestNode)));
                flag = 1;
                break;

            }

            }
 
        }while(!CheckCollision(temp_new_config) && inLimits(temp_new_config));// && checkConfigLimits(temp_new_config));
    }

    //check if config is in limit or not
    bool inLimits(Config config){
        for(size_t i=0; i<7; i++){
            if(config[i] < _lowerLimit[i] || config[i] > _upperLimit[i]){
                return false;
            }
        }
        return true;

    }
    //check for collision
    bool CheckCollision(Config config){

        _robot->SetActiveDOFValues(config);
        bool flag1 = _penv->CheckCollision(_robot);
        //bool flag2 = _penv->CheckSelfCollision(_robot);
        _robot->SetActiveDOFValues(_startConfig);
        return flag1 ;//|| flag2;
    }

    //execute trajectory Refference : OpenRave Examples
    void ExecuteTrajectory(){
    
        EnvironmentMutex& lock = _penv->GetMutex();
        lock.lock();
        TrajectoryBasePtr traj = RaveCreateTrajectory(_penv);
        traj->Init(_robot->GetActiveConfigurationSpecification());

        for(size_t i=0; i< _path.size(); i++){
            
            traj->Insert(0, _path[i]->getConfig());
        } 
            traj->Insert(0, _startConfig);
            // traj->Insert(0, _ggoalConfig);
            planningutils::RetimeActiveDOFTrajectory(traj, _robot);
            _robot->GetController()->SetPath(traj);
            lock.unlock();
    }


    //Draw a point at the end effector position
    void Draw(){

        std::vector<float> endEffector;
        float red[4] = {1,0,0,1};

        for(size_t i=0 ; i<_path.size() ; i++ ){
            _robot->SetActiveDOFValues(_path[i]->getConfig());
            _robot->SetActiveManipulator("leftarm");
            RobotBase::ManipulatorPtr hand;
            hand = _robot->GetActiveManipulator();
            RaveVector<dReal> point = hand->GetEndEffectorTransform().trans;
            std::vector<float> endEffector;
            endEffector.push_back(point.x);
            endEffector.push_back(point.y);
            endEffector.push_back(point.z);

            
            handler.push_back(_penv->plot3(&endEffector[0],1,1,5,red,0,true));
        }
    }

    
private:
    //openrave variables
    vector<RobotBasePtr> _robots;

    EnvironmentBasePtr _penv;
    RobotBasePtr _robot;

    Config _startConfig ;
    Config _goalConfig;
    Config _randomConfig; //get it from random generator
    Config _lowerLimit;
    Config _upperLimit;
    Config _stepSize; //get from config
    Config _extendConfig_new; //has new config from extend function
    Config _weight; //weight matrix used while calculating cost
    Config _ggoalConfig;    

    Node _startNode;
    Node _nearestNode;
    Node _extendNode_new;
    Node _goalNode;
    // Node _temp;

    NodeTree _mainTree;
    
    dReal _diffGoal;
    dReal _diffConfig;

    std::vector<GraphHandlePtr> handler;
    
    size_t count;

    vector< boost::shared_ptr<RRTNode> > _path; 
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

