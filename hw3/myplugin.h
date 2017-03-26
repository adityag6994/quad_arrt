//Aditya Gupta

#ifndef MYPLUGIN_H
#define MYPLUGIN_H

    #include <vector>
    #include <tuple>
    #include <algorithm>
    #include <boost/bind.hpp>
    #include <openrave/plugin.h>
    #include <openrave/openrave.h>

	using namespace std;
  using namespace OpenRAVE;

	typedef std::vector<OpenRAVE::dReal> Config;

  //Class Node	
	class RRTNode
    {
    	private:
    		Config _configration;
    		boost::shared_ptr<RRTNode> _parent;
        int uniqueId;
        static int CurrentID;

    	public:
          //Constructor
    			RRTNode(Config configration,boost::shared_ptr<RRTNode> parent){
                        _configration = configration;
                        _parent = parent;
                        CurrentID++;
                        uniqueId = 0;
                        // cout << "RRTNode Generated.." << endl;
                    }

                RRTNode(){
                	CurrentID++;
                  cout << "First RRTNode Generated.." << endl;
                  uniqueId = 0;
                }
                
                ~RRTNode();
    		  //set config
                void setConfig(Config config){_configration = config;}
          //set parent
                void setParent(boost::shared_ptr<RRTNode> parent){_parent = parent;}
          //set id
                void setUniqueId(int num){uniqueId = num;}

    			boost::shared_ptr<RRTNode> getParent() const {return _parent;}		
          //get config
    			Config getConfig() const {return _configration;}
          //get current id
                int getCurrentID() const {return CurrentID;}

                int getUniqueId(){return uniqueId;}
    };	  

    //Class Node Tree
    class NodeTree
    {

    	private:
    		vector< boost::shared_ptr<RRTNode> > _tree; 
            boost::shared_ptr<RRTNode> _rootNode;
    	
    	public:
    			NodeTree(){
            cout << "Node Tree Generated .." << endl;
          };
                
          NodeTree(boost::shared_ptr<RRTNode> root){
                    _rootNode = root;
          }
          ~NodeTree();
    //set root node		
          void setrootNode(boost::shared_ptr<RRTNode> newNode){
            _rootNode = newNode;
          }
    //add node
    			void nodeAdd(boost::shared_ptr<RRTNode> newNode){
                    _tree.push_back(newNode);
                }
    //get tree size
          int getTreeSize() const { return  _tree.size(); }  
		
          
    			boost::shared_ptr<RRTNode> getNode(int index) const{
    
                for(boost::shared_ptr<RRTNode> i : _tree){
                 	if(i->getCurrentID() == index){
                 		return i;
                 	}
                }
                throw 0;

          }
          
          void printFullTree(){
            for(size_t i=0 ; i<_tree.size() ; i++){
              printConfig("i ::: ",i,_tree[i]->getConfig());
            }
          }


          void printConfig(string s ,size_t a,Config config){
              cout << s   << a << "    ";
              
              for(size_t i = 0; i < 7 ; i++){
                cout << config[i] << ' ';
              }
              
              cout << endl;
          }


          boost::shared_ptr<RRTNode> getLast() {
            return _tree.back();
          }
          
            vector<boost::shared_ptr<RRTNode>> getfullPath(){
                    return _tree;
            }

            boost::shared_ptr<RRTNode> getlastNode(){return _tree.back();}

            //delete last node
            void deleteNode(){
                  //delete last node
                 _tree.pop_back();
            }

            void deleteNodeIndex(int index){
                    
                for(size_t i=0; i<_tree.size(); i++){

                  if(_tree[i]->getCurrentID() == index){
                    _tree.erase(_tree.begin()+i);
                  }

                }
            }

    };

 #endif
