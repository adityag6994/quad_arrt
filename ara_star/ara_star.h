//Aditya Gupta

#ifndef ARA_STAR_H
#define ARA_STAR_H

    #include <vector>
    #include <tuple>
    #include <algorithm>
    #include <boost/bind.hpp>
    #include <openrave/plugin.h>
    #include <openrave/openrave.h>

  using namespace std;
  using namespace OpenRAVE;

  // typedef std::vector<OpenRAVE::dReal> Config;
  typedef vector<OpenRAVE::dReal> Point;
  typedef boost::shared_ptr<Node> NodePtr;
  typedef vector< NodePtr >  NodeTree;


  // class ARAPoint:
  //   {
  //   	private:
  //             Point _p;
  //             // Point _y;

  //   	public:
  //             ARAPoint(Point a){
  //               _p = a;
  //               // _y = b;
  //             }

  //             ~ARAPoint();

  //             Point getPoint(){
  //               return _p;
  //             }

  //             setPoint(Point a){
  //               _p = a;
  //             }
        	
  //   };	  

    //Class Node Tree
    class Node
    {

    	private:
    		    Point        _point;
            NodePtr      _parent;
            dReal        _H;
            dReal        _G;
    	
    	public:
    	      Node(Point a, NodePtr p, dReal h, dReal g){
              _point  = a;
              _parent = p;
              _H      = h;
              _G      = g;
            }

            Node(){
              cout << "Node Generated.." << endl;
            }

            ~Node();

            void setPoint(Point ){
              _point = a;
            };

            dReal getH(){return _H;}	

            dReal getG(){return _G;}

            dReal getCost(){return (_H + _G);}

            NodePtr getParent(){return _parent;}

            Point getPoint(){
              return _point;
            }

    };

    class NodeList{
        private:
          NodeTree _tree;
          NodePtr _nearest;

        public:
          NodeList(){}

          ~NodeList();

          addNode(NodePtr a){
            _tree.push_back(a);
          }

          void sortTree(){
              sort(_tree.begin(), _tree.end(), less_than_key());
          }

          struct less_than_key{
              inline bool operator() (const NodePtr& node1, const NodePtr& node2){
                     return (node1->getCost() < node2->getCost());
              }
          };

          NodePtr getNearest(){
            return _nearest;
          }

          void setNearest(){
            _nearest = _tree.back();
          }

    };

 #endif

