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
  typedef std::vector<OpenRAVE::dReal> Point;
  typedef boost::shared_ptr<Node> NodePtr;


  class ARAPoint:
    {
    	private:
              Point _p;
              // Point _y;

    	public:
              ARAPoint(Point a){
                _p = a;
                // _y = b;
              }

              ~ARAPoint();

              Point getPoint(){
                return _p;
              }

              setPoint(Point a){
                _p = a;
              }
        	
    };	  

    //Class Node Tree
    class Node
    {

    	private:
    		    ARAPoint     _point;
            NodePtr      _parent;
            dReal        _H;
            dReal        _G;
    	
    	public:
    	      Node(Point a, NodePtr p, dReal h, dReal g){
              _point.setPoint(a);
              _parent = p;
              _H = h;
              _G = g;
            }

            ~Node();

            dReal getH(){return _H;}	

            dReal getG(){return _G;}

            NodePtr getParent(){return _parent;}

            get

    };

 #endif

