#ifndef _FACTORY_HPP_
#define _FACTORY_HPP_
#include "decision_maker/state.hpp"
#include "decision_maker/context.hpp"

class Ready : public State
{
public :
	void onEnter()
	{
		std::cout << "Enter Ready" << std::endl;
	}

  void onUpdate()
	{
    TransState("LaneKeep");
	}

	void onExit()
	{
		std::cout << "Exit Ready" << std::endl;
	}
  
};

class LaneKeep : public State
{
public:

  void onEnter(){
    std::cout << "Enter LaneKeep" << std::endl;

  }
  
  void onUpdate(){
    decision_maker_->ObstacleToFrenet();
    std::string current_scene = decision_maker_->determineCurrentScene();
    if(current_scene == "Complete"){
      TransState("Complete");
    }else if(current_scene == "Error"){
      TransState("Error");
    }else if(current_scene == "OverTake"){
      TransState("OverTake");
    }else if(current_scene == "CarFollow"){
      TransState("CarFollow");
    }
  }

  void onExit(){
    std::cout << "Exit LaneKeep" << std::endl;
 
  }

};


class OverTake : public State
{
public:
  void onEnter(){
    std::cout << "Enter OverTake" << std::endl;
    prepare = false;
  }
  
  void onUpdate(){
    if(decision_maker_->isApproachingGoal()){
      TransState("Complete");
    }
    double lateral_offset = decision_maker_->getCurrentLateralOffset();

    if(!prepare){
      if(std::abs(lateral_offset) < 1.0){
        decision_maker_->ObstacleToFrenet();
        std::string current_scene = decision_maker_->determineCurrentScene();
        if(current_scene == "Complete"){
          TransState("Complete");
        }else if(current_scene == "Error"){
          TransState("Error");
        }else if(current_scene == "OverTake"){
          TransState("OverTake");
        }else if(current_scene == "CarFollow"){
          TransState("CarFollow");
        }
      }else{
        prepare = true;
        std::cout << "prepare" << std::endl;
      }
    }

    if(prepare && std::abs(lateral_offset) < 0.4){
      TransState("LaneKeep");
    }
  }

  void onExit(){
    std::cout << "Exit OverTake" << std::endl;
  }

private:
  bool prepare = false;

};

class CarFollow : public State
{
public:

  void onEnter(){
    std::cout << "Enter CarFollow" << std::endl;
  }
  
  void onUpdate(){
    decision_maker_->ObstacleToFrenet();
    std::string current_scene = decision_maker_->determineCurrentScene();
    if(current_scene == "Complete"){
      TransState("Complete");
    }else if(current_scene == "Error"){
      TransState("Error");
    }else if(current_scene == "OverTake"){
      TransState("OverTake");
    }else if(current_scene == "LaneKeep"){
      TransState("LaneKeep");
    }
  }

  void onExit(){
    std::cout << "Exit CarFollow!!!" << std::endl;
  }

};

class Complete : public State
{
public:

  void onEnter(){
    std::cout << "Enter Complete" << std::endl;
  }
  
  void onUpdate(){

  }
  
  void onExit(){
    std::cout << "Exit Complete" << std::endl;
  }
};

class Error : public State
{
public:
  void onEnter(){
    std::cout << "Enter Error" << std::endl;
  }

  void onUpdate(){
    TransState("Ready");
  }

  void onExit(){
    std::cout << "Exit Error" << std::endl;
  }
};

class Factory
{
public :
	static State* CreateState(Context* context, std::string name, std::string parent_name = "")
	{
		State* state = nullptr;
		if (name == "Ready")
		{
			state = new Ready();
		}
		else if (name == "LaneKeep")
		{
			state = new LaneKeep();
		}
		else if (name == "OverTake")
		{
			state = new OverTake();
		}else if (name == "CarFollow")
		{
			state = new CarFollow();
		}
		else if (name == "Complete")
		{
			state = new Complete();
		}
		else if (name == "Error")
		{
			state = new Error();
		} 
		context->CreateState(state, name, parent_name);
		return state;
	}
};

#endif // _FACTORY_HPP_