#ifndef _STATE_HPP_
#define _STATE_HPP_
#include <string>

class Context;
class DecisionMakerNode;
class State{
public:   
  
  virtual void onEnter() = 0;
  
  virtual void onUpdate() = 0;

  virtual void onExit() = 0;

  void SetContext(Context* context);

  void SetDecisionMaker(DecisionMakerNode *decision_maker);

  void TransState(std::string name);

  DecisionMakerNode *decision_maker_;
  
private:
  
  Context *context_;
  
};

#endif // _STATE_HPP_