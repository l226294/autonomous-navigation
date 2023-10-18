#ifndef _CONTEXT_HPP_
#define _CONTEXT_HPP_
#include <string>
#include <unordered_map>

class State;
class DecisionMakerNode;

struct NodeState
{
  NodeState& operator = (const NodeState& n)
  {
    state_ = n.state_;
    father_name_ = n.father_name_;
    return *this;
  }
  State* state_;
  std::string father_name_;
};

class Context
{
public:
  friend class State;

  Context();

  ~Context();

  bool Start(std::string name);

  State* CreateState(State* state, std::string name, std::string father_name = "");
 
  void Update();

  std::string GetCurStateName();

  void SetDecisionMaker(DecisionMakerNode *decision_maker);

private:
  void TransForState(std::string name);

  std::unordered_map<std::string, NodeState> states_;
  NodeState cur_node_state_;
  std::string cur_name_;
  std::string root_name_;
};
#endif // _CONTEXT_HPP_