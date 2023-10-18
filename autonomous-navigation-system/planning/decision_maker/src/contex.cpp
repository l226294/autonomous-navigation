#include "decision_maker/context.hpp"
#include "decision_maker/state.hpp"

Context::Context()
{

}
 
Context::~Context()
{
  for (auto iter : states_)
  {
    if (iter.second.state_)
    {
      delete iter.second.state_;
      iter.second.state_ = nullptr;
    }
  }
  states_.clear();
}

bool Context::Start(std::string name)
{
  std::unordered_map<std::string, NodeState>::iterator iter_map = states_.find(name);
  if (iter_map != states_.end())
  {
    cur_node_state_ = iter_map->second;
    cur_name_ = iter_map->first;
    iter_map->second.state_->onEnter();
  }
  return false;
}

State* Context::CreateState(State* state, std::string name, std::string father_name)
{
  NodeState node_state;
  node_state.state_ = state;
  node_state.state_->SetContext(this);
  node_state.father_name_ = father_name;
  states_[name] = node_state;
  return state;
}

void Context::Update()
{
  cur_node_state_.state_->onUpdate();
}

std::string Context::GetCurStateName()
{
  return cur_name_;
}

void Context::TransForState(std::string name)
{
  std::string str_name = std::string(name);
  std::unordered_map<std::string, NodeState>::iterator iter_map = states_.find(str_name);
  if (iter_map != states_.end())
  {
    cur_node_state_.state_->onExit();
    cur_node_state_ = iter_map->second;
    cur_name_ = iter_map->first;
    cur_node_state_.state_->onEnter();
  }
}

void Context::SetDecisionMaker(DecisionMakerNode *decision_maker)
{
  for (auto iter : states_)
  {
    if (iter.second.state_)
    {
      iter.second.state_->SetDecisionMaker(decision_maker);
    }
  }
}