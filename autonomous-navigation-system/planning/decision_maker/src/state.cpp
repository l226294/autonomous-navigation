#include "decision_maker/context.hpp"
#include "decision_maker/state.hpp"

void State::SetContext(Context* context)
{
  context_ = context;
}
void State::SetDecisionMaker(DecisionMakerNode *decision_maker)
{
  decision_maker_ = decision_maker;
}
void State::TransState(std::string name)
{
  context_->TransForState(name);
}
