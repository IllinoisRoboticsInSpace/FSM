#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

ros::NodeHandle nh;
enum state_t {wait_start, find_location, move_to_mine, mine, move_to_bin, turn_to_bin, dump, manual};



class State
{
  protected:
    state_t this_state;
  public:
    State(state_t new_state) : this_state(new_state) { };
    virtual State * change_state() = 0;
    virtual void perform_task() = 0;
    bool checkFailed();
}

class StartState : public State
{
  public:
    StartState() : State(wait_start) { };
    State * change_state()
    {
      if (checkFailed())
        reinterpret_cast<ManualState *>(this)->ManualState::ManualState();
      else
        reinterpret_cast<MoveGoalState *>(this)->MoveGoalState::MoveGoalState(move_to_mine);
      return this;
    };
    void perform_task();
}

class LocalizeState : public State
{
  private:
    state_t exit_state;
  public:
    LocalizeState(state_t choice) : State(localize), exit_state(choice) { };
    State * change_state()
    {
      if (checkFailed())
        reinterpret_cast<ManualState *>(this)->ManualState::ManualState();
      else
        reinterpret_cast<MoveGoalState *>(this)->MoveGoalState::MoveGoalState(exit_state);
      return this;
    };
    void perform_task();
};

class MoveGoalState : public State
{
  private:
    geometry_msgs::Pose goal;
  public:
    MoveGoalState(state_t target) : State(target)
    {
      if (target == move_mine) //???
        goal = MINE_LOC;
      else if (target == move_bin) //???
        goal = BIN_LOC;
      else
        goal = geometry_msgs::Pose();
    };
    State * change_state()
    {
      if (checkFailed())
        reinterpret_cast<ManualState *>(this)->ManualState::ManualState();
      else if(this_state == move_mine)
        reinterpret_cast<MiningState *>(this)->MiningState::MiningState();
      else if(this_state == move_bin)
        reinterpret_cast<DumpingState *>(this)->DumpingState::DumpingState();
      return this;
    };
    State * re_localize()
    {
      reinterpret_cast<LocalizeState *>(this)->LocalizeState::LocalizeState(this_state);
      return this;
    };
    void perform_task();
}

class MiningState : public State
{
  public:
    MiningState() : State(mining) { };
    State * change_state()
    {
      if (checkFailed())
        reinterpret_cast<ManualState *>(this)->ManualState::ManualState();
      else
        reinterpret_cast<MoveGoalState *>(this)->MoveGoalState::MoveGoalState(move_bin);
      return this;
    };
    void perform_task();
}

class DumpingState : public State
{
  public:
    DumpingState() : State(dumping) { };
    State * change_state()
    {
      if (checkFailed())
        reinterpret_cast<ManualState *>(this)->ManualState::ManualState();
      else
        reinterpret_cast<MoveGoalState *>(this)->MoveGoalState::MoveGoalState(move_mine);
      return this;
    };
    void perform_task
}

class ManualState : public State
{
  public:
    ManualState() : State(manual);
    State * change_state()
    {
      return this;
    };
    void perform_task();
}

void StartState::perform_task()
{
}

void LocalizeState::perform_task()
{
}

void MoveGoalState::perform_task()
{
}

void MiningState::perform_task()
{
}

void DumpingState::perform_task()
{
}

void ManualState::perform_task()
{
}
