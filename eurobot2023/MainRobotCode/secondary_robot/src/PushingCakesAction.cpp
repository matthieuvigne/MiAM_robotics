#include <secondary_robot/PushingCakesAction.h>

bool PushingCakesAction::performAction(AbstractStrategy* strategy)
{
    bool pushingMoveSucceeded = 
        strategy->go_to_straight_line(end_position) &&
        strategy->go_forward(-200);

    // perform action
    // action was successful
    return(pushingMoveSucceeded);
}