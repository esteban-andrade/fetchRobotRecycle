function isGoalReached = exampleHelperIsStateInWorkspaceGoalRegion(planner, ~, newState)
%EXAMPLEHELPERISSTATEINWORKSPACEGOALREGION
dist = planner.StateSpace.distanceToWorkspaceGoalRegion(newState);
isGoalReached = dist < 0.02;
end

