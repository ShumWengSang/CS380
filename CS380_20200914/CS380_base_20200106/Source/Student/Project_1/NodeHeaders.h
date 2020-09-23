#pragma once

// Include all node headers in this file

// Example Control Flow Nodes
#include "ControlFlow/C_ParallelSequencer.h"
#include "ControlFlow/C_RandomSelector.h"
#include "ControlFlow/C_Selector.h"
#include "ControlFlow/C_Sequencer.h"

// Student Control Flow Nodes
#include "ControlFlow/C_Sequencer_Fail.h"
#include "ControlFlow/C_RandomSelector_OneFail.h"

// Example Decorator Nodes
#include "Decorator/D_Delay.h"
#include "Decorator/D_InvertedRepeater.h"
#include "Decorator/D_RepeatFourTimes.h"

// Student Decorator Nodes
#include "Decorator/D_IsInsideBox.h"
#include "Decorator/D_HasJob.h"
#include "Decorator/D_isNotCrazy.h"

// Example Leaf Nodes
#include "Leaf/L_CheckMouseClick.h"
#include "Leaf/L_Idle.h"
#include "Leaf/L_MoveToFurthestAgent.h"
#include "Leaf/L_MoveToMouseClick.h"
#include "Leaf/L_MoveToRandomPosition.h"

// Student Leaf Nodes
#include "Leaf/L_MoveBehindTarget.h"
#include "Leaf/L_MoveAwayFromBox.h"
#include "Leaf/L_FindClosestHen.h"
#include "Leaf/L_WolfInRange.h"
#include "Leaf/L_GoToJob.h"
#include "Leaf/L_WorkingJob.h"
#include "Leaf/L_AssignJob.h"
#include "Leaf/L_AssignJobTwo.h"
#include "Leaf/L_AssignJobThree.h"
#include "Leaf/L_AssignJobFour.h"
#include "Leaf/L_AssignJobFive.h"
#include "Leaf/L_MoveToOffice.h"
#include "Leaf/L_RandomMovements.h"
#include "Leaf/L_BeCrazy.h"