// FActionStack.cpp

#include "FActionStack.h"

void FActionStack::PushAction(UAG_ActionBase* Action)
{
	if (!Action)
	{
		return;
	}

	// Remove any existing instance first
	Stack.Remove(Action);

	// Add to the back = top of stack
	Stack.Add(Action);

	// Force a re-begin next Update
	if (CurrentAction && CurrentAction != Action)
	{
		CurrentAction = nullptr;
	}
}

void FActionStack::UpdateActions()
{
	if (bIsUpdating)
	{
		return;
	}

	bIsUpdating = true;

	if (Stack.Num() == 0)
	{
		CurrentAction = nullptr;
		bIsUpdating = false;
		return;
	}

	// If we don't have a valid current action, pick the top
	if (!CurrentAction)
	{
		CurrentAction = Stack.Last();

		if (!IsValid(CurrentAction))
		{
			Stack.Pop();
			CurrentAction = nullptr;
			bIsUpdating = false;
			return;
		}

		const bool bFirstTime = !FirstTimeActions.Contains(CurrentAction);
		FirstTimeActions.Add(CurrentAction);

		CurrentAction->OnBegin(bFirstTime);
	}

	if (!IsValid(CurrentAction))
	{
		CurrentAction = nullptr;
		bIsUpdating = false;
		return;
	}

	// OnUpdate may push/pop/change stack
	UAG_ActionBase* ActionBeforeUpdate = CurrentAction;
	CurrentAction->OnUpdate();

	// If stack emptied or current is no longer top, reset and return
	if (Stack.Num() == 0 || Stack.Last() != ActionBeforeUpdate)
	{
		CurrentAction = nullptr;
		bIsUpdating = false;
		return;
	}

	// If still top, check IsDone
	if (CurrentAction->IsDone())
	{
		UAG_ActionBase* Finished = CurrentAction;
		Stack.Pop(); // pop back
		Finished->OnEnd();
		FirstTimeActions.Remove(Finished);
		CurrentAction = nullptr;
	}

	bIsUpdating = false;
}
