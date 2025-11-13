// Fill out your copyright notice in the Description page of Project Settings.


#include "FActionStack.h"


FActionStack::~FActionStack()
{
	for (IAction* Action : Stack)
	{
		if (Action)
		{
			Action->OnEnd();
		}
	}
}

void FActionStack::PushAction(IAction* Action)
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
	if (Stack.Num() == 0)
	{
		CurrentAction = nullptr;
		return;
	}

	// If we don't have a valid current action, pick the top
	if (!CurrentAction)
	{
		CurrentAction = Stack.Last();

		const bool bFirstTime = !FirstTimeActions.Contains(CurrentAction);
		FirstTimeActions.Add(CurrentAction);

		CurrentAction->OnBegin(bFirstTime);
	}

	// OnUpdate may push/pop/change stack
	IAction* ActionBeforeUpdate = CurrentAction;
	CurrentAction->OnUpdate();

	// If stack emptied or current is no longer top, reset and return
	if (Stack.Num() == 0 || Stack.Last() != ActionBeforeUpdate)
	{
		CurrentAction = nullptr;
		return;
	}

	// If still top, check IsDone
	if (CurrentAction->IsDone())
	{
		IAction* Finished = CurrentAction;
		Stack.Pop(); // pop back
		Finished->OnEnd();
		FirstTimeActions.Remove(Finished);
		CurrentAction = nullptr;
	}
}
