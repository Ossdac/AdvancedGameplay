#include "FActionStack.h"

void FActionStack::PushAction(UAG_ActionBase* Action)
{
	if (!Action)
	{
		return;
	}

	Stack.Remove(Action);
	Stack.Add(Action);

	if (CurrentAction && CurrentAction != Action)
	{
		CurrentAction = nullptr;
	}
}

void FActionStack::PopToRoot()
{
	if (Stack.Num() <= 1)
	{
		CurrentAction = nullptr;
		return;
	}

	while (Stack.Num() > 1)
	{
		UAG_ActionBase* Removed = Stack.Pop();
		FirstTimeActions.Remove(Removed);
	}

	CurrentAction = nullptr;
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

	UAG_ActionBase* ActionBeforeUpdate = CurrentAction;
	CurrentAction->OnUpdate();

	if (Stack.Num() == 0 || Stack.Last() != ActionBeforeUpdate)
	{
		CurrentAction = nullptr;
		bIsUpdating = false;
		return;
	}

	if (CurrentAction->IsDone())
	{
		UAG_ActionBase* Finished = CurrentAction;
		Stack.Pop();
		Finished->OnEnd();
		FirstTimeActions.Remove(Finished);
		CurrentAction = nullptr;
	}

	bIsUpdating = false;
}