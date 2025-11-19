#pragma once

#include "CoreMinimal.h"
#include "Actions/AG_ActionBase.h"
#include "FActionStack.generated.h"

/**
 * Only the action at the top of the stack is active.
 * Actions receive Begin/Update/End callbacks and can push
 * or pop other actions during updates.
 */
USTRUCT(BlueprintType)
struct ADVANCEDGAMEPLAY_API FActionStack
{
	GENERATED_BODY()

public:
	// Add an action to the top of the stack
	void PushAction(UAG_ActionBase* Action);

	// Drive the stack once per frame (call from GameMode::Tick)
	void UpdateActions();

	bool IsEmpty() const { return CurrentAction == nullptr && Stack.Num() == 0; }

	const TArray<TObjectPtr<UAG_ActionBase>>& GetStack() const { return Stack; }
	UAG_ActionBase* GetCurrentAction() const { return CurrentAction; }

	// Top of stack is at the back
	UPROPERTY()
	TArray<TObjectPtr<UAG_ActionBase>> Stack;

	// Current active action (same as Stack.Last() when not null)
	UPROPERTY()
	TObjectPtr<UAG_ActionBase> CurrentAction = nullptr;

	// Tracks if we've ever begun a given action
	UPROPERTY()
	TSet<TObjectPtr<UAG_ActionBase>> FirstTimeActions;

	// Reentrancy guard
	bool bIsUpdating = false;
};
