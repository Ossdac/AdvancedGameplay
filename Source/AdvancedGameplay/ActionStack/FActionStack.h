#pragma once

#include "CoreMinimal.h"
#include "Actions/AG_ActionBase.h"
#include "FActionStack.generated.h"

USTRUCT(BlueprintType)
struct ADVANCEDGAMEPLAY_API FActionStack
{
	GENERATED_BODY()

public:
	void PushAction(UAG_ActionBase* Action);
	void UpdateActions();

	// Minimal helper for this assignment:
	// keep the root action (main menu), remove everything above it.
	void PopToRoot();

	bool IsEmpty() const { return CurrentAction == nullptr && Stack.Num() == 0; }

	const TArray<TObjectPtr<UAG_ActionBase>>& GetStack() const { return Stack; }
	UAG_ActionBase* GetCurrentAction() const { return CurrentAction; }

	UPROPERTY()
	TArray<TObjectPtr<UAG_ActionBase>> Stack;

	UPROPERTY()
	TObjectPtr<UAG_ActionBase> CurrentAction = nullptr;

	UPROPERTY()
	TSet<TObjectPtr<UAG_ActionBase>> FirstTimeActions;

	bool bIsUpdating = false;
};