// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "AG_ActionBase.generated.h"

/**
 * 
 */
UCLASS()
class ADVANCEDGAMEPLAY_API UAG_ActionBase : public UObject
{
	GENERATED_BODY()
	
public:
	// Called when this action becomes the current top action
	virtual void OnBegin(bool bFirstTime) {}

	// Called every frame while this is the current action
	virtual void OnUpdate() {}

	// Called when the action is popped / finished
	virtual void OnEnd() {}

	// Returns true when this action is finished and should be removed
	virtual bool IsDone() const { return true; }

	// Optional debug name
	virtual FString GetDebugName() const { return GetName(); }
};

