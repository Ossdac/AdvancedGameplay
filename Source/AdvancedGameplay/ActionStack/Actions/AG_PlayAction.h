// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AdvancedGameplay/ActionStack/FActionStack.h"
#include "UObject/Object.h"
#include "AG_PlayAction.generated.h"

/**
 * 
 */
UCLASS()
class ADVANCEDGAMEPLAY_API UAG_PlayAction : public UObject, public IAction
{
	GENERATED_BODY()

public:
	
	// IAction interface
	virtual void OnBegin(bool bFirstTime) override;
	virtual void OnUpdate() override;
	virtual void OnEnd() override;
	virtual bool IsDone() const override;

private:
	bool bDone = false;
};
