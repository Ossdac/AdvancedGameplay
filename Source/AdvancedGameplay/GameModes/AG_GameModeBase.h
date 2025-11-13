// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "AG_GameModeBase.generated.h"

/**
 * 
 */
UCLASS()
class ADVANCEDGAMEPLAY_API AAG_GameModeBase : public AGameModeBase
{
	GENERATED_BODY()
	virtual void Tick(float DeltaSeconds) override;

protected:
	virtual void BeginPlay() override;

private:
	UAG_GameInstance* CachedGI = nullptr;
};
