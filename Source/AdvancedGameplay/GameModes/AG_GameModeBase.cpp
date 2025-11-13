// Fill out your copyright notice in the Description page of Project Settings.


#include "AG_GameModeBase.h"

AAG_GameModeBase::AAG_GameModeBase()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
}

void AAG_GameModeBase::BeginPlay()
{
	Super::BeginPlay();
	CachedGI = Cast<UAG_GameInstance>(GetGameInstance());
}

void AAG_GameModeBase::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (CachedGI)
	{
		CachedGI->GetActionStack().UpdateActions();
	}
}