// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AdvancedGameplay/ActionStack/FActionStack.h"
#include "UObject/Object.h"
#include "TestAction.generated.h"

/**
 * Simple test action used for validating the ActionStack.
 * Prints text on begin/update and completes after a set duration.
 */
UCLASS(Blueprintable)
class ADVANCEDGAMEPLAY_API UTestAction : public UObject, public IAction
{
	GENERATED_BODY()

public:
	// Text printed when this action begins
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="TestAction")
	FString BeginText = TEXT("TestAction Begin");

	// Text printed each update
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="TestAction")
	FString UpdateText = TEXT("TestAction Update");

	// How long before this action finishes
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="TestAction")
	float Duration = 2.0f;

public:
	// IAction interface
	virtual void OnBegin(bool bFirstTime) override;
	virtual void OnUpdate() override;
	virtual void OnEnd() override;
	virtual bool IsDone() const override;

private:
	float Elapsed = 0.0f;
	bool bDone = false;
};