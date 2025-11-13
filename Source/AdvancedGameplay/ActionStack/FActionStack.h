// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

/**
 * A lightweight stack-based state controller.
 * 
 * Only the action at the top of the stack is active.
 * Actions receive Begin/Update/End callbacks and can push
 * or pop other actions during updates.
 */

class IAction
{
public:
	virtual ~IAction() = default;

	// Called when this action becomes the current top action
	virtual void OnBegin(bool bFirstTime) = 0;

	// Called every frame while this is the current action
	virtual void OnUpdate() = 0;

	// Called when the action is popped / finished
	virtual void OnEnd() = 0;

	// Returns true when this action is finished and should be removed
	virtual bool IsDone() const = 0;
};

// Optional base class, like Action in C#
class FAction : public IAction
{
public:
	virtual void OnBegin(bool bFirstTime) override {}
	virtual void OnUpdate() override {}
	virtual void OnEnd() override {}
	virtual bool IsDone() const override { return true; }

	virtual FString ToString() const { return TEXT("FAction"); }
};


class ADVANCEDGAMEPLAY_API FActionStack
{
public:

	virtual ~FActionStack();
	// Add an action to the top of the stack
	void PushAction(IAction* Action);

	// Drive the stack once per frame (call from GameMode::Tick)
	void UpdateActions();

	bool IsEmpty() const { return CurrentAction == nullptr && Stack.Num() == 0; }

	const TArray<IAction*>& GetStack() const { return Stack; }
	IAction* GetCurrentAction() const { return CurrentAction; }

private:
	// Treated as a Stack from the outside
	TArray<IAction*> Stack;

	// Tracks if we've ever begun a given action (for bFirstTime flag)
	TSet<IAction*> FirstTimeActions;

	// Current active action
	IAction* CurrentAction = nullptr;
};