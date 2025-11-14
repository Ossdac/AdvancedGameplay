// Fill out your copyright notice in the Description page of Project Settings.


#include "AG_GameInstance.h"
#include "ActionStack/Actions/AG_MainMenuAction.h"


void UAG_GameInstance::Init()
{
	Super::Init();

	// Create and push the root main menu action once for the entire app lifetime
	UAG_MainMenuAction* MainMenu = NewObject<UAG_MainMenuAction>(this);
	ActionStack.PushAction(MainMenu);
}
