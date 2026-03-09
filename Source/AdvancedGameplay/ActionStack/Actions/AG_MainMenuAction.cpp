#include "AG_MainMenuAction.h"

#include "AG_PlayAction.h"
#include "AdvancedGameplay/AG_GameInstance.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/KismetSystemLibrary.h"

void UAG_MainMenuAction::OnBegin(bool bFirstTime)
{
	bQuitRequested = false;

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	APlayerController* PC = World->GetFirstPlayerController();
	if (!PC)
	{
		return;
	}

	PC->SetIgnoreMoveInput(true);
	PC->SetIgnoreLookInput(true);
}

void UAG_MainMenuAction::OnUpdate()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	APlayerController* PC = World->GetFirstPlayerController();
	if (!PC)
	{
		return;
	}

	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(
			1001,
			0.0f,
			FColor::Cyan,
			TEXT("MAIN MENU")
		);

		GEngine->AddOnScreenDebugMessage(
			1002,
			0.0f,
			FColor::Cyan,
			TEXT("Press P to Play")
		);

		GEngine->AddOnScreenDebugMessage(
			1003,
			0.0f,
			FColor::Cyan,
			TEXT("Press Q to Quit")
		);
	}

	if (PC->WasInputKeyJustPressed(EKeys::P))
	{
		StartGame();
		return;
	}

	if (PC->WasInputKeyJustPressed(EKeys::Q))
	{
		RequestQuit();
	}
}

void UAG_MainMenuAction::OnEnd()
{
	if (!bQuitRequested)
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	APlayerController* PC = World->GetFirstPlayerController();
	UKismetSystemLibrary::QuitGame(World, PC, EQuitPreference::Quit, false);
}

bool UAG_MainMenuAction::IsDone() const
{
	return bQuitRequested;
}

void UAG_MainMenuAction::StartGame()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	UAG_GameInstance* GI = Cast<UAG_GameInstance>(World->GetGameInstance());
	if (!GI)
	{
		return;
	}

	UAG_PlayAction* PlayAction = NewObject<UAG_PlayAction>(GI);
	GI->GetActionStack().PushAction(PlayAction);
}

void UAG_MainMenuAction::RequestQuit()
{
	bQuitRequested = true;
}