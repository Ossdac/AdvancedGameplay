#include "AG_MainMenuAction.h"

#include "AG_PlayAction.h"
#include "AdvancedGameplay/AG_GameInstance.h"
#include "Engine/Engine.h"
#include "Kismet/KismetSystemLibrary.h"

void UAG_MainMenuAction::OnBegin(bool bFirstTime)
{
	bDone = false;

	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(
			-1, 5.0f, FColor::Cyan,
			TitleText + ": OnBegin"
		);
	}
}

void UAG_MainMenuAction::OnUpdate()
{
	UWorld* World = GetWorld();
	if (!World) { return; }

	APlayerController* PC = World->GetFirstPlayerController();
	if (!PC) { return; }

	// if (GEngine)
	// {
	// 	GEngine->AddOnScreenDebugMessage(
	// 		-1, .01f, FColor::Cyan,
	// 		TitleText
	// 	);
	// }

	if (PC->WasInputKeyJustPressed(EKeys::Enter))
	{
		StartGame();
	}

	if (PC->WasInputKeyJustPressed(EKeys::Escape))
	{
		RequestExit();
	}
}

void UAG_MainMenuAction::OnEnd()
{
	if (bDone)
	{
		UWorld* World = GetWorld();
		if (World)
		{
			APlayerController* PC = World->GetFirstPlayerController();
			UKismetSystemLibrary::QuitGame(World, PC, EQuitPreference::Quit, false);
		}
	}
}

bool UAG_MainMenuAction::IsDone() const
{
	// Only true when the player has explicitly requested exit.
	return bDone;
}

void UAG_MainMenuAction::StartGame()
{
	UWorld* World = GetWorld();
	if (!World) { return; }

	UAG_GameInstance* GI = Cast<UAG_GameInstance>(World->GetGameInstance());
	if (!GI) { return; }

	UAG_PlayAction* PlayAction = NewObject<UAG_PlayAction>(GI);
	GI->GetActionStack().PushAction(PlayAction);
}

void UAG_MainMenuAction::RequestExit()
{
	bDone = true;
}
