#include "AG_ResultAction.h"

#include "AdvancedGameplay/AG_GameInstance.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Engine.h"

void UAG_ResultAction::OnBegin(bool bFirstTime)
{
	bDone = false;

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

void UAG_ResultAction::OnUpdate()
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
			3001,
			0.0f,
			FColor::Yellow,
			ResultText
		);

		GEngine->AddOnScreenDebugMessage(
			3002,
			0.0f,
			FColor::Yellow,
			TEXT("Press Enter to return to menu")
		);
	}

	if (PC->WasInputKeyJustPressed(EKeys::Enter))
	{
		bDone = true;
	}
}

void UAG_ResultAction::OnEnd()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	UAG_GameInstance* GI = Cast<UAG_GameInstance>(World->GetGameInstance());
	if (GI)
	{
		GI->GetActionStack().PopToRoot();
	}

	const FString LevelName = UGameplayStatics::GetCurrentLevelName(World, true);
	UGameplayStatics::OpenLevel(World, FName(*LevelName));
}

bool UAG_ResultAction::IsDone() const
{
	return bDone;
}