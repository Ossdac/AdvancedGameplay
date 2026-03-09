#include "AG_PlayAction.h"

#include "AG_ResultAction.h"
#include "AdvancedGameplay/AG_GameInstance.h"
#include "AdvancedGameplay/Spider/SpiderCharacter.h"

#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"

void UAG_PlayAction::OnBegin(bool bFirstTime)
{
	bDone = false;
	bResultShown = false;

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	APlayerController* PC = World->GetFirstPlayerController();
	if (PC)
	{
		PC->SetIgnoreMoveInput(false);
		PC->SetIgnoreLookInput(false);
	}

	FindActors();
}

void UAG_PlayAction::OnUpdate()
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
			2001,
			0.0f,
			FColor::Green,
			TEXT("GAME RUNNING")
		);

		GEngine->AddOnScreenDebugMessage(
			2002,
			0.0f,
			FColor::Green,
			TEXT("Push the ball off the platform")
		);
	}

	if (!BallActor.IsValid() || !SpiderActor.IsValid())
	{
		FindActors();
		return;
	}

	if (!bResultShown && BallActor->GetActorLocation().Y <= BallWinY)
	{
		ShowResult(TEXT("CONGRATULATIONS"));
		return;
	}

	if (!bResultShown && SpiderActor->GetActorLocation().Y <= SpiderLoseY)
	{
		ShowResult(TEXT("YOU LOST"));
		return;
	}

	// Optional quick escape back to menu.
	if (PC->WasInputKeyJustPressed(EKeys::Escape))
	{
		ShowResult(TEXT("YOU LOST"));
	}
}

void UAG_PlayAction::OnEnd()
{
}

bool UAG_PlayAction::IsDone() const
{
	return bDone;
}

void UAG_PlayAction::FindActors()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	SpiderActor = Cast<ASpiderCharacter>(
		UGameplayStatics::GetActorOfClass(World, ASpiderCharacter::StaticClass())
	);

	TArray<AActor*> BallActors;
	UGameplayStatics::GetAllActorsWithTag(World, FName(TEXT("Ball")), BallActors);

	if (BallActors.Num() > 0)
	{
		BallActor = BallActors[0];
	}
}

void UAG_PlayAction::ShowResult(const FString& ResultText)
{
	if (bResultShown)
	{
		return;
	}

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

	UAG_ResultAction* ResultAction = NewObject<UAG_ResultAction>(GI);
	ResultAction->ResultText = ResultText;

	bResultShown = true;
	GI->GetActionStack().PushAction(ResultAction);
}