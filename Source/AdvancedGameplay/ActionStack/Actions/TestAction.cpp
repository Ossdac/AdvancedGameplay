#include "TestAction.h"
#include "Engine/Engine.h"

void UTestAction::OnBegin(bool bFirstTime)
{
	Elapsed = 0.0f;
	bDone = false;

	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(
			-1, 2.0f, FColor::Green,
			FString::Printf(TEXT("[Action] %s (FirstTime=%s)"),
				*BeginText,
				bFirstTime ? TEXT("true") : TEXT("false"))
		);
	}
}

void UTestAction::OnUpdate()
{
	if (bDone)
	{
		return;
	}

	Elapsed += GetWorld()->GetDeltaSeconds();

	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(
			-1, 0.1f, FColor::Yellow,
			FString::Printf(TEXT("[Action] %s (%.2f / %.2f)"),
				*UpdateText,
				Elapsed,
				Duration)
		);
	}

	if (Elapsed >= Duration)
	{
		bDone = true;
	}
}

void UTestAction::OnEnd()
{
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(
			-1, 2.0f, FColor::Red,
			TEXT("[Action] End")
		);
	}
}

bool UTestAction::IsDone() const
{
	return bDone;
}
