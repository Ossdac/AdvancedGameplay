#include "AG_SpiderAnimInstance.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/Character.h"

void UAG_SpiderAnimInstance::NativeInitializeAnimation()
{
	Super::NativeInitializeAnimation();

	OwningPawn = TryGetPawnOwner();

	LegSwingDegrees.SetNum(8);
	for (int32 Index = 0; Index < LegSwingDegrees.Num(); ++Index)
	{
		LegSwingDegrees[Index] = 0.0f;
	}
}

void UAG_SpiderAnimInstance::NativeUpdateAnimation(float DeltaSeconds)
{
	Super::NativeUpdateAnimation(DeltaSeconds);

	if (OwningPawn == nullptr)
	{
		OwningPawn = TryGetPawnOwner();
	}

	if (OwningPawn == nullptr)
	{
		return;
	}

	const FVector Vel = OwningPawn->GetVelocity();
	Speed = FVector(Vel.X, Vel.Y, 0.0f).Size();
	bIsMoving = Speed > 5.0f;

	// Normalized gait intensity 0..1
	const float Alpha = FMath::Clamp(Speed / SpeedForFullGait, 0.0f, 1.0f);

	if (!bIsMoving || Alpha <= 0.01f)
	{
		for (int32 Index = 0; Index < LegSwingDegrees.Num(); ++Index)
		{
			LegSwingDegrees[Index] = 0.0f;
		}
		return;
	}

	GaitTime += DeltaSeconds;

	// Simple alternating pattern (tetrapod-ish): legs 0,2,5,7 vs 1,3,4,6
	// You can swap indices if your numbering reads differently in motion.
	const float Omega = 2.0f * PI * StepFrequencyHz;
	const float Base = FMath::Sin(GaitTime * Omega) * MaxSwingDeg * Alpha;

	// Group A
	LegSwingDegrees[0] = Base;
	LegSwingDegrees[2] = Base;
	LegSwingDegrees[5] = Base;
	LegSwingDegrees[7] = Base;

	// Group B (opposite phase)
	LegSwingDegrees[1] = -Base;
	LegSwingDegrees[3] = -Base;
	LegSwingDegrees[4] = -Base;
	LegSwingDegrees[6] = -Base;
}