#include "SpiderAnimInstance.h"
#include "ProceduralSpiderGaitComponent.h"
#include "GameFramework/Pawn.h"

void USpiderAnimInstance::NativeUpdateAnimation(float DeltaSeconds)
{
	Super::NativeUpdateAnimation(DeltaSeconds);

	APawn* Pawn = TryGetPawnOwner();
	if (!Pawn) return;

	SpiderGait = Pawn->FindComponentByClass<UProceduralSpiderGaitComponent>();
	if (!SpiderGait) return;

	USkeletalMeshComponent* SkelComp = GetSkelMeshComponent();
	if (!SkelComp) return;

	// IMPORTANT: don't "create validity" by SetNum every tick
	if (IKTargets.Num() != 8)
	{
		IKTargets.SetNum(8);
		bTargetsValid = false;
	}

	const FTransform ComponentWorld = SkelComp->GetComponentTransform();

	bool bGotAtLeastOne = false;

	for (int32 i = 0; i < 8; ++i)
	{
		const ESpiderLeg Leg = static_cast<ESpiderLeg>(i);

		FTransform TargetWorld;
		if (SpiderGait->GetIKTarget(Leg, TargetWorld))
		{
			IKTargets[i] = TargetWorld.GetRelativeTransform(ComponentWorld);
			bGotAtLeastOne = true;
		}
	}

	if (bGotAtLeastOne)
	{
		bTargetsValid = true;
	}
}

void USpiderAnimInstance::NativeInitializeAnimation()
{
	Super::NativeInitializeAnimation();

	IKTargets.SetNum(8);
	bTargetsValid = false;
}

bool USpiderAnimInstance::GetIKTarget(ESpiderLeg Leg, FTransform& OutWorldTarget) const
{
	if (!bTargetsValid)
	{
		return false;
	}

	const int32 Index = static_cast<int32>(Leg);
	if (!IKTargets.IsValidIndex(Index))
	{
		return false;
	}

	OutWorldTarget = IKTargets[Index];
	return true;
}