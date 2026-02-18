#include "SpiderAnimInstance.h"
#include "ProceduralSpiderGaitComponent.h"
#include "GameFramework/Pawn.h"

void USpiderAnimInstance::NativeUpdateAnimation(float DeltaSeconds)
{
	APawn* Pawn = TryGetPawnOwner();
	if (!Pawn) return;

	SpiderGait = Pawn->FindComponentByClass<UProceduralSpiderGaitComponent>();
	if (!SpiderGait) return;

	IKTargets.SetNum(8);
	
	USkeletalMeshComponent* SkelComp = GetSkelMeshComponent();
	
	for (int i = 0; i < 8; ++i)
	{
		ESpiderLeg Leg = static_cast<ESpiderLeg>(i);
		FTransform Target;
		if (SpiderGait->GetIKTarget(Leg, Target))
		{
			if (SkelComp)
			{
				IKTargets[i] = Target.GetRelativeTransform(SkelComp->GetComponentTransform());
			}
		}
	}
}

bool USpiderAnimInstance::GetIKTarget(ESpiderLeg Leg, FTransform& OutWorldTarget) const
{
	const int32 Index = static_cast<int32>(Leg);
	if (!IKTargets.IsValidIndex(Index))
	{
		return false;
	}

	OutWorldTarget = IKTargets[Index];
	return true;
}
