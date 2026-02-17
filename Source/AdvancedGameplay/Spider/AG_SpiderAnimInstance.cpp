#include "AG_SpiderAnimInstance.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/Character.h"
#include "Kismet/KismetSystemLibrary.h"

static FVector SafeNormalOrUp(const FVector& N)
{
	const float S = N.SizeSquared();
	return (S > 1e-6f) ? (N / FMath::Sqrt(S)) : FVector::UpVector;
}

void UAG_SpiderAnimInstance::NativeInitializeAnimation()
{
	Super::NativeInitializeAnimation();
	SkelComp = GetSkelMeshComponent();
	OwningPawn = TryGetPawnOwner();
	EnsureSizes();

	// Default tetrapod-ish offsets (adjust if your leg indexing differs).
	// Indices assumed: 0..3 left Leg1..4, 4..7 right Leg1..4
	// Group A: 0,2,5,7 (offset 0), Group B: 1,3,4,6 (offset 0.5)
	if (LegRuntime.Num() == 8)
	{
		LegRuntime[0].PhaseOffset = 0.0f;
		LegRuntime[2].PhaseOffset = 0.0f;
		LegRuntime[5].PhaseOffset = 0.0f;
		LegRuntime[7].PhaseOffset = 0.0f;

		LegRuntime[1].PhaseOffset = 0.5f;
		LegRuntime[3].PhaseOffset = 0.5f;
		LegRuntime[4].PhaseOffset = 0.5f;
		LegRuntime[6].PhaseOffset = 0.5f;
	}
}

void UAG_SpiderAnimInstance::NativeUpdateAnimation(float DeltaSeconds)
{
	Super::NativeUpdateAnimation(DeltaSeconds);

	if (SkelComp == nullptr)
	{
		SkelComp = GetSkelMeshComponent();
	}
	if (OwningPawn == nullptr)
	{
		OwningPawn = TryGetPawnOwner();
	}
	if (SkelComp == nullptr || OwningPawn == nullptr)
	{
		return;
	}

	EnsureSizes();
	UpdateVelocity(DeltaSeconds);
	UpdatePhase(DeltaSeconds);
	UpdateLegs(DeltaSeconds);
}

void UAG_SpiderAnimInstance::EnsureSizes()
{
	if (Legs.Num() != 8)
	{
		Legs.SetNum(8);
	}
	if (NominalFootCS.Num() != 8)
	{
		NominalFootCS.SetNum(8);
	}
	if (LegRuntime.Num() != 8)
	{
		LegRuntime.SetNum(8);
	}
}

float UAG_SpiderAnimInstance::Wrap01(float X)
{
	X = FMath::Fmod(X, 1.0f);
	return (X < 0.0f) ? (X + 1.0f) : X;
}

void UAG_SpiderAnimInstance::UpdateVelocity(float /*DeltaSeconds*/)
{
	const FVector V = OwningPawn->GetVelocity();
	Speed = FVector(V.X, V.Y, 0.0f).Size();

	// Desired velocity for foot planning: use planar velocity.
	DesiredVelWS = FVector(V.X, V.Y, 0.0f);
}

void UAG_SpiderAnimInstance::UpdatePhase(float DeltaSeconds)
{
	const float Freq = FMath::Max(0.01f, StepFrequencyHz);
	GaitPhase = Wrap01(GaitPhase + DeltaSeconds * Freq);
}

bool UAG_SpiderAnimInstance::TraceGroundAt(const FVector& QueryWS, FVector& OutPointWS, FVector& OutNormalWS) const
{
	if (SkelComp == nullptr)
	{
		return false;
	}

	const FVector Up = SkelComp->GetUpVector();
	const FVector Start = QueryWS + Up * TraceUp;
	const FVector End = QueryWS - Up * TraceDown;

	FHitResult Hit;
	FCollisionQueryParams Params(SCENE_QUERY_STAT(AG_SpiderFootTrace), false);
	Params.AddIgnoredActor(OwningPawn);

	const bool bHit = GetWorld()->LineTraceSingleByChannel(Hit, Start, End, GroundChannel, Params);
	if (bHit)
	{
		OutPointWS = Hit.ImpactPoint;
		OutNormalWS = SafeNormalOrUp(Hit.ImpactNormal);
		return true;
	}

	return false;
}

void UAG_SpiderAnimInstance::UpdateLegs(float DeltaSeconds)
{
	const float DF = FMath::Clamp(DutyFactor, 0.05f, 0.95f);
	const FTransform CompToWorld = SkelComp->GetComponentTransform();

	for (int32 i = 0; i < 8; ++i)
	{
		FAG_LegRuntime& R = LegRuntime[i];

		// Determine desired phase (stance vs swing) from gait phase + per-leg offset.
		const float P = Wrap01(GaitPhase + R.PhaseOffset);

		const bool bShouldBeStance = (P < DF);
		const EAG_LegPhase Wanted = bShouldBeStance ? EAG_LegPhase::Stance : EAG_LegPhase::Swing;

		// Update normalized phase time within the active segment
		if (Wanted == EAG_LegPhase::Stance)
		{
			R.PhaseT = (DF > 1e-4f) ? (P / DF) : 0.0f;
		}
		else
		{
			const float SwingLen = (1.0f - DF);
			R.PhaseT = (SwingLen > 1e-4f) ? ((P - DF) / SwingLen) : 0.0f;
		}

		// Compute nominal stance query in WS, offset by predicted motion.
		const FVector NominalWS = CompToWorld.TransformPosition(NominalFootCS[i]);

		// Turning-aware planning (simple): shift target forward along desired velocity.
		const FVector Predict = DesiredVelWS * StepHorizonSeconds;
		const FVector QueryWS = NominalWS + Predict;

		// Trace ground at query
		FVector GroundP, GroundN;
		R.bHasGround = TraceGroundAt(QueryWS, GroundP, GroundN);
		if (R.bHasGround)
		{
			R.GroundPointWS = GroundP;
			R.GroundNormalWS = GroundN;
		}

		// Determine foot target on ground
		if (R.bHasGround)
		{
			R.FootTargetWS = R.GroundPointWS + R.GroundNormalWS * FootClearance;
		}
		else
		{
			// fallback: keep nominal height
			R.FootTargetWS = QueryWS;
			R.GroundNormalWS = SkelComp->GetUpVector();
		}

		// Transition logic
		if (R.Phase != Wanted)
		{
			R.Phase = Wanted;
			// When entering stance: lock current at the best grounded position
			if (R.Phase == EAG_LegPhase::Stance)
			{
				R.bFootHasLock = true;
				// Lock to current if it exists, else to target.
				R.FootLockWS = (R.FootCurrentWS.IsNearlyZero()) ? R.FootTargetWS : R.FootCurrentWS;
			}
			else
			{
				// entering swing: ensure lock exists
				if (!R.bFootHasLock)
				{
					R.bFootHasLock = true;
					R.FootLockWS = R.FootTargetWS;
				}
			}
		}

		// Step trigger: if in stance and foot drifted too far from nominal, schedule swing by forcing lock->target.
		if (R.Phase == EAG_LegPhase::Stance && R.bFootHasLock)
		{
			const float Err = FVector::Dist2D(R.FootLockWS, NominalWS);
			if (Err > StepTriggerDist)
			{
				// Force this leg to swing by setting its offset slightly ahead into swing region.
				// Minimal: just update lock to current and keep target; the gait function will swing it when its time comes.
				R.FootLockWS = R.bHasGround ? (R.GroundPointWS + R.GroundNormalWS * FootClearance) : NominalWS;
			}

			R.FootCurrentWS = R.FootLockWS;

			// Ensure not below ground plane
			if (R.bHasGround)
			{
				const float Pen = FVector::DotProduct(R.FootCurrentWS - R.GroundPointWS, R.GroundNormalWS);
				if (Pen < FootClearance)
				{
					R.FootCurrentWS += R.GroundNormalWS * (FootClearance - Pen);
				}
			}
		}
		else // Swing
		{
			const float S = FMath::Clamp(R.PhaseT, 0.0f, 1.0f);
			const FVector A = R.bFootHasLock ? R.FootLockWS : R.FootTargetWS;
			const FVector B = R.FootTargetWS;

			FVector Pos = FMath::Lerp(A, B, S);

			// Swing arc along component up
			const FVector Up = SkelComp->GetUpVector();
			const float Lift = SwingHeight * FMath::Sin(PI * S);
			Pos += Up * Lift;

			R.FootCurrentWS = Pos;

			// Clamp to not penetrate ground
			if (R.bHasGround)
			{
				const float Pen = FVector::DotProduct(R.FootCurrentWS - R.GroundPointWS, R.GroundNormalWS);
				if (Pen < FootClearance)
				{
					R.FootCurrentWS += R.GroundNormalWS * (FootClearance - Pen);
				}
			}
		}
	}
}