#include "AG_SpiderCharacter.h"
#include "GameFramework/Controller.h"
#include "GameFramework/CharacterMovementComponent.h"

AAG_SpiderCharacter::AAG_SpiderCharacter()
{
	UCharacterMovementComponent* MoveComp = GetCharacterMovement();
	MoveComp->MaxWalkSpeed = 250.0f;
	MoveComp->bOrientRotationToMovement = true;

	bUseControllerRotationYaw = false;
	bUseControllerRotationPitch = false;
	bUseControllerRotationRoll = false;
}

void AAG_SpiderCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAxis("MoveForward", this, &AAG_SpiderCharacter::MoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &AAG_SpiderCharacter::MoveRight);
	PlayerInputComponent->BindAxis("Turn", this, &AAG_SpiderCharacter::Turn);
	PlayerInputComponent->BindAxis("LookUp", this, &AAG_SpiderCharacter::LookUp);
}

void AAG_SpiderCharacter::MoveForward(float Value)
{
	if (Controller == nullptr || FMath::IsNearlyZero(Value))
	{
		return;
	}

	const FRotator YawRot(0.0f, Controller->GetControlRotation().Yaw, 0.0f);
	const FVector Dir = FRotationMatrix(YawRot).GetUnitAxis(EAxis::X);
	AddMovementInput(Dir, Value);
}

void AAG_SpiderCharacter::MoveRight(float Value)
{
	if (Controller == nullptr || FMath::IsNearlyZero(Value))
	{
		return;
	}

	const FRotator YawRot(0.0f, Controller->GetControlRotation().Yaw, 0.0f);
	const FVector Dir = FRotationMatrix(YawRot).GetUnitAxis(EAxis::Y);
	AddMovementInput(Dir, Value);
}

void AAG_SpiderCharacter::Turn(float Value)
{
	if (FMath::IsNearlyZero(Value))
	{
		return;
	}

	AddControllerYawInput(Value * TurnRateDegPerSec * GetWorld()->GetDeltaSeconds());
}

void AAG_SpiderCharacter::LookUp(float Value)
{
	if (FMath::IsNearlyZero(Value))
	{
		return;
	}

	AddControllerPitchInput(Value * LookUpRateDegPerSec * GetWorld()->GetDeltaSeconds());
}