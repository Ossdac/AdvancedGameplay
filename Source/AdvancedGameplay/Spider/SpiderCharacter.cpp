#include "SpiderCharacter.h"
#include "ProceduralSpiderGaitComponent.h"
#include "AdvancedGameplay/Physics/AG_RigidbodyComponent.h"

#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Components/InputComponent.h"
#include "Components/CapsuleComponent.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"

ASpiderCharacter::ASpiderCharacter()
{
	PrimaryActorTick.bCanEverTick = true;

	bUseControllerRotationYaw = false;
	bUseControllerRotationPitch = false;
	bUseControllerRotationRoll = false;

	GetCharacterMovement()->Deactivate();
	GetCharacterMovement()->SetComponentTickEnabled(false);

	SpiderGait = CreateDefaultSubobject<UProceduralSpiderGaitComponent>(TEXT("SpiderGait"));
	SpiderBody = CreateDefaultSubobject<UAG_RigidbodyComponent>(TEXT("SpiderBody"));
}

void ASpiderCharacter::BeginPlay()
{
	Super::BeginPlay();

	SpringArm = FindComponentByClass<USpringArmComponent>();
	Camera = FindComponentByClass<UCameraComponent>();

	APlayerController* PlayerController = Cast<APlayerController>(GetController());
	if (PlayerController)
	{
		PlayerController->SetInputMode(FInputModeGameOnly());
		PlayerController->bShowMouseCursor = false;
	}

	UCapsuleComponent* Capsule = GetCapsuleComponent();
	if (SpiderBody && Capsule)
	{
		SpiderBody->SetUpdatedComponent(Capsule);
		SpiderBody->SetRotationEnabled(false, true);
	}
}

void ASpiderCharacter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (bGameplayActive)
	{
		HandleRawMovement();
		UpdateVisualFacing();
	}

	if (SpiderGait)
	{
		if (bGameplayActive && SpiderBody)
		{
			SpiderGait->CommandedSpeed = SpiderBody->GetPlanarSpeed();
		}
		else
		{
			SpiderGait->CommandedSpeed = 0.0f;
		}
	}
}

void ASpiderCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void ASpiderCharacter::HandleRawMovement()
{
	APlayerController* PlayerController = Cast<APlayerController>(GetController());
	if (!PlayerController || !Controller || !SpiderBody)
	{
		return;
	}

	float MouseX = 0.0f;
	float MouseY = 0.0f;
	PlayerController->GetInputMouseDelta(MouseX, MouseY);

	if (!FMath::IsNearlyZero(MouseX))
	{
		AddControllerYawInput(MouseX * MouseYawSpeed);
	}

	if (!FMath::IsNearlyZero(MouseY))
	{
		AddControllerPitchInput(-MouseY * MousePitchSpeed);
	}

	float ForwardValue = 0.0f;
	if (PlayerController->IsInputKeyDown(EKeys::W))
	{
		ForwardValue = 1.0f;
	}

	const FRotator ControlRotation = Controller->GetControlRotation();
	const FRotator YawRotation(0.0f, ControlRotation.Yaw, 0.0f);
	const FVector CameraForward = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X);

	if (!FMath::IsNearlyZero(ForwardValue))
	{
		SpiderBody->AddDriveDirectionInput(CameraForward, ForwardValue, DriveAcceleration, DriveMaxSpeed);
	}
	else
	{
		SpiderBody->StopMotion();
	}
}

void ASpiderCharacter::UpdateVisualFacing()
{
	if (!Controller)
	{
		return;
	}

	const FRotator ControlRotation = Controller->GetControlRotation();
	const FRotator YawRotation(0.0f, ControlRotation.Yaw, 0.0f);
	SetActorRotation(YawRotation);
}

FVector ASpiderCharacter::GetMovementVelocity() const
{
	if (SpiderBody)
	{
		return SpiderBody->GetPlanarVelocity();
	}

	return GetVelocity();
}

void ASpiderCharacter::SetGameplayActive(bool bActive)
{
	bGameplayActive = bActive;

	if (!bGameplayActive && SpiderBody)
	{
		SpiderBody->StopMotion(true, true, true);
	}

	if (SpiderGait)
	{
		SpiderGait->CommandedSpeed = 0.0f;
	}
}