#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "SpiderCharacter.generated.h"

class UProceduralSpiderGaitComponent;
class USpringArmComponent;
class UCameraComponent;
class UPrimitiveComponent;
class UAG_RigidbodyComponent;

UCLASS()
class ADVANCEDGAMEPLAY_API ASpiderCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	ASpiderCharacter();

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Spider")
	float MaxWalkSpeed = 800.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Spider")
	float RotationRateDegrees = 360.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Spider|Push")
	float PushForce = 250000.0f;

protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	void HandleRawMovement();

	UFUNCTION()
	void OnSpiderHit(
		UPrimitiveComponent* HitComponent,
		AActor* OtherActor,
		UPrimitiveComponent* OtherComp,
		FVector NormalImpulse,
		const FHitResult& Hit);

public:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Spider")
	TObjectPtr<UProceduralSpiderGaitComponent> SpiderGait;

	UPROPERTY(BlueprintReadOnly, Category="Camera")
	TObjectPtr<USpringArmComponent> SpringArm = nullptr;

	UPROPERTY(BlueprintReadOnly, Category="Camera")
	TObjectPtr<UCameraComponent> Camera = nullptr;
};