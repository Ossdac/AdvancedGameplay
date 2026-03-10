#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "SpiderCharacter.generated.h"

class UProceduralSpiderGaitComponent;
class USpringArmComponent;
class UCameraComponent;
class UAG_RigidbodyComponent;

UCLASS()
class ADVANCEDGAMEPLAY_API ASpiderCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	ASpiderCharacter();

protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	void HandleRawMovement();
	void UpdateVisualFacing();

public:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Spider")
	TObjectPtr<UProceduralSpiderGaitComponent> SpiderGait;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Spider")
	TObjectPtr<UAG_RigidbodyComponent> SpiderBody;

	UPROPERTY(BlueprintReadOnly, Category="Camera")
	TObjectPtr<USpringArmComponent> SpringArm = nullptr;

	UPROPERTY(BlueprintReadOnly, Category="Camera")
	TObjectPtr<UCameraComponent> Camera = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Spider|Movement")
	float DriveAcceleration = 4500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Spider|Movement")
	float DriveMaxSpeed = 700.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Spider|Movement")
	float BrakeStrength = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Spider|Look")
	float MouseYawSpeed = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Spider|Look")
	float MousePitchSpeed = 1.0f;
	
	UFUNCTION(BlueprintCallable, BlueprintPure)
	FVector GetMovementVelocity() const;
	
public:
	UFUNCTION(BlueprintCallable)
	void SetGameplayActive(bool bActive);

	UFUNCTION(BlueprintCallable, BlueprintPure)
	bool IsGameplayActive() const { return bGameplayActive; }

private:
	bool bGameplayActive = false;
};