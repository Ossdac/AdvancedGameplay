#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "SpiderCharacter.generated.h"

class UProceduralSpiderGaitComponent;

UCLASS()
class ADVANCEDGAMEPLAY_API ASpiderCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	ASpiderCharacter();

protected:
	virtual void BeginPlay() override;

public:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Spider")
	TObjectPtr<UProceduralSpiderGaitComponent> SpiderGait;
};