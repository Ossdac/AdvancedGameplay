#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "AdvancedGameplay/ActionStack/FActionStack.h"
#include "AG_GameInstance.generated.h"

UCLASS()
class ADVANCEDGAMEPLAY_API UAG_GameInstance : public UGameInstance
{
	GENERATED_BODY()

public:
	virtual void Init() override;
	
	FActionStack& GetActionStack() { return ActionStack; }
	const FActionStack& GetActionStack() const { return ActionStack; }

private:
	FActionStack ActionStack;
};
