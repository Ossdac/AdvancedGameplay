#pragma once

#include "CoreMinimal.h"
#include "AG_ActionBase.h"
#include "UObject/Object.h"
#include "AG_MainMenuAction.generated.h"

/*
 * Main Menu Action that acts as a root of the ActionStack
 */

UCLASS(Blueprintable)
class ADVANCEDGAMEPLAY_API UAG_MainMenuAction : public UAG_ActionBase
{
	GENERATED_BODY()

public:
	// Text to show in the menu
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="MainMenu")
	FString TitleText = TEXT("MAIN MENU - Press Enter to Start");

	// IAction interface
	virtual void OnBegin(bool bFirstTime) override;
	virtual void OnUpdate() override;
	virtual void OnEnd() override;
	virtual bool IsDone() const override;

private:
	// Only true when Exiting the game to keep the main menu action as a root
	bool bDone = false;

	void StartGame();
	void RequestExit();
};
