// Fill out your copyright notice in the Description page of Project Settings.


#include "AG_PlayAction.h"

void UAG_PlayAction::OnBegin(bool bFirstTime)
{
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(
			-1, 5.f, FColor::Green,
			"Game Started: OnBegin"
		);
	}
}

void UAG_PlayAction::OnUpdate()
{
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(
			-1, .1f, FColor::Green,
			"Holy cow this is fun playing the game!"
		);
	}
}

void UAG_PlayAction::OnEnd()
{
}

bool UAG_PlayAction::IsDone() const
{
	return bDone;
}
