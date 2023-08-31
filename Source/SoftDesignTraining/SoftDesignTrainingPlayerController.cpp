// Copyright 1998-2015 Epic Games, Inc. All Rights Reserved.

#include "SoftDesignTrainingPlayerController.h"
#include "SoftDesignTraining.h"
//#include "AI/Navigation/NavigationSystem.h"
#include "SoftDesignTrainingCharacter.h"

#include "DrawDebugHelpers.h"

ASoftDesignTrainingPlayerController::ASoftDesignTrainingPlayerController()
{
}

void ASoftDesignTrainingPlayerController::SetupInputComponent()
{
    Super::SetupInputComponent();

    InputComponent->BindAxis("MoveForward", this, &ASoftDesignTrainingPlayerController::MoveForward);
    InputComponent->BindAxis("MoveRight", this, &ASoftDesignTrainingPlayerController::MoveRight);
}

void ASoftDesignTrainingPlayerController::MoveForward(float value)
{
    APawn* const pawn = GetPawn();

    if (pawn)
    {
        pawn->AddMovementInput(FVector(value, 0.0f, 0.0f));
    }
}

void ASoftDesignTrainingPlayerController::MoveRight(float value)
{
    APawn* const pawn = GetPawn();

    if (pawn)
    {
        pawn->AddMovementInput(FVector(0.0f, value, 0.0f));
    }
}