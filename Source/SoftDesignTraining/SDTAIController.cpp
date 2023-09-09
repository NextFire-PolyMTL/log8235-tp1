// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTAIController.h"
#include "SoftDesignTraining.h"

void ASDTAIController::BeginPlay()
{
    Super::BeginPlay();
    auto *chara = GetCharacter();
    auto *moveComp = chara->GetCharacterMovement();
    moveComp->MaxWalkSpeed = MaxSpeed;
    moveComp->MaxAcceleration = Acceleration;
}

void ASDTAIController::Tick(float deltaTime)
{
    Super::Tick(deltaTime);
    Move();
}

void ASDTAIController::Move()
{
    auto *pawn = GetPawn();
    // rotate
    auto rot = FQuat::FindBetween(pawn->GetActorForwardVector(), TargetDir);
    pawn->AddActorWorldRotation(rot);
    // move forward
    pawn->AddMovementInput(pawn->GetActorForwardVector());
    // debug print
    GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Yellow, FString::Printf(TEXT("[%s] Velocity: %f m/s"), *pawn->GetName(), pawn->GetVelocity().Size()));
}
