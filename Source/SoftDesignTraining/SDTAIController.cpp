// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTAIController.h"
#include "SoftDesignTraining.h"

void ASDTAIController::Tick(float deltaTime)
{
    Super::Tick(deltaTime);
    auto *pawn = GetPawn();
    auto targetDir = FVector::LeftVector; // placeholder
    // rotate
    auto rot = FQuat::FindBetween(pawn->GetActorForwardVector(), targetDir);
    pawn->AddActorWorldRotation(rot);
    // move forward
    m_Speed = FMath::Min(m_Speed + Acceleration * deltaTime, MaxSpeed);
    pawn->AddMovementInput(pawn->GetActorForwardVector(), m_Speed * deltaTime);
    // debug print
    GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Yellow, FString::Printf(TEXT("[%s] Velocity: %f m/s"), *pawn->GetName(), pawn->GetVelocity().Size()));
}
