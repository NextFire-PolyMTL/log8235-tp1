// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTAIController.h"
#include "SoftDesignTraining.h"
#include "SDTUtils.h"

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
    DetectWalls();
    Move();
}

void ASDTAIController::DetectWalls()
{
    auto *world = GetWorld();
    auto *pawn = GetPawn();

    auto forwardVector = pawn->GetActorForwardVector();
    auto rightVector = pawn->GetActorRightVector();
    auto leftVector = -rightVector;

    auto loc = pawn->GetActorLocation();
    auto forwardHit = SDTUtils::Raycast(world, loc, loc + forwardVector * WallRayCastDist);
    auto leftHit = SDTUtils::Raycast(world, loc, loc + leftVector * WallRayCastDist);
    auto rightHit = SDTUtils::Raycast(world, loc, loc + rightVector * WallRayCastDist);

    DrawDebugLine(world, loc, loc + leftVector * WallRayCastDist, FColor::Red, false, -1.0f, 0u, leftHit ? 20.0f : 0.0f);
    DrawDebugLine(world, loc, loc + forwardVector * WallRayCastDist, FColor::Green, false, -1.0f, 0u, forwardHit ? 20.0f : 0.0f);
    DrawDebugLine(world, loc, loc + rightVector * WallRayCastDist, FColor::Blue, false, -1.0f, 0u, rightHit ? 20.0f : 0.0f);

    if (forwardHit)
    {
        if (leftHit && rightHit) // dead end
        {
            TargetDir = -forwardVector;
        }
        else if (leftHit && !rightHit)
        {
            TargetDir = rightVector;
        }
        else if (rightHit && !leftHit)
        {
            TargetDir = leftVector;
        }
        else
        {
            // pick a random direction for now
            TargetDir = FMath::RandBool() ? leftVector : rightVector;
        }
    }
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
